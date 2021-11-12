/**
 * @file CodeReleaseKickAtGoalCard.cpp
 *
 * This file implements a basic striker behavior for the code release.
 * Normally, this would be decomposed into at least
 * - a ball search behavior card
 * - a skill for getting behind the ball
 *
 * @author Arne Hasselbring
 */


#include <fstream>
#include <iostream>
#include <math.h>
#include <unistd.h>
#include <string>


#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Module/Blackboard.h"
#include "Tools/Math/BHMath.h"
#include "Tools/Math/Eigen.h"
#include "Tools/Module/Module.h"
#include "Tools/NeuralNetwork/CompiledNN.h"
#include "Tools/NeuralNetwork/Model.h"
#include "Tools/NeuralNetwork/Tensor.h"
#include "Tools/NeuralNetwork/json.h"
#include "Tools/NeuralNetwork/SimpleNN.h"
#include "Tools/Streams/OutStreams.h"
#include "Controller/RoboCupCtrl.h"

#define STATS_GO_INLINE
#define STATS_DONT_USE_OPENMP
#define STATS_ENABLE_EIGEN_WRAPPERS
#define STATS_ENABLE_STDVEC_WRAPPERS
#include "Tools/NeuralNetwork/stats.hpp"
#include <fstream>


#define PI 3.14159265
#define BATCHSIZE 20
#define DEBUG_MODE true


int batchStep  = 0; // frame index number within batch we are on
int episodeStep = -1; // the episode needs to complete one step before we can start logging


      

auto trajectories = json::object{}; // an object to hold the curret batch of trajectories
std::ifstream metadataFile("../metadata.json");
json::value metadata = json::parse(metadataFile);
std::string actionLengthString = to_string(metadata["action_length"]);
unsigned int actionLength = (unsigned int)(std::stoi(actionLengthString));
std::string observationLengthString = to_string(metadata["observation_length"]);
unsigned int observationLength = (unsigned int)(std::stoi(observationLengthString));
std::vector<float> prevObservation;
std::vector<float> currentObservation;
int prevDone = 1; // whether this frame is the first frame of an episode or not
std::vector<float> prevAction;
std::vector<float> currentAction;
double prevLogProb;
double currentLogProb;
float currentValue;
float prevValue;


 

std::string getCurrentDirectory()
{
  char buff[FILENAME_MAX]; //create string buffer to hold path
  getcwd( buff, FILENAME_MAX );
  std::string currentWorkingDir(buff);
  return currentWorkingDir;
}



void debugPrintString(std::string s){
  if(DEBUG_MODE){
  std::cout << s << std::endl;
  }
}


void debugPrintFloatVector(std::vector<float> v){
  if (DEBUG_MODE)
  {
  for (auto i : v)
  {
    std::cout << i << std::endl;
  }
  }
}


// derived from https://stackoverflow.com/questions/12774207/fastest-way-to-check-if-a-file-exist-using-standard-c-c11-14-17-c


  bool doesFileExist (const std::string& name) {
    std::ifstream f(name.c_str());
    return f.good();
  }


void waitForNewPolicy()
{
  while (true)
  {
    if (!doesFileExist(getCurrentDirectory() + "/../shared_policy.h5"))
    {
      debugPrintString("waiting for shared policy");
      continue;
    }
    if (!doesFileExist(getCurrentDirectory() +"/../action_policy.h5"))
    {
      debugPrintString("waiting for action policy");
      continue;
    }
    if (!doesFileExist(getCurrentDirectory() +"/../value_policy.h5"))
    {
      debugPrintString("waiting for value policy");
      continue;
    }
    if (!doesFileExist(getCurrentDirectory() +"/../metadata.json"))
    {
      debugPrintString("waiting for meta data");
      continue;
    }
    break;
  }
}




json::object newTrajectoriesJSON()
{
        json::object emptyTrajectories = json::object{};
        emptyTrajectories.insert("last_values", json::array{});
        emptyTrajectories.insert("length", BATCHSIZE);
        emptyTrajectories.insert("observations", json::array{});
        emptyTrajectories.insert("episode_starts", json::array{});
        emptyTrajectories.insert("values",json::array{});
        emptyTrajectories.insert("actions",json::array{});
        emptyTrajectories.insert("log_probs", json::array{});
        return emptyTrajectories;
}

json::array floatVectToJSON(std::vector<float> inputVector)
{
   
        json::array output = json::array{};
        for (float i: inputVector)
        {
          output.push_back(i);
          //std::cout << i << std::endl;
        }
        return output;
}



// returns an observation from the environment as a vector
std::vector<float> getObservation(GroundTruthRobotPose pose)
{
  std::vector<float> observationVector(4); // observation length is hardcoded for the time being
  double x = pose.translation[0];
  double y = pose.translation[1];
  double angle = pose.rotation;
  double sinAngle = sin(angle* (PI/180));
  double cosAngle = cos(angle* (PI/180));
  observationVector[0] = x/4500.0;
  observationVector[1] = y/3000.0;
  observationVector[2] = sinAngle;
  observationVector[3] = cosAngle;
  return observationVector;
}


CARD(CodeReleaseKickAtGoalCard,
{,
  CALLS(Activity),
  CALLS(InWalkKick),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(1000) initialWaitTime,
    (int)(7000) ballNotSeenTimeout,
    (Angle)(5_deg) ballAlignThreshold,
    (float)(500.f) ballNearThreshold,
    (Angle)(10_deg) angleToGoalThreshold,
    (float)(400.f) ballAlignOffsetX,
    (float)(100.f) ballYThreshold,
    (Angle)(2_deg) angleToGoalThresholdPrecise,
    (float)(150.f) ballOffsetX,
    (Rangef)({140.f, 170.f}) ballOffsetXRange,
    (float)(40.f) ballOffsetY,
    (Rangef)({20.f, 50.f}) ballOffsetYRange,
    (int)(10) minKickWaitTime,
    (int)(3000) maxKickWaitTime,
  }),
});

class CodeReleaseKickAtGoalCard : public CodeReleaseKickAtGoalCardBase
{
  bool preconditions() const override
  {
    return true;
  }

  bool postconditions() const override
  {
    return true;
  }

  option
  {
    theActivitySkill(BehaviorStatus::codeReleaseKickAtGoal);

    initial_state(start)
    {
      // transition
      // {
      //   if(state_time > initialWaitTime)
      //     goto turnToBall;
      // }

      action
      {
        // theLookForwardSkill();
        // theStandSkill();
        {


       if(Blackboard::getInstance().exists("GroundTruthRobotPose")){
       const GroundTruthRobotPose& theGroundTruthRobotPose = static_cast<const GroundTruthRobotPose&>(Blackboard::getInstance()["GroundTruthRobotPose"]); 



        NeuralNetwork::CompilationSettings settings;  // not sure if this is necessary but it's unused since we're not compiling but was required
        // for apply function signature
       
        
       
        NeuralNetwork::Model sharedModel("shared_policy.h5");
        NeuralNetwork::Model actionModel("action_policy.h5");
        NeuralNetwork::Model valueModel("value_policy.h5");

        


        SimRobotCore2::Scene* scene = (SimRobotCore2::Scene*)RoboCupCtrl::application->resolveObject("RoboCup", SimRobotCore2::scene);

        /*
        if (scene != NULL) {
          episodeStep = scene->getStep();
        }
        else{
          std::cout << "FATAL ERROR: scene is NULL" << std::endl;
          exit(1);
        }
        */
        currentObservation = getObservation(theGroundTruthRobotPose);



        if (episodeStep == 0)
        {
          prevDone = true;
        }
        else if(episodeStep > 0){
          prevDone = false;
        }


        if (batchStep == 0) // setting up our first trajectories file descriptor, this will be exchanged on each 
        {
          trajectories = newTrajectoriesJSON();
        }
  

       

        //outputFile << "test" << '\n';

        
        //std::cout << "exists" << std::endl;
        



        //std::ifstream metadataFile("/home/john/BHumanCodeRelease/Config/NeuralNets/metadata.json");
        


        /*std::cout <<  "OBSERVATION LENGTH" << std::endl;
        std::cout << observationLength << std::endl;

        std::cout <<  "ACTION LENGTH" << std::endl;
        std::cout << actionLengthString << std::endl;*/

        auto logStdArray = metadata["log_stds"];


        Eigen::MatrixXd  stdDevs(1,1);
        stdDevs.resize(actionLength,1);
        //std::cout << "Reached" << std::endl;


        const json::array &stdArray = as_array(logStdArray);
        int index = 0;
        for (auto i =  stdArray.begin(); i != stdArray.end(); i++)
        {
          const json::value &logStd = *i;

          double logStdDouble = (std::stod(to_string(logStd)));
          //std::cout << "Reached" << std::endl;

          stdDevs(index) = exp(logStdDouble);
          index += 1;
          

        }
        //std::cout << "Reached" << std::endl;

        
        
        Eigen::MatrixXd covarianceMatrix(1,1);
        covarianceMatrix.resize(actionLength, actionLength);
        //std::cout << "Reached" << std::endl;

        covarianceMatrix = stdDevs.array().matrix().asDiagonal();
        


     
    
        

        std::vector<NeuralNetwork::TensorXf> sharedOutputs(sharedModel.getOutputs().size());

        std::vector<NeuralNetwork::TensorXf> observationInput(sharedModel.getInputs().size());
        //std::cout << "policy load and input setup complete" << std::endl;
        //reshaping but not sure why, derived from check.cpp
        const std::vector<NeuralNetwork::TensorLocation>& inputs = sharedModel.getInputs();
        for(std::size_t i = 0; i < observationInput.size(); ++i)
        {
          observationInput[i].reshape(inputs[i].layer->nodes[inputs[i].nodeIndex].outputDimensions[inputs[i].tensorIndex]);
        }



        observationInput[0][0] = currentObservation[0];
        observationInput[0][1] = currentObservation[1];
        observationInput[0][2] = currentObservation[2];
        observationInput[0][3] = currentObservation[3];

        //std::cout << "OBSERVATION :" << std::endl;

        /*
        auto newObs = json::array{};
        for (float i: observationInput[0])
        {
          newObs.push_back(i);
          //std::cout << i << std::endl;
        }
        trajectories["observations"].push_back(newObs);
        */
        
        
        //std::cout << "reached pre apply" << std::endl;
        NeuralNetwork::SimpleNN::apply(observationInput, sharedOutputs, sharedModel, [&settings](const NeuralNetwork::Node& node, const std::vector<const NeuralNetwork::TensorXf*>& inputs, const std::vector<NeuralNetwork::TensorXf*>& outputs)
        {
        });


        //std::cout << "simpleNN test" << std::endl;

        for (float i: sharedOutputs[0])
        {
          //std::cout << i << ",";
        }
        //std::cout << "" << std::endl;
        



        //std::cout << "simplNN test complete" << std::endl;


        NeuralNetwork::TensorXf latentAction =  sharedOutputs[0];
        NeuralNetwork::TensorXf latentValue =  sharedOutputs[1];



        //std::cout << "SHARED LAYERS OUTPUT 1 :" << std::endl;

        for (float i: latentAction)
        {
          //std::cout << i << std::endl;
        }
        //std::cout << "SHARED LAYERS OUTPUT 2 :" << std::endl;

        for (float i: latentValue)
        {
          //std::cout << i << std::endl;
        }

        



        std::vector<NeuralNetwork::TensorXf> valueInput(valueModel.getInputs().size());
        valueInput[0] = latentValue;

        std::vector<NeuralNetwork::TensorXf> valueOutput(valueModel.getOutputs().size());


        NeuralNetwork::SimpleNN::apply(valueInput, valueOutput, valueModel, [&settings](const NeuralNetwork::Node& node, const std::vector<const NeuralNetwork::TensorXf*>& inputs, const std::vector<NeuralNetwork::TensorXf*>& outputs)
        {
        });


        //std::cout << "VALUE NET OUTPUT :" << std::endl;
        NeuralNetwork::TensorXf valueEstimate =  valueOutput[0];

        for (float i: valueEstimate)
        {
          //std::cout << i << std::endl;
        }
        //trajectories["values"].push_back(valueEstimate(0));
        currentValue = valueEstimate(0);


        std::vector<NeuralNetwork::TensorXf> actionPolicyInput(actionModel.getInputs().size());
        actionPolicyInput[0] = latentAction;

        std::vector<NeuralNetwork::TensorXf> actionPolicyOutput(actionModel.getOutputs().size());


        NeuralNetwork::SimpleNN::apply(actionPolicyInput, actionPolicyOutput, actionModel, [&settings](const NeuralNetwork::Node& node, const std::vector<const NeuralNetwork::TensorXf*>& inputs, const std::vector<NeuralNetwork::TensorXf*>& outputs)
        {
        });




        //std::cout << "ACTION NET OUTPUT: " << std::endl;
        NeuralNetwork::TensorXf actionMeans =  actionPolicyOutput[0];

        Eigen::MatrixXd actionEigen(actionLength,1);

        
        for (unsigned int i = 0; i < actionMeans.size(); i ++)
        {
          actionEigen(i) = actionMeans[i];
        }

        /*
        std::cout << "mu vector: " << std::endl;

        std::cout << actionEigen << std::endl;
        std::cout << "cov matrix: " << std::endl;

        std::cout << covarianceMatrix << std::endl;
        */


        Eigen::MatrixXd actionChoice = stats::rmvnorm(actionEigen, covarianceMatrix, true);
            
        //std::cout << "action: " << std::endl;

        //std::cout << actionChoice << std::endl;

        currentLogProb = stats::dmvnorm(actionChoice,actionEigen, covarianceMatrix, true);



        //trajectories["log_probs"].push_back(logProb);
        //std::cout << "action log prob: " << std::endl;

        //std::cout << logProb << std::endl;

        currentAction = std::vector<float>();
        for (unsigned int i = 0; i < actionChoice.size(); i++)
        {
          currentAction.push_back(actionChoice(i));
        }

        

        if (episodeStep > -1)
        {
          trajectories["episode_starts"].push_back(prevDone);
          trajectories["observations"].push_back(floatVectToJSON(prevObservation));
          trajectories["actions"].push_back(floatVectToJSON(prevAction));
          trajectories["values"].push_back(prevValue);
          trajectories["last_values"].push_back(currentValue);
          trajectories["log_probs"].push_back(prevLogProb);



          debugPrintFloatVector(prevObservation);
          debugPrintFloatVector(currentObservation);
          std::cout << "open" << std::endl;
          std::cout << currentValue << std::endl;
          std::cout << prevValue << std::endl;
          std::cout << "close" << std::endl;

          //log here
        }


        //logging here




          std::cout<<episodeStep<<std::endl;            
          
         

        
        
        
        
        if (episodeStep > -1)
        {
          batchStep += 1;
        }
        episodeStep += 1;


        if (episodeStep >= 10) {
          std::cout << stringify(trajectories, json::PrettyPrint) << std::endl;
          RoboCupCtrl::application->resetSimulation();
          episodeStep = 0;
          if (batchStep >= BATCHSIZE)
          {
            waitForNewPolicy();
            //load new policy
            //delete new policy from filesystem
            trajectories = newTrajectoriesJSON();
          }
          
        }
      
        prevObservation = std::vector<float>(currentObservation);
        prevAction = std::vector<float>(currentAction);
        prevValue = currentValue;
        prevLogProb = currentLogProb;

        theWalkAtRelativeSpeedSkill(Pose2f(actionChoice(0),actionChoice(1), actionChoice(2)));



        //currentObservation = getObservation(theGroundTruthRobotPose);
        //debugPrintFloatVector(prevObservation);
        //debugPrintFloatVector(currentObservation);
        //exit(1);


        }
        else{
       
                theWalkAtRelativeSpeedSkill(Pose2f(0.f, 0.f, 0.f));
        }

        }
      
      }
    }

    // state(turnToBall)
    // {
    //   transition
    //   {
    //     if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
    //       goto searchForBall;
    //     if(std::abs(theFieldBall.positionRelative.angle()) < ballAlignThreshold)
    //       goto walkToBall;
    //   }

    //   action
    //   {
    //     theLookForwardSkill();
    //     theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theFieldBall.positionRelative.angle(), 0.f, 0.f));
    //   }
    // }

    // state(walkToBall)
    // {
    //   transition
    //   {
    //     if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
    //       goto searchForBall;
    //     if(theFieldBall.positionRelative.squaredNorm() < sqr(ballNearThreshold))
    //       goto alignToGoal;
    //   }

    //   action
    //   {
    //     theLookForwardSkill();
    //     theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), theFieldBall.positionRelative);
    //   }
    // }

    // state(alignToGoal)
    // {
    //   const Angle angleToGoal = calcAngleToGoal();

    //   transition
    //   {
    //     if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
    //       goto searchForBall;
    //     if(std::abs(angleToGoal) < angleToGoalThreshold && std::abs(theFieldBall.positionRelative.y()) < ballYThreshold)
    //       goto alignBehindBall;
    //   }

    //   action
    //   {
    //     theLookForwardSkill();
    //     theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballAlignOffsetX, theFieldBall.positionRelative.y()));
    //   }
    // }

    // state(alignBehindBall)
    // {
    //   const Angle angleToGoal = calcAngleToGoal();

    //   transition
    //   {
    //     if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
    //       goto searchForBall;
    //     if(std::abs(angleToGoal) < angleToGoalThresholdPrecise && ballOffsetXRange.isInside(theFieldBall.positionRelative.x()) && ballOffsetYRange.isInside(theFieldBall.positionRelative.y()))
    //       goto kick;
    //   }

    //   action
    //   {
    //     theLookForwardSkill();
    //     theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
    //   }
    // }

    // state(kick)
    // {
    //   const Angle angleToGoal = calcAngleToGoal();

    //   transition
    //   {
    //     if(state_time > maxKickWaitTime || (state_time > minKickWaitTime && theInWalkKickSkill.isDone()))
    //       goto start;
    //   }

    //   action
    //   {
    //     theLookForwardSkill();
    //     theInWalkKickSkill(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
    //   }
    // }

    // state(searchForBall)
    // {
    //   transition
    //   {
    //     if(theFieldBall.ballWasSeen())
    //       goto turnToBall;
    //   }

    //   action
    //   {
    //     theLookForwardSkill();
    //     theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
    //   }
    // }
  }

  // Angle calcAngleToGoal() const
  // {
  //   return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
  // }
};

MAKE_CARD(CodeReleaseKickAtGoalCard);
