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



#define STATS_GO_INLINE
#define STATS_DONT_USE_OPENMP
#define STATS_ENABLE_EIGEN_WRAPPERS
#define STATS_ENABLE_STDVEC_WRAPPERS
#include "Tools/NeuralNetwork/stats.hpp"


#define PI 3.14159265

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
        std::cout << "exists" << std::endl;
        const GroundTruthRobotPose& theGroundTruthRobotPose = static_cast<const GroundTruthRobotPose&>(Blackboard::getInstance()["GroundTruthRobotPose"]); \
        double x = theGroundTruthRobotPose.translation[0];
        double y = theGroundTruthRobotPose.translation[1];
        double angle = theGroundTruthRobotPose.rotation;

        std::cout<< "x" << std::endl;
        std::cout << x << std::endl;
        std::cout<< "y" << std::endl;
        std::cout << y << std::endl;
        std::cout<< "angle" << std::endl;
        std::cout << angle << std::endl;
        double sinAngle = sin(angle* (PI/180));
        double cosAngle = cos(angle* (PI/180));

        char buff[FILENAME_MAX]; //create string buffer to hold path
        getcwd( buff, FILENAME_MAX );
        std::string currentWorkingDir(buff);
        std::cout << currentWorkingDir << std::endl;
                
        //std::ifstream metadataFile("/home/john/BHumanCodeRelease/Config/NeuralNets/metadata.json");
        std::ifstream metadataFile("../../Config/NeuralNets/metadata.json");

        json::value metadata = json::parse(metadataFile);
        
        std::string observationLengthString = to_string(metadata["observation_length"]);
        unsigned int observationLength = (unsigned int)(std::stoi(observationLengthString));

        std::string actionLengthString = to_string(metadata["action_length"]);
        unsigned int actionLength = (unsigned int)(std::stoi(actionLengthString));


        std::cout <<  "OBSERVATION LENGTH" << std::endl;
        std::cout << observationLength << std::endl;

        std::cout <<  "ACTION LENGTH" << std::endl;
        std::cout << actionLengthString << std::endl;

        auto logStdArray = metadata["log_stds"];


        Eigen::MatrixXd  stdDevs(1,1);
        stdDevs.resize(actionLength,1);
        std::cout << "Reached" << std::endl;


        const json::array &stdArray = as_array(logStdArray);
        int index = 0;
        for (auto i =  stdArray.begin(); i != stdArray.end(); i++)
        {
          const json::value &logStd = *i;

          double logStdDouble = (std::stod(to_string(logStd)));
          std::cout << "Reached" << std::endl;

          stdDevs(index) = exp(logStdDouble);
          index += 1;
          

        }
        std::cout << "Reached" << std::endl;

        
        
        Eigen::MatrixXd covarianceMatrix(1,1);
        covarianceMatrix.resize(actionLength, actionLength);
        std::cout << "Reached" << std::endl;

        covarianceMatrix = stdDevs.array().matrix().asDiagonal();
        


        NeuralNetwork::CompilationSettings settings;  // not sure if this is necessary but it's unused since we're not compiling but was required
        // for apply function signature
        NeuralNetwork::Model sharedModel("NeuralNets/shared_policy.h5");
        NeuralNetwork::Model actionModel("NeuralNets/action_policy.h5");
        NeuralNetwork::Model valueModel("NeuralNets/value_policy.h5");

        
    
        

        std::vector<NeuralNetwork::TensorXf> sharedOutputs(sharedModel.getOutputs().size());

        std::vector<NeuralNetwork::TensorXf> observation(sharedModel.getInputs().size());
        std::cout << "policy load and input setup complete" << std::endl;
        //reshaping but not sure why, derived from check.cpp
        const std::vector<NeuralNetwork::TensorLocation>& inputs = sharedModel.getInputs();
        for(std::size_t i = 0; i < observation.size(); ++i)
        {
          observation[i].reshape(inputs[i].layer->nodes[inputs[i].nodeIndex].outputDimensions[inputs[i].tensorIndex]);
        }



        observation[0][0] = x;
        observation[0][1] = y;
        observation[0][2] = sinAngle;
        observation[0][3] = cosAngle;

        std::cout << "OBSERVATION :" << std::endl;
        for (float i: observation[0])
        {
          std::cout << i << std::endl;
        }
       
        std::cout << "reached pre apply" << std::endl;
        NeuralNetwork::SimpleNN::apply(observation, sharedOutputs, sharedModel, [&settings](const NeuralNetwork::Node& node, const std::vector<const NeuralNetwork::TensorXf*>& inputs, const std::vector<NeuralNetwork::TensorXf*>& outputs)
        {
        });


        std::cout << "simpleNN test" << std::endl;

        for (float i: sharedOutputs[0])
        {
          std::cout << i << ",";
        }
        std::cout << "" << std::endl;
        



        std::cout << "simplNN test complete" << std::endl;


        NeuralNetwork::TensorXf latentAction =  sharedOutputs[0];
        NeuralNetwork::TensorXf latentValue =  sharedOutputs[1];



        std::cout << "SHARED LAYERS OUTPUT 1 :" << std::endl;

        for (float i: latentAction)
        {
          std::cout << i << std::endl;
        }
        std::cout << "SHARED LAYERS OUTPUT 2 :" << std::endl;

        for (float i: latentValue)
        {
          std::cout << i << std::endl;
        }

        



        std::vector<NeuralNetwork::TensorXf> valueInput(valueModel.getInputs().size());
        valueInput[0] = latentValue;

        std::vector<NeuralNetwork::TensorXf> valueOutput(valueModel.getOutputs().size());


        NeuralNetwork::SimpleNN::apply(valueInput, valueOutput, valueModel, [&settings](const NeuralNetwork::Node& node, const std::vector<const NeuralNetwork::TensorXf*>& inputs, const std::vector<NeuralNetwork::TensorXf*>& outputs)
        {
        });


        std::cout << "VALUE NET OUTPUT :" << std::endl;
        NeuralNetwork::TensorXf valueEstimate =  valueOutput[0];

        for (float i: valueEstimate)
        {
          std::cout << i << std::endl;
        }
      

        std::vector<NeuralNetwork::TensorXf> actionPolicyInput(actionModel.getInputs().size());
        actionPolicyInput[0] = latentAction;

        std::vector<NeuralNetwork::TensorXf> actionPolicyOutput(actionModel.getOutputs().size());


        NeuralNetwork::SimpleNN::apply(actionPolicyInput, actionPolicyOutput, actionModel, [&settings](const NeuralNetwork::Node& node, const std::vector<const NeuralNetwork::TensorXf*>& inputs, const std::vector<NeuralNetwork::TensorXf*>& outputs)
        {
        });




        std::cout << "ACTION NET OUTPUT: " << std::endl;
        NeuralNetwork::TensorXf actionMeans =  actionPolicyOutput[0];

        Eigen::MatrixXd actionEigen(actionLength,1);

        
        for (unsigned int i = 0; i < actionMeans.size(); i ++)
        {
          actionEigen(i) = actionMeans[i];
        }

        std::cout << "mu vector: " << std::endl;

        std::cout << actionEigen << std::endl;
        std::cout << "cov matrix: " << std::endl;

        std::cout << covarianceMatrix << std::endl;
        


        Eigen::MatrixXd actionChoice = stats::rmvnorm(actionEigen, covarianceMatrix, true);
            
        std::cout << "action: " << std::endl;

        std::cout << actionChoice << std::endl;

        double logProb = stats::dmvnorm(actionChoice,actionEigen, covarianceMatrix, true);

        std::cout << "action log prob: " << std::endl;

        std::cout << logProb << std::endl;

      
        
        theWalkAtRelativeSpeedSkill(Pose2f(actionChoice(0),actionChoice(1), actionChoice(2)));



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
