/**
 * @file WalkToTarget.cpp
 *
 * This file implements the implementation of the WalkToTarget skill.
 *
 * @author Arne Hasselbring
 */


#include <fstream>
#include <iostream>
#include <math.h> 


#include "Representations/BehaviorControl/Libraries/LibCheck.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/MotionControl/MotionInfo.h"
#include "Representations/MotionControl/MotionRequest.h"

#include "Tools/Math/Eigen.h"
#include "Tools/Module/Module.h"
#include "Tools/NeuralNetwork/CompiledNN.h"
#include "Tools/NeuralNetwork/Model.h"
#include "Tools/NeuralNetwork/Tensor.h"
#include "Tools/NeuralNetwork/json.h"
#include "Tools/Streams/OutStreams.h"

#define STATS_GO_INLINE
#define STATS_DONT_USE_OPENMP
#define STATS_ENABLE_EIGEN_WRAPPERS
#define STATS_ENABLE_STDVEC_WRAPPERS
#include "Tools/NeuralNetwork/stats.hpp"

SKILL_IMPLEMENTATION(WalkToTargetImpl,
{,
  IMPLEMENTS(WalkToTarget),
  REQUIRES(LibCheck),
  REQUIRES(MotionInfo),
  MODIFIES(MotionRequest),
});

class WalkToTargetImpl : public WalkToTargetImplBase
{
  void execute(const WalkToTarget& p) override
  {
    
    std::ifstream metadataFile("/home/john/BHumanCodeRelease/Config/NeuralNets/metadata.json");
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

    //unsigned int observationLength = 24;

  /*
    for (int i =0; i < action_length; i++)
    {
      stdDevs[i] = 
    //stdDevs << exp(-0.03979301452636719), exp(-0.059911955147981644), exp(-0.08187924325466156), exp(-0.034654323011636734);
    }*/
    
    Eigen::MatrixXd covarianceMatrix(1,1);
    covarianceMatrix.resize(actionLength, actionLength);
    std::cout << "Reached" << std::endl;

    covarianceMatrix = stdDevs.array().matrix().asDiagonal();
    
    

    
    //loading the shared feature extractor
    NeuralNetwork::CompiledNN sharedPolicy;
    std::unique_ptr<NeuralNetwork::Model> sharedModel;

    sharedModel = std::make_unique<NeuralNetwork::Model>("NeuralNets/shared_policy.h5");
    sharedPolicy.compile(*sharedModel);
    

    //loading the action network

    NeuralNetwork::CompiledNN actionPolicy;
    std::unique_ptr<NeuralNetwork::Model> actionModel;

    actionModel = std::make_unique<NeuralNetwork::Model>("NeuralNets/action_policy.h5");
    actionPolicy.compile(*actionModel);
    
    //loading the value network 

    NeuralNetwork::CompiledNN valuePolicy;
    std::unique_ptr<NeuralNetwork::Model> valueModel;

    valueModel = std::make_unique<NeuralNetwork::Model>("NeuralNets/value_policy.h5");
    valuePolicy.compile(*valueModel);
    

    std::vector<unsigned int> sizeOfInput {observationLength};
    NeuralNetwork::TensorXf inputTensor(sizeOfInput,0);

    std::cout << "OBSERVATION :" << std::endl;
    for (float i: inputTensor)
    {
      std::cout << i << std::endl;
    }



    sharedPolicy.input(0) = inputTensor;
    sharedPolicy.apply();




    std::cout << "SHARED LAYERS OUTPUT 1 :" << std::endl;
    NeuralNetwork::TensorXf latentAction =  sharedPolicy.output(0);

    for (float i: latentAction)
    {
      std::cout << i << std::endl;
    }
    std::cout << "SHARED LAYERS OUTPUT 2 :" << std::endl;
    NeuralNetwork::TensorXf latentValue =  sharedPolicy.output(1);

    for (float i: latentValue)
    {
      std::cout << i << std::endl;
    }

    valuePolicy.input(0) = latentValue;
    valuePolicy.apply();

    std::cout << "VALUE NET OUTPUT :" << std::endl;
    NeuralNetwork::TensorXf valueEstimate =  valuePolicy.output(0);

    for (float i: valueEstimate)
    {
      std::cout << i << std::endl;
    }


    actionPolicy.input(0) = latentAction;
    actionPolicy.apply();

    std::cout << "ACTION NET OUTPUT: " << std::endl;
    NeuralNetwork::TensorXf actionMeans =  actionPolicy.output(0);

    Eigen::MatrixXd actionEigen(actionLength,1);

    
    for (unsigned int i = 0; i < actionMeans.size(); i ++)
    {
      actionEigen(i) = actionMeans[i];
    }

    std::cout << "mu vector: " << std::endl;

    std::cout << actionEigen << std::endl;
    std::cout << "cov matrix: " << std::endl;

    std::cout << covarianceMatrix << std::endl;
    


    Eigen::MatrixXd test = stats::rmvnorm(actionEigen, covarianceMatrix, true);
        
    std::cout << "action: " << std::endl;

    std::cout << test << std::endl;

    double logProb = stats::dmvnorm(test,actionEigen, covarianceMatrix, true);

    std::cout << "action log prob: " << std::endl;

    std::cout << logProb << std::endl;

    //OutMapFile stream("./test.log");
    //stream<<actionMeans;



    //exit(0);
  
    theMotionRequest.motion = MotionRequest::walk;



    /*
    //Assigning the input for the policy
    std::vector<unsigned int> sizeOfInput {2};
    NeuralNetwork::TensorXf inputTensor(sizeOfInput,0);
    inputTensor[0] = rand() % 2;
    inputTensor[1] = rand() % 2;
    policy.input(0) = inputTensor;
    policy.apply();
    
    //Outputting the result into a file
    OutTextFile stream("test.txt");
    stream<<policy.output(0)[0];

    if(policy.output(0)[0] > 0.5)
      theMotionRequest.motion = MotionRequest::walk;
    else
      theMotionRequest.motion = MotionRequest::stand;
    */
    theMotionRequest.walkRequest.mode = WalkRequest::targetMode;
    theMotionRequest.walkRequest.target = p.target;
    theMotionRequest.walkRequest.speed = p.speed;
    theMotionRequest.walkRequest.walkKickRequest = WalkRequest::WalkKickRequest();
    theLibCheck.inc(LibCheck::motionRequest);
  }

  bool isDone(const WalkToTarget&) const override
  {
    return theMotionInfo.motion == MotionRequest::walk;
  }
};

MAKE_SKILL_IMPLEMENTATION(WalkToTargetImpl);
