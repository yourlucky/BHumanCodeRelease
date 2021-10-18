/**
 * @file WalkToTarget.cpp
 *
 * This file implements the implementation of the WalkToTarget skill.
 *
 * @author Arne Hasselbring
 */

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
#include "Tools/Streams/OutStreams.h"
#include "Tools/NeuralNetwork/date.h"

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
    
    

    Date d(1,1,1);


    
    const int observationLength = 24;
    
    Eigen::Matrix< double, 4, 1> stdDevs;
    stdDevs << exp(-0.03979301452636719), exp(-0.059911955147981644), exp(-0.08187924325466156), exp(-0.034654323011636734);
    Eigen::Matrix< double, 4, 4> covarianceMatrix = stdDevs.array().matrix().asDiagonal();
    std::cout << covarianceMatrix << std::endl;
    //vector<int> stdDevs{ -0.03979301452636719, -0.059911955147981644, -0.08187924325466156, -0.034654323011636734 };


    
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
    sharedPolicy.input(0) = inputTensor;
    sharedPolicy.apply();




    /*
    std::cout << "SHARED LAYERS OUTPUT 1 :" << std::endl;
    std::vector<unsigned int> result1 =  sharedPolicy.output(0).dims();
    for (unsigned int i: result1)
    {
      std::cout << i << std::endl;
    }

     std::cout << "SHARED LAYERS OUTPUT 2 :" << std::endl;
    std::vector<unsigned int> result2 =  sharedPolicy.output(1).dims();
    for (unsigned int i: result2)
    {
      std::cout << i << std::endl;
    }
    */
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

    Eigen::Matrix< double, 4, 1> actionEigen;

    
    for (unsigned int i = 0; i < actionMeans.size(); i ++)
    {
      actionEigen[i] = actionMeans[i];
      std::cout << actionMeans[i] << std::endl;
    }

    std::cout << "mu vector: " << std::endl;

    std::cout << actionEigen << std::endl;
    std::cout << "cov matrix: " << std::endl;

    std::cout << covarianceMatrix << std::endl;
    


    Eigen::Matrix<double, 4, 1> test = stats::rmvnorm(actionEigen, covarianceMatrix, true);
        
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
