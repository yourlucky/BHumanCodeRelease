/**
 * @file WalkToTarget.cpp
 *
 * This file implements the implementation of the WalkToTarget skill.
 *
 * @author Arne Hasselbring
 */

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
    //Neural Net Definitions
    NeuralNetwork::CompiledNN policy;
    std::unique_ptr<NeuralNetwork::Model> model;

    //Reading and compiling model of a hdf5 file
    model = std::make_unique<NeuralNetwork::Model>("NeuralNets/test.h5");
    policy.compile(*model);
    
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
