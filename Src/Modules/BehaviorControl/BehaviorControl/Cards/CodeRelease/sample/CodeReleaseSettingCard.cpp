/**
 * @file CodeReleaseKickAtGoalCard.cpp
 *
 * This file is just practice
 *
 * @author 
 */

#include <math.h> 
#include <iostream>
#include <string>

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"

#include "Representations/Sensing/FallDownState.h"

#include "Representations/Communication/RobotInfo.h"


#include "Tools/BehaviorControl/Framework/Card/Dealer.h"
//#include "Modules/BehaviorControl/BehaviorControl/Cards/CodeRelease/striker/Striker.h"
//#include "Modules/BehaviorControl/BehaviorControl/Cards/CodeRelease/supporter/Supporter.h"
//#include "Modules/BehaviorControl/BehaviorControl/Cards/CodeRelease/keeper/Keeper.h"

#include "Tools/Module/Blackboard.h"
#include "Representations/Infrastructure/GroundTruthWorldState.h"

#include "Tools/Math/Angle.h"


CARD(CodeReleaseSettingCard,
    { ,
      CALLS(Activity),
      CALLS(InWalkKick),
      CALLS(LookForward),
      CALLS(Stand),
      CALLS(Say),
      CALLS(SpecialAction),
      CALLS(WalkAtRelativeSpeed),
      CALLS(WalkToTarget),
      REQUIRES(FallDownState),
      REQUIRES(FieldBall),
      REQUIRES(FieldDimensions),
      REQUIRES(RobotPose),
      REQUIRES(RobotInfo),
      DEFINES_PARAMETERS(
  {,
    (float)(1.0f) walkSpeed,
  }),
    });
    

class CodeReleaseSettingCard : public CodeReleaseSettingCardBase
{    
    bool preconditions() const override
    {
        return true;
    }

    bool postconditions() const override
    {
        return true;
    }
  void execute() override
  {

    if(theRobotInfo.number == 1)
    {
       theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 4000.f,0));
    }

    if(theRobotInfo.number == 2)
    {
       theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 400.f,0));
    }

    if(theRobotInfo.number == 3)
    {
       theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 400.f,0));
    }
        
   
  }
 
  void reset() override
  {
    dealer.reset();
  } 
    PriorityListDealer dealer;
};

MAKE_CARD(CodeReleaseSettingCard);

