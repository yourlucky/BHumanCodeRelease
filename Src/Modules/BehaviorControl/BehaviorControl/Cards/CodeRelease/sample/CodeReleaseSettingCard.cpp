/**
 * @file CodeReleaseKickAtGoalCard.cpp
 *
 * This file is just practice
 *
 * @author 
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"

#include "Representations/Sensing/FallDownState.h"

#include "Representations/Communication/RobotInfo.h"

#include "Modules/BehaviorControl/BehaviorControl/Cards/CodeRelease/striker/Striker.h"
#include "Modules/BehaviorControl/BehaviorControl/Cards/CodeRelease/supporter/Supporter.h"
#include "Modules/BehaviorControl/BehaviorControl/Cards/CodeRelease/keeper/Keeper.h"


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
        (float)(0.2f) walkSpeed,
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

option
{
  initial_state(start)
  {
      transition
      {
          if(state_time > 1000)
              goto judge;
      }
      action
      {
         theLookForwardSkill();
         theStandSkill();
      }
  }
    
    state(judge)
    {
        transition
        {
            if(theRobotInfo.number == 1)
                goto keeper;
            else if(theRobotInfo.number == 2 || theRobotInfo.number == 5)
                goto striker;
            else if(theRobotInfo.number == 4 || theRobotInfo.number == 3 )
                goto supporter;
            else
                goto stand;
        }
    }
    
    state(keeper)
    {
        action
        {
            theSaySkill("I am goal keeper");
        }
    }
    
    state(striker)
    {
      
        action
        {
          //if(state_time > initialWaitTime)
            theSaySkill("striker striker");
        }
    }
    
    state(supporter)
    {
        action
        {
            theSaySkill("supporter supporter");
        }
    }
    
    state(stand)
    {
        action
        {
          theLookForwardSkill();
          theStandSkill();
        }
    }

  }      
        
};

MAKE_CARD(CodeReleaseSettingCard);
