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
        (float)(0.5f) walkSpeed,
        (int)(500) initialWaitTime,
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
         if (state_time > initialWaitTime)
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
                goto striker;
            else
                goto notmove;
        }
    } 
    
    state(striker)
    {
      
        transition
        {
          if (!theFieldBall.ballWasSeen(ballNotSeenTimeout))
            goto searchForBall;
        if (std::abs(theFieldBall.positionRelative.angle()) < ballAlignThreshold)
        {
            goto walkToBall_1;
        }
        }

        action
        {
          theLookForwardSkill();
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theFieldBall.positionRelative.angle(), 0.f, 0.f));
          theSaySkill("turn to ball");
        }

    }

    state(walkToBall_1) //walk speed 0.2
      {
        transition
        {
          if (!theFieldBall.ballWasSeen(ballNotSeenTimeout))
            goto searchForBall;

        }

        action
        {
          theLookForwardSkill();
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), theFieldBall.positionRelative);
          theSaySkill("speed one");
        }
      }

     state(searchForBall)
      {
        transition
        {
          if (theFieldBall.ballWasSeen())
            goto striker;
        }

        action
        {
          theLookForwardSkill();
          theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
          theSaySkill("search");
        }
      }
    
    

    state(notmove)
    {
        action
        {
          theLookForwardSkill();
          theStandSkill();
          theSaySkill("Setting");
        }
    }

  }      
        
};

MAKE_CARD(CodeReleaseSettingCard);
