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

#include "Modules/BehaviorControl/BehaviorControl/Cards/CodeRelease/striker/Striker.h"
#include "Modules/BehaviorControl/BehaviorControl/Cards/CodeRelease/supporter/Supporter.h"
#include "Modules/BehaviorControl/BehaviorControl/Cards/CodeRelease/keeper/Keeper.h"

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
  int b_r_d_1 = 2147000000; 
  int b_r_d_2 = 2147000000; 

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
              goto giverole;
      }
      action
      {
         theLookForwardSkill();
         theStandSkill();
      }
  }
    
    state(giverole)
    {

        transition
        {
          if(theRobotInfo.number == 4)
            goto skeeper;


          int x_d = pow((theFieldBall.positionRelative.x()-theRobotPose.translation.x()),2);
          int y_d = pow((theFieldBall.positionRelative.y()-theRobotPose.translation.y()),2);
          
          if(theRobotInfo.number == 3)
            goto notmove;

          if(theRobotInfo.number == 1) {
            std::cout << "robot 2 distance by #1" << std::endl;
            std::cout << b_r_d_2 << std::endl;

            b_r_d_1 = x_d+y_d;
            if (b_r_d_1 < b_r_d_2)
              goto shuffle_dance;            
          }

          if(theRobotInfo.number == 2) {
            std::cout << "robot 1 distance by #2" << std::endl;
            std::cout << b_r_d_1 << std::endl;
            
            b_r_d_2 = x_d+y_d;
            if (b_r_d_2 < b_r_d_1)
              goto shuffle_dance;            
          }
            
        }
        action
        {
          theLookForwardSkill();
          if(theRobotInfo.number==1)
            theSaySkill("Supporter");
          theStandSkill();
        }
    } 

     state(skeeper)
      {
        action
        {
          theLookForwardSkill();
          Angle v_angle =(-5/6)*pi;
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed),Pose2f(v_angle,4.5,0.0));
        }
      }

       state(searchForBall)
      {
        transition
        {
          if (theFieldBall.ballWasSeen())
            goto giverole;
        }

        action
        {
          theLookForwardSkill();
          theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
        }
      }

    state(shuffle_dance)
    {
        action
        {
          const GroundTruthWorldState&theGroundTruthWorldState =
          static_cast<const GroundTruthWorldState&>(Blackboard::getInstance()["GroundTruthWorldState"]);
          const Pose2f _ownPosition = theGroundTruthWorldState.ownPose; 
          theLookForwardSkill();
          theStandSkill();
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed),Pose2f(_ownPosition));
        }
    }
    
    state(turnaround)
    {
        action
        {
          theLookForwardSkill();
          theStandSkill();
          theSaySkill("turn");
          theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
        }
    }

    state(notmove)
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
