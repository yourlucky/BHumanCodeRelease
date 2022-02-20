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

CARD(CodeReleaseRunSpeedCard,
    { ,
      CALLS(Activity),
      CALLS(InWalkKick),
      CALLS(LookForward),
      CALLS(Stand),
      CALLS(SpecialAction),
      CALLS(WalkAtRelativeSpeed),
      CALLS(WalkToTarget),
      REQUIRES(FallDownState),
      REQUIRES(FieldBall),
      REQUIRES(FieldDimensions),
      REQUIRES(RobotPose),
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

class CodeReleaseRunSpeedCard : public CodeReleaseRunSpeedCardBase
{
    int c_time = 0; //current time

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
        transition
        {
          if (state_time > initialWaitTime)
            goto turnToBall;
        }

        action
        {
          theLookForwardSkill();
          theStandSkill();
        }
      }

      state(getUP)
      {
          transition
          {
            if (theFallDownState.state != FallDownState::fallen)
                goto turnToBall;
          }

          action
          {
            theStandSkill();
          }
      }

      state(turnToBall)
      {
        transition
        {
          if (!theFieldBall.ballWasSeen(ballNotSeenTimeout))
            goto searchForBall;
        if (std::abs(theFieldBall.positionRelative.angle()) < ballAlignThreshold)
        {
            c_time = state_time;
            goto walkToBall_1;
        }
        }

        action
        {
          theLookForwardSkill();
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theFieldBall.positionRelative.angle(), 0.f, 0.f));
        }
      }

      state(walkToBall_1) //walk speed 0.2
      {
        transition
        {
          if (!theFieldBall.ballWasSeen(ballNotSeenTimeout))
            goto searchForBall;
          
          if (theFallDownState.state == FallDownState::fallen)
            goto getUP;

          if (state_time > c_time + 5000) //if not fallen for 0.5 secs
          {
              c_time = state_time;
              walkSpeed += 0.3;
              goto walkToBall_2;
          }
        }

        action
        {
          theLookForwardSkill();
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), theFieldBall.positionRelative);
        }
      }

      state(walkToBall_2) //walk speed 0.5
      {
          transition
          {
            if (!theFieldBall.ballWasSeen(ballNotSeenTimeout))
              goto searchForBall;

            if (theFallDownState.state == FallDownState::fallen)
                goto getUP;

            if (state_time > c_time + 5000) //if not fallen for 10secs
            {
                c_time = state_time;
                walkSpeed += 0.3;
                goto walkToBall_3;
            }
          }

          action
          {
            theLookForwardSkill();
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), theFieldBall.positionRelative);
          }
      }

      state(walkToBall_3) //walk speed 0.8
      {
          transition
          {
            if (!theFieldBall.ballWasSeen(ballNotSeenTimeout))
              goto searchForBall;

            if (theFallDownState.state == FallDownState::fallen)
                goto getUP;

            if (state_time > c_time + 5000) //if not fallen for 10secs
            {
                c_time = state_time;
                walkSpeed += 0.2;
                goto walkToBall_4;
            }
          }

          action
          {
            theLookForwardSkill();
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), theFieldBall.positionRelative);
          }
      }

      state(walkToBall_4) //walk speed 1.0
      {
          transition
          {
            if (!theFieldBall.ballWasSeen(ballNotSeenTimeout))
              goto searchForBall;

            if (theFallDownState.state == FallDownState::fallen)
                goto getUP;

            if (state_time > c_time + 10000) //if not fallen for 10secs
            {
                c_time = state_time;
                walkSpeed += 0.2;
                goto walkToBall_3;
            }
          }

           action
          {
            theLookForwardSkill();
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), theFieldBall.positionRelative);
          }
      }

      state(searchForBall)
      {
        transition
        {
          if (theFieldBall.ballWasSeen())
            goto turnToBall;
        }

        action
        {
          theLookForwardSkill();
          theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
        }
      }
    }

        Angle calcAngleToGoal() const
    {
        return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
    }
};

MAKE_CARD(CodeReleaseRunSpeedCard);
