/**
 * @file CodeReleasePositionForKickOffCard.cpp
 *
 * This file implements nothing.
 *
 * @author Arne Hasselbring
 */

#include <stdlib.h>
#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"

CARD(CodeReleasePositionForKickOffCard,
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

class CodeReleasePositionForKickOffCard : public CodeReleasePositionForKickOffCardBase
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
      transition
      {
        if(state_time > initialWaitTime)
          goto turnToOwnSide;
      }

      action
      {
        theLookForwardSkill();
        theStandSkill();
      }
    }

    state(turnToOwnSide)
    {
      transition
      {
        if(std::abs((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyMark, theFieldDimensions.yPosLeftPenaltyArea)).angle()) < ballAlignThreshold)
          goto walkToOwnSide;
      }
      
      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyMark, theFieldDimensions.yPosLeftPenaltyArea)).angle(), 0.f, 0.f));
      }
    }

    state(walkToOwnSide)
    {
      transition
      {
        if((theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyMark, theFieldDimensions.yPosLeftPenaltyArea)).squaredNorm() < sqr(ballNearThreshold))
          //implement wait/completion
          goto doNothing;
      }

      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnPenaltyMark, theFieldDimensions.yPosLeftPenaltyArea)));

      }
    }

    state(doNothing)
    {
      transition
      {
        //nothing
      }
      action
      {
        theLookForwardSkill();
      }
    }
  }

  Angle calcAngleToGoal() const
  {
    return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
  }

  Angle calcAngleToCenter() const
  {
    return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosHalfWayLine, theFieldDimensions.yPosCenterGoal)).angle();
  }

};

MAKE_CARD(CodeReleasePositionForKickOffCard);

