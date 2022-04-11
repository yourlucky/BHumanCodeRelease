/**
 * @file CodeReleaseKeeperCatchBallCard.cpp
 *
 * This file states the behavior that keeper handle the situation where
 * ball move towards own goal fast.
 * @author Chen Chao
 */


#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include <cmath>

CARD(CodeReleaseKeeperCatchBallCard,
{,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(SpecialAction),
  CALLS(Say),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  DEFINES_PARAMETERS(
  {,
/*
    (int)(1000) initialWaitTime,
    (int)(7000) ballNotSeenTimeout,
    (float)(500.f) ballNearThreshold,
    (Angle)(25_deg) angleToGoalThreshold,
    (float)(400.f) ballAlignOffsetX,
    (float)(100.f) ballYThreshold,
    (Angle)(2_deg) angleToGoalThresholdPrecise,
    (float)(150.f) ballOffsetX,
    (Rangef)({140.f, 170.f}) ballOffsetXRange,
    (float)(40.f) ballOffsetY,
    (Rangef)({20.f, 50.f}) ballOffsetYRange,
    (int)(10) minKickWaitTime,
    (int)(3000) maxKickWaitTime,
    (Angle)(15_deg) ballAlignThreshold,
    (float)(0.8f) walkSpeed,
*/
    (float)(80.f) sitBlockLen,
  }),
});

class CodeReleaseKeeperCatchBallCard : public CodeReleaseKeeperCatchBallCardBase
{
  bool preconditions() const override
  {

    return theFieldBall.isInsideOwnPenaltyArea && theFieldBall.isRollingTowardsOwnGoal;//TO BE FIXED
  }

  bool postconditions() const override
  {
    return !theFieldBall.isInsideOwnPenaltyArea ||  !theFieldBall.isRollingTowardsOwnGoal;//TO BE FIXED
  }
  
  void execute() override
  {
    theActivitySkill(BehaviorStatus::codeReleaseKeeperCatchBall);
    theLookForwardSkill();
    theSaySkill("I'm catching!");
    if (theFieldBall.intersectionPositionWithOwnYAxis.y() >  sitBlockLen )
       theSpecialActionSkill(SpecialActionRequest::groundPunchLeft);
    else if (theFieldBall.intersectionPositionWithOwnYAxis.y() < sitBlockLen)
        theSpecialActionSkill(SpecialActionRequest::groundPunchRight);
    else
        theSpecialActionSkill(SpecialActionRequest::sumo);
  }
};

MAKE_CARD(CodeReleaseKeeperCatchBallCard);
