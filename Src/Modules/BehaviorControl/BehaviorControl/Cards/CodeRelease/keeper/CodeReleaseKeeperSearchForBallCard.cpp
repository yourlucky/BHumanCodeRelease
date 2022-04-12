/**
 * @file CodeReleaseKeeperSearchForBallCard.cpp
 *
 * This file states the behavior that make keeper search for ball if ball lost.
 *
 * @author Chen Chao
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include <cmath>

#define mymax(a,b) (((a) > (b)) ? (a) : (b))
#define mymin(a,b) (((a) < (b)) ? (a) : (b))

CARD(CodeReleaseKeeperSearchForBallCard,
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
*/
    
    (float)(0.8f) walkSpeed,
    (int)(6000) ballNotSeenTimeout,
    (float)(-4000.f) desiredPosX,
  }),
});

class CodeReleaseKeeperSearchForBallCard : public CodeReleaseKeeperSearchForBallCardBase
{
  bool preconditions() const override
  {
    return theFieldBall.timeSinceTeamBallWasValid > ballNotSeenTimeout;
  }

  bool postconditions() const override
  {
    return theFieldBall.timeSinceTeamBallWasValid <= ballNotSeenTimeout;
  }
  
  void execute() override
  {
    //theActivitySkill(BehaviorStatus::codeReleaseKeeperSearchForBall);
    theLookForwardSkill();
    if (theRobotPose.translation.x() < desiredPosX )
    {
      theWalkAtRelativeSpeedSkill(Pose2f(0.f,0.3f,0.f));//move foward?
    }
    else
    {
        theWalkAtRelativeSpeedSkill(Pose2f(1.f,0.1f,0.f));//go around?
    }
  }
};

MAKE_CARD(CodeReleaseKeeperSearchForBallCard);
