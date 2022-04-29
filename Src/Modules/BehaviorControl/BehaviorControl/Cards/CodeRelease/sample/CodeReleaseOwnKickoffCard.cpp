/**
 * @file CodeReleaseKickAtGoalCard.cpp
 *
 * This file is just practice
 *
 * @author 
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"

#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"


#include "Representations/BehaviorControl/FieldBall.h"



#include "Tools/BehaviorControl/Framework/Card/Card.h"


//#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
//#include "Tools/Math/BHMath.h"
//#include "Representations/Communication/RobotInfo.h"
//#include "Tools/Module/Blackboard.h"
//#include "Representations/Infrastructure/GroundTruthWorldState.h"

CARD(CodeReleaseOwnKickoffCard,
 {,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(Say),
  CALLS(Stand),
   CALLS(WalkToTarget),
    REQUIRES(FieldDimensions),
   DEFINES_PARAMETERS(
  {,
    (float)(1.0f) walkSpeed,
    }),
});

class CodeReleaseOwnKickoffCard : public CodeReleaseOwnKickoffCardBase
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
   
    //theActivitySkill(BehaviorStatus::codeReleasePositionForKickOff);
    theActivitySkill(BehaviorStatus::codeReleaseOwnKickoff);
    theLookForwardSkill();
    theStandSkill();
    // Not implemented in the Code Release.
    //theSaySkill("yes Goal Own Kick off card");
    theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f,Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)));

  }
};

MAKE_CARD(CodeReleaseOwnKickoffCard);
