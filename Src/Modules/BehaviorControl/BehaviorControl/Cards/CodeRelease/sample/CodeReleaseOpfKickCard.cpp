/**
 * @file CodeReleaseKickAtGoalCard.cpp
 *
 * This file is just practice
 *
 * @author 
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"


CARD(CodeReleaseOpfKickCard,
 {,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(Say),
  CALLS(Stand),
  CALLS(WalkToTarget),
  DEFINES_PARAMETERS(
  {,
    (float)(0.2f) walkSpeed,
  }),
});

class CodeReleaseOpfKickCard : public CodeReleaseOpfKickCardBase
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
    theActivitySkill(BehaviorStatus::codeReleaseOpfKick);
    //theLookForwardSkill();
    //theStandSkill();
    // Not implemented in the Code Release.
    theSaySkill("suffle");
    theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed),Vector2f(0.f,0.f));
  }
};

MAKE_CARD(CodeReleaseOpfKickCard);
