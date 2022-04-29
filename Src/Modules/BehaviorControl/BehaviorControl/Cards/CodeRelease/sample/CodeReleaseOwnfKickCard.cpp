/**
 * @file CodeReleaseKickAtGoalCard.cpp
 *
 * This file is just practice
 *
 * @author 
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"


CARD(CodeReleaseOwnfKickCard,
 {,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(Say),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  DEFINES_PARAMETERS(
  {,
    (float)(1.0f) walkSpeed,
  }),

});

class CodeReleaseOwnfKickCard : public CodeReleaseOwnfKickCardBase
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
    theActivitySkill(BehaviorStatus::codeReleaseOwnfKick);
    //theLookForwardSkill();
    //theStandSkill();
    // Not implemented in the Code Release.
    theSaySkill("Turn");
    theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
  }
};

MAKE_CARD(CodeReleaseOwnfKickCard);
