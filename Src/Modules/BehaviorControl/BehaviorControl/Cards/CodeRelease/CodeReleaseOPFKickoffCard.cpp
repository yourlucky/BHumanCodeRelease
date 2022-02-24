/**
 * @file CodeReleaseKickAtGoalCard.cpp
 *
 * This file is just practice
 *
 * @author 
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"


CARD(CodeReleaseOPFKickoffCard,
 {,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(Say),
  CALLS(Stand),
});

class CodeReleaseOPFKickoffCard : public CodeReleaseOPFKickoffCardBase
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
    //theLookForwardSkill();
    //theStandSkill();
    // Not implemented in the Code Release.
    theSaySkill("Opponent Kick off");
  }
};

MAKE_CARD(CodeReleaseOPFKickoffCard);
