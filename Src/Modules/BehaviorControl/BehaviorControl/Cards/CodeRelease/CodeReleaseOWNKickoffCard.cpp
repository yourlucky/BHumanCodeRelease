/**
 * @file CodeReleaseKickAtGoalCard.cpp
 *
 * This file is just practice
 *
 * @author 
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"


CARD(CodeReleaseOWNKickoffCard,
 {,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(Say),
  CALLS(Stand),
});

class CodeReleaseOWNKickoffCard : public CodeReleaseOWNKickCardBase
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
    theSaySkill("Own Kick off card");
  }
};

MAKE_CARD(CodeReleaseOWNKickoffCard);