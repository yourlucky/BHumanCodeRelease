/**
 * @file CodeReleaseKickAtGoalCard.cpp
 *
 * This file is just practice
 *
 * @author 
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"


CARD(CodeReleaseOwnKickoffCard,
 {,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(Say),
  CALLS(Stand),
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
    theSaySkill("yes Goal Own Kick off card");
  }
};
<<<<<<< HEAD
//git fixing
=======
//fix git
//fix git2
>>>>>>> 54e00088be1ef0d3d4c336210eb69c3700cc1d2f
MAKE_CARD(CodeReleaseOwnKickoffCard);
