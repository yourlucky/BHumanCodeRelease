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

option
{
    theActivitySkill(BehaviorStatus::codeReleaseOwnfKick);
    //theLookForwardSkill();
    //theStandSkill();
    // Not implemented in the Code Release.
    theSaySkill("Own Kick");
<<<<<<< HEAD
    theSaySkill("I can assign the role");


  initial_state(start)
  {
      transition
      {
          if(state_time > 1000)
              goto judge;
      }
      action
      {
          LookForward();
          Stand();
      }
  }
    
    state(judge)
    {
        transition
        {
            if(theRobotInfo.number == 1)
                goto keeper;
            else if(theRobotInfo.number == 2 || theRobotInfo.number == 5)
                goto striker;
            else if(theRobotInfo.number == 4 || theRobotInfo.number == 3 )
                goto supporter;
            else
                goto stand;
        }
    }
    
    state(keeper)
    {
        action
        {
            keeper();
        }
    }
    
    state(striker)
    {
        action
        {
            striker();
        }
    }
    
    state(supporter)
    {
        action
        {
            supporter();
        }
    }
    
    state(stand)
    {
        action
        {
            LookForward();
            Stand();
        }
    }

=======
>>>>>>> d7a2c0fc (new setting)
  }
  }
};

MAKE_CARD(CodeReleaseOwnfKickCard);
