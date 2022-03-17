/**
 * @file CodeReleaseKickAtGoalCard.cpp
 *
 * This file is just practice
 *
 * @author 
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"

#include "Representations/Sensing/FallDownState.h"

CARD(CodeReleaseSettingCard,
    { ,
      CALLS(Activity),
      CALLS(LookForward),
      CALLS(Stand),
      CALLS(Say),
      REQUIRES(RobotPose),
      DEFINES_PARAMETERS(
      {,
        (float)(0.2f) walkSpeed,
        (int)(1000) initialWaitTime,
 
      }),
    });

class CodeReleaseSettingCard : public CodeReleaseSettingCardBase
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
      //theActivitySkill(BehaviorStatus::codeReleaseKickAtGoal);
      theActivitySkill(BehaviorStatus::codeReleaseSetting);


      initial_state(start)
      {
        transition
        {
          if (state_time > initialWaitTime)
            //goto turnToBall;
        }

        action
        {
          theLookForwardSkill();
          theStandSkill();
          theSaySkill("Yes, I can assign the role");
        }
      }

      
};

MAKE_CARD(CodeReleaseSettingCard);
