/**
 * @file CodeReleaseKickAtGoalCard.cpp
 *
 * This file is just practice
 *
 * @author 
 */

#include <math.h> 
#include <iostream>
#include <string>

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"

#include "Representations/Sensing/FallDownState.h"

#include "Representations/Communication/RobotInfo.h"


#include "Tools/BehaviorControl/Framework/Card/Dealer.h"
//#include "Modules/BehaviorControl/BehaviorControl/Cards/CodeRelease/striker/Striker.h"
//#include "Modules/BehaviorControl/BehaviorControl/Cards/CodeRelease/supporter/Supporter.h"
//#include "Modules/BehaviorControl/BehaviorControl/Cards/CodeRelease/keeper/Keeper.h"

#include "Tools/Module/Blackboard.h"
#include "Representations/Infrastructure/GroundTruthWorldState.h"

#include "Tools/Math/Angle.h"


CARD(CodeReleaseSettingCard,
    { ,

      CALLS(Activity),
      CALLS(InWalkKick),
      CALLS(LookForward),
      CALLS(Stand),
      CALLS(Say),
      CALLS(SpecialAction),
      CALLS(WalkAtRelativeSpeed),
      CALLS(WalkToTarget),
      REQUIRES(FallDownState),
      REQUIRES(FieldBall),
      REQUIRES(FieldDimensions),
      REQUIRES(RobotPose),
      REQUIRES(RobotInfo),
 LOADS_PARAMETERS(
  {,
    (DeckOfCards<CardRegistry>) one,
    (DeckOfCards<CardRegistry>) two,
     (DeckOfCards<CardRegistry>) three,
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
  void execute() override
  {
        
        switch (theRobotInfo.number)
        {       
          case 1: {dealer.deal(one)->call();}break;
          case 3: {dealer.deal(two)->call();}break;
          case 4: {dealer.deal(three)->call();}break;
          default :break;
        }
  }
 
  void reset() override
  {
    dealer.reset();
  } 
    PriorityListDealer dealer;
};

MAKE_CARD(CodeReleaseSettingCard);
