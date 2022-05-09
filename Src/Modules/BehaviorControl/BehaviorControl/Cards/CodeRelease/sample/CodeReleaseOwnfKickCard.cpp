/**
 * @file CodeReleaseKickAtGoalCard.cpp
 *
 * This file is just practice
 *
 * @author 
 */

#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"

#include "Representations/Communication/RobotInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Representations/Communication/TeamData.h"


CARD(CodeReleaseOwnfKickCard,
 {,
  CALLS(Activity),
  CALLS(LookForward),
  CALLS(Say),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  REQUIRES(RobotInfo),
  REQUIRES(RobotPose),
   REQUIRES(TeamData),
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
    if(theRobotInfo.number == 2) {
    //theWalkAtRelativeSpeedSkill(Pose2f(0.9f, 0.f, 0.f));}
    theSaySkill("good setting");
    }

    if(theRobotInfo.number == 1){
        
        float _x = theTeamData.teammates[0].theRobotPose.translation.x()-theRobotPose.translation.x();
        float _y = theTeamData.teammates[0].theRobotPose.translation.y()-theRobotPose.translation.y();
        
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f,_x,0.f));
}
     


  }
};

MAKE_CARD(CodeReleaseOwnfKickCard);
