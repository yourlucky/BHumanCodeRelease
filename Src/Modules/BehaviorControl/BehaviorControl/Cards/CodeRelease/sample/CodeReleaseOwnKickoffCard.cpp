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
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Infrastructure/GroundTruthWorldState.h"

//#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
//#include "Tools/Math/BHMath.h"

//#include "Tools/Module/Blackboard.h"
//#include "Representations/Infrastructure/GroundTruthWorldState.h"

CARD(CodeReleaseOwnKickoffCard,
 {,
    CALLS(Activity),
    CALLS(LookForward),
    CALLS(Say),
    CALLS(Stand),
    CALLS(WalkToTarget),
    REQUIRES(RobotPose),
    REQUIRES(RobotInfo),
    REQUIRES(FieldDimensions),
   DEFINES_PARAMETERS(
  {,
    (float)(0.99f) walkSpeed,
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
   
    theActivitySkill(BehaviorStatus::codeReleasePositionForKickOff);
    //theActivitySkill(BehaviorStatus::codeReleaseOwnKickoff);
    theLookForwardSkill();
    theStandSkill();
    // Not implemented in the Code Release.
    //theSaySkill("yes Goal Own Kick off card");
    
    
    //Pose2f _ownPosition = theGroundTruthWorldState.ownPose;
    
    if(theRobotInfo.number == 1)
    {
    const GroundTruthWorldState&theGroundTruthWorldState =
     static_cast<const GroundTruthWorldState&>(Blackboard::getInstance()["GroundTruthWorldState"]);
    
    Pose2f _firstteam = theGroundTruthWorldState.firstTeamPlayers[0].pose;
    Angle v_angle=(theRobotPose.inversePose * Vector2f(_firstteam.translation.x(),_firstteam.translation.y())).angle();
    theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(Pose2f(v_angle,0.f,0.f )));
    
    
    }
    
    else if(theRobotInfo.number == 2)
   {
    const GroundTruthWorldState&theGroundTruthWorldState_2=
    static_cast<const GroundTruthWorldState&>(Blackboard::getInstance()["GroundTruthWorldState"]);   
       
    Pose2f _firstteam = theGroundTruthWorldState_2.firstTeamPlayers[0].pose;
   Angle v_angle=(theRobotPose.inversePose * Vector2f(_firstteam.translation.x(),_firstteam.translation.y())).angle();
   theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(Pose2f(v_angle,0.f,0.f )));}
   
   else{
       Angle v_angle=0.f;
       theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(Pose2f(v_angle,0.f,0.f )));
   }
       
    //float _x= _firstteam.translation.x()-_ownPosition.translation.x();
    //float _y= _firstteam.translation.y()-_ownPosition.translation.y();
    
    // if(theRobotInfo.number == 1) {
             
         //theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f,Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)));

  }
};

MAKE_CARD(CodeReleaseOwnKickoffCard);
