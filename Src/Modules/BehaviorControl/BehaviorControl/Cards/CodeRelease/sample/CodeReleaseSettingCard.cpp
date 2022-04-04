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
      DEFINES_PARAMETERS(
      {,
        (float)(0.5f) walkSpeed,
        (int)(500) initialWaitTime,
        (int)(7000) ballNotSeenTimeout,
        (Angle)(5_deg) ballAlignThreshold,
        (float)(500.f) ballNearThreshold,
        (Angle)(10_deg) angleToGoalThreshold,
        (float)(400.f) ballAlignOffsetX,
        (float)(100.f) ballYThreshold,
        (Angle)(2_deg) angleToGoalThresholdPrecise,
        (float)(150.f) ballOffsetX,
        (Rangef)({140.f, 170.f}) ballOffsetXRange,
        (float)(40.f) ballOffsetY,
        (Rangef)({20.f, 50.f}) ballOffsetYRange,

        (float)(0.f)ball_I,
        (float)(0.f)ball_F,
        //(float)(0.f)ball_S,
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
     
  initial_state(start)
  {     
      transition
      {
         if (state_time > initialWaitTime)
              goto giverole;
      }
      action
      {
         theLookForwardSkill();
         theStandSkill();
      }
  }  
    state(giverole)
    {
        transition
        {
          if(theRobotInfo.number == 4)
            goto skeeper;
          
          if(theRobotInfo.number == 1 ||theRobotInfo.number == 3) {
            if (ball_I > ball_F) {
              goto shuffle_dance;
            }
            if (ball_I < ball_F) {
              goto notmove;
            }


          }
                     
        }
        action
        {     
          const GroundTruthWorldState&theGroundTruthWorldState =
          static_cast<const GroundTruthWorldState&>(Blackboard::getInstance()["GroundTruthWorldState"]);
          const Pose2f _ownPosition = theGroundTruthWorldState.ownPose;
          const Pose2f _firstteam = theGroundTruthWorldState.firstTeamPlayers[0].pose;
          const Pose2f _secondteam = theGroundTruthWorldState.secondTeamPlayers[0].pose;

          const GroundTruthRobotPose &theGroundTruthRobotPose =
          static_cast<const GroundTruthRobotPose &>( Blackboard::getInstance()["GroundTruthRobotPose"]);
          const Vector2f _ballPosition = theGroundTruthWorldState.balls[0].position.head<2>(); 
                    //my position and ball distance
          ball_I = pow((_ownPosition.translation.x()-_ballPosition(0)),2) + pow((_ownPosition.translation.y()-_ballPosition(1)),2);
          ball_F = pow((_firstteam.translation.x()-_ballPosition(0)),2) + pow((_firstteam.translation.y()-_ballPosition(1)),2);
          //ball_S = pow((_secondteam.translation.x()-_ballPosition(0)),2) + pow((_secondteam.translation.y()-_ballPosition(1)),2);
          
          theLookForwardSkill();
          theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));

        }
    } 
    // state(movetoother)
    // {
    //     transition
    //     {
    //       if (!theFieldBall.ballWasSeen(ballNotSeenTimeout))
    //         goto searchForBall;

    //       //my position and ball distance
    //       //float ball_I = pow((_ownPosition.translation.x()-_ballPosition(0)),2) + pow((_ownPosition.translation.y()-_ballPosition(1)),2);
    //       //float ball_F = pow((_firstteam.translation.x()-_ballPosition(0)),2) + pow((_firstteam.translation.y()-_ballPosition(1)),2);
    //       //float ball_S = pow((_secondteam.translation.x()-_ballPosition(0)),2) + pow((_secondteam.translation.y()-_ballPosition(1)),2);

    //       //if(ball_I < ball_F || ball_I < ball_S)
    //         //goto giverole;             
    //     }


    //     action
    //     {
    //       const GroundTruthWorldState&theGroundTruthWorldState =
    //       static_cast<const GroundTruthWorldState&>(Blackboard::getInstance()["GroundTruthWorldState"]);
    //       const Pose2f _ownPosition = theGroundTruthWorldState.ownPose;
    //       const Pose2f _firstteam = theGroundTruthWorldState.firstTeamPlayers[0].pose;
    //       const Pose2f _secondteam = theGroundTruthWorldState.secondTeamPlayers[0].pose;

    //       const GroundTruthRobotPose &theGroundTruthRobotPose =
    //       static_cast<const GroundTruthRobotPose &>( Blackboard::getInstance()["GroundTruthRobotPose"]);
    //       const Vector2f _ballPosition = theGroundTruthWorldState.balls[0].position.head<2>(); 

    //       float _x = _firstteam.translation.x() * -1;
    //       float _y = _firstteam.translation.y()* -1;

    //       theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed),Vector2f(_x,_y));
    //     }
    // }

     state(skeeper)
      {
        action
        { 
          float x_ = 1600;
          float y_ = -2000;
          Angle v_angle =-0.8*pi;
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed),Pose2f(v_angle,Vector2f(x_,y_)));
          //theLookForwardSkill();
          //theStandSkill();
        }
      }

       state(searchForBall)
      {
        transition
        {
          if (theFieldBall.ballWasSeen())
            goto giverole;
        }

        action
        {
          theLookForwardSkill();
          theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
        }
      }

    state(shuffle_dance)
    {
        transition
        {
          if (!theFieldBall.ballWasSeen(ballNotSeenTimeout))
            goto searchForBall;

          
          if(ball_I < ball_F)
            goto notmove;
               
        }


        action
        {
          const GroundTruthWorldState&theGroundTruthWorldState =
          static_cast<const GroundTruthWorldState&>(Blackboard::getInstance()["GroundTruthWorldState"]);
          const Pose2f _ownPosition = theGroundTruthWorldState.ownPose;
          const Pose2f _firstteam = theGroundTruthWorldState.firstTeamPlayers[0].pose;
          const Pose2f _secondteam = theGroundTruthWorldState.secondTeamPlayers[0].pose;

          const GroundTruthRobotPose &theGroundTruthRobotPose =
          static_cast<const GroundTruthRobotPose &>( Blackboard::getInstance()["GroundTruthRobotPose"]);
          const Vector2f _ballPosition = theGroundTruthWorldState.balls[0].position.head<2>(); 
            
          ball_I = pow((_ownPosition.translation.x()-_ballPosition(0)),2) + pow((_ownPosition.translation.y()-_ballPosition(1)),2);
          ball_F = pow((_firstteam.translation.x()-_ballPosition(0)),2) + pow((_firstteam.translation.y()-_ballPosition(1)),2);
          if(theRobotInfo.number == 1)
            theSaySkill("shuffle");
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed),Vector2f(0.f,0.f));
        }
    }
    
    state(turn)
    {
        transition
        {
          if (!theFieldBall.ballWasSeen(ballNotSeenTimeout))
            goto searchForBall;

        // if(ball_I > ball_F && ball_I > ball_S)
        //     goto shuffle_dance;

        // if(ball_I < ball_F && ball_I < ball_S)
        //     goto notmove;
        
            
        }
        action
        {
          theLookForwardSkill();
          theStandSkill();
          theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
          //ball_I = pow((_ownPosition.translation.x()-_ballPosition(0)),2) + pow((_ownPosition.translation.y()-_ballPosition(1)),2);
          //ball_F = pow((_firstteam.translation.x()-_ballPosition(0)),2) + pow((_firstteam.translation.y()-_ballPosition(1)),2);
        }
    }

    state(notmove)
    {
      transition
        {
          if (!theFieldBall.ballWasSeen(ballNotSeenTimeout))
            goto searchForBall;

          if(ball_I > ball_F)
            goto shuffle_dance;

            
        }
        action
        {
          const GroundTruthWorldState&theGroundTruthWorldState =
          static_cast<const GroundTruthWorldState&>(Blackboard::getInstance()["GroundTruthWorldState"]);
          const Pose2f _ownPosition = theGroundTruthWorldState.ownPose;
          const Pose2f _firstteam = theGroundTruthWorldState.firstTeamPlayers[0].pose;
          const Pose2f _secondteam = theGroundTruthWorldState.secondTeamPlayers[0].pose;

          const GroundTruthRobotPose &theGroundTruthRobotPose =
          static_cast<const GroundTruthRobotPose &>( Blackboard::getInstance()["GroundTruthRobotPose"]);
          const Vector2f _ballPosition = theGroundTruthWorldState.balls[0].position.head<2>(); 
            
          ball_I = pow((_ownPosition.translation.x()-_ballPosition(0)),2) + pow((_ownPosition.translation.y()-_ballPosition(1)),2);
          ball_F = pow((_firstteam.translation.x()-_ballPosition(0)),2) + pow((_firstteam.translation.y()-_ballPosition(1)),2);
          if(theRobotInfo.number == 1)
            theSaySkill("change");
          theLookForwardSkill();
          theStandSkill();
        }
    }
  }      
        
};

MAKE_CARD(CodeReleaseSettingCard);
