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


#include "Representations/Communication/RobotInfo.h"

#include "Tools/Module/Blackboard.h"
#include "Representations/Infrastructure/GroundTruthWorldState.h"

//#include "Representations/Sensing/FallDownState.h"

#include <cmath> 

CARD(CodeReleaseKickndribbleCard,
{,
  CALLS(Activity),
  CALLS(Kick),
  CALLS(InWalkKick),
  CALLS(LookForward),
  CALLS(PathToTarget),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(Say),
  CALLS(PassTarget),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(RobotInfo),
  DEFINES_PARAMETERS(
  {,
    (float)(1.0f) walkSpeed,
    (int)(1000) initialWaitTime,
    (int)(4000) ballNotSeenTimeout,
    (Angle)(5_deg) ballAlignThreshold,
    (float)(500.f) ballNearThreshold,
    (Angle)(20_deg) angleToGoalThreshold,
    (float)(400.f) ballAlignOffsetX,
    (float)(100.f) ballYThreshold,
    (Angle)(2_deg) angleToGoalThresholdPrecise,
    //(float)(80.f) ballOffsetX,
    (float)(150.f) ballOffsetX,
    (Rangef)({140.f, 170.f}) ballOffsetXRange,
   // (float)(80.f) ballOffsetY,
    (float)(40.f) ballOffsetY,
    
    (Rangef)({20.f, 50.f}) ballOffsetYRange,
    (int)(10) minKickWaitTime,
    (int)(3000) maxKickWaitTime,
    
    (float)(0.f)ball_X,
    (float)(0.f)ball_Y,
    
    (int)(-20) c_time,
  }),
});

class CodeReleaseKickndribbleCard : public CodeReleaseKickndribbleCardBase
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
    theActivitySkill(BehaviorStatus::codeReleaseKickndribble);

    initial_state(start)
    {
      transition
      {
        if(state_time > initialWaitTime)
          goto giverole;
      }

      action
      {
        //theLookForwardSkill();
        theStandSkill();
      }
    }

    state(giverole)
    {
        transition
        {
          if(theRobotInfo.number == 1)
          {
            if (state_time > c_time + 13000 ) //if not fallen for 10secs
            {
              c_time = state_time;
              goto InitialWait;
            }
          }  
          else
            goto runner;
          }                     
        action
        {              
           if(theRobotInfo.number == 1)
                theLookForwardSkill();
        }
    }

    state(InitialWait)
    {
        transition
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
      float ball_I = pow((_ownPosition.translation.x()-_ballPosition(0)),2) + pow((_ownPosition.translation.y()-_ballPosition(1)),2);
      float ball_F = pow((_firstteam.translation.x()-_ballPosition(0)),2) + pow((_firstteam.translation.y()-_ballPosition(1)),2);
            
            if(state_time > c_time -5000  && ball_I < ball_F) 
                goto searchForBall; 
        }
        action {
        theLookForwardSkill();
        }
    }        
          
    state(runner)
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
      float ball_I = pow((_ownPosition.translation.x()-_ballPosition(0)),2) + pow((_ownPosition.translation.y()-_ballPosition(1)),2);
      float ball_F = pow((_firstteam.translation.x()-_ballPosition(0)),2) + pow((_firstteam.translation.y()-_ballPosition(1)),2);

      transition
        {
          if (state_time > 15000 && ball_I <ball_F)
              goto searchForBall;            
        }
        action
        {
          //theLookForwardSkill();
          //const Angle v_angle=(theRobotPose.inversePose * Vector2f(4000,500)).angle();
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 4000,0));
          //theSaySkill("one"); 
        }
    }

    state(searchForBall)
    {
      transition
      {
        if(theFieldBall.ballWasSeen())
          goto turnToBall;
      }

      action
      {
        theLookForwardSkill();
        //theSaySkill("search for ball");
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));

      }
    }
  
    state(turnToBall)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(std::abs(theFieldBall.positionRelative.angle()) < ballAlignThreshold)
          goto walkToBall;
      }

      action
      {
        //theSaySkill("turn to ball");
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theFieldBall.positionRelative.angle(), 0.f, 0.f));
      }
    }

    state(walkToBall)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto giverole;
        if(theFieldBall.positionRelative.squaredNorm() < sqr(ballNearThreshold))
          goto alignkick;      
      }

      action
      {
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theFieldBall.positionRelative.angle(), theFieldBall.positionRelative.x(),theFieldBall.positionRelative.y()));
      }
    }

    state(kick)
    {     
      transition
      {
        if(state_time > maxKickWaitTime || (state_time > minKickWaitTime && theInWalkKickSkill.isDone()))
          goto InitialWait;
      }

      action
      {   
        const GroundTruthWorldState&theGroundTruthWorldState =
        static_cast<const GroundTruthWorldState&>(Blackboard::getInstance()["GroundTruthWorldState"]);
        
        const Pose2f _ownPosition = theGroundTruthWorldState.ownPose;
        const Vector2f _ballPosition = theGroundTruthWorldState.balls[0].position.head<2>();
        const Pose2f _firstteam = theGroundTruthWorldState.firstTeamPlayers[0].pose;                    
        const Angle v_angle=(theRobotPose.inversePose * Vector2f(_firstteam.translation.x(),_firstteam.translation.y())).angle();
              
        theInWalkKickSkill(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(v_angle, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
        //thePassTargetSkill(2,Vector2f(theFieldBall.positionRelative.x()-ballOffsetX,theFieldBall.positionRelative.y()-ballOffsetY));
               
        float D_x =pow( _firstteam.translation.x()-theFieldBall.positionRelative.x(),2);
        float D_y =pow ( _firstteam.translation.y()-theFieldBall.positionRelative.y(),2);
        
        float D_t = pow((D_x+D_y),0.5);
        //float D_t = 5.5f;
        
        
         theLookForwardSkill();
        //theKickSkill(KickRequest::kickForward,true,D_t,false);
        theSaySkill("kick");
      }
    }

    state(alignkick)
    {       
        const GroundTruthWorldState&theGroundTruthWorldState =
        static_cast<const GroundTruthWorldState&>(Blackboard::getInstance()["GroundTruthWorldState"]);        
        const Pose2f _ownPosition = theGroundTruthWorldState.ownPose;
        const Vector2f _ballPosition = theGroundTruthWorldState.balls[0].position.head<2>();
        const Pose2f _firstteam = theGroundTruthWorldState.firstTeamPlayers[0].pose;
        const Angle v_angle =(theRobotPose.inversePose * Vector2f(_firstteam.translation.x(),_firstteam.translation.y())).angle();
        //const Angle v_angle_goal =(theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline*-1, 0.f)).angle();
        const Angle v_angle_goal =(theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
        
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto giverole;
        
        if(theRobotInfo.number == 1){
            if(std::abs(v_angle) < angleToGoalThreshold && std::abs(theFieldBall.positionRelative.y()) < ballYThreshold)
            goto alignBehindBall;           
            }
        if(theRobotInfo.number == 2){
            if(std::abs(v_angle_goal) < angleToGoalThreshold && std::abs(theFieldBall.positionRelative.y()) < ballYThreshold)
                 goto alignBehindBall; 
            }
      }
      action
      {          
        
        ball_X = (_ownPosition.translation.x()-_ballPosition(0))*2;        
        float target_X = theFieldBall.positionRelative.x() + ball_X;

        theSaySkill("align first");
          theLookForwardSkill();
          if(theRobotInfo.number == 1)
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(v_angle, target_X, theFieldBall.positionRelative.y()));
          if (theRobotInfo.number == 2) {
             theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(v_angle_goal, theFieldBall.positionRelative.x()-ballOffsetX, theFieldBall.positionRelative.y()));
             //theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(v_angle_goal, target_X, theFieldBall.positionRelative.y()));
          }
      }
    }
    

    state(alignBehindBall)
    {
        const GroundTruthWorldState&theGroundTruthWorldState =
        static_cast<const GroundTruthWorldState&>(Blackboard::getInstance()["GroundTruthWorldState"]);        
        const Pose2f _ownPosition = theGroundTruthWorldState.ownPose;
        const Vector2f _ballPosition = theGroundTruthWorldState.balls[0].position.head<2>();
        const Pose2f _firstteam = theGroundTruthWorldState.firstTeamPlayers[0].pose;
        const Angle v_angle =(theRobotPose.inversePose * Vector2f(_firstteam.translation.x(),_firstteam.translation.y())).angle();
        const Angle v_angle_goal =(theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
        
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto giverole;
       if( theRobotInfo.number == 1) {
           if( std::abs(v_angle) < angleToGoalThresholdPrecise && ballOffsetYRange.isInside(theFieldBall.positionRelative.y())) {
               goto kick; }
       }
              if( theRobotInfo.number == 2) {
          if( std::abs(v_angle_goal) < angleToGoalThresholdPrecise && ballOffsetYRange.isInside(theFieldBall.positionRelative.y())) {
          goto walktotarget; }
      }
      }

      action
      {        
        theLookForwardSkill();
        theSaySkill("align second");
         if(theRobotInfo.number == 1)
         theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(v_angle, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
         if(theRobotInfo.number == 2)
         theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(v_angle_goal, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
      }
    }
    
    state(walktotarget)
    {
        const GroundTruthWorldState&theGroundTruthWorldState =
        static_cast<const GroundTruthWorldState&>(Blackboard::getInstance()["GroundTruthWorldState"]);        
        const Pose2f _ownPosition = theGroundTruthWorldState.ownPose;
        const Vector2f _ballPosition = theGroundTruthWorldState.balls[0].position.head<2>();
        const Pose2f _firstteam = theGroundTruthWorldState.firstTeamPlayers[0].pose;
        const Angle v_angle =(theRobotPose.inversePose * Vector2f(_firstteam.translation.x(),_firstteam.translation.y())).angle();
        const Angle v_angle_goal =(theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
        
        transition
        {
            if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
              goto giverole;
           if(std::abs(v_angle_goal) > angleToGoalThreshold)
            goto alignBehindBall;
          
            
        }
        action
        {
        const GroundTruthWorldState&theGroundTruthWorldState =
        static_cast<const GroundTruthWorldState&>(Blackboard::getInstance()["GroundTruthWorldState"]);        
        const Pose2f _ownPosition = theGroundTruthWorldState.ownPose;
        const Vector2f _ballPosition = theGroundTruthWorldState.balls[0].position.head<2>();
        const Pose2f _firstteam = theGroundTruthWorldState.firstTeamPlayers[0].pose;
        const Angle v_angle =(theRobotPose.inversePose * Vector2f(_firstteam.translation.x(),_firstteam.translation.y())).angle();
        const Angle v_angle_goal =(theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
            
             theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(v_angle_goal,theFieldBall.positionRelative.x(),theFieldBall.positionRelative.y()));
        }
    }

 }

  
  
};

MAKE_CARD(CodeReleaseKickndribbleCard);
