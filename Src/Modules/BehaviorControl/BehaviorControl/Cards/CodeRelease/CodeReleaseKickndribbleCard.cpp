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
  CALLS(InWalkKick),
  CALLS(LookForward),
  CALLS(PathToTarget),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(Say),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(RobotInfo),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(1000) initialWaitTime,
    (int)(7000) ballNotSeenTimeout,
    (Angle)(5_deg) ballAlignThreshold,
    (float)(500.f) ballNearThreshold,
    (Angle)(20_deg) angleToGoalThreshold,
    (float)(400.f) ballAlignOffsetX,
    (float)(100.f) ballYThreshold,
    (Angle)(2_deg) angleToGoalThresholdPrecise,
    (float)(150.f) ballOffsetX,
    (Rangef)({140.f, 170.f}) ballOffsetXRange,
    (float)(100.f) ballOffsetY,
    (Rangef)({-20.f, 500.f}) ballOffsetYRange,
    (int)(10) minKickWaitTime,
    (int)(3000) maxKickWaitTime,
    
    (float)(-0.01f) addAngle,

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
    //theActivitySkill(BehaviorStatus::codeReleaseKickndribble);

    initial_state(start)
    {
      transition
      {
        if(state_time > initialWaitTime)
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
          if(theRobotInfo.number == 1)
          {
            if (state_time > c_time + 5000) //if not fallen for 10secs
            {
              c_time = state_time;
              goto walkToBall;
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

    state(runner)
    {
      transition
        {
          if (state_time < -1)
              goto giverole;            
        }
        action
        {
          //theLookForwardSkill();
          walkSpeed =1.f;
          Angle v_angle =0.f*pi;
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(v_angle, 3500,0));
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
        theSaySkill("time up");     
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theFieldBall.positionRelative.angle(), theFieldBall.positionRelative.x(),theFieldBall.positionRelative.y()));
      }
    }

    state(kick)
    {
      transition
      {
        if(state_time > maxKickWaitTime || (state_time > minKickWaitTime && theInWalkKickSkill.isDone()))
          goto giverole;
      }

      action
      {
          
        theSaySkill("kick");
        const GroundTruthWorldState&theGroundTruthWorldState =
        static_cast<const GroundTruthWorldState&>(Blackboard::getInstance()["GroundTruthWorldState"]);
        
        const Pose2f _ownPosition = theGroundTruthWorldState.ownPose;
        const Vector2f _ballPosition = theGroundTruthWorldState.balls[0].position.head<2>();
        const Pose2f _firstteam = theGroundTruthWorldState.firstTeamPlayers[0].pose;          
          
        const Angle v_angle=(theRobotPose.inversePose * Vector2f(_firstteam.translation.x(),_firstteam.translation.y())).angle();
        theLookForwardSkill();
        theInWalkKickSkill(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(v_angle, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
      }
    }

    state(alignkick)
    {
     //const Angle k_angle =180;
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto giverole;
        if(std::abs(theFieldBall.positionRelative.y()) < ballYThreshold)
          goto alignBehindBall;
      }

      action
      {
        const GroundTruthWorldState&theGroundTruthWorldState =
        static_cast<const GroundTruthWorldState&>(Blackboard::getInstance()["GroundTruthWorldState"]);
        
        const Pose2f _ownPosition = theGroundTruthWorldState.ownPose;
        const Vector2f _ballPosition = theGroundTruthWorldState.balls[0].position.head<2>();
        const Pose2f _firstteam = theGroundTruthWorldState.firstTeamPlayers[0].pose;
        
        ball_X = std::abs(_ownPosition.translation.x()-_ballPosition(0));
        ball_Y =  std::abs(_ownPosition.translation.y()-_ballPosition(1));
        
        const Angle v_angle =(theRobotPose.inversePose * Vector2f(_firstteam.translation.x(),_firstteam.translation.y())).angle();


        theSaySkill("align first");
          theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(v_angle, theFieldBall.positionRelative.x() - ballAlignOffsetX, theFieldBall.positionRelative.y()));
      }
    }

    state(alignBehindBall)
    {
        const GroundTruthWorldState&theGroundTruthWorldState =
        static_cast<const GroundTruthWorldState&>(Blackboard::getInstance()["GroundTruthWorldState"]);
        
        const Pose2f _ownPosition = theGroundTruthWorldState.ownPose;
        const Vector2f _ballPosition = theGroundTruthWorldState.balls[0].position.head<2>();
        const Pose2f _firstteam = theGroundTruthWorldState.firstTeamPlayers[0].pose;
        
        int I_X =  std::abs(_firstteam.translation.x()-_ownPosition.translation.x());
        int I_Y = std::abs( _firstteam.translation.y()-_ownPosition.translation.y());
        
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto giverole;
       if(I_X+ I_Y > 2000  && ballOffsetYRange.isInside(theFieldBall.positionRelative.y()))
          goto kick;
      }

      action
      {        
        const Angle v_angle=(theRobotPose.inversePose * Vector2f(_firstteam.translation.x(),_firstteam.translation.y())).angle();
        theLookForwardSkill();
        theSaySkill("align second");
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(v_angle, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
      }
    }

 }

  
  
};

MAKE_CARD(CodeReleaseKickndribbleCard);