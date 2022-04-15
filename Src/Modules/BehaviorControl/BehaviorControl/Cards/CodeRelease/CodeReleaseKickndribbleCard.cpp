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
  CALLS(PassTarget),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  REQUIRES(RobotInfo),
  DEFINES_PARAMETERS(
  {,
    (float)(1.0f) walkSpeed,
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
    theActivitySkill(BehaviorStatus::codeReleaseKickndribbleCard);

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
            if (state_time > c_time - 5000) //if not fallen for 10secs
            {
              c_time = state_time;
              goto searchForBall;
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
          //const Angle v_angle=(theRobotPose.inversePose * Vector2f(4000,500)).angle();
          
          //theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(0.f, 4000,500));
          theSaySkill("one"); 
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
        //theSaySkill("time up");     
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
        //theInWalkKickSkill(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(v_angle, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
        Skills::thePassTarget(3,Vector2f(theFieldBall.positionRelative.x(),theFieldBall.positionRelative.y()));
      
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

      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto giverole;
        
        if(std::abs(v_angle) < angleToGoalThreshold && std::abs(theFieldBall.positionRelative.y()) < ballYThreshold)
          goto alignBehindBall;
      }

      action
      {
        
        ball_X = (_ownPosition.translation.x()-_ballPosition(0))*2;
        
        float target_X = theFieldBall.positionRelative.x() + ball_X;

        theSaySkill("align first");
          theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(v_angle, target_X, theFieldBall.positionRelative.y()));
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
        
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto giverole;
       if(  std::abs(v_angle) < angleToGoalThresholdPrecise && ballOffsetYRange.isInside(theFieldBall.positionRelative.y()))
          goto kick;
      }

      action
      {        
        theLookForwardSkill();
        theSaySkill("align second");
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(v_angle, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
      }
    }

 }

  
  
};

MAKE_CARD(CodeReleaseKickndribbleCard);
