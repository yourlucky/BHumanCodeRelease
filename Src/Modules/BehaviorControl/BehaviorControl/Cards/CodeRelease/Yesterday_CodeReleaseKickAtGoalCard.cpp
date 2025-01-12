/**
 * @file CodeReleaseKickAtGoalCard.cpp
 *
 * This file implements a basic striker behavior for the code release.
 * Normally, this would be decomposed into at least
 * - a ball search behavior card
 * - a skill for getting behind the ball
 *
 * @author Arne Hasselbring
 */

/*

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"


#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"

#include "Tools/Module/Blackboard.h"
#include "Representations/Infrastructure/GroundTruthWorldState.h"

#include "Tools/Math/Angle.h"



CARD(CodeReleaseKickAtGoalCard,
{,
  CALLS(Activity),
  CALLS(InWalkKick),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(Say),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
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
    (int)(10) minKickWaitTime,
    (int)(3000) maxKickWaitTime,

    (int)(-0.1f) angleParm,
  }),
});

class CodeReleaseKickAtGoalCard : public CodeReleaseKickAtGoalCardBase
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
    theActivitySkill(BehaviorStatus::codeReleaseKickAtGoal);

    const GroundTruthRobotPose &theGroundTruthRobotPose =
    static_cast<const GroundTruthRobotPose &>( Blackboard::getInstance()["GroundTruthRobotPose"]);

    const GroundTruthWorldState&theGroundTruthWorldState =
    static_cast<const GroundTruthWorldState&>(Blackboard::getInstance()["GroundTruthWorldState"]);

    initial_state(start)
    {
      transition
      {
        if(state_time > initialWaitTime)
          goto turnToBall;
      }

      action
      {
        theLookForwardSkill();
        theStandSkill();
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

        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theFieldBall.positionRelative.angle(), 0.f, 0.f));
        theSaySkill("inital");
      }
    }

    state(walkToBall)
    {
      transition
      {
        //if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          //goto searchForBall;
        if(theFieldBall.positionRelative.squaredNorm() < sqr(ballNearThreshold))
          goto notmove;
      }

      action
      {

        const Vector2f _ballPosition = theGroundTruthWorldState.balls[0].position.head<2>(); 
        const Pose2f _ownPosition = theGroundTruthWorldState.ownPose;  
        const Pose2f _FriendPosition = theGroundTruthWorldState.firstTeamPlayers[0].pose;

        float x_firstTeamPlayers = _FriendPosition.translation(0)*-1;
        float y_firstTeamPlayers = _FriendPosition.translation(1)*-1;


        //float x_ownPosition = _ownPosition.translation(0)+300;
        //float y_ownPosition = _ownPosition.translation(1)+4000;

        //float virtualBallXPosition = _ballPosition(0)*-1;
        //float virtualBallYPosition = _ballPosition(1)*-1;
      

      //float virtualBallXPosition = -1*_ballPosition[0];
      //float virtualBallYPosition = -1*_ballPosition[1];
      //float virtualBallXPosition = theFieldBall.positionRelative.x();
      //float virtualBallYPosition = theFieldBall.positionRelative.y();
      //float myownXposition = _ownPosition[0]*2; 
      //float myownXposition = _ownPosition[0]*3; 


       Angle v_angle =1.45*pi;
       theLookForwardSkill();
       theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed),Vector2f(x_firstTeamPlayers,y_firstTeamPlayers));

        //theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed),theFieldBall.positionRelative);//chagned
        //theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed),_ballPosition);

        //theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed),Pose2f(virtualBallXPosition,virtualBallYPosition));

        //theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed),Vector2f(virtualBallXPosition,virtualBallYPosition));

        
        theSaySkill("walk to ball");
      
      }
    }
    
    state(notmove)
    {

      action
      {
        theSaySkill("not move");
      }
    }


    state(alignToGoal)
    {
      const Angle angleToGoal = calcAngleToGoal();

      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(std::abs(angleToGoal) < angleToGoalThreshold && std::abs(theFieldBall.positionRelative.y()) < ballYThreshold)
          goto alignBehindBall;
      }

      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballAlignOffsetX, theFieldBall.positionRelative.y()));
        theSaySkill("align to goal");
      }
    }

    state(alignBehindBall)
    {
      const Angle angleToGoal = calcAngleToGoal();

      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(std::abs(angleToGoal) < angleToGoalThresholdPrecise && ballOffsetXRange.isInside(theFieldBall.positionRelative.x()) && ballOffsetYRange.isInside(theFieldBall.positionRelative.y()))
          goto kick;
      }

      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
        theSaySkill("align behind ball");
      }
    }

    state(kick)
    {
      const Angle angleToGoal = calcAngleToGoal();

      transition
      {
        if(state_time > maxKickWaitTime || (state_time > minKickWaitTime && theInWalkKickSkill.isDone()))
          goto start;
      }

      action
      {
        theLookForwardSkill();
        theInWalkKickSkill(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
        theSaySkill("kick");
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
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
        theSaySkill("search for ball");
      }
    }
  }

  Angle calcAngleToGoal() const
  {
    return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle()*(1/10)+angleParm;
  }
};



MAKE_CARD(CodeReleaseKickAtGoalCard);

*/

/**
 * @file CodeReleaseKickAtGoalCard.cpp
 *
 * This file implements a basic striker behavior for the code release.
 * Normally, this would be decomposed into at least
 * - a ball search behavior card
 * - a skill for getting behind the ball
 *
 * @author Arne Hasselbring
 */

/*

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"


#include "Representations/BehaviorControl/Skills.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"

#include "Tools/Module/Blackboard.h"
#include "Representations/Infrastructure/GroundTruthWorldState.h"

#include "Tools/Math/Angle.h"



CARD(CodeReleaseKickAtGoalCard,
{,
  CALLS(Activity),
  CALLS(InWalkKick),
  CALLS(LookForward),
  CALLS(Stand),
  CALLS(WalkAtRelativeSpeed),
  CALLS(WalkToTarget),
  CALLS(Say),
  REQUIRES(FieldBall),
  REQUIRES(FieldDimensions),
  REQUIRES(RobotPose),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
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
    (int)(10) minKickWaitTime,
    (int)(3000) maxKickWaitTime,

    (int)(-0.1f) angleParm,
  }),
});

class CodeReleaseKickAtGoalCard : public CodeReleaseKickAtGoalCardBase
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
    theActivitySkill(BehaviorStatus::codeReleaseKickAtGoal);

    const GroundTruthRobotPose &theGroundTruthRobotPose =
    static_cast<const GroundTruthRobotPose &>( Blackboard::getInstance()["GroundTruthRobotPose"]);

    const GroundTruthWorldState&theGroundTruthWorldState =
    static_cast<const GroundTruthWorldState&>(Blackboard::getInstance()["GroundTruthWorldState"]);

    initial_state(start)
    {
      transition
      {
        if(state_time > initialWaitTime)
          goto turnToBall;
      }

      action
      {
        theLookForwardSkill();
        theStandSkill();
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

        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theFieldBall.positionRelative.angle(), 0.f, 0.f));
        theSaySkill("inital");
      }
    }

    state(walkToBall)
    {
      transition
      {
        //if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          //goto searchForBall;
        if(theFieldBall.positionRelative.squaredNorm() < sqr(ballNearThreshold))
          goto notmove;
      }

      action
      {

        const Vector2f _ballPosition = theGroundTruthWorldState.balls[0].position.head<2>(); 
        const Pose2f _ownPosition = theGroundTruthWorldState.ownPose;  
        const Pose2f _FriendPosition = theGroundTruthWorldState.firstTeamPlayers[0].pose;

        float x_firstTeamPlayers = _FriendPosition.translation(0)*-1;
        float y_firstTeamPlayers = _FriendPosition.translation(1)*-1;


        //float x_ownPosition = _ownPosition.translation(0)+300;
        //float y_ownPosition = _ownPosition.translation(1)+4000;

        //float virtualBallXPosition = _ballPosition(0)*-1;
        //float virtualBallYPosition = _ballPosition(1)*-1;
      

      //float virtualBallXPosition = -1*_ballPosition[0];
      //float virtualBallYPosition = -1*_ballPosition[1];
      //float virtualBallXPosition = theFieldBall.positionRelative.x();
      //float virtualBallYPosition = theFieldBall.positionRelative.y();
      //float myownXposition = _ownPosition[0]*2; 
      //float myownXposition = _ownPosition[0]*3; 


       Angle v_angle =1.45*pi;
       theLookForwardSkill();
       theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed),Vector2f(x_firstTeamPlayers,y_firstTeamPlayers));

        //theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed),theFieldBall.positionRelative);//chagned
        //theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed),_ballPosition);

        //theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed),Pose2f(virtualBallXPosition,virtualBallYPosition));

        //theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed),Vector2f(virtualBallXPosition,virtualBallYPosition));

        
        theSaySkill("walk to ball");
      
      }
    }
    
    state(notmove)
    {

      action
      {
        theSaySkill("not move");
      }
    }


    state(alignToGoal)
    {
      const Angle angleToGoal = calcAngleToGoal();

      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(std::abs(angleToGoal) < angleToGoalThreshold && std::abs(theFieldBall.positionRelative.y()) < ballYThreshold)
          goto alignBehindBall;
      }

      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballAlignOffsetX, theFieldBall.positionRelative.y()));
        theSaySkill("align to goal");
      }
    }

    state(alignBehindBall)
    {
      const Angle angleToGoal = calcAngleToGoal();

      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(std::abs(angleToGoal) < angleToGoalThresholdPrecise && ballOffsetXRange.isInside(theFieldBall.positionRelative.x()) && ballOffsetYRange.isInside(theFieldBall.positionRelative.y()))
          goto kick;
      }

      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
        theSaySkill("align behind ball");
      }
    }

    state(kick)
    {
      const Angle angleToGoal = calcAngleToGoal();

      transition
      {
        if(state_time > maxKickWaitTime || (state_time > minKickWaitTime && theInWalkKickSkill.isDone()))
          goto start;
      }

      action
      {
        theLookForwardSkill();
        theInWalkKickSkill(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(angleToGoal, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
        theSaySkill("kick");
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
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
        theSaySkill("search for ball");
      }
    }
  }

  Angle calcAngleToGoal() const
  {
    return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle()*(1/10)+angleParm;
  }
};

MAKE_CARD(CodeReleaseKickAtGoalCard);

*/