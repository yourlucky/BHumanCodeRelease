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

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Communication/RobotInfo.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include "Tools/Math/BHMath.h"
#include <cmath>
#include <iostream>

#define mymax(a,b) (((a) > (b)) ? (a) : (b))
#define mymin(a,b) (((a) < (b)) ? (a) : (b))

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
  REQUIRES(RobotInfo),
  DEFINES_PARAMETERS(
  {,
    (float)(0.8f) walkSpeed,
    (int)(1000) initialWaitTime,
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
    (float)(3800.f) maxAlertLen,
    (float)(200.f) spaceX,
    (float)(120.f) spaceY,
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

    initial_state(giveRole)
    {
      std::string str = "Robot Number:"+theRobotInfo.number;
      str = str+"yes";
      std::cout<<str;

      theSaySkill(str);
      if(theRobotInfo.number == 3)
        goto goalkeeper;
      else
        goto start;
      
    }

    state(goalkeeper)
    {
      transition{
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBallKeeper;
        
        if(theFieldBall.isInsideOwnPenaltyArea)
          goto walkToBallKeeper;
      }
      action{
        Pose2f pos = getThePose();
        if(theFieldBall.distanceToOwnPenaltyArea > maxAlertLen)
        {
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), 
            Pose2f(theFieldBall.positionRelative.angle(), pos.translation.x(), pos.translation.y()));
        }
        else
        {
          if(std::abs(theFieldBall.positionRelative.angle()) > ballAlignThreshold)
          {
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), 
            Pose2f(theFieldBall.positionRelative.angle(),0.f, 0.f));
          }
          else
          {
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), 
            Pose2f(theFieldBall.positionRelative.angle(), pos.translation.x(), pos.translation.y()));
          }
        }
      }
    }

    state(walkToBallKeeper)
    {
      transition
      {
        if(!theFieldBall.isInsideOwnPenaltyArea)
          goto goalkeeper;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBallKeeper;
        if(theFieldBall.positionRelative.squaredNorm() < sqr(ballNearThreshold))
          goto alignToGoalKeeper;
      }

      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), theFieldBall.positionRelative);
      }
    }

    state(alignToGoalKeeper)
    {
      const Angle angleAwayFromGoal = calcAngleAwayFromGoal();

      transition
      {
        if(!theFieldBall.isInsideOwnPenaltyArea)
          goto goalkeeper;
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBallKeeper;
        if(std::abs(angleAwayFromGoal) < angleToGoalThreshold && std::abs(theFieldBall.positionRelative.y()) < ballYThreshold)
          goto alignBehindBallKeeper;
      }

      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleAwayFromGoal, theFieldBall.positionRelative.x() - ballAlignOffsetX, theFieldBall.positionRelative.y()));
      }
    }

    state(alignBehindBallKeeper)
    {
      
      const Angle angleAwayFromGoal = calcAngleAwayFromGoal();

      transition
      {
        if(!theFieldBall.isInsideOwnPenaltyArea)
          goto goalkeeper;

        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBallKeeper;
        if(ballOffsetXRange.isInside(theFieldBall.positionRelative.x()) && ballOffsetYRange.isInside(theFieldBall.positionRelative.y()))
          goto kickKeeper;
      }

      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(angleAwayFromGoal, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
      }
    }

    state(kickKeeper)
    {
      const Angle angleAwayFromGoal = calcAngleAwayFromGoal();
      //if(std::abs(

      transition
      {
        if(!theFieldBall.isInsideOwnPenaltyArea)
          goto goalkeeper;

        if(state_time > maxKickWaitTime || (state_time > minKickWaitTime && theInWalkKickSkill.isDone()))
          goto goalkeeper;
      }

      action
      {
        theLookForwardSkill();
        theInWalkKickSkill(WalkKickVariant(WalkKicks::forward, Legs::left), Pose2f(angleAwayFromGoal, theFieldBall.positionRelative.x() - ballOffsetX, theFieldBall.positionRelative.y() - ballOffsetY));
      }
    }

    state(searchForBallKeeper)
    {
      transition
      {
        if(theFieldBall.ballWasSeen())
          goto goalkeeper;
      }

      action
      {
        theLookForwardSkill();
        theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
      }
    }

    state(start)
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
      }
    }

    state(walkToBall)
    {
      transition
      {
        if(!theFieldBall.ballWasSeen(ballNotSeenTimeout))
          goto searchForBall;
        if(theFieldBall.positionRelative.squaredNorm() < sqr(ballNearThreshold))
          goto alignToGoal;
      }

      action
      {
        theLookForwardSkill();
        theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), theFieldBall.positionRelative);
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
      }
    }
  }

  Angle calcAngleAwayFromGoal() const
  {
    float selfDistanceToOwnGoal = std::abs(theRobotPose.translation.x() - theFieldDimensions.xPosOwnGoal);
    float ballDistanceToOwnGoal = std::abs(theFieldBall.positionOnField.x() - theFieldDimensions.xPosOwnGoal);
    // if(distanceToOwnGoal > ballDistanceToOwnGoal)
    // {
    //   if(theFieldBall.positionRelative.y() > 0.f)
    //   {
    //     return (theRobotPose.inversePose * Vector2f(theFieldBall.positionOnField.x(), theFieldDimensions.yPosLeftFieldBorder)).angle();
    //   }
    //   return (theRobotPose.inversePose * Vector2f(theFieldBall.positionOnField.x(), theFieldDimensions.yPosRightFieldBorder)).angle();
    // }

    //Angle angleToSelfGoal = (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOwnGroundline, 0.f)).angle();
    //std::cout<<"Angle"<<angleToSelfGoal;
    if(ballDistanceToOwnGoal<selfDistanceToOwnGoal)
    {
         if(theFieldBall.positionRelative.y() - theRobotPose.translation.y() > 0.f)
      {
        return (theRobotPose.inversePose * Vector2f(theFieldBall.positionOnField.x(), theFieldDimensions.yPosLeftFieldBorder)).angle();
      }
      return (theRobotPose.inversePose * Vector2f(theFieldBall.positionOnField.x(), theFieldDimensions.yPosRightFieldBorder)).angle();

    }
    return Angle(0.f);
    //return (theRobotPose.inversePose * Vector2f(theFieldBall.positionOnField.x(), theFieldBall.positionOnField.y())).angle();
  }

  Angle calcAngleToGoal() const
  {
    return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
  }
  

  Pose2f getThePose()
  {
    float angle1,angle2,angle;
    if (fabs(theFieldDimensions.xPosOwnGoalPost-theFieldBall.positionOnField.x()) > 1e-4)
    {
    
      angle1 = (float)atan2((theFieldDimensions.yPosRightGoal-theFieldBall.positionOnField.y()),(theFieldDimensions.xPosOwnGoalPost-theFieldBall.positionOnField.x()));
    	angle2 = (float)atan2((theFieldDimensions.yPosLeftGoal-theFieldBall.positionOnField.y()),(theFieldDimensions.xPosOwnGoalPost-theFieldBall.positionOnField.x()));
    }
    else
    {
    
      angle1 = angle2 = 1.57f;
    }
    angle = (angle1+angle2)/2;
    float ratio = (float)tan(angle);
    float x1,y1,x2,y2,x,y;
    x2 = theFieldDimensions.xPosOwnPenaltyArea - spaceX;
    y2 = ratio*(x2-theFieldBall.positionOnField.x())+theFieldBall.positionOnField.y();
    if (ratio < 0)
    {
    
      y1 = theFieldDimensions.yPosRightPenaltyArea + spaceY;
      x1 = (y1 - theFieldBall.positionOnField.y())/ratio + theFieldBall.positionOnField.x();
      x = mymin(x1,x2);
      y = mymax(y1,y2);
    }
    else
    {
    
      y1 = theFieldDimensions.yPosLeftPenaltyArea - spaceY;
      x1 = (y1 - theFieldBall.positionOnField.y())/ratio + theFieldBall.positionOnField.x();
      x = mymin(x1,x2);
      y = mymin(y1,y2);
    }
    return theRobotPose.inversePose * Vector2f(x, y);

  }

};

MAKE_CARD(CodeReleaseKickAtGoalCard);
