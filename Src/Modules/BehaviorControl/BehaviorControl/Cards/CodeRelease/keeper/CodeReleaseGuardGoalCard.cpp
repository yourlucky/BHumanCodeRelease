/**
 * @file CodeReleaseGuardGoalCard.cpp
 *
 * This file states the default behavior that make keeper stand in a proper position.
 *
 * @author CC
 */

#include "Representations/BehaviorControl/FieldBall.h"
#include "Representations/BehaviorControl/Skills.h"
#include "Representations/Configuration/FieldDimensions.h"
#include "Representations/Modeling/RobotPose.h"
#include "Tools/BehaviorControl/Framework/Card/Card.h"
#include "Tools/BehaviorControl/Framework/Card/CabslCard.h"
#include <cmath>

#define mymax(a,b) (((a) > (b)) ? (a) : (b))
#define mymin(a,b) (((a) < (b)) ? (a) : (b))

CARD(CodeReleaseGuardGoalCard,
    {
        ,
      CALLS(Activity),
      CALLS(LookForward),
      CALLS(WalkToTarget),
      CALLS(SpecialAction),
      CALLS(Say),
      REQUIRES(FieldBall),
      REQUIRES(FieldDimensions),
      REQUIRES(RobotPose),
      DEFINES_PARAMETERS(
      {
        ,
        (float)(0.8f) walkSpeed,
        (Angle)(15_deg) ballAlignThreshold,
        (float)(3800.f) maxAlertLen,
        (float)(200.f) spaceX,
        (float)(120.f) spaceY,
      }),
    });

class CodeReleaseGuardGoalCard : public CodeReleaseGuardGoalCardBase
{

    bool preconditions() const override
    {

        return true;
    }

    bool postconditions() const override
    {

        return true;
    }

    Pose2f calFittingPose()
    {

        float angle1, angle2, angle;
        if (fabs(theFieldDimensions.xPosOwnGoalPost - theFieldBall.positionOnField.x()) > 1e-4)
        {

            angle1 = (float)atan2((theFieldDimensions.yPosRightGoal - theFieldBall.positionOnField.y()), (theFieldDimensions.xPosOwnGoalPost - theFieldBall.positionOnField.x()));
            angle2 = (float)atan2((theFieldDimensions.yPosLeftGoal - theFieldBall.positionOnField.y()), (theFieldDimensions.xPosOwnGoalPost - theFieldBall.positionOnField.x()));
        }
        else
        {

            angle1 = angle2 = 1.57f;
        }
        angle = (angle1 + angle2) / 2;
        float ratio = (float)tan(angle);
        float x1, y1, x2, y2, x, y;
        x2 = theFieldDimensions.xPosOwnPenaltyArea - spaceX;
        y2 = ratio * (x2 - theFieldBall.positionOnField.x()) + theFieldBall.positionOnField.y();
        if (ratio < 0)
        {

            y1 = theFieldDimensions.yPosRightPenaltyArea + spaceY;
            x1 = (y1 - theFieldBall.positionOnField.y()) / ratio + theFieldBall.positionOnField.x();
            x = mymin(x1, x2);
            y = mymax(y1, y2);
        }
        else
        {

            y1 = theFieldDimensions.yPosLeftPenaltyArea - spaceY;
            x1 = (y1 - theFieldBall.positionOnField.y()) / ratio + theFieldBall.positionOnField.x();
            x = mymin(x1, x2);
            y = mymin(y1, y2);
        }
        return theRobotPose.inversePose * Vector2f(x, y);

    }
    void execute() override
    {

        theActivitySkill(BehaviorStatus::codeReleaseGuardGoal);//BehaviorStatus.h
        theLookForwardSkill();
        if (theFieldBall.distanceToOwnPenaltyArea > maxAlertLen)
        {

            theSaySkill("none of my business");
            theSpecialActionSkill(SpecialActionRequest::playDead);
        }
        else
        {

            theSaySkill("I am in");
            Pose2f pos = calFittingPose();
            if (std::abs(theFieldBall.positionRelative.angle()) > ballAlignThreshold)
            {

                theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theFieldBall.positionRelative.angle(), 0.f, 0.f));
            }
            else
            {

                theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theFieldBall.positionRelative.angle(), pos.translation.x(), pos.translation.y()));
            }
        }
    }
};

MAKE_CARD(CodeReleaseGuardGoalCard);