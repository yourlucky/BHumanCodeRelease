/*
option(Striker)
    {
      //theActivitySkill(BehaviorStatus::codeReleaseKickAtGoal);
      //theActivitySkill(BehaviorStatus::codeReleaseRunSpeed);


      initial_state(start)
      {
        
        transition
        {
          if (state_time > initialWaitTime)
            goto turnToBall;
        }

        action
        {
          theLookForwardSkill();
          theStandSkill();
          theSaySkill("good good good striker Card");
        }
      }

      state(getUP)
      {
          transition
          {
            if (theFallDownState.state != FallDownState::fallen)
                goto turnToBall;
          }

          action
          {
            theStandSkill();
            theSaySkill("get up");
          }
      }

      state(turnToBall)
      {
        transition
        {
          if (!theFieldBall.ballWasSeen(ballNotSeenTimeout))
            goto searchForBall;
        if (std::abs(theFieldBall.positionRelative.angle()) < ballAlignThreshold)
        {
            c_time = state_time;
            goto walkToBall_1;
        }
        }

        action
        {
          theLookForwardSkill();
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), Pose2f(theFieldBall.positionRelative.angle(), 0.f, 0.f));
          theSaySkill("turn to ball");
        }
      }

      state(walkToBall_1) //walk speed 0.2
      {
        transition
        {
          if (!theFieldBall.ballWasSeen(ballNotSeenTimeout))
            goto searchForBall;
          
          if (theFallDownState.state == FallDownState::fallen)
            goto getUP;

          if (state_time > c_time + 5000) //if not fallen for 0.5 secs
          {
              c_time = state_time;
              walkSpeed += 0.3;
              goto walkToBall_2;
          }
        }

        action
        {
          theLookForwardSkill();
          theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), theFieldBall.positionRelative);
          theSaySkill("speed one");
        }
      }

      state(walkToBall_2) //walk speed 0.5
      {
          transition
          {
            if (!theFieldBall.ballWasSeen(ballNotSeenTimeout))
              goto searchForBall;

            if (theFallDownState.state == FallDownState::fallen)
                goto getUP;

            if (state_time > c_time + 5000) //if not fallen for 10secs
            {
                c_time = state_time;
                walkSpeed += 0.3;
                goto walkToBall_3;
            }
          }

          action
          {
            theLookForwardSkill();
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), theFieldBall.positionRelative);
            theSaySkill("speed two");
          }
      }

      state(walkToBall_3) //walk speed 0.8
      {
          transition
          {
            if (!theFieldBall.ballWasSeen(ballNotSeenTimeout))
              goto searchForBall;

            if (theFallDownState.state == FallDownState::fallen)
                goto getUP;

            if (state_time > c_time + 5000) //if not fallen for 10secs
            {
                c_time = state_time;
                walkSpeed += 0.2;
                goto walkToBall_4;
            }
          }

          action
          {
            theLookForwardSkill();
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), theFieldBall.positionRelative);
            theSaySkill("speed three");
          }
      }

      state(walkToBall_4) //walk speed 1.0
      {
          transition
          {
            if (!theFieldBall.ballWasSeen(ballNotSeenTimeout))
              goto searchForBall;

            if (theFallDownState.state == FallDownState::fallen)
                goto getUP;

            if (state_time > c_time + 10000) //if not fallen for 10secs
            {
                c_time = state_time;
                walkSpeed += 0.2;
                goto walkToBall_3;
            }
          }

           action
          {
            theLookForwardSkill();
            theWalkToTargetSkill(Pose2f(walkSpeed, walkSpeed, walkSpeed), theFieldBall.positionRelative);
            theSaySkill("speed four");
          }
      }

      state(searchForBall)
      {
        transition
        {
          if (theFieldBall.ballWasSeen())
            goto turnToBall;
        }

        action
        {
          theLookForwardSkill();
          theWalkAtRelativeSpeedSkill(Pose2f(walkSpeed, 0.f, 0.f));
          theSaySkill("search");
        }
      }
    }

        Angle calcAngleToGoal() const
    {
        return (theRobotPose.inversePose * Vector2f(theFieldDimensions.xPosOpponentGroundline, 0.f)).angle();
    }
*/