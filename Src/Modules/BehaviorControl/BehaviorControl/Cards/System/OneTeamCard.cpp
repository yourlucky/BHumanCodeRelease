/**
 * @file NoTeamCard.cpp
 *
 * This file implements card that is active in situations where there is no team behavior.
 *
 * @author Arne Hasselbring
 */

#include "Representations/BehaviorControl/TeamSkills.h"
#include "Tools/BehaviorControl/Framework/Card/TeamCard.h"

TEAM_CARD(OneTeamCard,
{,
  CALLS(Role),
  CALLS(TeamActivity),
  CALLS(TeammateRoles),
  CALLS(TimeToReachBall),
});

class OneTeamCard : public OneTeamCardBase
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
    theTeamActivitySkill(TeamBehaviorStatus::noTeam);
    theTimeToReachBallSkill(TimeToReachBall());
    theTeammateRolesSkill(TeammateRoles());

    Role roleone;
    roleone.isGoalkeeper = false;//initial : false
    roleone.playBall = true;//initial : false
    roleone.supporterIndex = -2; //initial : -1
    roleone.numOfActiveSupporters = 5;
    theRoleSkill(roleone);
  }
};

MAKE_TEAM_CARD(OneTeamCard);
