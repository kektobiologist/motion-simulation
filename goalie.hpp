#ifndef GOALIE_HPP
#define GOALIE_HPP

#include <list>
#include "beliefstate.h"
#include "geometry.h"
#include "pose.h"
#include "constants.h"
#include <algorithm>
#include <fstream>

class TGoalie
{
public:
TGoalie(){
} // TGoalie

~TGoalie()
{ } // ~TGoalKeeping
inline bool isActiveTactic(void) const
{
  return false;
}

    bool isGoalKeeperInPosition(const BeliefState* state, int botID)
{
  if ((state->homeX[botID] >  (-HALF_FIELD_MAXX + GOAL_DEPTH)) &&
      (state->homeX[botID] <= (-HALF_FIELD_MAXX + GOAL_DEPTH + BOT_RADIUS*2)) &&
      (state->homeY[botID] >= OUR_GOAL_MINY - DBOX_HEIGHT) &&
      (state->homeY[botID] <= (OUR_GOAL_MAXY + DBOX_HEIGHT)))
    return true;
  else
    return false;
}

Pose execute(BeliefState *state, int botID)
{
 Vector2D<int> homePos(state->homeX[botID], state->homeY[botID]);
 Vector2D<int> ballPos(state->ballX, state->ballY);
  float dist = Vector2D<int>::dist(ballPos, homePos);
  if (!isGoalKeeperInPosition(state, botID) && dist > 0.5 * BOT_BALL_THRESH)
  {
    return Pose(-HALF_FIELD_MAXX + GOAL_DEPTH + BOT_RADIUS*1.5, 0, PI/2);
  }
  Vector2D<int> botDestination ;
  float ang1;
     if(abs(state->ballVx)<10)
         ang1 = 0;
     else
          ang1 = atan(state->ballVy/state->ballVx);
   //in case of ball traveling directly from the oponent's half
   botDestination.y = state->ballY - ((state->ballX) - (-HALF_FIELD_MAXX + DBOX_WIDTH + BOT_RADIUS*1.5))*tan(ang1) ;
   if(botDestination.y >=  OUR_GOAL_MAXY)
       botDestination.y = OUR_GOAL_MAXY - BOT_RADIUS;
   if(botDestination.y <= OUR_GOAL_MINY)
       botDestination.y = OUR_GOAL_MINY + BOT_RADIUS;
   qDebug() << "bot dest y " << botDestination.y << endl;
    botDestination.x = (-HALF_FIELD_MAXX + GOAL_DEPTH+ 1.5*BOT_RADIUS); //+ 100;   //set your threshold ********

    return Pose(botDestination.x, botDestination.y, PI/2);
}
};// class TGoalKeepingOurside
#endif // GOALIE_H
