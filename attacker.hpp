#ifndef TTATTACK_HPP
#define TTATTACK_HPP

#include <list>
#include "beliefstate.h"
#include "geometry.h"
#include "pose.h"
#include "constants.h"
#define ANGLE_TO_DIST 0


class TAttack
{
  static const int offset = 400;
  // Corner Case: wall, ball, bot in line
  Point2D<int> prevBotPos;
  float prevBotAngle;
  public:
    TAttack()
    {
      iState = APPROACHING;
    } // TAttack

    ~TAttack()
    { } // ~TAttack

    enum InternalState
    {
      APPROACHING,
      ATTACKING
    } iState;
    int hasAchievedOffset;
    inline bool isActiveTactic(void) const
    {
      return true;
    }

    bool pointxInField(Vector2D<int> final)
    {
      if((final.x < HALF_FIELD_MAXX - (BALL_AT_CORNER_THRESH) && final.x > -HALF_FIELD_MAXX + (BALL_AT_CORNER_THRESH)))
      {
        if((final.y < HALF_FIELD_MAXY - BALL_AT_CORNER_THRESH && final.y > -HALF_FIELD_MAXY + BALL_AT_CORNER_THRESH))
        {
          return true;
        }
        else return false;
      }
      else  return false;
    }


    Pose execute(BeliefState *state,int botID)
    {
        Vector2D<int> ballPos(state->ballX,state->ballY);
        Vector2D<int> homePos(state->homeX[botID], state->homeY[botID]);
        Vector2D<int> opp_goal(HALF_FIELD_MAXX, 0);

        static Vector2D<float> lastVel[10];
                    static int index = 0;
                    if(index < 10) {
                        lastVel[index].x = state->ballVx;
                        lastVel[index].y = state->ballVy;
                        index = (index + 1) % 10;
                    }
                    Vector2D<float> avgBallVel(0.0,0.0);
                    for(int i=0;i<10;i++) {
                        avgBallVel.x += lastVel[i].x;
                        avgBallVel.y += lastVel[i].y;
                    }
                    avgBallVel.x /= 10.0;
                    avgBallVel.y /= 10.0;

        float dist = Vector2D<int>::dist(ballPos, homePos);

        if(dist < BOT_BALL_THRESH)
        {
            /* Ball is with bot. So go to goal */
            printf("\n Going to Ball \n ");
            //sParam.GoToPointP.align = false;
            //tState = RUNNING;

            Pose appro(HALF_FIELD_MAXX,0,Vector2D<int>::angle(opp_goal, ballPos));
            iState = ATTACKING;
            return appro;
        }
        else
        {          
          /* Ball is not with bot so go to ball first */
          //sParam.GoToPointP.align = true;
          float ballgoaldist = Vector2D<int>::dist(ballPos, opp_goal);
          float factor = (int)Vector2D<int>::dist(ballPos,homePos);
          factor /= 5*MAX_BOT_SPEED;
          factor =0;
          //int ballPosX = state->ballPos.x;// + factor * state->ballVel.x;
          //int ballPosY = state->ballPos.y;// + factor * state->ballVel.y;
          
          float offset = TAttack::offset;//TAttack::offset * state->ballVel.abs()/6000.0+
                    if(homePos.x < ballPos.x)
						offset = 0;
                    int x3 = (ballPos.x * (ballgoaldist + offset) - offset * HALF_FIELD_MAXX) / ballgoaldist;
          int y3 = (ballPos.y * (ballgoaldist + offset)) / ballgoaldist;
          /// logarithmic search to place offset point in field. */
        //float offset = 600;
					
					
					
					/****************************** added velocity factor in offset *****************/
				 factor = 0.4;
				 int targetX=0,targetY=0;					
					{
						printf("Ball Veclocity : x = %f  y = %f\n",avgBallVel.x,avgBallVel.y);
                        int ballBotDist = (int)Vector2D<int>::dist(homePos,ballPos);
                        printf("delta x = %d   delta y = %d\n",(int)(factor * state->ballVx),(int)(factor * state->ballVy));
						
                        targetX = ballPos.x + (int)(factor * avgBallVel.x);
                        targetY = ballPos.y + (int)(factor * avgBallVel.y);
						//x3 = targetX;//(targetX * (ballgoaldist + offset) - offset * OPP_GOAL_X) / ballgoaldist;
						//y3 = targetY;//(targetY * (ballgoaldist + offset)) / ballgoaldist; 
					}
					/**************************** done velocity factor ****************************/
					
					/*************************** Weighted offset *********************************/
					//weighted value of previously calculated offset and one using ball velocity
					// P = (P1 + W*P2)/(W + 1)
                    float weight = (avgBallVel.x < -100 ? -(avgBallVel.x)-100 : 0.00)*sqrt(avgBallVel.y*avgBallVel.y) / (200.0);
					Vector2D<int> final(0,0);
					final.x = x3 + weight * targetX;			
					final.y = y3 + weight * targetY;
					final.x = final.x/(weight + 1);
					final.y = final.y/(weight + 1);
					
					x3 = final.x, y3 = final.y;
					/************************ Done offset ***************************************/
					
                    //SkillSet::comm->addCircle(x3,y3,300);

          while(isPointinField(Point2D<int>(x3, y3)))
          {
            if(isPointinField(ballPos))
            {
              offset= 0;
              x3 = (ballPos.x * (ballgoaldist + offset) - offset * HALF_FIELD_MAXX) / ballgoaldist;
              y3 = (ballPos.y * (ballgoaldist + offset)) / ballgoaldist;
              break;
            }
            offset /= 1.25;
            if(offset <= 1.0)
              break;
            x3 = (ballPos.x * (ballgoaldist + offset) - offset * HALF_FIELD_MAXX) / ballgoaldist;
            y3 = (ballPos.y * (ballgoaldist + offset)) / ballgoaldist;
          }
          offset/=1.25;
          /// log search to place offset at a point not co-inciding with a bot.
          Point2D<int> offsetpt(x3,y3);
          /*while(1)
          {
            bool flag = false;
            for(int i=0; i < 5; i++)
            {
                Vector2D<int> homePosi(state->homeX[i],state->awayX[i]);
              if(intersects(offsetpt, homePosi, (int)BOT_RADIUS*3))
              {
                if(i == botID)
                  continue;
                flag = true;
                offset /= 1.1;
              }
            }
            for(int i=0; i < 5; i++)
            {
                Vector2D<int> awayPosi(state->awayX[i], state->awayY[i]);
              if(intersects(offsetpt, awayPosi, (int)BOT_RADIUS*3))
              {
                flag = true;
                offset /= 1.1;
              }
            }
            //x3 = (ballPosX * (ballgoaldist + offset) - offset * OPP_GOAL_X) / ballgoaldist;
            //y3 = (ballPosY * (ballgoaldist + offset)) / ballgoaldist;
            if(!flag)
              break;
            if(offset <= 2.0)
            {
              offset = 0;
              //x3 = (ballPosX * (ballgoaldist + offset) - offset * OPP_GOAL_X) / ballgoaldist;
              //y3 = (ballPosY * (ballgoaldist + offset)) / ballgoaldist;
              break;
            }
          }*/

          int dist2 = Vector2D<int>::dist(offsetpt, homePos);
          if(dist2 < 200)
            hasAchievedOffset = 1;
          else if(dist > 2 * offset)
            hasAchievedOffset = 0;

          if(ballPos.x  < homePos.x)
            hasAchievedOffset = 0;

          Pose attackp(x3,y3,Vector2D<int>::angle(opp_goal,ballPos));

          if(hasAchievedOffset)
          {

            Pose attackpt(ballPos.x,ballPos.y,Vector2D<int>::angle(ballPos, homePos));
            return attackpt;
          }
          else
          {
              return attackp;
          }
        }
      }
}; // class TAttack

#endif // TTCharge_HPP
