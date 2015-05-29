#ifndef TDEFEND_HPP
#define TDEFEND_HPP

///TO BE CHANGED

#include <list>
#include "beliefstate.h"
#include "geometry.h"
#include "pose.h"
#include "constants.h"
#define ANGLE_TO_DIST 0


class TDefend
{
  static const int offset = 400;
  // Corner Case: wall, ball, bot in line
  Point2D<int> prevBotPos;
  float prevBotAngle;

  public:
    TDefend()
    {
      iState = APPROACHING;
    } // TDefend

    ~TDefend()
    { } // ~TDefend

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

    Pose executeattack(BeliefState *state, int botID)
    {
        Vector2D<int> ballPos(state->ballX,state->ballY);
        Vector2D<int> homePos(state->homeX[botID], state->homeY[botID]);
        Vector2D<int> opp_goal(HALF_FIELD_MAXX, 0);
        static Vector2D<float> lastVel[10];
                    static int index = 0;
                    if(index < 10)
                    {
                        lastVel[index].x = state->ballVx;
                        lastVel[index].y = state->ballVy;
                        index = (index + 1) % 10;
                    }
                    Vector2D<float> avgBallVel(0.0,0.0);
                    for(int i=0;i<10;i++)
                    {
                        avgBallVel.x += lastVel[i].x;
                        avgBallVel.y += lastVel[i].y;
                    }
                    avgBallVel.x /= 10.0;
                    avgBallVel.y /= 10.0;


                    float dist = Vector2D<int>::dist(ballPos, homePos);
          /* Ball is not with bot so go to ball first */
          //sParam.GoToPointP.align = true;
          float ballgoaldist = Vector2D<int>::dist(ballPos, opp_goal);
          float factor = (int)Vector2D<int>::dist(ballPos,homePos);
          factor /= 5*MAX_BOT_SPEED;
          factor =0;
          //int ballPosX = state->ballPos.x;// + factor * state->ballVel.x;
          //int ballPosY = state->ballPos.y;// + factor * state->ballVel.y;
          
          float offset = TDefend::offset;//TAttack::offset * state->ballVel.abs()/6000.0+
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
        
          }

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

            return Pose(ballPos.x,ballPos.y,Vector2D<int>::angle(ballPos, homePos));
          }
          else
          {
              return attackp;
          }
    }

    Pose execute(BeliefState *state,int botID)
    {
        Vector2D<int> ballPos(state->ballX,state->ballY);
        Vector2D<int> homePos(state->homeX[botID], state->homeY[botID]);
        Vector2D<int> opp_goal(HALF_FIELD_MAXX, 0);
        ////needs to be corrected opp_pos
        //Vector2D<int> opp_pos(state->awayX[state->oppBotNearestToBall],state->awayY[state->oppBotNearestToBall]);
        float ang1,temp1;
        float dist2=100000000;
        int i;
        for(i=0;i<5;i++)
        {
            Vector2D <int> awayPos(state->awayX[i], state->awayY[i]);
          temp1=Vector2D<int>::dist(ballPos,awayPos);
          if(dist2>temp1)dist2=temp1;

        }

        float dist = Vector2D<int>::dist(ballPos, homePos);
        //float dist2= Vector2D<int>::dist(ballPos,opp_pos);

        /////start changes from here //////
        if(ballPos.x>0)
          {
            if(ballPos.x>HALF_FIELD_MAXX-DBOX_WIDTH)
            {
              return Pose(-0.4*(HALF_FIELD_MAXX-DBOX_WIDTH),ballPos.y,Vector2D<int>::angle(opp_goal, ballPos));
            }
            else if( abs(state->ballVx) > 50 && state->ballVx<0 )
            {
                  ang1 = atan(state->ballVy/state->ballVx);
                  temp1=ballPos.y -((ballPos.x)-(-HALF_FIELD_MAXX + DBOX_WIDTH))*tan(ang1);
                  if(temp1>HALF_FIELD_MAXY)temp1 =  HALF_FIELD_MAXY;
                  if(temp1<-HALF_FIELD_MAXY)temp1=-HALF_FIELD_MAXY;
                  return Pose(-0.7*(HALF_FIELD_MAXX-DBOX_WIDTH),temp1,0);
            }
            else
            {

                return Pose((0.3*(ballPos.x +(HALF_FIELD_MAXX-DBOX_WIDTH ))-(HALF_FIELD_MAXX-DBOX_WIDTH)),ballPos.y,0);
            }

          }
          else
          {

             if(abs (dist-dist2)<2*BOT_BALL_THRESH &&  ballPos.x > -HALF_FIELD_MAXX+1.5*GOAL_DEPTH )
             {
                 ///ATTACK FUNCTION
                return  executeattack(state, botID);
                qDebug()<<"1 is called \n";
                //return Pose(ballPos.x,ballPos.y,0);

             }
            // else return  Pose(HALF_FIELD_MAXX,ballPos.y,0);

             if (ballPos.x < homePos.x &&  ballPos.x >-HALF_FIELD_MAXX+1.5*GOAL_DEPTH )
             {
              ////ATTACK THE BALL
               return  executeattack(state, botID);
                //return Pose(ballPos.x,ballPos.y,0);

             }
             if(ballPos.x > homePos.x &&  ballPos.x >-HALF_FIELD_MAXX+1.5*GOAL_DEPTH )
             {
                   if(state->ballVx<50)
                   {

                       //Working properly
                     return Pose(ballPos.x,ballPos.y,0);

                   }

                    if(ballPos.x > -0.7*(HALF_FIELD_MAXX-DBOX_WIDTH))
                      {
                        if(state->ballVx > 100){
                           ang1 = atan(state->ballVy/state->ballVx);
                            temp1=ballPos.y -((ballPos.x)-(-HALF_FIELD_MAXX + DBOX_WIDTH))*tan(ang1);
                            if(temp1>HALF_FIELD_MAXY || temp1 < -HALF_FIELD_MAXY)
                            temp1 =  ballPos.y;
                        }
                        else{
                            temp1 = ballPos.y;
                        }

                             ///CAN BE CHANGED TO HALFFIELDMAXX
                            qDebug() <<"3 is called \n";

                            return Pose(-0.7*(HALF_FIELD_MAXX-DBOX_WIDTH),temp1,0);
                        }
                        else
                          {
                             qDebug()<<"4 is called \n";
                            //return  Pose(ballPos.x,ballPos.y,0);

                               return  executeattack(state, botID);
                              ////Attack the ball
                          }
                }
             if(ballPos.x <-HALF_FIELD_MAXX+1.5*GOAL_DEPTH && abs(ballPos.y)>OUR_GOAL_MAXY + 1.8*BOT_RADIUS)
             {
             qDebug()<<"5 is called \n";
                //return Pose(ballPos.x,ballPos.y,0);

               return  executeattack(state, botID);
              ///ATTACK  THE BALL
             }

          }

        return Pose(ballPos.x, ballPos.y, 0);

    }
}; // class TDefend

#endif // TDEFEND_HPP
