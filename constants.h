#ifndef CONSTANTS_H
#define CONSTANTS_H

const int GOAL_DEPTH                   = 300;

const int CENTER_X                     = 0;

const int CENTER_Y                     = 0;

const int HALF_FIELD_MAXX              = 3025; //actual 225 (rugged surface at end)

const int HALF_FIELD_MAXY              = 2050;


const int OUR_GOAL_MAXY                = 600;

const int OUR_GOAL_MINY                = -600;

const int OPP_GOAL_MAXY                = 600;

const int OPP_GOAL_MINY                = -600;

const int OUR_GOAL_WIDTH               = OUR_GOAL_MAXY - OUR_GOAL_MINY;

const int OPP_GOAL_WIDTH               = OPP_GOAL_MAXY - OPP_GOAL_MINY;

const int CENTER_CIRCLE_DIAMETER       = 1000;

const int DBOX_WIDTH                   = 1200;       //Along X direction

const int DBOX_HEIGHT                  = 855;     //Along Y direction(half height in each y direction)

const int DBOX_DEPTH                   = 10;

const int BALL_AT_CORNER_THRESH        = 20;

/* Bot Parameteres configuration */

              // mm

const float BOT_RADIUS                 = 150;                       // mm

const float BALL_RADIUS                = 40;                       // mm

const float SAFE_RADIUS                = (BOT_RADIUS * 2);

const float COLLISION_DIST             = (BOT_RADIUS * 7);

const int DRIBBLER_BALL_THRESH         =  500;            // mm


const int BOT_BALL_THRESH              = 150;                  // mm

const int BOT_BALL_THRESH_FOR_PR       = 200;                  // mm

//const int BOT_POINT_THRESH             = 147;                     // mm



// Parameters useful for camera's data transformation.

const int OUR_GOAL_Y = 500;

const int OPP_GOAL_Y = 0;



#endif // CONSTANTS_H
