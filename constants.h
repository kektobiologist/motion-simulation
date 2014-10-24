#ifndef CONSTANTS_H
#define CONSTANTS_H

extern const int GOAL_DEPTH                   = 300;

extern const int CENTER_X                     = 0;

extern const int CENTER_Y                     = 0;

extern const int HALF_FIELD_MAXX              = 3025; //actual 225 (rugged surface at end)

extern const int HALF_FIELD_MAXY              = 2050;


extern const int OUR_GOAL_MAXY                = 600;

extern const int OUR_GOAL_MINY                = -600;

extern const int OPP_GOAL_MAXY                = 600;

extern const int OPP_GOAL_MINY                = -600;

extern const int OUR_GOAL_WIDTH               = OUR_GOAL_MAXY - OUR_GOAL_MINY;

extern const int OPP_GOAL_WIDTH               = OPP_GOAL_MAXY - OPP_GOAL_MINY;

extern const int CENTER_CIRCLE_DIAMETER       = 1000;

extern const int DBOX_WIDTH                   = 1200;       //Along X direction

extern const int DBOX_HEIGHT                  = 855;     //Along Y direction(half height in each y direction)

extern const int DBOX_DEPTH                   = 10;

extern const int BALL_AT_CORNER_THRESH        = 20;

/* Bot Parameteres configuration */

              // mm

extern const float BOT_RADIUS                 = 150;                       // mm

extern const float BALL_RADIUS                = 40;                       // mm

extern const float SAFE_RADIUS                = (BOT_RADIUS * 2);

extern const float COLLISION_DIST             = (BOT_RADIUS * 7);

extern const int DRIBBLER_BALL_THRESH         =  500;            // mm


extern const int BOT_BALL_THRESH              = 150;                  // mm

extern const int BOT_BALL_THRESH_FOR_PR       = 200;                  // mm

//extern const int BOT_POINT_THRESH             = 147;                     // mm



// Parameters useful for camera's data transformation.

extern const int OUR_GOAL_Y = 500;

extern const int OPP_GOAL_Y = 0;



#endif // CONSTANTS_H
