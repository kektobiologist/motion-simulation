#ifndef CONSTANTS_H
#define CONSTANTS_H

namespace Constants {
const double FINAL_VEL = 0;
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
static const double d          = 6.5; //distance between wheels in cm
static const double ticksToCmS = 1.107; //still only approximate... v = v_ticks * ticksToCmS
static const double fieldXConvert = 23.79; // now im always using xconvert as standard conversion from strategy -> cm and vice versa.
static const double fieldYConvert = 22.02;
// NOTE(arpit): Uncertainties should be non-zero when simulating. Currently 0 since bot data is fetched from vision.
static const double xUncertainty = 0;//0.5; // Uncertainty is in %age of max value. eg. 1% where fabs(x) <= 1000 means fabs(error) <= 10
static const double yUncertainty = 0;//0.5;
static const double thetaUncertainty = 0;//3;
// NOTE(arpit): numPacketDelay and update() specified here is only used in simulation.
static const int numPacketDelay = 0; // num of packets to delay in update

static const double vwmax      = 200; // cm/s^2
static const double vsat       = ticksToCmS*100.; // cm/s
static const double atmax      = 200*ticksToCmS; // cm/s^2, need to measure this, need to take inertia into account
static const double awmax      = 1000; // 1/s^2, no idea how to measure this, need to take inertia into account
}


#endif // CONSTANTS_H
