#ifndef BELIEFSTATE_H
#define BELIEFSTATE_H

class BeliefState
{
public:
    BeliefState();
    double ballX, ballY; // everything in strategy coordinates
    double homeX[5], homeY[5], homeTheta[5];
    double awayX[5], awayY[5], awayTheta[5];
    bool homeIsPresent[5], awayIsPresent[5], ballIsPresent;

    // vision calculated velocities
    double homeVl[5], homeVr[5];
    double awayVl[5], awayVr[5];
    // ball velocity in strategy coordinates per second.
    double ballVx, ballVy;
};

#endif // BELIEFSTATE_H
