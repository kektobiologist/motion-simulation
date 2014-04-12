#include "beliefstate.h"

BeliefState::BeliefState()
{
    ballX = ballY = 0;
    ballIsPresent = false;
    for(int i =0 ;i < 5; i++) {
        homeX[i] = homeY[i] = homeTheta[i] = 0;
        awayX[i] = awayY[i] = awayTheta[i] = 0;
        homeIsPresent[i] = false;
        awayIsPresent[i] = false;
    }
}
