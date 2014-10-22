#ifndef VISIONVELOCITY_HPP
#define VISIONVELOCITY_HPP
// For calculating bot, ball velocity (in ticks) from vision data
// useful for:
// * measuring actuation + transmission delay of velocity signal
// * measuring disparity in target and actual velocity at some point of time.
// *
namespace VisionVelocity {
struct BotPose {
    int x, y;
    float theta;
};

void calcBotVelocity(BotPose p1, BotPose p2, float timeMs, float &vl, float &vr);
}
#endif // VISIONVELOCITY_HPP
