#include "collision-checking.h"
#include <gsl/gsl_poly.h>
/* got this idea from
 * https://www.particleincell.com/2013/cubic-line-intersection/
 */
namespace CollisionChecking {
std::vector<double> cubic_LineSegmentIntersection(const double cX[4], const double cY[4], double u_low,
    double u_high, const LineSegment &ls) {
    // create line given by the linesegment
    // A*x + B*y + C = 0
    double A = ls.y2 - ls.y1;
    double B = ls.x1 - ls.x2;
    double C = ls.x1*(ls.y1-ls.y2) + ls.y1*(ls.x2-ls.x1);

    // the final equation to solve is given by the replacing x
    // and y in the above linear equation with:
    // x(u) = a0 + a1*u + a2*u^2 + a3*u^3;
    // and y(u) similarly.

    // coeffs for final cubic
    double c[4];
    c[3] = A * cX[3] + B * cY[3];
    c[2] = A * cX[2] + B * cY[2];
    c[1] = A * cX[1] + B * cY[1];
    c[0] = A * cX[0] + B * cY[0] + C;
    assert(c[3] != 0); // ensure its still cubic
    // solve cubic
    double u[3];
    int roots = gsl_poly_solve_cubic(c[2]/c[3], c[1]/c[3], c[0]/c[3], u, u+1, u+2);

    vector<double> u_result;
    for (int i = 0; i < roots; i++) {
        // check if within line segment
        double x = cX[0] + cX[1]*u[i] + cX[2]*u[i]*u[i] + cX[3]*u[i]*u[i]*u[i];
        double y = cY[0] + cY[1]*u[i] + cY[2]*u[i]*u[i] + cY[3]*u[i]*u[i]*u[i];
        double s;
        if (ls.x2-ls.x1 != 0) // not vertical
            s = (x-ls.x1)/(ls.x2-ls.x1);
        else
            s = (y-ls.y1)/(ls.y2-ls.y1);
        if (s < 0 || s > 1.0)
            continue;
        // check if within u range
        if (u[i] >= u_low && u[i] <= u_high)
            u_result.push_back(u[i]);
    }
    return u_result;
}

std::vector<Pose> cubicSpline_LineSegmentIntersection(const CubicSpline &s, const LineSegment &ls) {
    // unpack the x and y splines
    using namespace alglib;
    real_2d_array tbly, tblx;
    ae_int_t ny, nx;
    alglib::spline1dunpack(s.getSplineX(), nx, tblx);
    alglib::spline1dunpack(s.getSplineY(), ny, tbly);
    // both should have same number of pieces
    assert(nx == ny);
    int n = nx;
    vector<Pose> results;
    for (int i = 0; i < n-1; i++) {
        // confirm that u intervals are same for both x and y splines
        assert(tbly[i][0] == tblx[i][0] && tbly[i][1] == tblx[i][1]);
        double u_low = tbly[i][0], u_high = tbly[i][1];
        double cX[4], cY[4];
        for (int j = 0; j < 4; j++) {
            cX[j] = tblx[i][j+2];
            cY[j] = tbly[i][j+2];
        }
        vector<double> u = cubic_LineSegmentIntersection(cX, cY, u_low, u_high, ls);
        for (int i = 0; i < u.size(); i++) {
            double x = cX[0] + cX[1]*u[i] + cX[2]*u[i]*u[i] + cX[3]*u[i]*u[i]*u[i];
            double y = cY[0] + cY[1]*u[i] + cY[2]*u[i]*u[i] + cY[3]*u[i]*u[i]*u[i];
            results.push_back(Pose(x*fieldXConvert, y*fieldXConvert, 0));
        }
    }
    return results;
}
}
