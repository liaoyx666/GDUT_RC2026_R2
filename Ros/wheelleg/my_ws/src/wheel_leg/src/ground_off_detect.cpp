#include "wheel_leg/ground_off_detect.hpp"


GroundOffDetect::GroundOffDetect()
{
}

GroundOffDetect::~GroundOffDetect()
{
}

double GroundOffDetect::calc_Fn()
{
    return 
        F * cos(theta) + 
        mw * (
                g + 
                zM_a - 
                L0_a * cos(theta) + 
                2.0 * L0_v * theta_v * sin(theta) +
                L0 * theta_a * sin(theta) +
                L0 * theta_v * theta_v * cos(theta)
            );
}




 double GroundOffDetect::calc_F(double ag, double tq)
 {
    double a = ag + M_PI / 4.0;

    double l0 = sqrt(l4 * l4 + l2 * l2 - 2.0 * l4 * l2 * cos(a));
    double ag4 = acos((l2 * l2 + l0 * l0 - l4 * l4) / (2.0 * l2 * l0));
    double ag0 = acos((l3 * l3 + l5 * l5 - l0 * l0) / (2.0 * l3 * l5));
    double ag3 = acos((l0 * l0 + l5 * l5 - l3 * l3) / (2.0 * l0 * l5));
    double ag6 = M_PI - ag3 - ag4;
    double l6 = sqrt(l1 * l1 + l2 * l2 - 2.0 * l1 * l2 * cos(ag6));
    double ag2 = acos((l1 * l1 + l6 * l6 - l2 * l2) / (2.0 * l1 * l6));
    double ag1 = acos((l6 * l6 + l2 * l2 - l1 * l1) / (2.0 * l6 * l2));

    double af1 = ag0 + ag2;
    double ag20 = M_PI - ag0 - ag2;


    double L0 = sin(ag0) * (l1 + l5) / sin(ag20);

    double L2 = sqrt(L0 * L0 + l1 * l1 - 2.0 * L0 * l1 * cos(ag2));

    double af2 = acos((L0 * L0 + L2 * L2 - l1 * l1) / (2.0 * L0 * L2));

    double a1 = ag1 - af2;


    return (((tq / l2) / sin(a1)) / sin(af1)) * sin(M_PI - af1 - af2);
 }