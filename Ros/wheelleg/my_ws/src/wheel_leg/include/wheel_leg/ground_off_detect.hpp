#pragma once
#include <cmath>


class GroundOffDetect
{
private:
    double L0 = 0;
    double L0_v = 0;
    double L0_a = 0;

    double last_L0 = 0;
    double last_L0_v = 0;

    double theta = 0;
    double theta_v = 0;
    double theta_a = 0;

    double last_theta_v = 0;

    double F;
    

    double mw = 0.3470 / 2.0;// 轮子质量

    double g = 9.8;

    double zM_a = 0;








    double l1 = 0.282;
    double l2 = 0.2856;
    double l3 = 0.2928;
    double l4 = 0.13067;
    double l5 = 0.05702;





   

public:
    GroundOffDetect();
    virtual ~GroundOffDetect();
   
    void paramsUpdate(
        double L0_, 
        double theta_, double theta_v_, 
        double F_,
        double zM_a_
    ) {
        L0 = L0_;
        L0_v = (L0 - last_L0);
        L0_a = (L0_v - last_L0_v);
        last_L0 = L0;
        last_L0_v = L0_v;

        theta = theta_;
        theta_v = theta_v_;
        theta_a = (theta_v - last_theta_v);
        last_theta_v = theta_v;

        F = F_;

        zM_a = zM_a_;
    }

    double calc_F(double ag, double tq);
    double calc_Fn();
};

