#ifndef _LEGCONTROL_H
#define _LEGCONTROL_H

namespace Leg {
    #define PI 3.14159265358979323846
    class Pid{
    private:
        double kp=0;
        double ki=0;
        double kd=0;
        double pre_error;
        double integral;
    public:
        Pid(double p, double i, double d): kp(p), ki(i), kd(d), pre_error(0), integral(0) {}
        double compute(double target, double current, double dt) {
            double error = target - current;
            integral += error * dt;
            double derivative = (error - pre_error) / dt;
            pre_error = error;
            return kp * error + ki * integral + kd * derivative;
        }
    };

    class LegControl:public Pid {
    private:
        const double L1 = 0.282; //ED
        const double L2 = 0.2856;//AD
        const double L3 = 0.29281;//BC
        const double L4 = 0.13067;//AB
        const double L23 = 0.5702;//DC
        double angle_out1;
        double angle_out2;
        double x =0;
        double belta;
        double output;
    public:
        double y =-0.3;
        LegControl(double kp,double ki,double kd):Pid(kp, ki, kd) {}
        ~LegControl(){};
        double Leg_Control(double now_angle,double target_L,double dt);
        void State_INFO();
        void Forward_kinematics(double alpha);//正解
        void Inverse_kinematics(double X,double Y);//逆解
    };

}

#endif