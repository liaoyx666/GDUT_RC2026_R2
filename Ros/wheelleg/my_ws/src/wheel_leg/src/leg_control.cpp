#include "ros/ros.h"
#include "wheel_leg/leg_control.hpp"
#include <cmath>

namespace Leg {
  void LegControl::Forward_kinematics(double alpha) {
      // 正解代码实现
      double angle_BAD = PI/4 + alpha;
      double L_BD = sqrt(L2*L2 + L4*L4 -2*L2*L4*cos(angle_BAD));
      double middle_val1 = (L_BD*L_BD + L2*L2 - L4*L4) / (2*L_BD*L2);
      double middle_val2 = (L_BD*L_BD + L23*L23 - L3*L3) / (2*L_BD*L23);
      belta = PI-alpha - acos(middle_val1) - acos(middle_val2);
      if(belta > PI/2){
          belta = PI - belta;
      }
      x = L2*cos(alpha) - L1*cos(belta);
      y = -L2*sin(alpha) - L1*sin(belta);
  }

  void LegControl::Inverse_kinematics(double X,double Y) {
      // 逆解代码实现
      double phi = atan(-Y/X);
      double R = sqrt(X*X +Y*Y);
      double middle_val3 = 2*L2*R;
      double middle_val4 = L2*L2 -L1*L1 + R*R;
      double acos_arg = middle_val4 / middle_val3;
      angle_out1 = -phi - acos(acos_arg)+0.58;
      angle_out2 = -phi + acos(acos_arg)+0.58;
  }

  double LegControl::Leg_Control(double now_angle,double target_L,double dt) {
      Forward_kinematics(now_angle);//获取当前足端位置x,y
      LegControl::LegControl::Inverse_kinematics(x,y);//获取当前关节角度angle_out1,angle_out2
      output = -compute(target_L,y, dt);//PID计算
      return output;
  }

  void LegControl::State_INFO(){
      ROS_INFO("x: %f , y: %f , belta: %f\n"
        " angle_out1: %f , angle_out2: %f\n"
        "output:%.4f" ,x, y, 
        belta, angle_out1, angle_out2,output);
  }
}
