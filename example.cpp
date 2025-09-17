#include "speed_plan.h"
#include "stdlib.h"
int  main() 
{
    float path_targets[] = {50.0f, 30.0f, 80.0f, 0.0f};
    int path_count = sizeof(path_targets) / sizeof(path_targets[0]);

    TrapePlanner planner(5.0f, 5.0f, 10.0f, 0.1f);
    SPlanner planner1(10.0f, 10.0f, 10.0f, 10.0f, 0.01f);
    PolynomialPlanner planner2(10.0f, 10.0f, 10.0f, 0.01f);


    planner.SetMotionParams(0.0f, path_targets[1], 0.0f, 0.0f);
    planner1.SetMotionParams(0.0f, 10.0f, 0.0f, 0.0f);
    planner2.SetMotionParams(0.0f, 10.0f, 0.0f, 0.0f);

     while (!planner.GetArrivedFlag())
    {

    float vel = planner.Plan(0.0f,0.0f);
    float vel1 = planner1.Plan(0.0f,0.0f);
    float vel2 = planner2.Plan(0.0f,0.0f);  

    }




    return 0;
}