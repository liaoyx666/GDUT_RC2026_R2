#pragma once


#ifdef __cplusplus


namespace path_plan {
    class PathPlan_Base {
    protected:
        float x_start;
        float y_start;
        float x_end;
        float y_end;
        float x_distance;//x方向距离
        float y_distance;//y方向距离
        float x_position;//x当前位置
        float y_position;//y当前位置
    public:
        PathPlan_Base() = default;
        virtual ~PathPlan_Base() = default;
        virtual void plan_path() = 0;
    };

    class LinePath_plan : public PathPlan_Base {
    protected:
        
    public:
        LinePath_plan(float x_start, float y_start, float x_end, float y_end){
            this->x_start = x_start;
            this->y_start = y_start;
            this->x_end = x_end;
            this->y_end = y_end;
            this->x_distance = x_end - x_start;
            this->y_distance = y_end - y_start;
            this->x_position = x_start;
            this->y_position = y_start;
        }
        
        ~LinePath_plan() override = default;
        void plan_path() override;
    };
    
}


#endif 

