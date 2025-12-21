#pragma once

#ifdef __cplusplus
namespace data
{
	class RobotPose
    {
    public:
		RobotPose();
		virtual ~RobotPose() {}
			
		float x;
		float y;
		float z;
			
		float yaw;
		float roll;
		float pitch;
		
    protected:
    
    private:
    
    };
	













}
#endif
