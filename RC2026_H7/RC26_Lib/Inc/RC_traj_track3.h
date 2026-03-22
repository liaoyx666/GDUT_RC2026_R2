#pragma once
#include "RC_path3.h"
#include "RC_nonlinear_pid.h"

#ifdef __cplusplus
namespace path
{
	class TrajTrack
    {
    public:
		TrajTrack();
		virtual ~TrajTrack() {}
		
		
    protected:
		
    private:
		Path3* path;
		pid::NonlinearPid tan_pid;
		pid::NonlinearPid nor_pid;
	
		
    };
}
#endif
