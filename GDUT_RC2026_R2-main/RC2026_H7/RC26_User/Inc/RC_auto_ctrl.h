#pragma once

#ifdef __cplusplus
namespace ctrl
{
	/*A1:一区， A3:三区*/
	enum AutoState
	{
		AUTO_START_A1 = 0,
		AUTO_GET_WEAPON,
		AUTO_DOCK,
		AUTO_PASS_MF_GET_KFS,
		AUTO_PUT_KFS_2L,
		AUTO_COMBINE,
		AUTO_PUT_KFS_3L,
		AUTO_UNCOMBINE,
		
	};
	
	
	class AutoCtrl
    {
    public:
		AutoCtrl();
		~AutoCtrl() = default;
		
    protected:
		
    private:
		
    };
}
#endif

