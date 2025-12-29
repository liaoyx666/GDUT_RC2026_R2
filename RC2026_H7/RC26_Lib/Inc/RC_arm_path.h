#pragma once
#include <math.h>
#include "RC_vector3d.h"
#include "RC_tim.h"
#define ARM_JOINTS 4
#define ARM_PATH_MAX_TRAJ_POINTS 7000
#define PATH_CMD_QUEUE_MAX 64

#ifdef __cplusplus
namespace path
{

using Vector3D = vector3d::Vector3D;

struct Pose3D
{
    Vector3D pos;
    float pitch;
};

#define POS(x,y,z,p) Pose3D{Vector3D(x,y,z),(p)}

struct JointArray
{
    float j[ARM_JOINTS];
};

// =============================================
//        ★★ 新类：TimePlanner ★★
//   独立负责速度规划与时间参数化
// =============================================
class TimePlanner : public tim::TimHandler
{
public:
    struct Profile
    {
        float L;        // 长度
        float v_max;
        float a_max;
        float t_acc;
        float t_cruise;
        float t_total;
    };

    static Profile compute(float L, float v_max, float a_max);

		
		
    // 将 t 映射到 s∈[0,1]
    static float timeToS(const Profile& pf, float t);

			private:

};


// =============================================
//               PathCommand（原有）
// =============================================
enum PathType
{
    LINEAR = 0,
    BEZIER2 = 1
};

struct PathCommand
{
    PathType type;
    Pose3D p0;
    Pose3D p1;
    Pose3D p2;
};


// =============================================
//           ★★ CartesianPlanner ★★
// =============================================
class CartesianPlanner
{
public:
    CartesianPlanner();
    void clear();

    // 原有接口
    bool addLinear(const Pose3D& a, const Pose3D& b);
    bool addBezier2(const Pose3D& a, const Pose3D& ctrl, const Pose3D& b);

    // 新接口：用户只需要 addPoint
    bool addPoint(const Pose3D& p, float smooth, float v_max, float a_max);

    // dt = 0.001f (1KHz)
    bool build();

    Pose3D sampleBezier2(const Pose3D& p0, const Pose3D& p1, const Pose3D& p2, float t);

    JointArray trajectory_[ARM_PATH_MAX_TRAJ_POINTS];
    int trajectoryLen_;

    static bool reachedAll(const JointArray& a, const JointArray& b, float tol);

private:
    float shortestDiff(float a, float b);

    // 几何 + 时间标定段
    struct Segment
    {
        Pose3D p0, p1, ctrl;
        bool bezier;
        float smooth;

        float v_max, a_max;
        float length;

        TimePlanner::Profile profile;
    };

    Segment segs_[PATH_CMD_QUEUE_MAX];
    int segCount_;
    bool hasStart_;
    Pose3D lastPose_;

    float estimateBezierLength(const Pose3D& a, const Pose3D& c, const Pose3D& b);
};


// =============================================
//             TrajectoryExecutor
// =============================================
class TrajectoryExecutor
{
public:
    TrajectoryExecutor();

    void bind(CartesianPlanner* p);
    void switchToNew();
    void reset();

    bool run(const JointArray& cur);

    bool hasTrajectory() const;

    const JointArray& getCurrentCmd() const;
static bool reachedGoal(const JointArray& current, const JointArray& target, float threshold);
    int currentIndex_;

private:
    CartesianPlanner* planner_;
    JointArray active_[ARM_PATH_MAX_TRAJ_POINTS];
    int len_;
    float threshold_;
};

} // namespace path
#endif
