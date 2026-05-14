#include "RC_arm_path.h"
#include "RC_arm.h"

namespace path
{

// =============================================
//            TimePlanner（速度规划）
// =============================================

TimePlanner::Profile TimePlanner::compute(float L, float v_max, float a_max)
{
    Profile pf{};
    pf.L = L; pf.v_max = v_max; pf.a_max = a_max;

    float t_acc = v_max / a_max;
    float d_acc = 0.5f * a_max * t_acc * t_acc;

    if (2*d_acc >= L)
    {
        t_acc = sqrtf(L / a_max);
        pf.t_acc = t_acc;
        pf.t_cruise = 0;
        pf.t_total = 2*t_acc;
    }
    else
    {
        float d_cruise = L - 2*d_acc;
        float t_cruise = d_cruise / v_max;

        pf.t_acc = t_acc;
        pf.t_cruise = t_cruise;
        pf.t_total = 2*t_acc + t_cruise;
    }
    return pf;
}

float TimePlanner::timeToS(const Profile& pf, float t)
{
    float a = pf.a_max;
    float v = pf.v_max;
    float L = pf.L;

    if (t <= 0) return 0;
    if (t >= pf.t_total) return 1;

    if (t < pf.t_acc)
    {
        float d = 0.5f*a*t*t;
        return d / L;
    }

    if (t < pf.t_acc + pf.t_cruise)
    {
        float d_acc = 0.5f*a*pf.t_acc*pf.t_acc;
        float d = d_acc + v*(t - pf.t_acc);
        return d / L;
    }

    float td = t - pf.t_acc - pf.t_cruise;
    float d_acc = 0.5f*a*pf.t_acc*pf.t_acc;
    float d_cruise = v*pf.t_cruise;
    float d_dec = v*td - 0.5f*a*td*td;

    return (d_acc + d_cruise + d_dec) / L;
}




// =============================================
//             CartesianPlanner
// =============================================

CartesianPlanner::CartesianPlanner()
{
    trajectoryLen_ = 0;
    segCount_ = 0;
    hasStart_ = false;
}

void CartesianPlanner::clear()
{
    trajectoryLen_ = 0;
    segCount_ = 0;
    hasStart_ = false;
}

// ----------------- Bezier sample -----------------
Pose3D CartesianPlanner::sampleBezier2(const Pose3D& p0,
                                       const Pose3D& p1,
                                       const Pose3D& p2,
                                       float t)
{
    float mt = 1-t;
    float mt2 = mt*mt;
    float t2 = t*t;

    Pose3D o;
    o.pos = Vector3D(
        mt2*p0.pos.x() + 2*mt*t*p1.pos.x() + t2*p2.pos.x(),
        mt2*p0.pos.y() + 2*mt*t*p1.pos.y() + t2*p2.pos.y(),
        mt2*p0.pos.z() + 2*mt*t*p1.pos.z() + t2*p2.pos.z()
    );
    o.pitch = mt2*p0.pitch + 2*mt*t*p1.pitch + t2*p2.pitch;
    return o;
}

// ----------------- addPoint -----------------
bool CartesianPlanner::addPoint(const Pose3D& p, float smooth, float v_max, float a_max)
{
    if (!hasStart_)
    {
        lastPose_ = p;
        hasStart_ = true;
        return true;
    }

    if (segCount_ >= PATH_CMD_QUEUE_MAX) return false;

    Segment& s = segs_[segCount_];

    s.p0 = lastPose_;
    s.p1 = p;
    s.smooth = smooth;
    s.v_max = v_max;
    s.a_max = a_max;

    if (smooth == 0)
    {
        s.bezier = false;
        s.length = (s.p1.pos - s.p0.pos).length();
    }
    else
    {
        s.bezier = true;
        s.ctrl.pos   = s.p0.pos   + (s.p1.pos - s.p0.pos)*smooth;
        s.ctrl.pitch = s.p0.pitch + (s.p1.pitch - s.p0.pitch)*smooth;
        s.length = estimateBezierLength(s.p0, s.ctrl, s.p1);
    }

    s.profile = TimePlanner::compute(s.length, v_max, a_max);

    segCount_++;
    lastPose_ = p;
    return true;
}

// 贝塞尔长度估计
float CartesianPlanner::estimateBezierLength(const Pose3D& a,
                                             const Pose3D& c,
                                             const Pose3D& b)
{
    float L = 0;
    Pose3D prev = sampleBezier2(a,c,b,0);
    for (int i=1;i<=20;i++)
    {
        float t = i*0.05f;
        Pose3D cur = sampleBezier2(a,c,b,t);
        L += (cur.pos - prev.pos).length();
        prev = cur;
    }
    return L;
}


// =============================================
//         build（按时间采样轨迹）
// =============================================
bool CartesianPlanner::build()
{
    float dt = 0.001f;
    trajectoryLen_ = 0;
		arm::ArmKinematics ik;
    for (int i=0; i<segCount_; i++)
    {
        Segment& s = segs_[i];

        float t = 0;
        while (t <= s.profile.t_total && trajectoryLen_ < ARM_PATH_MAX_TRAJ_POINTS)
        {
            float u = TimePlanner::timeToS(s.profile, t);

            Pose3D p;

            if (!s.bezier)
            {
                p.pos = Vector3D::lerp(s.p0.pos, s.p1.pos, u);
                p.pitch = s.p0.pitch + (s.p1.pitch - s.p0.pitch)*u;
            }
            else
            {
                p = sampleBezier2(s.p0, s.ctrl, s.p1, u);
            }

            // IK
           
            arm::EndEffectorPos tgt{p.pos.x(),p.pos.y(),p.pos.z(),p.pitch};
            arm::JointAngles sol;

            if (!ik.inverse(tgt, sol)) return false;

            trajectory_[trajectoryLen_++] =
                JointArray{sol.theta1, sol.theta2, sol.theta3, sol.theta4};

            t += dt;
        }
    }

    segCount_ = 0;
    hasStart_ = false;
    return true;
}


// =============================================
//             TrajectoryExecutor
// =============================================
TrajectoryExecutor::TrajectoryExecutor()
: planner_(nullptr), len_(0), currentIndex_(0), threshold_(0.2f)
{}
	
	
bool TrajectoryExecutor::reachedGoal(const JointArray& current, const JointArray& target, float threshold)
{
    for (int i = 0; i < ARM_JOINTS; i++)
    {
        // 计算绝对误差
        float error = fabsf(target.j[i] - current.j[i]);
        
        // 如果任何一个关节的误差超过阈值，则认为未到达
        if (error > threshold)
        {
            return false;
        }
    }
    // 所有关节都在阈值内
    return true;
}


void TrajectoryExecutor::bind(CartesianPlanner* p)
{
    planner_ = p;
    len_ = p->trajectoryLen_;
    for (int i=0;i<len_;i++)
        active_[i] = p->trajectory_[i];
    currentIndex_ = 0;
}

void TrajectoryExecutor::switchToNew()
{
    bind(planner_);
}

void TrajectoryExecutor::reset()
{
    currentIndex_ = 0;
}


bool TrajectoryExecutor::run(const JointArray& cur)
{
    if (len_ == 0) return true;

    const JointArray& target = active_[currentIndex_];

    // 只有到达当前点，才允许前进
    if (reachedGoal(cur, target, threshold_))
    {
        if (currentIndex_ < len_ - 1)
        {
            currentIndex_++;
            return false;
        }
        else
        {
            return true;
        }
    }

    return false;
}


bool TrajectoryExecutor::hasTrajectory() const
{
    return len_ > 0;
}

const JointArray& TrajectoryExecutor::getCurrentCmd() const
{
    static JointArray last{};
    if (len_ == 0) return last;
    return active_[currentIndex_];
}

} // namespace path
