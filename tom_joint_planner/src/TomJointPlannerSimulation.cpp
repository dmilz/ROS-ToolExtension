#include <tom_joint_planner/TomJointPlannerSimulation.h>
#include <functional>

namespace selfception
{

    //! Default static const  values pre-set
    const TomJointPlannerSimulation::SVector TomJointPlannerSimulation::DEFAULT_JOINT_NAMES {"r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_elbow_joint", "r_wrist_1_joint", "r_wrist_2_joint", "r_wrist_3_joint"};
    const TomJointPlannerSimulation::MovementFunction TomJointPlannerSimulation::DEFAULT_MOVEMENT_FUNCTION { nullptr };
    const double TomJointPlannerSimulation::DEFAULT_RATE = 50.0;
    const bool TomJointPlannerSimulation::DEFAULT_INTERPOLATION_STATUS = true;

    /** --- Constructor --- **/
    TomJointPlannerSimulation::TomJointPlannerSimulation(ros::NodeHandle nh)
        : nh_(nh), priv_nh_("~"), ok(false)
    {

        pub_joints_ = nh_.advertise<JointState>("/right_arm_joint_states", 1);
        sub_joints_ = nh_.subscribe<JointState>("/joint_states", 10, &TomJointPlannerSimulation::processJoints, this);
        ROS_INFO_STREAM("Planner started... ");
    }

    /** --- Destructor --- **/
    TomJointPlannerSimulation::~TomJointPlannerSimulation() {}


    void TomJointPlannerSimulation::setQInit(const JointState& qinit)
    {
        m_qInit=qinit;
        ROS_INFO_STREAM("Initial value set up.");
    }


    void TomJointPlannerSimulation::setQHome(const JointState& qhome)
    {
        m_qHome=qhome;
        ROS_INFO_STREAM("Home value set up.");
    }

    void TomJointPlannerSimulation::setQDesired(const JointState& qdesired)
    {
        m_qDesired=qdesired;
        ROS_INFO_STREAM("Desired value set up.");
    }

    void TomJointPlannerSimulation::processJoints(const sensor_msgs::JointStateConstPtr& js)
    {
       m_qCurrent = JointState(*js);
       ok = true;
    }

    bool TomJointPlannerSimulation::start(const SVector& joint_names, const MovementFunction movement_function /* = nullptr */,
                                          const double rate_ /* = 50.0 */, const bool interpolate /* = true */)
    {
        /** --- Create JointState message --- **/
        sensor_msgs::JointState djs; // desired joint state
        djs.header.seq = 1;
        djs.header.stamp = ros::Time::now();
        djs.header.frame_id = "";

        if(!joint_names.empty()) { // if joint names were given
            for(auto elm : joint_names)
                djs.name.push_back(elm);
        } else { // if no joint names were given, use default one
            djs.name.push_back("r_shoulder_pan_joint");
            djs.name.push_back("r_shoulder_lift_joint");
            djs.name.push_back("r_elbow_joint");
            djs.name.push_back("r_wrist_1_joint");
            djs.name.push_back("r_wrist_2_joint");
            djs.name.push_back("r_wrist_3_joint");
        }



        DVector sift;
        const double sstep = M_PI/(2.0*djs.name.size());
        double s = 0;
        for (uint i = 0; i < djs.name.size(); ++i, s += sstep) {
            djs.position.push_back(0);
            djs.velocity.push_back(0);
            djs.effort.push_back(0);
            sift.push_back(s);
        }


        // initialize current starting state
        ros::spinOnce();
        while (!ok && ros::ok()) {
            ROS_WARN("Waiting for incoming Joints data");
            ros::spinOnce();
            ros::Duration(0.5).sleep();
        }

        if(ok)
            setQInit(m_qCurrent);

        // set desired state
        setQDesired(djs);
        ros::Time begin = ros::Time::now();
        double t = 0;
        const double A = M_PI/2.0, w = 0.2;

        ros::Rate rate(rate_);
        while (ros::ok() && ok) {
            t = (ros::Time::now() - begin).toSec();
            for (uint i = 0; i < sift.size(); ++i)
                djs.position[i] = A*sin(w*t+sift[i]);

            pub_joints_.publish(djs);
            ros::spinOnce();
            rate.sleep();
        }
        return true;
    }

    bool TomJointPlannerSimulation::stop()
    {
        ok = false;
        return true;
    }

} // end namespace


int main(int argc, char** argv)
{
    ros::init(argc, argv, "tom_joint_planner_simulation");
    ros::NodeHandle nh;
    selfception::TomJointPlannerSimulation node(nh);


    selfception::TomJointPlannerSimulation::SVector joint_names;
    joint_names.push_back("r_shoulder_pan_joint");
    joint_names.push_back("r_shoulder_lift_joint");
    joint_names.push_back("r_elbow_joint");
    joint_names.push_back("r_wrist_1_joint");
    joint_names.push_back("r_wrist_2_joint");
    joint_names.push_back("r_wrist_3_joint");

    /*
    joint_names.push_back("l_shoulder_pan_joint");
    joint_names.push_back("l_shoulder_lift_joint");
    joint_names.push_back("l_elbow_joint");
    joint_names.push_back("l_wrist_1_joint");
    joint_names.push_back("l_wrist_2_joint");
    joint_names.push_back("l_wrist_3_joint");
    */

    node.start(joint_names, nullptr, 50.0, true);
    ros::shutdown();
    return 0;
}
