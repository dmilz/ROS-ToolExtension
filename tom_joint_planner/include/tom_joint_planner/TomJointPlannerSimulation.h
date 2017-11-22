#ifndef SELFCEPTION_TOM_JOINT_PLANNER_SIMULATION_H
#define SELFCEPTION_TOM_JOINT_PLANNER_SIMULATION_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <functional>

using namespace std;

namespace selfception
{

    // A bundle of predefined movements as a lambda expression
    namespace movements
    {
        typedef std::vector<double> DVector;
        typedef std::function<const DVector(const double&, const unsigned int)>  MovementFunction;

        /** Bundle of different movement functions **/

        // Simple sin movement of all joints
        static const MovementFunction sine_movement = [](const double& time, const unsigned int size) -> const DVector {
            static const double A = M_PI/2.0, w = 0.2;
            const double delta_s = M_PI/(2.0*size);
            double s = 0;
            DVector djs = DVector(size, 0.0);
            for (unsigned int i = 0; i < size; ++i, s+= delta_s)
                djs[i] = A*sin(w*time+s);

            return djs;
        };

        // Minimal (safe) sin movement that avoids colliding with the robot
        static const MovementFunction minimal_sine_movement = [](const double& time, const unsigned int size) -> const DVector {
            static const double A = M_PI/4.0, w = 0.2;

//#if (__GNUC__ > 3) || (__GNUC__ == 2 && __GNUC_MINOR__ > 5) /* Test for GCC > 2.5 */
//            double middle[size] = { [0 ... size] = 0 }; // Only supported in gcc version >2.5
//#else
             double middle[size];
             for(int ii = 0; ii < size; middle[ii] = 0, ++ii) {} // Empty loop, logic is inside (..)
//#endif

             DVector djs = DVector(size, 0.0);
             for (unsigned int i = 0; i < size; ++i)
                 djs[i] = middle[i]+A*sin(w*time);

             return djs;
        };

    }


    class TomJointPlannerSimulation
    {
    public:
        typedef std::vector<std::string> SVector;
        typedef std::vector<double> DVector;
        typedef movements::MovementFunction MovementFunction;
        typedef sensor_msgs::JointState JointState;

    private:

        //! The node handle
        ros::NodeHandle nh_;
        //! Node handle in the private namespace
        ros::NodeHandle priv_nh_;

        //! Data checking
        bool ok;

        //! Publisher for joints
        ros::Publisher pub_joints_;
        //! subs to joints
        ros::Subscriber sub_joints_;

        //! Joint states
        JointState m_qHome;
        JointState m_qInit;
        JointState m_qDesired;
        JointState m_qCurrent;

        //! Default values pre-set
        static const SVector DEFAULT_JOINT_NAMES;// {"r_shoulder_pan_joint", "r_shoulder_lift_joint", "r_elbow_joint", "r_wrist_1_joint", "r_wrist_2_joint", "r_wrist_3_joint"};
        static const MovementFunction DEFAULT_MOVEMENT_FUNCTION;// { nullptr };
        static const double DEFAULT_RATE;// = 50.0;
        static const bool DEFAULT_INTERPOLATION_STATUS;// = true;

    public:
        TomJointPlannerSimulation(ros::NodeHandle nh);
        virtual ~TomJointPlannerSimulation();

        void setQInit   (const JointState&        qinit);
        void setQHome   (const JointState&        qhome);
        void setQDesired(const JointState&        qdesired);

        bool start      (const SVector&           joint_names        = TomJointPlannerSimulation::DEFAULT_JOINT_NAMES,
                         const MovementFunction   movement_function   = TomJointPlannerSimulation::DEFAULT_MOVEMENT_FUNCTION,
                         const double             rate_               = TomJointPlannerSimulation::DEFAULT_RATE,
                         const bool               interpolate         = TomJointPlannerSimulation::DEFAULT_INTERPOLATION_STATUS
        ); // bool start(...)

        bool stop();

    private:
        bool init();
        void processJoints(const sensor_msgs::JointStateConstPtr& js);
    };


}



#endif // SELFCEPTION_TOM_JOINT_PLANNER_SIMULATION_H

