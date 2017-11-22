#ifndef TOOLEXTENSION_LWPR_NODE_H
#define TOOLEXTENSION_LWPR_NODE_H

#include "toolextension_lwpr_node.h"
#define LWPRLEARNER_VERBOSE
#include "learner/lwpr_learner.h"

namespace toolextension_lwpr
{

    // For printing vectors
    template<typename T>
    std::ostream &operator <<(std::ostream &os, const std::vector<T> &v)
    {
       using namespace std;
       os<<"[";
       copy(v.begin(), v.end()-1, ostream_iterator<T>(os, ","));
       os<<v.back()<<"]";
       return os;
    }


    typedef std::vector<double> Vector;
    /*enum    STATE
    {
        training = 0,
        predicting = 1,
        both = 2
    };*/
    struct  KinematicState
    {
        int state[9];
        Vector Q,       Q_dot;      ros::Time Q_time;
        Vector X_ee,    X_ee_dot;   ros::Time X_ee_time;
        Vector X_te,    X_te_dot;   ros::Time X_te_time;
        Vector J_ee,    J_te;       ros::Time J_ee_time, J_te_time;
        Vector R_ee;                ros::Time R_ee_time;
        void reset() { state[0] = (state[1] = (state[2] = (state[3] = (state[4] = (state[5] = (state[6] =(state[7] = (state[8] = 0)))))))); }
        bool valid() { bool b; for(int i = 0; i < 9; i++){ b = b && (state[i] > 0);}; return b;}
        KinematicState() { X_ee={0,0,0}; X_ee_dot={0,0,0}; X_te={0,0,0}; X_te_dot={0,0,0}; Q={0,0,0,0,0}; Q_dot={0,0,0,0,0}; R_ee={0,0,0,0}; }
    };


    class Node
    {
    public:
        Node                    (const ros::NodeHandle &n, std::string configName);
        ~Node                   ();

        // Callback functions
        void    cb_Qin          (const sensor_msgs::JointState::ConstPtr& msg);
        void    cb_Xeein        (const tf::StampedTransform& transform);
        void    cb_Xtein        (const tf::StampedTransform& transform);
        void    cb_Save_Model   (const std_msgs::String::ConstPtr& location);
        void    cb_Set_State    (const std_msgs::Int16::ConstPtr& stat);

        // Approximation
        Vector approximateQdot  (const Vector &Q1,   const Vector &Q2,   double t1, double t2);
        Vector approximateXeedot(const Vector &Xee1, const Vector &Xee2, double t1, double t2);
        Vector approximateXtedot(const Vector &Xte1, const Vector &Xte2, double t1, double t2);
        Vector approximateJee   (const Vector &Q1,   const Vector &Q2, const Vector &Xee1, const Vector &Xee2);
        Vector approximateJte   (const Vector &Q1,   const Vector &Q2, const Vector &Xte1, const Vector &Xte2);

        // Training
        Vector trainEE          (const Vector &Q,                      const Vector &X_ee);
        Vector trainEEdot       (const Vector &Q, const Vector &Q_dot, const Vector &X_ee_dot);
        Vector trainEEjaccobi   (const Vector &Q, const Vector &Q_dot, const Vector &J_ee);
        Vector trainTE          (const Vector &Q,                      const Vector &X_te);
        Vector trainTEdot       (const Vector &Q, const Vector &Q_dot, const Vector &X_te_dot);
        Vector trainTEjaccobi   (const Vector &Q, const Vector &Q_dot, const Vector &J_te);
        Vector trainREE         (const Vector &Q, const Vector &R_ee);

        // Predicting
        Vector predictEE        (const Vector &Q);
        Vector predictEEdot     (const Vector &Q, const Vector &Q_dot);
        Vector predictEEjaccobi (const Vector &Q, const Vector &Q_dot);
        Vector predictTE        (const Vector &Q);
        Vector predictTEdot     (const Vector &Q, const Vector &Q_dot);
        Vector predictTEjaccobi (const Vector &Q, const Vector &Q_dot);
        Vector predictREE       (const Vector &Q);

        // Checker
        void checkEE         (double mse);
        void checkEEdot      (double mse);
        void checkEEjaccobian(double mse);
        void checkTE         (double mse);
        void checkTEdot      (double mse);
        void checkTEjaccobian(double mse);
        void checkREE        (double mse);

        // Main
        void   run              ();
        void   process          ();
        void   publishXee       ();
        void   publishXte       ();
        void   publishXeedotfroJ();
        void   publishXtedotfroJ();


    private:
        // Parameters
        const unsigned int _nJoints = 5;

        // Learner
        LWPR_Learner *_lwpr_learner_Xee, *_lwpr_learner_Xee_dot;// LWPR Learner for learning the endeffector kinematic
        LWPR_Learner *_lwpr_learner_Xte, *_lwpr_learner_Xte_dot;// LWPR Learner for learning the tool kinematic
        LWPR_Learner *_lwpr_learner_Jee, *_lwpr_learner_Jte;    // LWPR Learner for learning the current jacobian
        LWPR_Learner *_lwpr_learner_Ree, *_lwpr_learner_Rte;    // LWPR Learner for learning the current rotation
        double mse[7] = {0, 0, 0, 0, 0, 0, 0};  // Stores the current training MSE
        int error[7] = {0, 0, 0, 0, 0, 0, 0}; // Stores the amount of times the mse was above average


        // Kinematic states
        KinematicState *current, *previous, *predict;

        // Internal state
        STATE                       _state;
        bool                        _verbose;

        // ROS interface
        ros::Publisher              _pub_Xee, _pub_Xee_dot_froJ;
        ros::Publisher              _pub_Xte, _pub_Xte_dot_froJ;
        ros::NodeHandle             _n;
        ros::Subscriber             _sub_Qin, _sub_save,_sub_state;
        tf::TransformListener       _listener;
        tf::StampedTransform        _transform;

        // Config
        Config                      _config;
        std::string                 _configFolder;
        std::string                 _configName;


    };
}; /* namespace */


#endif /* TOOLEXTENSION_LWPR_NODE_H */
