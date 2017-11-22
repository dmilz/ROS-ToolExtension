#include <node.h>
#include <thread>
#define RATE 5

namespace toolextension_lwpr
{
    Node::Node(const ros::NodeHandle &n, std::string configName)
        : _state(both), _n(n), _verbose(true), _configName(configName)
    {
        /**********************************************************
         ************************* Config *************************
         **********************************************************/
        // init configFolder and configName
        this->_configFolder = boost::filesystem::path(this->_configName).parent_path().string();
        // read config, if it exists
        if(boost::filesystem::exists(this->_configName))
        {
                this->_config.read(this->_configName.c_str());
                if(this->_verbose)
                    ROS_DEBUG_STREAM("Read config: " << this->_configName);
        }
        if(this->_verbose)
            ROS_INFO_STREAM("Current config:" << std::endl << this->_config);

        /**********************************************************
         ************************* LEARNER ************************
         **********************************************************/
        // init learner
        this->_lwpr_learner_Xee     = new LWPR_Learner(this->_config,"lwpr_xee.");
        this->_lwpr_learner_Xee_dot = new LWPR_Learner(this->_config,"lwpr_xee_dot.");
        this->_lwpr_learner_Xte     = new LWPR_Learner(this->_config,"lwpr_xte.");
        this->_lwpr_learner_Xte_dot = new LWPR_Learner(this->_config,"lwpr_xte_dot.");
        this->_lwpr_learner_Jee     = new LWPR_Learner(this->_config,"lwpr_jee.");
        this->_lwpr_learner_Jte     = new LWPR_Learner(this->_config,"lwpr_jte.");
        this->_lwpr_learner_Ree     = new LWPR_Learner(this->_config,"lwpr_ree.");
        this->current  = new KinematicState();
        this->previous = new KinematicState();
        this->predict  = new KinematicState();

        /**********************************************************
         ************************* ROS ****************************
         **********************************************************/
        // init Ros interface
        // set up publisher and subscribers
        typedef geometry_msgs::PointStamped   	msg_pos;
        this->_pub_Xee  = _n.advertise<msg_pos> (this->_config.getString("xee_out_topic", ""), 2);
        this->_pub_Xte  = _n.advertise<msg_pos> (this->_config.getString("xte_out_topic", ""), 2);
        this->_pub_Xee_dot_froJ = _n.advertise<msg_pos> (this->_config.getString("xeedotfroj_out_topic", ""), 2);
        this->_pub_Xte_dot_froJ = _n.advertise<msg_pos> (this->_config.getString("xtedotfroj_out_topic", ""), 2);
        this->_sub_Qin  = _n.subscribe          (this->_config.getString("q_in_topic", ""), 1 , &Node::cb_Qin       , this);
        this->_sub_save = _n.subscribe          (this->_config.getString("save_model_in_topic", ""), 1 , &Node::cb_Save_Model, this);
        this->_sub_state= _n.subscribe          (this->_config.getString("set_state_in_topic",""), 1 , &Node::cb_Set_State , this);
    }

    Node::~Node()
    {
        /*if(this->_config.write(this->_configName.c_str()))
                ROS_ERROR_STREAM("Could not write config to " << this->_configName);
        */
        delete current;
        delete previous;
        delete predict;
        delete this->_lwpr_learner_Xee     ;
        delete this->_lwpr_learner_Xee_dot ;
        delete this->_lwpr_learner_Xte     ;
        delete this->_lwpr_learner_Xte_dot ;
        delete this->_lwpr_learner_Jee     ;
        delete this->_lwpr_learner_Jte     ;
        delete this->_lwpr_learner_Ree     ;

        ros::shutdown();
    }

    // Callback functions
    void Node::cb_Qin (const sensor_msgs::JointState::ConstPtr& msg)
    {
        // Structure inputs
        Vector Qin {0, 0, 0, 0, 0};
        Qin[0] = msg->position[4]; // r_shoulder_pan_joint
        Qin[1] = msg->position[5]; // r_shoulder_lift_joint
        Qin[2] = msg->position[6]; // r_elbow_joint
        Qin[3] = msg->position[7]; // r_wrist_1_joint
        Qin[4] = msg->position[8]; // r_wrist_2_joint

        if(this->_verbose)
            ROS_INFO("I heard: Q_in = [%3.5f %3.5f %3.5f %3.5f %3.5f]", Qin[0], Qin[1], Qin[2], Qin[3], Qin[4]);

        if(Qin.size() != this->_nJoints)
            ROS_INFO_STREAM("Qin.size() != this->_nJoints:" << Qin.size() << " != " << this->_nJoints);

        //Archivate old states
        this->previous->Q = this->current->Q;
        this->previous->Q_dot = this->current->Q_dot;


        KinematicState *kinstat = this->current;

        kinstat->Q         = Qin;
        kinstat->Q_time    = msg->header.stamp;
        kinstat->state[0] += 1;

        kinstat->Q_dot     = this->approximateQdot(this->previous->Q,
                                                   kinstat->Q,
                                                   this->previous->Q_time.toSec(),
                                                   kinstat->Q_time.toSec());
        kinstat->state[1] += 1;


        // Approximate X2ee dot, Jee, Xte dot, Jte
        kinstat->X_ee_dot  = this->approximateXeedot(this->previous->X_ee,
                                                     kinstat->X_ee,
                                                     this->previous->X_ee_time.toSec(),
                                                     kinstat->X_ee_time.toSec());
        kinstat->state[3] += 1;
        kinstat->J_ee      = this->approximateJee   (this->previous->Q,
                                                     kinstat->Q,
                                                     this->previous->X_ee,
                                                     kinstat->X_ee);
        kinstat->state[6] += 1;
        kinstat->X_te_dot  = this->approximateXtedot(this->previous->X_te,
                                                     kinstat->X_te,
                                                     this->previous->X_te_time.toSec(),
                                                     kinstat->X_te_time.toSec());
        kinstat->state[5] += 1;
        kinstat->J_te      = this->approximateJte   (this->previous->Q,
                                                     kinstat->Q,
                                                     this->previous->X_te,
                                                     kinstat->X_te);
        kinstat->state[7] += 1;

        this->process();
    }

    void Node::cb_Xeein (const tf::StampedTransform& transform)
    {
        if(this->_verbose)
            ROS_INFO_STREAM("Transform of X_ee: [" << transform.getOrigin().x() << " " << transform.getOrigin().y() << " " << transform.getOrigin().z() << "]");

        //Archivate old states
        this->previous->X_ee = this->current->X_ee;
        this->previous->X_ee_dot = this->current->X_ee_dot;
        this->previous->J_ee = this->current->J_ee;
        this->previous->R_ee = this->current->R_ee;

        KinematicState *kinstat;
        kinstat = this->current;
        kinstat->X_ee[0]  =  transform.getOrigin().x();
        kinstat->X_ee[1]   = transform.getOrigin().y();
        kinstat->X_ee[2]   = transform.getOrigin().z();
        kinstat->X_ee_time = transform.stamp_;
        kinstat->state[2] += 1;

        tf::Quaternion quat = transform.getRotation();

        // the tf::Quaternion has a method to acess roll pitch and yaw
        if(quat.w()<0.0)
        {
        kinstat->R_ee[0] = -quat.w();
        kinstat->R_ee[1] = -quat.x();
        kinstat->R_ee[2] = -quat.y();
        kinstat->R_ee[3] = -quat.z();
        }else{
        kinstat->R_ee[0] = quat.w();
        kinstat->R_ee[1] = quat.x();
        kinstat->R_ee[2] = quat.y();
        kinstat->R_ee[3] = quat.z();
        }
        ROS_INFO_STREAM("R_ee = "<<kinstat->R_ee);
        kinstat->state[8] += 1;

        this->process();
    }

    void   Node::cb_Xtein           (const tf::StampedTransform& transform)
    {
        if(this->_verbose)
            ROS_INFO_STREAM("Transform of X_te: [" << transform.getOrigin().x() << " " << transform.getOrigin().y() << " " << transform.getOrigin().z() << "]");

        //Archivate old states
        this->previous->X_te = this->current->X_te;
        this->previous->X_te_dot = this->current->X_te_dot;
        this->previous->J_te = this->current->J_te;

        KinematicState *kinstat = this->current;

        kinstat->X_te[0]   = transform.getOrigin().x();
        kinstat->X_te[1]   = transform.getOrigin().y();
        kinstat->X_te[2]   = transform.getOrigin().z();
        kinstat->X_te_time = transform.stamp_;
        kinstat->state[4] += 1;

        this->process();
    }

    void   Node::cb_Save_Model    (const std_msgs::String::ConstPtr& location)
    {
        if(this->_verbose)
            ROS_INFO("Save Model");
    }

    void   Node::cb_Set_State     (const std_msgs::Int16::ConstPtr& stat)
    {
        this->_state = (STATE) stat->data;
        if (this->_verbose && stat->data)
            ROS_INFO("Prediction state entered");
        else if (this->_verbose && !stat->data)
            ROS_INFO("Training state entered");
    }


    // Approximation
    Vector Node::approximateQdot  (const Vector &Q1, const Vector &Q2, const double t1, const double t2)
    {
        Vector Qdot = Vector(Q1.size());
        for(int i = 0; i < Q1.size(); i++)
            Qdot[i] = (Q2[i]-Q1[i])/(t2-t1);
        return Qdot;
    }

    Vector Node::approximateXeedot(const Vector &Xee1, const Vector &Xee2, const double t1, const double t2)
    {
        Vector Xeedot = Vector(Xee1.size());
        for(int i = 0; i < Xee1.size(); i++)
            Xeedot[i] = (Xee2[i]-Xee1[i])/(t2-t1);
        return Xeedot;
    }

    Vector Node::approximateXtedot(const Vector &Xte1, const Vector &Xte2, const double t1, const double t2)
    {
        Vector Xtedot = Vector(Xte1.size());
        for(int i = 0; i < Xte1.size(); i++)
            Xtedot[i] = (Xte2[i]-Xte1[i])/(t2-t1);
        return Xtedot;
    }

    Vector Node::approximateJee(const Vector &Q1, const Vector &Q2, const Vector &Xee1, const Vector &Xee2)
    {
        Vector Jee = Vector(Xee1.size()*Q1.size());
        for(int i1 = 0; i1 < Q1.size(); i1++)
            for(int i2 = 0; i2 < Xee1.size(); i2++)
                Jee[i1*i2] = ((Q2[i1]-Q1[i1])>0.000001) ? (Xee2[i2]-Xee1[i2])/(Q2[i1]-Q1[i1]) : 0.0;
        if(this->_verbose)
            ROS_INFO_STREAM("Appriximated Jee : " << Jee);
        return Jee;
    }

    Vector Node::approximateJte(const Vector &Q1, const Vector &Q2, const Vector &Xte1, const Vector &Xte2)
    {
        Vector Jte = Vector(Xte1.size()*Q1.size());
        for(int i1 = 0; i1 < Q1.size(); i1++)
            for(int i2 = 0; i2 < Xte1.size(); i2++)
                Jte[i1*i2] = ((Q2[i1]-Q1[i1])>0.000001) ? (Xte2[i2]-Xte1[i2])/(Q2[i1]-Q1[i1]) : 0.0;
        if(this->_verbose)
            ROS_INFO_STREAM("Appriximated Jte : " << Jte);
        return Jte;
    }


    // Training
    Vector Node::trainEE (const Vector &Q, const Vector &X_ee)
    {
        //ROS_INFO_STREAM("Q = "<<Q<<"     X_ee"<<X_ee);
        Vector predict = this->_lwpr_learner_Xee->train(Q, X_ee);
        mse[0] = 0;
        for(int i = 0; i < predict.size(); i++)
            mse[0] += (predict[i]-X_ee[i]);
        mse[0] /= predict.size();
        checkEE(mse[0]);
        if(this->_verbose)
             ROS_DEBUG("Trained X_ee with mse = \t\t%f",mse[0]);

        return predict;
    }

    Vector Node::trainEEdot (const Vector &Q, const Vector &Q_dot, const Vector &X_ee_dot)
    {
        Vector in = Q;
        in.insert( in.end(), Q_dot.begin(), Q_dot.end() );
        Vector predict = this->_lwpr_learner_Xee_dot->train(in, X_ee_dot);
        mse[1] = 0;
        for(int i = 0; i < predict.size(); i++)
            mse[1] += (predict[i]-X_ee_dot[i]);
        mse[1] /= predict.size();
        checkEEdot(mse[1]);
        if(this->_verbose)
             ROS_DEBUG("Trained X_ee_dot with mse = \t\t%f",mse[1]);

        return predict;
    }

    Vector Node::trainEEjaccobi (const Vector &Q, const Vector &Q_dot, const Vector &J_ee)
    {
        Vector in = Q;
        in.insert( in.end(), Q_dot.begin(), Q_dot.end() );
        Vector predict = this->_lwpr_learner_Jee->train(in, J_ee);
        mse[2] = 0;
        for(int i = 0; i < predict.size(); i++)
        {
            mse[2] += (predict[i]-J_ee[i]);
            ROS_DEBUG("trainJEE : %f += %f - %f", mse[2], predict[i],J_ee[i]);
        }
        mse[2] /= predict.size();
        checkEEjaccobian(mse[2]);
        if(this->_verbose)
             ROS_DEBUG("Trained J_ee with mse = \t\t%f",mse[2]);

        return predict;
    }

    Vector Node::trainTE (const Vector &Q, const Vector &X_te)
    {
        Vector predict = this->_lwpr_learner_Xte->train(Q, X_te);
        mse[3] = 0;
        for(int i = 0; i < predict.size(); i++)
            mse[3] += (predict[i]-X_te[i]);
        mse[3] /= predict.size();
        checkTE(mse[3]);
        if(this->_verbose)
             ROS_DEBUG("Trained X_te with mse = \t\t%f",mse[3]);

        return predict;
    }

    Vector Node::trainTEdot (const Vector &Q, const Vector &Q_dot, const Vector &X_te_dot)
    {
        Vector in = Q;
        in.insert( in.end(), Q_dot.begin(), Q_dot.end() );
        Vector predict = this->_lwpr_learner_Xte_dot->train(in, X_te_dot);
        mse[4] = 0;
        for(int i = 0; i < predict.size(); i++)
            mse[4] += (predict[i]-X_te_dot[i]);
        mse[4] /= predict.size();
        checkTEdot(mse[4]);
        if(this->_verbose)
             ROS_DEBUG("Trained X_te_dot with mse = \t\t%f",mse[4]);

        return predict;
    }

    Vector Node::trainTEjaccobi (const Vector &Q, const Vector &Q_dot, const Vector &J_te)
    {
        Vector in = Q;
        in.insert( in.end(), Q_dot.begin(), Q_dot.end() );
        Vector predict = this->_lwpr_learner_Jte->train(in, J_te);
        mse[5] = 0;
        for(int i = 0; i < predict.size(); i++)
        {
            mse[5] += (predict[i]-J_te[i]);
            ROS_DEBUG("trainJTE : %f += %f - %f", mse[5], predict[i],J_te[i]);
        }
        mse[5] /= predict.size();
        checkTEjaccobian(mse[5]);
        if(this->_verbose)
             ROS_DEBUG("Trained J_te with mse = \t\t%f",mse[5]);

        return predict;
    }

    Vector Node::trainREE (const Vector &Q, const Vector &R_ee)
    {
        Vector predict = this->_lwpr_learner_Ree->train(Q, R_ee);
        mse[6] = 0;
        for(int i = 0; i < predict.size(); i++)
        {
            mse[6] += (predict[i]-R_ee[i]);
            ROS_DEBUG("trainREE : %f += %f - %f", mse[6], predict[i],R_ee[i]);
        }
        mse[6] /= predict.size();
        checkREE(mse[6]);
        if(this->_verbose)
             ROS_DEBUG("Trained R_ee with mse = \t\t%f",mse[6]);

        return predict;
    }

    // Predicting
    Vector Node::predictEE (const Vector &Q)
    {
        if(this->_verbose)
            ROS_DEBUG_STREAM("Called predictEE(" << Q << ")" );
        Vector tol;
        return this->_lwpr_learner_Xee->predict(Q, tol);
    }

    Vector Node::predictEEdot (const Vector &Q, const Vector &Q_dot)
    {
        if(this->_verbose)
            ROS_DEBUG_STREAM("Called predictEEdot(" << Q << "\n              " << Q_dot << ")" );
        Vector tol, in = Q;
        in.insert( in.end(), Q_dot.begin(), Q_dot.end() );
        return this->_lwpr_learner_Xee_dot->predict(in, tol);
    }

    Vector Node::predictEEjaccobi (const Vector &Q, const Vector &Q_dot)
    {
        if(this->_verbose)
            ROS_DEBUG_STREAM("Called predictEEjacobi(" << Q << "\n              " << Q_dot << ")" );
        Vector tol, in = Q;
        in.insert( in.end(), Q_dot.begin(), Q_dot.end() );
        return this->_lwpr_learner_Jee->predict(in, tol);
    }

    Vector Node::predictTE (const Vector &Q)
    {
        if(this->_verbose)
            ROS_DEBUG_STREAM("Called predictTE(" << Q << ")" );
        Vector tol;
        return this->_lwpr_learner_Xte->predict(Q, tol);
    }

    Vector Node::predictTEdot (const Vector &Q, const Vector &Q_dot)
    {
        if(this->_verbose)
            ROS_DEBUG_STREAM("Called predictTEdot(" << Q << "\n              " << Q_dot << ")" );
        Vector tol, in = Q;
        in.insert( in.end(), Q_dot.begin(), Q_dot.end() );
        return this->_lwpr_learner_Xte_dot->predict(in, tol);
    }

    Vector Node::predictTEjaccobi (const Vector &Q, const Vector &Q_dot)
    {
        if(this->_verbose)
            ROS_DEBUG_STREAM("Called predictTEjacobi(" << Q << "\n              " << Q_dot << ")" );
        Vector tol, in = Q;
        in.insert( in.end(), Q_dot.begin(), Q_dot.end() );
        return this->_lwpr_learner_Jte->predict(in, tol);
    }

    Vector Node::predictREE (const Vector &Q)
    {
        if(this->_verbose)
            ROS_DEBUG_STREAM("Called predictREE(" << Q <<  ")" );
        Vector tol;
        return this->_lwpr_learner_Ree->predict(Q, tol);
    }

    // Checker
    void Node::checkEE(double mse)
    {
        // Forget everything TODO Config file ; Somthing is untrained if its quite wrong for 25 times
        if(mse > this->_config.getDouble("xee_mse_limit", 0.5))
        {
            if (++this->error[0] < 20)
                return;
            this->error[0] = 0;
            this->_lwpr_learner_Xee     = new LWPR_Learner(this->_config,"lwpr_xee.");
            if(this->_verbose)
                ROS_DEBUG("checkEE(%f) let forget everything",mse);
        }
    }

    void Node::checkEEdot(double mse)
    {
        if(mse > this->_config.getDouble("xee_dot_mse_limit", 0.5))
        {
            if (++this->error[1] < 20)
                return;
            this->error[1] = 0;
            if(this->_verbose)
                ROS_DEBUG("checkEEdot(%f) let forget everything",mse);
            this->_lwpr_learner_Xee_dot = new LWPR_Learner(this->_config,"lwpr_xee_dot.");
        }
    }

    void Node::checkEEjaccobian(double mse)
    {
        if(mse > this->_config.getDouble("jee_mse_limit", 0.5))
        {
            if (++this->error[2] < 20)
                return;
            this->error[2] = 0;
            if(this->_verbose)
                ROS_DEBUG("checkEEjacobian(%f) let forget everything",mse);
            this->_lwpr_learner_Jee     = new LWPR_Learner(this->_config,"lwpr_jee.");
        }
    }

    void Node::checkTE(double mse)
    {
        if(mse > this->_config.getDouble("xte_mse_limit", 0.5))
        {
            if (++this->error[3] < 20)
                return;
            this->error[3] = 0;
            if(this->_verbose)
                ROS_DEBUG("checkTE(%f) let forget everything",mse);
            this->_lwpr_learner_Xte     = new LWPR_Learner(this->_config,"lwpr_xte.");
        }
    }

    void Node::checkTEdot(double mse)
    {
        if(mse > this->_config.getDouble("xte_dot_mse_limit", 0.5))
        {
            if (++this->error[4] < 20)
                return;
            this->error[4] = 0;
            if(this->_verbose)
                ROS_DEBUG("checkTEdot(%f) let forget everything",mse);
            this->_lwpr_learner_Xte_dot = new LWPR_Learner(this->_config,"lwpr_xte_dot.");
        }
    }

    void Node::checkTEjaccobian(double mse)
    {
        if(mse > this->_config.getDouble("jte_mse_limit", 0.5))
        {
            if(++this->error[5] < 20)
                return;
            this->error[5] = 0;
            if(this->_verbose)
                ROS_DEBUG("checkTEjacobian(%f) let forget everything",mse);
            this->_lwpr_learner_Jte     = new LWPR_Learner(this->_config,"lwpr_jte.");
        }
    }

    void Node::checkREE(double mse)
    {
        if(mse > this->_config.getDouble("ree_mse_limit", 0.5))
        {
            if(++this->error[6] < 20)
                return;
            this->error[6] = 0;
          if(this->_verbose)
                ROS_DEBUG("checkREE(%f) let forget everything",mse);
            this->_lwpr_learner_Ree     = new LWPR_Learner(this->_config,"lwpr_ree.");
        }
    }

    // Main
    void Node::run ()
    {
        ros::Rate rate(RATE);
        while (_n.ok())
        {
            if(this->_verbose)
                ROS_INFO("----------------------New Step -------------------------");
            try
            {
                this->_listener.lookupTransform("/base_footprint", this->_config.getString("xee_in_topic", ""), ros::Time(0), this->_transform);
                this->cb_Xeein(this->_transform);
                this->_listener.lookupTransform(this->_config.getString("xee_in_topic", ""), this->_config.getString("xte_in_topic", ""), ros::Time(0), this->_transform);
                this->cb_Xtein(this->_transform);
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                ros::Duration(0.5).sleep();
                continue;
            }
            ros::spinOnce();
            rate.sleep();
        }
    }

    void Node::process ()
    {

        if(this->_verbose)
            ROS_INFO("Called process(%i,%i,%i,%i,%i,%i,%i,%i) in %s",this->current->state[0],this->current->state[1],this->current->state[2],this->current->state[3],this->current->state[4],this->current->state[5],this->current->state[6],this->current->state[7], (this->_state == training) ? "training state" : "prediction state");
        if(this->_state == training)
        {
            if(!this->current->valid())
                return;

            if(this->_verbose)
                ROS_INFO("[ Process() ] Valid kinematics state. Train models.");
            // Warning: only supported since C++11
            std::thread th_trainEE          ([this](){this->trainEE   (this->current->Q, this->current->X_ee)                           ;} );
            std::thread th_trainEEdot       ([this](){this->trainEEdot(this->current->Q, this->current->Q_dot, this->current->X_ee_dot) ;} );
            std::thread th_trainEEjaccobi   ([this](){this->trainEEjaccobi(this->current->Q, this->current->Q_dot, this->current->J_ee) ;} );
            std::thread th_trainTE          ([this](){this->trainTE   (this->current->Q, this->current->X_te)                           ;} );
            std::thread th_trainTEdot       ([this](){this->trainTEdot(this->current->Q, this->current->Q_dot, this->current->X_te_dot) ;} );
            std::thread th_trainTEjaccobi   ([this](){this->trainTEjaccobi(this->current->Q, this->current->Q_dot, this->current->J_te) ;} );
            std::thread th_trainREE         ([this](){this->trainREE(this->current->Q, this->current->R_ee)                             ;} );

            th_trainEE.join();
            th_trainEEdot.join();
            th_trainEEjaccobi.join();
            th_trainTE.join();
            th_trainTEdot.join();
            th_trainTEjaccobi.join();
            th_trainREE.join();

            if(this->_verbose)
            {
                ROS_INFO("Trained X_ee with mse = \t\t%f",mse[0]);
                ROS_INFO("Trained X_ee_dot with mse = \t\t%f",mse[1]);
                ROS_INFO("Trained J_ee with mse = \t\t%f",mse[2]);
                ROS_INFO("Trained X_te with mse = \t\t%f",mse[3]);
                ROS_INFO("Trained X_te_dot with mse = \t\t%f",mse[4]);
                ROS_INFO("Trained J_te with mse = \t\t%f",mse[5]);
                ROS_INFO("Trained R_ee with mse = \t\t%f",mse[6]);
                ROS_INFO("Trained R_te with mse = \t\t%f",mse[7]);
            }

            delete this->previous;
            this->previous = this->current;
            this->current = new KinematicState();
            this->current->reset();
        }
        else if(this->_state == predicting)
        {
            if( !((this->current->state[0] > 0) && (this->current->state[1] > 0)) )
                return;

            this->predict->Q = this->current->Q;         this->predict->state[0] = 1;
            this->predict->Q_dot = this->current->Q_dot; this->predict->state[1] = 1;
            if(this->_verbose)
                ROS_INFO("[ Process() ] Valid kinematics state. Predict models.");
            // Warning: only supported since C++11
            std::thread th_predictEE          ([this](){this->predict->X_ee     = this->predictEE       (this->predict->Q);});
            std::thread th_predictEEdot       ([this](){this->predict->X_ee_dot = this->predictEEdot    (this->predict->Q,this->predict->Q_dot);});
            std::thread th_predictEEjaccobi   ([this](){this->predict->J_ee     = this->predictEEjaccobi(this->predict->Q,this->predict->Q_dot);});
            std::thread th_predictTE          ([this](){this->predict->X_te     = this->predictTE       (this->predict->Q);});
            std::thread th_predictTEdot       ([this](){this->predict->X_te_dot = this->predictTEdot    (this->predict->Q,this->predict->Q_dot);});
            std::thread th_predictTEjaccobi   ([this](){this->predict->J_te     = this->predictTEjaccobi(this->predict->Q,this->predict->Q_dot);});
            std::thread th_predictREE         ([this](){this->predict->R_ee     = this->predictREE      (this->predict->Q);});

            th_predictEE.join();
            th_predictEEdot.join();
            th_predictEEjaccobi.join();
            th_predictTE.join();
            th_predictTEdot.join();
            th_predictTEjaccobi.join();
            th_predictREE.join();

            // Publish results
            publishXee();
            publishXte();
            publishXeedotfroJ();
            publishXtedotfroJ();

            delete this->previous;
            this->previous = this->predict;
            this->predict = new KinematicState();
            this->current->reset();
        }
        else if(this->_state == both)
        {


            if(!this->current->valid())
                return;

            if(this->_verbose)
                ROS_INFO("[ Process() ] Valid kinematics state. Train and predict models.");

            // 'Preprocess data'
            this->predict->Q = this->current->Q;         this->predict->state[0] = 1;
            this->predict->Q_dot = this->current->Q_dot; this->predict->state[1] = 1;


            // Warning: only supported since C++11
            std::thread th_bothEE          ([this](){this->predict->X_ee     = this->trainEE       (this->current->Q, this->current->X_ee)                           ;} );
            std::thread th_bothEEdot       ([this](){this->predict->X_ee_dot = this->trainEEdot    (this->current->Q, this->current->Q_dot, this->current->X_ee_dot) ;} );
            std::thread th_bothEEjaccobi   ([this](){this->predict->J_ee     = this->trainEEjaccobi(this->current->Q, this->current->Q_dot, this->current->J_ee)     ;} );
            std::thread th_bothTE          ([this](){this->predict->X_te     = this->trainTE       (this->current->Q, this->current->X_te)                           ;} );
            std::thread th_bothTEdot       ([this](){this->predict->X_te_dot = this->trainTEdot    (this->current->Q, this->current->Q_dot, this->current->X_te_dot) ;} );
            std::thread th_bothTEjaccobi   ([this](){this->predict->J_te     = this->trainTEjaccobi(this->current->Q, this->current->Q_dot, this->current->J_te)     ;} );
            std::thread th_bothREE         ([this](){this->predict->R_ee     = this->trainREE      (this->current->Q, this->current->R_ee)                           ;} );

            th_bothEE.join();
            th_bothEEdot.join();
            th_bothEEjaccobi.join();
            th_bothTE.join();
            th_bothTEdot.join();
            th_bothTEjaccobi.join();
            th_bothREE.join();

            if(this->_verbose)
            {
                ROS_INFO("Trained X_ee with mse = \t%f",      mse[0]);
                ROS_INFO("Trained X_ee_dot with mse = \t%f",  mse[1]);
                ROS_INFO("Trained J_ee with mse = \t%f",      mse[2]);
                ROS_INFO("Trained X_te with mse = \t%f",      mse[3]);
                ROS_INFO("Trained X_te_dot with mse = \t%f",  mse[4]);
                ROS_INFO("Trained J_te with mse = \t%f",      mse[5]);
                ROS_INFO("Trained R_ee with mse = \t%f",      mse[6]);
            }

            // Publish results
            publishXee();
            publishXte();
            publishXeedotfroJ();
            publishXtedotfroJ();

            delete this->previous;
            this->previous = this->predict;
            this->predict = new KinematicState();
            this->current->reset();

        }
        else
        {
            // ERROR
        }

    }

    void Node::publishXee ()
    {

        /* ---- Publish to toolextension_lwpr/xee_out ----- */
        geometry_msgs::PointStamped msg;

        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "/world";

        msg.point.x = this->predict->X_ee[0];
        msg.point.y = this->predict->X_ee[1];
        msg.point.z = this->predict->X_ee[2];
        if(this->_verbose)
            ROS_INFO("Publish Xee = ( %f , %f , %f )", msg.point.x, msg.point.y, msg.point.z);
        this->_pub_Xee.publish(msg);


        // ---- Publish to tf/xee_predicted ----- /
        static tf::TransformBroadcaster br_xee;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(msg.point.x, msg.point.y, msg.point.z) );
        tf::Quaternion q;
        q.setW(this->predict->R_ee[0]);
        q.setX(this->predict->R_ee[1]);
        q.setY(this->predict->R_ee[2]);
        q.setZ(this->predict->R_ee[3]);
        transform.setRotation(q);
        br_xee.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "xee_predicted"));

    }

    void Node::publishXte ()
    {
        /* ---- Publish to toolextension_lwpr/xte_out ----- */
        geometry_msgs::PointStamped msg;

        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "/xee_predicted";

        msg.point.x = this->predict->X_te[0];
        msg.point.y = this->predict->X_te[1];
        msg.point.z = this->predict->X_te[2];
        if(this->_verbose)
            ROS_INFO("Publish XTe = ( %f , %f , %f )", msg.point.x, msg.point.y, msg.point.z);
        this->_pub_Xte.publish(msg);

        // ---- Publish to tf/xte_predicted -----
        static tf::TransformBroadcaster br_xte;
        tf::Transform transform;
        transform.setOrigin( tf::Vector3(this->predict->X_te[0], this->predict->X_te[1], this->predict->X_te[2]) );
        tf::Quaternion q;
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        // FIXME ROTATIONS OF THE XEE_PREDICTED
        br_xte.sendTransform(tf::StampedTransform(transform, ros::Time::now(),  this->_config.getString("xee_predicted", "") ,this->_config.getString("xte_predicted", "")));


    }

    void Node::publishXeedotfroJ ()
    {
        geometry_msgs::PointStamped msg;

        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "/world";

        msg.point.x = (msg.point.y = (msg.point.z = 0));
        for(int i = 0; i < this->predict->Q_dot.size(); i++)
        {
            msg.point.x += this->predict->J_ee[3*i + 0] * this->predict->Q_dot[i];
            msg.point.y += this->predict->J_ee[3*i + 1] * this->predict->Q_dot[i];
            msg.point.z += this->predict->J_ee[3*i + 2] * this->predict->Q_dot[i];
        }
        if(this->_verbose)
            ROS_INFO("Publish Xee_dot_froJ = ( %f, %f, %f )",msg.point.x, msg.point.y, msg.point.z);
        this->_pub_Xee_dot_froJ.publish(msg);

    }

    void   Node::publishXtedotfroJ ()
    {
        geometry_msgs::PointStamped msg;

        msg.header.stamp = ros::Time::now();
        msg.header.frame_id = "/world";

        msg.point.x = (msg.point.y = (msg.point.z = 0));
        for(int i = 0; i < this->predict->Q_dot.size(); i++)
        {
            msg.point.x += (this->predict->J_ee[3*i + 0] + this->predict->J_te[3*i + 0]) * this->predict->Q_dot[i];
            msg.point.y += (this->predict->J_ee[3*i + 1] + this->predict->J_te[3*i + 1]) * this->predict->Q_dot[i];
            msg.point.z += (this->predict->J_ee[3*i + 2] + this->predict->J_te[3*i + 2]) * this->predict->Q_dot[i];
        }
        if(this->_verbose)
            ROS_INFO("Publish Xte_dot_froJ = ( %f, %f, %f)",msg.point.x, msg.point.y, msg.point.z);
        this->_pub_Xte_dot_froJ.publish(msg);

    }
}
