#include <alle_kinematics_skin/RobotTools/eigenUtils.h>
#include <ros/ros.h>

namespace alle_kinematics_skin{
namespace robotTools{
using namespace Tum;

/*********************************************************************************
    convert from geometry msgs Pose to Eigen pos/quat
**********************************************************************************/
void eigenUtils::pose2eigen_pos_quat(const geometry_msgs::Pose &pose,   // input
                                     Vector3d &pos,                     // output
                                     Quaterniond &quat)
{
    pos(0) = pose.position.x;
    pos(1) = pose.position.y;
    pos(2) = pose.position.z;
    quat.w() = pose.orientation.w;
    quat.x() = pose.orientation.x;
    quat.y() = pose.orientation.y;
    quat.z() = pose.orientation.z;
}

/*********************************************************************************
    convert from Eigen pos/quat to geometry msgs Pose
**********************************************************************************/
void eigenUtils::eigen_pos_quat2pose(const Vector3d &pos,
                                     const Quaterniond &quat,
                                     geometry_msgs::Pose &pose)
{
    pose.position.x = pos(0);
    pose.position.y = pos(1);
    pose.position.z = pos(2);

    pose.orientation.w = quat.w();
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
}

/*********************************************************************************
    convert from geometry msgs Point to Eigen pos
**********************************************************************************/
void eigenUtils::point2eigen_pos(const geometry_msgs::Point &point, // input
                                 Vector3d &pos                      // output
                                 )
{
    pos(0) = point.x;
    pos(1) = point.y;
    pos(2) = point.z;
}

/*********************************************************************************
    convert from Eigen pos to geometry msgs Point
**********************************************************************************/
void eigenUtils::eigen_pos2point(const Vector3d &pos,
                                 geometry_msgs::Point &point)
{
    point.x = pos(0);
    point.y = pos(1);
    point.z = pos(2);
}


/*********************************************************************************
    product of homogeneous transform with eign
**********************************************************************************/
void eigenUtils::product_transform(const Vector3d &trans_01,
                                   const Quaterniond &quat_01,
                                   const Vector3d &trans_12, // local linear transform
                                   const Quaterniond &quat_12,// local rotation
                                   Vector3d &trans_02,
                                   Quaterniond &quat_02)
{
    trans_02  = trans_01 + quat_01*trans_12;
    quat_02 = quat_01*quat_12;
}

/*********************************************************************************
    coordinate transform with eign
**********************************************************************************/
void eigenUtils::coordinate_transform_vect(const Vector3d &vec_in,
                                           const Vector3d &trans,
                                           const Quaterniond &quat,
                                           Vector3d &vec_out)
{
    vec_out  = vec_in + quat*trans;
}

/*********************************************************************************
    rotate vector with eigen
**********************************************************************************/
void eigenUtils::rotate_vect(const Vector3d &vec_in,
                                const Quaterniond &quat,
                                Vector3d &vec_out)
{
    vec_out  = quat*vec_in;
}

/*********************************************************************************
    print quaternion
**********************************************************************************/
void eigenUtils::print_quat(const Quaterniond &quat)
{
    ROS_INFO_STREAM("quat(w,x,y,z)=" << quat.w() << " " << quat.vec().transpose());
}

/**********************************************************************************
  check sign of quaternion
**********************************************************************************/
bool eigenUtils::checkQuaternionSign(const Quaterniond &quat)
{
    // return true if w < 0
    if(quat.w()<0.0)
        return true;
    return false;
}

/**********************************************************************************
  flip sign of quaternion
**********************************************************************************/
Quaterniond eigenUtils::flipQuaternionSign(const Quaterniond &quat)
{
    return Quaterniond(-quat.w(), -quat.x(), -quat.y(), -quat.z());
}

/**********************************************************************************
  check and flip sign of quaternion if necessary
**********************************************************************************/
Quaterniond eigenUtils::checkFlipQuaternionSign(const Quaterniond &quat)
{
    if(checkQuaternionSign(quat))
        return flipQuaternionSign(quat);
    return quat;
}

/**********************************************************************************
  RPY to rotation matrix
**********************************************************************************/
Matrix3d eigenUtils::RPY2RotMat(const Vector3d &RPY)
{
    Matrix3d R = Matrix3d::Identity();
    return R = AngleAxisd(RPY(2), Vector3d::UnitZ())
             * AngleAxisd(RPY(1), Vector3d::UnitY())
             * AngleAxisd(RPY(0), Vector3d::UnitX());
}

/**********************************************************************************
  rotation matrix to RPY
**********************************************************************************/
Vector3d eigenUtils::rotMat2RPY(const Matrix3d &R)
{
    /*
    Vector3d rpy = Vector3d::Zero();
    Vector3d euler = R.eulerAngles(2,1,0); // zyx euler angles

    rpy(0) = euler(2); // roll  = eular z
    rpy(1) = euler(1); // pitch = eular y
    rpy(2) = euler(0); // yaw   = eular x
    ////////////*///////////

    Vector3d rpy;
    rpy << 0.0, 0.0, 0.0;
    rpy(0) = atan2( R(2,1) , R(2,2) );
    rpy(1) = asin( -R(2,0) );
    rpy(2) = atan2( R(1,0) , R(0,0) );

    return rpy;
}

/**********************************************************************************
  xyz Euler Angles to rotation matrix
**********************************************************************************/
Matrix3d eigenUtils::xyzEuler2RotMat(const Vector3d &vAngle)
{
    Matrix3d R = Matrix3d::Identity();
    return R = AngleAxisd(vAngle(0), Vector3d::UnitX())
             * AngleAxisd(vAngle(1), Vector3d::UnitY())
             * AngleAxisd(vAngle(2), Vector3d::UnitZ());
}

/**********************************************************************************
  rotation matrix around x axis
**********************************************************************************/
Matrix3d eigenUtils::rotX(double angle)
{
    Matrix3d R = Matrix3d::Identity();
    return R = AngleAxisd(angle, Vector3d::UnitX());
}


/**********************************************************************************
  rotation matrix around y axis
**********************************************************************************/
Matrix3d eigenUtils::rotY(double angle)
{
    Matrix3d R = Matrix3d::Identity();
    return R = AngleAxisd(angle, Vector3d::UnitY());
}


/**********************************************************************************
  rotation matrix around z axis
**********************************************************************************/
Matrix3d eigenUtils::rotZ(double angle)
{
    Matrix3d R = Matrix3d::Identity();
    return R = AngleAxisd(angle, Vector3d::UnitZ());
}

/**********************************************************************************
  quaternion error (Siciliano (3.87))
**********************************************************************************/
Vector3d eigenUtils::quaternionError(const Quaterniond &quat_des,
                                     const Quaterniond &quat_current)
{
    // Matrix3d S = omega2SkewSymmetricMat(quat_des.vec());
    // Vector3d eo = quat_current.w()*quat_des.vec() - quat_des.w()*quat_current.vec() - S*quat_current.vec();

    Quaterniond delta_q = quat_des * quat_current.inverse();
    delta_q.normalize(); // normalize quaternion
    delta_q = eigenUtils::checkFlipQuaternionSign(delta_q); // flip sign if necessary
    Vector3d eo = delta_q.vec();
    return eo;
}

/**********************************************************************************
  quaternion log error (Shoemake, Murray, Righetti)
**********************************************************************************/
// definition 1 (cf. Shoemake)
// exp(q) = [cos(th), v*sin(th)]
// log(q) = [0, v*th]
// angular velocity: around v with 2*th
//
// th = acos(w)
// sin(th) = sin(acos(w))
// v = q.vec()/sin(th)
//
// eo = 2*v*th = 2*q.vec()/sin(acos(w)) * acos(w)
//             = 2*q.vec()/sinc(a), where a = acos(w)

// definition 2 (Murray, Righetti)
// q = [cos(th/2), v*sin(th/2)]
// log(q) = [0, v*th]
// angular velocity: around v with th
//
// th = 2*acos(w)
// sin(th/2) = sin(2*acos(w)/2) = sin(acos(w))
// v = q.vec()/sin(th/2)
//
// eo = v*th = q.vec()/sin(acos(w)) * 2*acos(w)
//           = 2*q.vec()/sin(acos(w)) * acos(w)
//           = 2*q.vec()/sinc(a), where a = acos(w)

// avoid division by zero when sin(acos(w)) ~= 0
// Taylor expansion of th/sin(th)   = 1 + 1/6*th^2
//                     th/sin(th/2) = 2*(1 + 1/6*th^2)

// caevat: to avoid division by 0 (singularity), it is necessary to consider two cases
// 1) sin(0) with quat.w() = 1. In this case, use Taylor expansion of th/sin(th) = 1 + 1/6*th^2
// 2) sin(pi) with quat.w() = -1. This shouldn't happen since we restrict quat.w() >= 0.


Vector3d eigenUtils::quaternionLogError(const Quaterniond &quat_des,
                                        const Quaterniond &quat_current)
{
    const double threshold = 0.001;
    Vector3d eo;
    Quaterniond delta_q = quat_des * quat_current.inverse();
    delta_q = eigenUtils::checkFlipQuaternionSign(delta_q); // flip sign if needed to make sure delta_q.w() > 0

    // acos(x) with x>1 returns nan. Thus, some care will be necessary to make sure x<=1
    // 1. normalize delta_q
    // 2. use atan2(delta_q.vec().norm(), delta_q.w()) instead for more nuemrical robustness

    delta_q.normalize(); // normalize quaternion
      //double theta = 2.0 * acos(delta_q.w());
    double theta = 2.0 * atan2(delta_q.vec().norm(), delta_q.w());
    //  ROS_INFO_STREAM("theta = " << theta);

    // avoid division by 0
    if(abs(1.0-(delta_q.w()*delta_q.w()) < threshold))
    {
        // Taylor expansion of th/sin(th) ~= 1 + 1/6*th^2
        // and th/(sin(th/2)) ~= 2*(1 + 1/6*th^2)
        eo = 2.0 * (1.0 + 1.0/6.0 * theta*theta) * delta_q.vec();
    }
    else
        eo = theta/sin(theta/2.0) * delta_q.vec();

  return eo;
}

/**********************************************************************************
  skew-symmetric operator for given angular velocity omega
**********************************************************************************/
Matrix3d eigenUtils::omega2SkewSymmetricMat(const Vector3d &w)
{
    Matrix3d S;
    S <<   0, -w(2),  w(1),
        w(2),    0 , -w(0),
       -w(1),  w(0),    0;
    return S;
}

/**********************************************************************************
  quaternion multiplication (Murray book p. 33)
**********************************************************************************/
Quaterniond eigenUtils::quaternionMult(const Quaterniond &quat_q,
                                                       const Quaterniond &quat_p)
{
    Quaterniond quat_qp;
    quat_qp.w()   = quat_q.w()*quat_p.w() - quat_q.vec().dot(quat_p.vec());
    quat_qp.vec() = quat_q.w()*quat_p.vec() + quat_p.w()*quat_q.vec() + quat_q.vec().cross(quat_p.vec());

    return quat_qp;
}

/**********************************************************************************
  Moore-Penrose Pseudo Inverse
**********************************************************************************/
MatrixXd pinv(const MatrixXd &a, double epsilon = std::numeric_limits<double>::epsilon() )
{
    Eigen::JacobiSVD< MatrixXd > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
    double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
    return svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

//template <typename R, typename T>
//R eigenUtils::getSpline5(const T &start, const T &goal, double t_current, double t_total)
//{
//    //Saturate time variable to prevent using the pol function out of the desired time interval
//    if(t_current>t_total)
//    {
//        t_current=t_total;
//    }
//    else if (t_current<0.0)
//    {
//        t_current=0.0;
//    }

//    double r = t_current/t_total;
//    double r2 =r*r;
//    double r3=r2*r;
//    double r4=r3*r;
//    double r5=r4*r;
//    double a1 = 10*r3 - 15*r4 + 6*r5;
//    double a2 = (30*r2 - 60*r3 + 30*r4)/(t_total);
//    double a3 = (60*r - 180*r2 + 120*r3)/(t_total*t_total);

//    R out;
//    T x_d, xp_d, xpp_d;

//    int sV=x_d.size();

//    for (int idx = 0; idx < sV; idx++)
//    {
//        x_d(idx) = a1*(goal(idx) - start(idx)) + start(idx);
//        xp_d(idx) = a2*(goal(idx) - start(idx));
//        xpp_d(idx) = a3*(goal(idx) - start(idx));
//    }

//    out.push_back(x_d);
//    out.push_back(xp_d);
//    out.push_back(xpp_d);

//    return out;
//}

//template VVector7d eigenUtils::getSpline5<VVector7d,Tum::Vector7d>(const Tum::Vector7d&, const Tum::Vector7d&, double, double);

}
}
