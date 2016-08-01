#ifndef POSITION_ESTIMATION_H
#define POSITION_ESTIMATION_H

#include <ros/ros.h>
#include <position_estimation/Anchor_msgs.h>
#include <random>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <math.h>
#include <position_estimation/Anchor_msgs.h>
#include <pcl/common/eigen.h>

struct measurements {
    double anc0_t1;
    double anc0_t2;
    double anc1_t1;
    double anc1_t2;
    double anc2_t1;
    double anc2_t2;
    double anc3_t1;
    double anc3_t2;
};

struct Pose {
    Eigen::Vector3d position;
    Eigen::Vector3d orientation;
};

double AnchorDistance(const Eigen::Vector3d& one, const Eigen::Vector3d& two) {
    Eigen::Vector3d tmp = one - two;
    return sqrt(tmp.adjoint()*tmp);
}

Eigen::Matrix3d getRotationMatrix(const Eigen::Vector3d& orientation) {
    Eigen::Matrix3d ret;
    ret = Eigen::AngleAxisd(orientation(2), Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(orientation(1), Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(orientation(0), Eigen::Vector3d::UnitX());
    return ret;
}

double calculateProbability(Eigen::VectorXd measured, Eigen::VectorXd expected) {
    return 3;
}


#endif // POSITION_ESTIMATION_H
