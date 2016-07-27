#define PARTICLE_NUMBER 100

#include <ros/ros.h>
#include <position_estimation/Anchor_msgs.h>
#include <random>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <math.h>
#include <position_estimation/Anchor_msgs.h>
#include <pcl/common/eigen.h>

struct anchors {
    double anc0;
    double anc1;
    double anc2;
    double anc3;
};

struct Pose {
    Eigen::Vector3d position;
    Eigen::Vector3d orientation;
};

Eigen::Vector3d anc0;
Eigen::Vector3d anc1;
Eigen::Vector3d anc2;
Eigen::Vector3d anc3;
Eigen::Vector3d tag1_position;
Eigen::Vector3d tag2_position;
Pose uavLocation;

anchors measuredDistance;
ros::Publisher debug_particle_cloud_pub;
ros::Publisher debug_anc0_point_pub;
ros::Publisher debug_anc1_point_pub;
ros::Publisher debug_anc2_point_pub;
ros::Publisher debug_anc3_point_pub;
ros::Publisher debug_position_pub;
ros::Publisher debug_tag1_pub;
ros::Publisher debug_tag2_pub;
ros::Publisher debug_position_tag1_pub;
ros::Publisher debug_position_tag2_pub;
ros::Publisher debug_naive_mean_estimate_pub;

visualization_msgs::Marker generic_marker;
visualization_msgs::Marker debug_anc0;
visualization_msgs::Marker debug_anc1;
visualization_msgs::Marker debug_anc2;
visualization_msgs::Marker debug_anc3;
Pose debug_position;
visualization_msgs::Marker debug_position_vis;
visualization_msgs::Marker debug_naive_mean_estimate;

std::default_random_engine generator;
std::uniform_real_distribution<double> particle_uniform_prior(-7,7);
std::normal_distribution<double> particle_uniform_drift(0, 0.1d);
std::uniform_real_distribution<double> particle_uniform_resample(0,1);
std::normal_distribution<double> debug_sensor_noise(0.0d,0.2d);
std::uniform_real_distribution<double> particle_angle(0, 2*M_PI);
std::normal_distribution<double> particle_angle_drift(0, 0.05);
visualization_msgs::MarkerArray debug_particles;
std::vector<Pose> particles(PARTICLE_NUMBER);

double AnchorDistance(const Eigen::Vector3d& one, const Eigen::Vector3d& two) {
    return one.adjoint()*two;
}

Eigen::Matrix3d getRotationMatrix(const Eigen::Vector3d& orientation) {
    Eigen::Matrix3d ret;
    ret = Eigen::AngleAxisd(orientation(2), Eigen::Vector3d::UnitZ())
            * Eigen::AngleAxisd(orientation(1), Eigen::Vector3d::UnitY())
            * Eigen::AngleAxisd(orientation(0), Eigen::Vector3d::UnitX());
    return ret;
}

int main(int argc, char** argv) {
    std::cout << "YOLO" << std::endl;
    ros::init(argc, argv, "positionestimation");
    ros::NodeHandle n;
    debug_particle_cloud_pub = n.advertise<visualization_msgs::MarkerArray>("/particle_cloud",10);
    debug_anc0_point_pub = n.advertise<visualization_msgs::Marker>("/anchor0", 10);
    debug_anc1_point_pub = n.advertise<visualization_msgs::Marker>("/anchor1", 10);
    debug_anc2_point_pub = n.advertise<visualization_msgs::Marker>("/anchor2", 10);
    debug_anc3_point_pub = n.advertise<visualization_msgs::Marker>("/anchor3", 10);
    debug_naive_mean_estimate_pub = n.advertise<visualization_msgs::Marker>("/naive_mean_estimate", 10);
    debug_position_pub = n.advertise<visualization_msgs::Marker>("/debug_position", 10);
    debug_tag1_pub = n.advertise<visualization_msgs::Marker>("debug_tag1", 10);
    debug_tag2_pub = n.advertise<visualization_msgs::Marker>("debug_tag2", 10);

    debug_position_tag1_pub = n.advertise<visualization_msgs::Marker>("position_tag1", 10);
    debug_position_tag2_pub = n.advertise<visualization_msgs::Marker>("position_tag2", 10);
    ros::Rate loop_rate(30);

    generic_marker.header.frame_id = "/map";
    generic_marker.header.stamp = ros::Time();
    generic_marker.ns = "my_namespace";
    generic_marker.id = 1001;
    generic_marker.type = visualization_msgs::Marker::SPHERE;
    generic_marker.pose.orientation.x = 0.0;
    generic_marker.pose.orientation.y = 0.0;
    generic_marker.pose.orientation.z = 0.0;
    generic_marker.pose.orientation.w = 1.0;
    generic_marker.scale.x = 0.1;
    generic_marker.scale.y = 0.1;
    generic_marker.scale.z = 0.1;
    generic_marker.color.a = 1.0;
    generic_marker.color.r = 0.0;
    generic_marker.color.g = 1.0;
    generic_marker.color.b = 0.0;

    debug_anc0 = generic_marker;
    debug_anc1 = generic_marker;
    debug_anc2 = generic_marker;
    debug_anc3 = generic_marker;
    debug_position_vis = generic_marker;
    debug_position_vis.color.r = 1.0;
    debug_position_vis.color.g = 0.0;
    debug_naive_mean_estimate = generic_marker;

    debug_anc0.id = 1002;
    debug_anc1.id = 1003;
    debug_anc2.id = 1004;
    debug_anc3.id = 1005;
    debug_position_vis.id = 1006;

    anc0(0) = -5; anc0(1) = -5; anc0(2) = 0;
    anc1(0) = -5; anc1(1) = 5; anc1(2) = 0;
    anc2(0) = 5; anc2(1) = 0; anc2(2) = 0;
    anc3(0) = 0; anc3(1) = 0; anc3(2) = 5;
    debug_position.position(0) = 1; debug_position.position(1) = 0; debug_position.position(2) = 2;
    debug_position.orientation(0) = 0; debug_position.orientation(1) = 0; debug_position.orientation(2) = 0;
    tag1_position(0) = 0; tag1_position(1) = 0.5; tag1_position(2) = 0;
    tag2_position(0) = 0; tag2_position(1) = -0.5; tag2_position(2) = 0;

    debug_anc0.pose.position.x = anc0(0); debug_anc0.pose.position.y = anc0(1); debug_anc0.pose.position.z = anc0(2);
    debug_anc1.pose.position.x = anc1(0); debug_anc1.pose.position.y = anc1(1); debug_anc1.pose.position.z = anc1(2);
    debug_anc2.pose.position.x = anc2(0); debug_anc2.pose.position.y = anc2(1); debug_anc2.pose.position.z = anc2(2);
    debug_anc3.pose.position.x = anc3(0); debug_anc3.pose.position.y = anc3(1); debug_anc3.pose.position.z = anc3(2);
    debug_position_vis.pose.position.x = debug_position.position(0); debug_position_vis.pose.position.y = debug_position.position(1); debug_position_vis.pose.position.z = debug_position.position(2);

    std::vector<visualization_msgs::Marker> prt(PARTICLE_NUMBER, generic_marker);

    std::vector<double> acuWeight(PARTICLE_NUMBER, 0);
    debug_particles.markers = prt;
    short i = 0;
    debug_naive_mean_estimate = generic_marker;
    debug_naive_mean_estimate.color.r = 1.0;
    double naive_mean_estimate_angle = 0;
    std::cout << "YOLO 2" << std::endl;
    for(int i = 0; i < PARTICLE_NUMBER; i++) {
        particles[i].position.fill(0);
        particles[i].orientation.fill(0);
    }

    double sigma2 = 1;
    double theta = 0;
    double r = 3;
    char skip = 0;

    double d1 = 0;
    double d2 = 0;
    double d3 = 0;
    double d4 = 0;
    double d5 = 0;
    double d6 = 0;
    double d7 = 0;
    double d8 = 0;
    double d9 = 0;

    while(ros::ok()) {
        debug_position.position(0) = r*cos(theta);
        debug_position.position(1) = r*sin(theta);
        debug_position.position(2) = r*cos(theta);
        debug_position.orientation(2) = theta;
        //debug_position.orientation(1) = theta;
        debug_position.orientation(0) = theta;
        for(int ddd = 0; ddd < 3; ddd++) {
            if(debug_position.orientation(ddd) > 2*M_PI) {
                debug_position.orientation(ddd)-= 2*M_PI;
            }
            if(debug_position.orientation(ddd) < 0) {
                debug_position.orientation(ddd) += 2*M_PI;
            }
        }
        theta += M_PI/200;
        if(theta >= 2*M_PI) {
            theta -= 2*M_PI;
        }
        if(theta < 0) {
            theta += 2*M_PI;
        }

        debug_position_vis.pose.position.x = debug_position.position(0); debug_position_vis.pose.position.y = debug_position.position(1); debug_position_vis.pose.position.z = debug_position.position(2);

        uavLocation.position.fill(0);
        uavLocation.orientation.fill(0);

        Eigen::Matrix3d R = getRotationMatrix(debug_position.orientation);
        //std::cout << "R: " << R << std::endl;
        Eigen::Vector3d tag1_tmp = debug_position.position + R*tag1_position;
        Eigen::Vector3d tag2_tmp = debug_position.position + R*tag2_position;

        visualization_msgs::Marker tag1_vis = generic_marker;
        tag1_vis.pose.position.x = tag1_tmp(0); tag1_vis.pose.position.y = tag1_tmp(1); tag1_vis.pose.position.z = tag1_tmp(2);

        visualization_msgs::Marker tag2_vis = generic_marker;
        tag2_vis.pose.position.x = tag2_tmp(0); tag2_vis.pose.position.y = tag2_tmp(1); tag2_vis.pose.position.z = tag2_tmp(2);

        if(skip == 5) {
            d1 = AnchorDistance(anc0, tag1_tmp) +  debug_sensor_noise(generator);
            d2 = AnchorDistance(anc1, tag1_tmp) +  debug_sensor_noise(generator);
            d3 = AnchorDistance(anc2, tag1_tmp) +  debug_sensor_noise(generator);
            d4 = AnchorDistance(anc3, tag1_tmp) +  debug_sensor_noise(generator);
            d5 = debug_position.position(2) +  debug_sensor_noise(generator);
            d6 = AnchorDistance(anc0, tag2_tmp) +  debug_sensor_noise(generator);
            d7 = AnchorDistance(anc1, tag2_tmp) +  debug_sensor_noise(generator);
            d8 = AnchorDistance(anc2, tag2_tmp) +  debug_sensor_noise(generator);
            d9 = AnchorDistance(anc3, tag2_tmp) +  debug_sensor_noise(generator);
            skip = 0;
        }
        else {
            skip++;
        }
        double maxValue = -9999999;
        double minValue = 9999999;
        double sumWeight = 0;
        Eigen::Vector3d coses;
        Eigen::Vector3d sines;
        coses.fill(0);
        sines.fill(0);
        for(int i = 0; i < PARTICLE_NUMBER; i++) {
            particles[i].position(0) += particle_uniform_drift (generator);
            particles[i].position(1) += particle_uniform_drift (generator);
            particles[i].position(2) += particle_uniform_drift (generator);

            particles[i].orientation(0) += particle_angle_drift(generator);
            particles[i].orientation(1) += particle_angle_drift(generator);
            particles[i].orientation(2) += particle_angle_drift(generator);

            for(int flubb = 0; flubb < 3; flubb++) {
                if(particles[i].orientation(flubb) > 2*M_PI) {
                    particles[i].orientation(flubb) -= 2*M_PI;
                }
                if(particles[i].orientation(flubb) < 0) {
                    particles[i].orientation(flubb) += 2*M_PI;
                }
            }
            Eigen::Matrix3d R = getRotationMatrix(particles[i].orientation);
            Eigen::Vector3d tg1 = particles[i].position + R*tag1_position;
            Eigen::Vector3d tg2 = particles[i].position + R*tag2_position;

            double w = 0;
            double d = d1 - AnchorDistance(tg1, anc0);
            w += d*d;
            d = d2 - AnchorDistance(tg1, anc1);
            w += d*d;
            d = d3 - AnchorDistance(tg1, anc2);
            w += d*d;
            d = d4 - AnchorDistance(tg1, anc3);
            w += d*d;
            //d = d5 - particles[i].position(2);
            //w += d*d;
            d = d6 - AnchorDistance(tg2, anc0);
            w += d*d;
            d = d7 - AnchorDistance(tg2, anc1);
            w += d*d;
            d = d8 - AnchorDistance(tg2, anc2);
            w += d*d;
            d = d9 - AnchorDistance(tg2, anc3);
            w += d*d;
            w = exp(w*-0.5/sigma2);
            acuWeight[i] = w+sumWeight;

            /*if(false && w < -10) {
            particle->pose.position.x = particle_uniform_prior(generator);
            particle->pose.position.y = particle_uniform_prior(generator);
            particle->pose.position.z = particle_uniform_prior(generator);
            particle->color.a = 0.5f;
        }*/
            uavLocation.position += w*particles[i].position;
            coses(0) += w*cos(particles[i].orientation(0));
            sines(0) += w*sin(particles[i].orientation(0));
            coses(1) += w*cos(particles[i].orientation(1));
            sines(1) += w*sin(particles[i].orientation(1));
            coses(2) += w*cos(particles[i].orientation(2));
            sines(2) += w*sin(particles[i].orientation(2));
            //naive_mean_estimate_angle += w*angle[i];
            sumWeight += w;
        }
        //std::cout << measuredDistance.anc0 << " " << measuredDistance.anc1 << " " << measuredDistance.anc2 << " " << measuredDistance.anc3 << std::endl;
        //std::cout << maxValue << std::endl;
        uavLocation.position /= sumWeight;
        coses /= sumWeight;
        sines /= sumWeight;
        uavLocation.orientation(0) = atan2(sines(0), coses(0));
        uavLocation.orientation(1) = atan2(sines(1), coses(1));
        uavLocation.orientation(2) = atan2(sines(2), coses(2));

        if (sumWeight < 0.5 * PARTICLE_NUMBER) {
            for(int i = 0; i < PARTICLE_NUMBER; i++) {
                acuWeight[i] /= sumWeight;
            }

            /*for(int i = 0; i < PARTICLE_NUMBER; i++) {
            std::cout << "AcuWeight[" << i << "]: " << acuWeight[i] << " ";
            }
            std::cout << std::endl;*/

            std::vector<Pose> ugly = particles;
            for(int j = 0; j < PARTICLE_NUMBER; j++) {
                double randd = particle_uniform_resample(generator);

                /*int i;
                for(i = 0; acuWeight[i] < randd && i < PARTICLE_NUMBER-1; i++) {
                    continue;
                }*/
                int a = PARTICLE_NUMBER-1;
                int b = 0;
                while (b + 5 < a) {
                    int index = (a+b)/2;
                    if(acuWeight[index] > randd) {
                        a = index;
                    }
                    else {
                        b = index;
                    }
                }

                for(b = b; acuWeight[b] < randd && b < PARTICLE_NUMBER-1; b++) {
                    continue;
                }
                /*if( j == 0) {
                    std::cout << i << " " << b << std::endl;
                }*/

                particles[j] = ugly[b];
            }
        }
        debug_naive_mean_estimate.pose.position.x = uavLocation.position(0);
        debug_naive_mean_estimate.pose.position.y = uavLocation.position(1);
        debug_naive_mean_estimate.pose.position.z = uavLocation.position(2);
        R = getRotationMatrix(uavLocation.orientation);
        Eigen::Vector3d tg1_tmp = uavLocation.position + R*tag1_position;
        Eigen::Vector3d tg2_tmp = uavLocation.position + R*tag2_position;

        visualization_msgs::Marker tg1 = debug_naive_mean_estimate;
        visualization_msgs::Marker tg2 = debug_naive_mean_estimate;
        tg1.pose.position.x = tg1_tmp(0); tg1.pose.position.y = tg1_tmp(1); tg1.pose.position.z = tg1_tmp(2);
        tg2.pose.position.x = tg2_tmp(0); tg2.pose.position.y = tg2_tmp(1); tg2.pose.position.z = tg2_tmp(2);

        // Publicera allt
        debug_anc0_point_pub.publish(debug_anc0);
        debug_anc1_point_pub.publish(debug_anc1);
        debug_anc2_point_pub.publish(debug_anc2);
        debug_anc3_point_pub.publish(debug_anc3);
        debug_position_pub.publish(debug_position_vis);
        debug_tag1_pub.publish(tag1_vis);
        debug_tag2_pub.publish(tag2_vis);
        debug_position_tag1_pub.publish(tg1);
        debug_position_tag2_pub.publish(tg2);
        debug_naive_mean_estimate_pub.publish(debug_naive_mean_estimate);
        loop_rate.sleep();
    }
    return 0;
}
