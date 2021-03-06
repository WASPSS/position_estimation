#define PARTICLE_NUMBER 2000
#define K_VALUE  0.00095d
#define M_VALUE 0.00005d
#define NEGHALFSIGMA2 -0.5/0.1

#include <ros/ros.h>
#include <position_estimation/Anchor_msgs.h>
#include <random>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseArray.h>
#include <math.h>
#include <position_estimation/Anchor_msgs.h>
#include <position_estimation/position_estimation.h>

Eigen::Vector3d anc0;
Eigen::Vector3d anc1;
Eigen::Vector3d anc2;
Eigen::Vector3d anc3;
Eigen::Vector3d tag1_position;
Eigen::Vector3d tag2_position;
Pose uavLocation;


measurements measuredDistance;

ros::Publisher debug_particle_cloud_pub;
ros::Publisher debug_anc0_point_pub;
ros::Publisher debug_anc1_point_pub;
ros::Publisher debug_anc2_point_pub;
ros::Publisher debug_anc3_point_pub;
ros::Publisher debug_tag1_pub;
ros::Publisher debug_tag2_pub;
ros::Publisher debug_naive_mean_estimate_pub;

ros::Subscriber tag_listener_sub;

visualization_msgs::Marker generic_marker;
visualization_msgs::Marker debug_anc0;
visualization_msgs::Marker debug_anc1;
visualization_msgs::Marker debug_anc2;
visualization_msgs::Marker debug_anc3;
Pose debug_position;
visualization_msgs::Marker debug_naive_mean_estimate;

std::default_random_engine generator;
std::uniform_real_distribution<double> particle_uniform_prior(0,5);
std::normal_distribution<double> particle_uniform_drift(0, 0.1d);
std::uniform_real_distribution<double> particle_uniform_resample(0,1);
std::normal_distribution<double> debug_sensor_noise(0.0d,0.2d);
std::uniform_real_distribution<double> particle_angle(0, 2*M_PI);
std::normal_distribution<double> particle_angle_drift(0, 0.05);
//std::uniform_real_distribution<double> parameter_drift(-0.00001, 0.00001);
visualization_msgs::MarkerArray debug_particles;
std::vector<Pose> particles(PARTICLE_NUMBER);

void tagListenerCallback(const position_estimation::Anchor_msgs::ConstPtr& msg) {
    if(msg->anc0_t1 > 0) {
        measuredDistance.anc0_t1 = (double) msg->anc0_t1 * K_VALUE + M_VALUE;
    }
    else {
        measuredDistance.anc0_t1 = -1;
    }

    if(msg->anc1_t1 > 0) {
        measuredDistance.anc1_t1 = (double) msg->anc1_t1 * K_VALUE + M_VALUE;
    }
    else {
        measuredDistance.anc1_t1 = -1;
    }

    if(msg->anc2_t1 > 0) {
        measuredDistance.anc2_t1 = (double) msg->anc2_t1 * K_VALUE + M_VALUE;
    }
    else {
        measuredDistance.anc2_t1 = -1;
    }

    if(msg->anc3_t1 > 0) {
        measuredDistance.anc3_t1 = (double) msg->anc3_t1 * K_VALUE + M_VALUE;
    }
    else {
        measuredDistance.anc3_t1 = -1;
    }

    if(msg->anc0_t2 > 0) {
        measuredDistance.anc0_t2 = (double) msg->anc0_t2 * K_VALUE + M_VALUE;
    }
    else {
        measuredDistance.anc0_t2 = -1;
    }

    if(msg->anc1_t2 > 0) {
        measuredDistance.anc1_t2 = (double) msg->anc1_t2 * K_VALUE + M_VALUE;
    }
    else {
        measuredDistance.anc1_t2 = -1;
    }

    if(msg->anc2_t2 > 0) {
        measuredDistance.anc2_t2 = (double) msg->anc2_t2 * K_VALUE + M_VALUE;
    }
    else {
        measuredDistance.anc2_t2 = -1;
    }

    if(msg->anc3_t2 > 0) {
        measuredDistance.anc3_t2 = (double) msg->anc3_t2 * K_VALUE + M_VALUE;
    }
    else {
        measuredDistance.anc3_t2 = -1;
    }
	//std::cout << "megsdf receeveid" << std::endl;
}

int main(int argc, char** argv) {
    std::cout << "POWER" << std::endl;
    ros::init(argc, argv, "positionestimation");
    ros::NodeHandle n;
    debug_particle_cloud_pub = n.advertise<visualization_msgs::MarkerArray>("/particle_cloud",10);
    debug_anc0_point_pub = n.advertise<visualization_msgs::Marker>("/anchor0", 10);
    debug_anc1_point_pub = n.advertise<visualization_msgs::Marker>("/anchor1", 10);
    debug_anc2_point_pub = n.advertise<visualization_msgs::Marker>("/anchor2", 10);
    debug_anc3_point_pub = n.advertise<visualization_msgs::Marker>("/anchor3", 10);
    debug_tag1_pub = n.advertise<visualization_msgs::Marker>("/tag1", 10);
        debug_tag2_pub = n.advertise<visualization_msgs::Marker>("/tag2", 10);
    debug_naive_mean_estimate_pub = n.advertise<visualization_msgs::Marker>("/naive_mean_estimate", 10);

    tag_listener_sub = n.subscribe("/anchor_dist", 1, tagListenerCallback);
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
    debug_naive_mean_estimate = generic_marker;

    debug_anc0.id = 1002;
    debug_anc1.id = 1003;
    debug_anc2.id = 1004;
    debug_anc3.id = 1005;
	/* X, y, z points in order for anchors*/

    anc0(0) = 0.917; anc0(1) = 0.885; anc0(2) = 2.174;
    anc1(0) = 4.667; anc1(1) = 0.98; anc1(2) = 1.955;
    anc2(0) = 4.649; anc2(1) = 2.603; anc2(2) = 1.98;
    anc3(0) = 0.259; anc3(1) = 3.168; anc3(2) = 1.920;
	/* X, y, z points of tags with respect to centre of the UAV on same axis as Anchors*/
    tag1_position(0) = 0; tag1_position(1) = 0.25; tag1_position(2) = 0;
    tag2_position(0) = 0; tag2_position(1) = -0.25; tag2_position(2) = 0;

    debug_anc0.pose.position.x = anc0(0); debug_anc0.pose.position.y = anc0(1); debug_anc0.pose.position.z = anc0(2);
    debug_anc1.pose.position.x = anc1(0); debug_anc1.pose.position.y = anc1(1); debug_anc1.pose.position.z = anc1(2);
    debug_anc2.pose.position.x = anc2(0); debug_anc2.pose.position.y = anc2(1); debug_anc2.pose.position.z = anc2(2);
    debug_anc3.pose.position.x = anc3(0); debug_anc3.pose.position.y = anc3(1); debug_anc3.pose.position.z = anc3(2);

    std::vector<visualization_msgs::Marker> prt(PARTICLE_NUMBER, generic_marker);

    std::vector<double> acuWeight(PARTICLE_NUMBER, 0);
    debug_particles.markers = prt;
    short i = 0;
    debug_naive_mean_estimate = generic_marker;
    debug_naive_mean_estimate.color.r = 1.0;
    double naive_mean_estimate_angle = 0;
    for(int i = 0; i < PARTICLE_NUMBER; i++) {
        particles[i].position.fill(0);
        particles[i].orientation.fill(0);
    }

    Eigen::Vector3d coses;
    Eigen::Vector3d sines;

    Eigen::Matrix3d R;
    while(ros::ok()) {
        ros::spinOnce();
        uavLocation.position.fill(0);
        uavLocation.orientation.fill(0);
        coses.fill(0);
        sines.fill(0);
        double sumWeight = 0;

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

            R = getRotationMatrix(particles[i].orientation);
            Eigen::Vector3d tg1 = particles[i].position + R*tag1_position;
            Eigen::Vector3d tg2 = particles[i].position + R*tag2_position;

            double w = 0;
            double d = 0;
            if(measuredDistance.anc0_t1 > 0) {
                d = measuredDistance.anc0_t1 - AnchorDistance(tg1, anc0);
                w += d*d;
            }
            if(measuredDistance.anc1_t1 > 0) {
                d = measuredDistance.anc1_t1 - AnchorDistance(tg1, anc1);
                w += d*d;
            }
            if(measuredDistance.anc2_t1 > 0) {
                d = measuredDistance.anc2_t1 - AnchorDistance(tg1, anc2);
                w += d*d;
            }
            if(measuredDistance.anc3_t1 > 0) {
                d = measuredDistance.anc3_t1 - AnchorDistance(tg1, anc3);
                w += d*d;
            }
            if(measuredDistance.anc0_t2 > 0) {
                d = measuredDistance.anc0_t2 - AnchorDistance(tg2, anc0);
                w += d*d;
            }
            if(measuredDistance.anc1_t2 > 0) {
                d = measuredDistance.anc1_t2 - AnchorDistance(tg2, anc1);
                w += d*d;
            }
            if(measuredDistance.anc2_t2 > 0) {
                d = measuredDistance.anc2_t2 - AnchorDistance(tg2, anc2);
                w += d*d;
            }
            if(measuredDistance.anc3_t2 > 0) {
                d = measuredDistance.anc3_t2 - AnchorDistance(tg2, anc3);
                w += d*d;
            }
            w = exp(NEGHALFSIGMA2*w);
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
        tg1.color.b = 1.0;
        visualization_msgs::Marker tg2 = debug_naive_mean_estimate;
        tg2.color.r = 0.0;
        tg1.pose.position.x = tg1_tmp(0); tg1.pose.position.y = tg1_tmp(1); tg1.pose.position.z = tg1_tmp(2);
        tg2.pose.position.x = tg2_tmp(0); tg2.pose.position.y = tg2_tmp(1); tg2.pose.position.z = tg2_tmp(2);

        std::cout << "Yaw: " << uavLocation.orientation.transpose() << std::endl;

        // Publicera allt
        debug_anc0_point_pub.publish(debug_anc0);
        debug_anc1_point_pub.publish(debug_anc1);
        debug_anc2_point_pub.publish(debug_anc2);
        debug_anc3_point_pub.publish(debug_anc3);
        debug_tag1_pub.publish(tg1);
        debug_tag2_pub.publish(tg2);
        debug_naive_mean_estimate_pub.publish(debug_naive_mean_estimate);
        loop_rate.sleep();
    }
    return 0;
}


/*int main(int argc, char** argv) {
    ros::init(argc, argv, "positionestimation");
    ros::NodeHandle n;
    particle_cloud_pub = n.advertise<visualization_msgs::MarkerArray>("/particle_cloud",10);
    anc0_point_pub = n.advertise<visualization_msgs::Marker>("/anchor0", 10);
    anc1_point_pub = n.advertise<visualization_msgs::Marker>("/anchor1", 10);
    anc2_point_pub = n.advertise<visualization_msgs::Marker>("/anchor2", 10);
    anc3_point_pub = n.advertise<visualization_msgs::Marker>("/anchor3", 10);
    naive_mean_estimate_pub = n.advertise<visualization_msgs::Marker>("/naive_mean_estimate", 10);
    tag_listener_sub = n.subscribe("/anchor_dist", 1, tagListenerCallback);
    ros::Rate loop_rate(60);

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

    anc0 = generic_marker;
    anc1 = generic_marker;
    anc2 = generic_marker;
    anc3 = generic_marker;

    anc0.id = 1002;
    anc1.id = 1003;
    anc2.id = 1004;
    anc3.id = 1005;

    anc0.pose.position.x = 0; anc0.pose.position.y = 0; anc0.pose.position.z = 0.631;
    anc1.pose.position.x = 2.195; anc1.pose.position.y = 0; anc1.pose.position.z = 0.631;
    anc2.pose.position.x = 2.195; anc2.pose.position.y = 2.696; anc2.pose.position.z = 0.631;
    anc3.pose.position.x = 0; anc3.pose.position.y = 2.696; anc3.pose.position.z = 0.631;
    std::vector<visualization_msgs::Marker> prt(PARTICLE_NUMBER, generic_marker);
    std::vector<double> acuWeight(PARTICLE_NUMBER, 1/PARTICLE_NUMBER);
    particles.markers = prt;
    short i = 0;
    visualization_msgs::Marker naive_mean_estimate = generic_marker;
    visualization_msgs::Marker prev_naive_mean_estimate = naive_mean_estimate;
    naive_mean_estimate.color.r = 1.0;
    int number_of_probable_particles = 0;

    for(auto particle = particles.markers.begin(); particle != particles.markers.end(); ++particle) {
        particle->id = i; i++;
        particle->pose.position.x = particle_uniform_prior(generator);
        particle->pose.position.y = particle_uniform_prior(generator);
        particle->pose.position.z = particle_uniform_prior(generator);
    }

    measuredDistance.anc0 = 0;
    measuredDistance.anc1 = 0;
    measuredDistance.anc2 = 0;
    measuredDistance.anc3 = 0;

    double sigma2 = 0.05;
    while(ros::ok()) {
        ros::spinOnce();
        double delta_x = (prev_naive_mean_estimate.pose.position.x - naive_mean_estimate.pose.position.x);
        double delta_y = (prev_naive_mean_estimate.pose.position.y - naive_mean_estimate.pose.position.y);
        double delta_z = (prev_naive_mean_estimate.pose.position.z - naive_mean_estimate.pose.position.z);

        prev_naive_mean_estimate = naive_mean_estimate;
        naive_mean_estimate.pose.position.x = 0;
        naive_mean_estimate.pose.position.y = 0;
        naive_mean_estimate.pose.position.z = 0;

        particle_cloud_pub.publish(particles);

        double maxValue = -9999999;
        double sumWeight = 0;
        std::cout << "Iteration begin" << std::endl;
        for(int i = 0; i < PARTICLE_NUMBER; i++) {
            particles.markers[i].pose.position.x += particle_uniform_drift (generator) + delta_x;

            particles.markers[i].pose.position.y += particle_uniform_drift (generator) + delta_y;

            particles.markers[i].pose.position.z += particle_uniform_drift (generator) + delta_z;
            double w = 1;
            double d = 0;
            if(measuredDistance.anc0 > 0) {
                d = measuredDistance.anc0 - MarkerDistance(particles.markers[i], anc0);
                w += d*d;
            }
            if(measuredDistance.anc1 > 0) {
                d = measuredDistance.anc1 - MarkerDistance(particles.markers[i], anc1);
                w += d*d;
            }
            if(measuredDistance.anc2 > 0) {
                d = measuredDistance.anc2 - MarkerDistance(particles.markers[i], anc2);
                w += d*d;
            }
            if(measuredDistance.anc3 > 0) {
                d = measuredDistance.anc3 - MarkerDistance(particles.markers[i], anc3);
                w += d*d;
            }
            w = exp(-0.5*w/sigma2);

            acuWeight[i] = w+sumWeight;

            if(w > maxValue) {
                maxValue = w;
            }

            /*if(false && w < -10) {
            particle->pose.position.x = particle_uniform_prior(generator);
            particle->pose.position.y = particle_uniform_prior(generator);
            particle->pose.position.z = particle_uniform_prior(generator);
            particle->color.a = 0.5f;
        }
            naive_mean_estimate.pose.position.x += w*particles.markers[i].pose.position.x;
            naive_mean_estimate.pose.position.y += w*particles.markers[i].pose.position.y;
            naive_mean_estimate.pose.position.z += w*particles.markers[i].pose.position.z;
            number_of_probable_particles++;
            sumWeight += w;
        }

        //std::cout << measuredDistance.anc0 << " " << measuredDistance.anc1 << " " << measuredDistance.anc2 << " " << measuredDistance.anc3 << std::endl;
        std::cout << maxValue << std::endl;
        naive_mean_estimate.pose.position.x /= sumWeight;
        naive_mean_estimate.pose.position.y /= sumWeight;
        naive_mean_estimate.pose.position.z /= sumWeight;

        if (sumWeight < 0.5 * PARTICLE_NUMBER) {
            for(int i = 0; i < PARTICLE_NUMBER; i++) {
                acuWeight[i] /= sumWeight;
            }

            /*for(int i = 0; i < PARTICLE_NUMBER; i++) {
            std::cout << "AcuWeight[" << i << "]: " << acuWeight[i] << " ";
            }
            std::cout << std::endl;

            std::vector<visualization_msgs::Marker> ugly = particles.markers;

            for(int j = 0; j < PARTICLE_NUMBER; j++) {
                double randd = particle_uniform_resample(generator);

                /*int i;
                for(i = 0; acuWeight[i] < randd && i < PARTICLE_NUMBER-1; i++) {
                    continue;
                }
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
                }

                particles.markers[j].pose.position.x = ugly[b].pose.position.x;
                particles.markers[j].pose.position.y = ugly[b].pose.position.y;
                particles.markers[j].pose.position.z = ugly[b].pose.position.z;
            }
        }
        // Publicera allt
        anc0_point_pub.publish(anc0);
        anc1_point_pub.publish(anc1);
        anc2_point_pub.publish(anc2);
        anc3_point_pub.publish(anc3);
        naive_mean_estimate_pub.publish(naive_mean_estimate);
        particle_cloud_pub.publish(particles);
        loop_rate.sleep();
    }
    return 0;
}*/
