/*
 *
 *	Robot Navigation Challenge
 *
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>

#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <stdio.h>

#include <math.h>
#include <vector>
#include <fstream>
#include <sstream>
#include <sensor_msgs/LaserScan.h>

// Representation (RVIZ)
#include <visualization_msgs/Marker.h>

/**
 * Our class to control the robot
 * It has members to store the robot pose, and
 * methods to control the robot by publishing data
 */

double soi = 1;
//double  = 0.2;

class Turtlebot {
public:
    
    Turtlebot();
    double getR(double gx, double gy);
    bool command(double goal_x, double goal_y, double dir_angle);
    double computeAngulo(double rx, double ry);
    
private:

    ros::NodeHandle nh_;

    //2D robot pose
    double x, y, theta;
    // Scan
    sensor_msgs::LaserScan data_scan;

    //Publisher and subscribers
    ros::Publisher vel_pub_;
    ros::Subscriber kinect_sub_;
    ros::Subscriber pose_sub_;
   
    //double fr[];

    double angleDiff(double dest_angle);
    double computeW(double dest_angle);
    double computeV(double gx, double gy);
    double getDist(double gx, double gy);
    

    void publish(double angular_vel, double linear_vel);
    void receivePose(const nav_msgs::OdometryConstPtr & pose);
    void receiveKinect(const sensor_msgs::LaserScan & laser_kinect);

};

Turtlebot::Turtlebot() {
    
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);
    pose_sub_ = nh_.subscribe("/odom", 1, &Turtlebot::receivePose, this);
    kinect_sub_ = nh_.subscribe("/scan", 1, &Turtlebot::receiveKinect, this);
    
    //fr[0] = 0.0;
    //fr[1] = 0.0;

}

double Turtlebot::getR(double gx, double gy){
    
    double m = 5; // constante
    double den = sqrt(pow(gx - x, 2) + pow(gy - y, 2));
    
    double fax = m * ((gx - x) / den);
    double fay = m * ((gy - y) / den);
    
    //double rx = fax + fr[0];
    double rx = fax;
    //double ry = fay + fr[1];
    double ry = fay;
    
    std::cout << "Fuerza de atraccion X: " << fax << ", Y: " << fay << std::endl;
    std::cout << "Fuerza Resultante Rx: " << rx << ", Ry: " << ry << std::endl;
    
    return computeAngulo(rx, ry);

}




double Turtlebot::angleDiff(double dest_angle) {

    double diff = fmod(dest_angle - theta + 180, 360) - 180;
    return diff < -180 ? diff + 360 : diff;

}


double Turtlebot::computeW(double dest_angle) {

    double diff = angleDiff(dest_angle);
    double base_w = 0.1;
    double w = base_w * diff < base_w ? base_w : base_w * diff;
    return diff >= 0 ? w : -w;

}


double Turtlebot::getDist(double gx, double gy) {

    // distancia euclidea
    double dx = gx - x;
    double dy = gy - y;

    return sqrt((dx * dx) + (dy * dy));

}


double Turtlebot::computeV(double gx, double gy) {

    // distancia euclidea
    double dist = getDist(gx, gy);

    // calculo de la velocidad
    double v_base = 0.1;
    return v_base * dist > v_base ? v_base * dist : v_base;

}


double Turtlebot::computeAngulo(double gx, double gy) {
    return atan2((gy - y), (gx - x));
}


bool Turtlebot::command(double gx, double gy, double dir_angle) {

    bool ret_val = false;
    double linear_vel;
    double angular_vel;
    double margen = 0.05;

    std::cout << "Dif X: " << gx << " - " << x << " = " << gx - x << std::endl;
    std::cout << "Dif Y: " << gy << " - " << y << " = " << gy - y << std::endl;

    if (fabs(gx - x) < margen && fabs(gy - y) < margen) {

        linear_vel = 0.0;
        angular_vel = 0.0;
        ret_val = true;
        std::cout << "Hemos llegado! " << std::endl;

    } else {

        // angulo de R
        double angulo = dir_angle;
        std::cout << "Theta: " << theta << std::endl;
        std::cout << "Angulo: " << angulo << std::endl;

        if (fabs(angulo - theta) < margen) {

            // calculo de V
            linear_vel = computeV(gx, gy);
            // corrección del angualo sobre la marcha
            angular_vel = computeW(angulo);
            std::cout << "Desplazando a  " << linear_vel << " m/s Girando a " << angular_vel << " rad/s" << std::endl;

        } else {

            // calculo de W
            angular_vel = computeW(angulo);
            std::cout << "Girando a " << angular_vel << " rad/s" << std::endl;

        }
    }

    publish(angular_vel, linear_vel);
    return ret_val;
}


void Turtlebot::publish(double angular, double linear) {
    geometry_msgs::Twist vel;
    vel.angular.z = angular;
    vel.linear.x = linear;

    vel_pub_.publish(vel);

    return;
}


void Turtlebot::receivePose(const nav_msgs::OdometryConstPtr& msg) {

    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;

    theta = tf::getYaw(msg->pose.pose.orientation);

    std::cout << "Pose: " << x << ", " << y << " ," << theta << std::endl;


}


void Turtlebot::receiveKinect(const sensor_msgs::LaserScan& msg) {

//    data_scan = msg;
//    
//    double frx = 0;
//    double fry = 0;
//    unsigned i = 0;
//    
//    while (i < data_scan.ranges.size()) {
////        if (data_scan.ranges[i] <= margen_min){
////            
////            // PARA QUE NOS LA PEGAMOS
////        }
//        
//        if (msg.ranges[i] <= soi) {
//            
//            double gamma = msg.angle_min + (msg.angle_increment * i);
//            double w = sin(gamma) * msg.ranges[i];
//            double h = cos(gamma) * msg.ranges[i];
//            
//            double d0 = msg.ranges[i];
//            double m = (soi - d0) / d0;
//            
//            double rx = m * (h / d0);
//            double ry = m * (w / d0);
//            
//            frx = frx + rx;
//            fry = fry + ry;
//            
//        }
//        i++;
//    }
//    
//    fr[0] = frx;
//    fr[1] = fry;
//    std::cout << "Fuerza de Repulsion Total: X " << frx << ", Y: " << fry << std::endl;

}

std::vector<geometry_msgs::Pose> loadPlan(const char *filename);


int main(int argc, char** argv) {
    
    ros::init(argc, argv, "robot_control");
    Turtlebot robot;
    ros::NodeHandle n;

    if(argc<2)
    {
          std::cout << "Insufficient number of parameters" << std::endl;
          std::cout << "Usage: robot_control <filename>" << std::endl;
          return 0;
    }

    std::vector<geometry_msgs::Pose> plan = loadPlan(argv[1]);
    unsigned int cont_wp = 0;
    double resultante;
    ros::Rate loop_rate(20);


    while (ros::ok() && cont_wp < plan.size()){

        ros::spinOnce();
        resultante = robot.getR(plan[0].position.x, plan[0].position.y);
        
        std::cout << "RESULTANTE: " << resultante << std::endl;
        
        bool ret_val = robot.command(plan[0].position.x, plan[0].position.y, resultante);

        if (ret_val == true) {
            cont_wp++;
        }
        
        std::cout << "------------------------------------------\n" << std::endl;
        usleep(1*500000);
    }

    return 0;

}


std::vector<geometry_msgs::Pose> loadPlan(const char *filename) {
    std::vector<geometry_msgs::Pose> plan;
    double x, y;

    std::ifstream is(filename);

    while (is.good()) {
        is >> x;
        if (is.good()) {
            is >> y;
            geometry_msgs::Pose curr_way;
            curr_way.position.x = x;
            curr_way.position.y = y;
            plan.push_back(curr_way);
            ROS_INFO("Loaded waypoint (%f, %f).", x, y);
        }
    }
    ROS_INFO("Plan loaded successfully.");
    return plan;
}

