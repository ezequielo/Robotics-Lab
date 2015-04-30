/*
 *
 *           KINECT SENSOR TEST 2
 *       Reaching a single goal avoiding obstacles
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
class Turtlebot {
public:
    Turtlebot();

    /*
     * This function should command the robot to reach the goal
     * It should compute the commands to the robot by knowing the current position
     * and the goal position.
     * This function will return true only if the goal has been reached.
     */
    bool command(double goal_x, double goal_y);
    bool isPathClear();
    float findAltGoal();

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

    
    bool isClear(int read);
    
    //!Publish the command to the turtlebot
    void publish(double angular_vel, double linear_vel);

    //!Callback for robot position
    void receivePose(const nav_msgs::OdometryConstPtr & pose);
    //!Callback for kinect
    void receiveKinect(const sensor_msgs::LaserScan & laser_kinect);


};

Turtlebot::Turtlebot() {
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);

    pose_sub_ = nh_.subscribe("/odom", 1, &Turtlebot::receivePose, this);

    kinect_sub_ = nh_.subscribe("/scan", 1, &Turtlebot::receiveKinect, this);


}


/*
    command 
 */

bool Turtlebot::command(double gx, double gy) {

    double linear_vel = 0.0;
    double angular_vel = 0.0;
    double dist_x, dist_y, distance, prod;
    double angulo;
    bool ret_val = false;


    linear_vel = 0.1;
    
    publish(angular_vel,linear_vel);    

    receiveKinect(data_scan);

    return ret_val;
}



bool Turtlebot::isClear(int k){
    
    bool clear = true;
    if (data_scan.ranges[k] <= 10)
        clear = false;
    
    return clear;
}

/*
 *   findAltGoal finds a pair of alternatives coordenates when an 
 *   obstacle is detected 
 */

float Turtlebot::findAltGoal() {
    
    /*  
     * 
     *      Afecta a la trayectoria??? 
     * 
     */
    
        
    std::cout << "Angle min " << data_scan.angle_min << " rad" << ", " << data_scan.angle_min * (180/M_PI) <<" degrees" << std::endl;
    std::cout << "Angle max " << data_scan.angle_max << " rad" << ", " << data_scan.angle_max * (180/M_PI) <<" degrees" << std::endl;
    
    // Get read #
    
    bool encontrado = false;
    bool obstcl_init;          // true -> osbtacle; false -> clear
    bool obstcl_read;        // true -> osbtacle; false -> clear
    int k;         // laserScan read number
    
    obstcl_init = isClear(0);

    for (int i = 0; i < data_scan.ranges.size(); i++){
        //std::cout << "Lectura #" << i << " Resultado: " << data_scan.ranges[i] << std::endl;
        obstcl_read = isClear(i);
        
        if (obstcl_init =! obstcl_read && encontrado == false){
            bool read_plus1 = isClear(i+1);
            bool read_plus2 = isClear(i+2);
            
            if (read_plus1 == read_plus2 && read_plus1 == obstcl_read) {
                k = i;
                encontrado = true;
            }
        }
    }
    
    std::cout << "Lectura #" << k << std::endl;

    // Get angle
    
    float gamma = data_scan.angle_min + (data_scan.angle_increment * k);
    
    std::cout << "Angulo Gamma" << gamma << " rad" << ", " << gamma * (180/M_PI) <<" degrees" << std::endl;
    
    // Get X point coordenates
    
    float w = sin(gamma) * data_scan.ranges[k];
    float h = cos(gamma) * data_scan.ranges[k];
    
    std::cout << "R coordenates: " << x << ", " << y << std::endl;
    std::cout << "(h, w): " << h << ", " << w << std::endl;
    std::cout << "X coordenates: " << x + h << ", " << y + w << std::endl;

    return 0.0;
}


/*
 *   isPathClear returns true if there is not any obstacle
 */

bool Turtlebot::isPathClear() {

    bool isClear = true;
    float min = 10.0;
    int i = 0;
    
    while (i < data_scan.ranges.size() && isClear == true){
        if (data_scan.ranges[i] < min) 
            isClear = false;
        i++;  
    }
    return isClear;
}


//Publish the command to the turtlebot

void Turtlebot::publish(double angular, double linear) {
    geometry_msgs::Twist vel;
    vel.angular.z = angular;
    vel.linear.x = linear;

    vel_pub_.publish(vel);

    return;
}


//Callback for robot position

void Turtlebot::receivePose(const nav_msgs::OdometryConstPtr& msg) {

    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;

    theta = tf::getYaw(msg->pose.pose.orientation);

    std::cout << "Pose: " << x << ", " << y << " ," << theta << std::endl;


}

//Callback for robot position

void Turtlebot::receiveKinect(const sensor_msgs::LaserScan& msg) {
    
    data_scan=msg;
    /*

    unsigned int num_readings = 100;
    double laser_frequency = 40;
    double ranges[num_readings];
    double intensities[num_readings];

    data_scan=msg;

    std::cout << "\tRANGE: " << msg.ranges.size() << std::endl;

    float min = 10.0;
    float angular_vel = 0.0;
    float linear_vel = 0.5;

    for (unsigned int i = 0; i < msg.ranges.size(); i++ ){

            std::cout << "\tRANGE: " << msg.ranges[i] << std::endl;

            if (msg.ranges[i] < min ){
                    min = msg.ranges[i];
            }

            if (min <= 2.0) {
                    linear_vel = 0.0;
                    angular_vel = 0.3;
                    std::cout << "\tTurtlebot detenido !" << std::endl;
                    std::cout << "\tDistancia al obstaculo: " << min << std::endl;
            }
    }
     */
    //publish(angular_vel,linear_vel);
}

std::vector<geometry_msgs::Pose> loadPlan(const char *filename);
void visualizePlan(const std::vector<geometry_msgs::Pose> &plan, ros::Publisher &marker_pub);


int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_control");
    Turtlebot robot;
    ros::NodeHandle n;

    if (argc < 3) {
        std::cout << "Insuficient number of parameters" << std::endl;
        std::cout << "Usage: c1 Goal_coordinate_x Goal_coordinate_y" << std::endl;
        return 0;
    }

    float xGoal = std::atof(argv[1]);
    float yGoal = std::atof(argv[2]);

    ros::Rate loop_rate(20);

    while (ros::ok()) {

        if (robot.isPathClear() == true){

            std::cout << "Camino hacia el objetivo despejado!" << std::endl;
            ros::spinOnce();
            robot.command(xGoal, yGoal);

        } else {

            std::cout << "El obstaculo obliga a cambiar la trayectoria!" << std::endl;
            float altGoal = robot.findAltGoal();

            //std::cout << "Reconduciendo a las coordenadas: " << altGoal[0] << ", " << altGoal[1] << std::endl;
            //robot.command(altGoal[0], altGoal[1]);
        }
        loop_rate.sleep();
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

void visualizePlan(const std::vector< geometry_msgs::Pose >& plan, ros::Publisher &marker_pub) {
    ros::NodeHandle n;

    uint32_t shape = visualization_msgs::Marker::CUBE;

    for (unsigned int i = 0; i < plan.size(); i++) {
        visualization_msgs::Marker marker;
        // Set the frame ID and timestamp.  See the TF tutorials for information on these.
        marker.header.frame_id = "/odom"; // This is the default fixed frame in order to show the move of the robot
        marker.header.stamp = ros::Time::now();

        // Set the namespace and id for this marker.  This serves to create a unique ID
        // Any marker sent with the same namespace and id will overwrite the old one
        marker.ns = "plan";
        marker.id = i;

        // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
        marker.type = shape;

        // Set the marker action.  Options are ADD and DELETE
        marker.action = visualization_msgs::Marker::ADD;

        // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
        marker.pose.position.x = plan[i].position.x;
        marker.pose.position.y = plan[i].position.y;
        marker.pose.position.z = 0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        // Set the scale of the marker -- 1x1x1 here means 1m on a side
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;

        // Set the color -- be sure to set alpha to something non-zero!
        marker.color.r = 0.0f;
        marker.color.g = 1.0f;
        marker.color.b = 0.0f;
        marker.color.a = 1.0;

        marker.lifetime = ros::Duration(); // Eternal marker

        // Publish the marker
        marker_pub.publish(marker);
    }
}


