/*
 *
 *           KINECT SENSOR TEST 2
 *       Obstacle avoidance (1 obstacle, clear region visible)
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
    
    bool command(double goal_x, double goal_y);
    bool isPathClear();
    bool alignToGoal(double x, double y);
    std::vector<float> findAltGoal();
    
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
    
    double angleDiff(double dest_angle);
    double computeW(double dest_angle);
    double computeV(double gx, double gy);
    bool isClear(int k);
    int findClearRegion(int k);
    double computeAngulo(double gx, double gy);
    
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

double Turtlebot::computeV(double gx, double gy) {

    // distancia euclidea
    double dx = gx - x;
    double dy = gy - y;
    double dist = sqrt((dx * dx) + (dy * dy));

    // calculo de la velocidad
    double v_base = 0.1;
    return v_base * dist > v_base ? v_base * dist : v_base;

}

double Turtlebot::computeAngulo(double gx, double gy){
    return atan2((gy - y), (gx - x));
}

bool Turtlebot::alignToGoal(double x, double y){
    
    bool align = false;
    double margen = 0.05;
    double angulo = computeAngulo(x, y);
    
    if (fabs(angulo - theta) > margen)
        publish(computeW(angulo), 0.0);
    else
        align = true;
    
    return align;

}


bool Turtlebot::command(double gx, double gy) {

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

        // calcular angulo
        double angulo = computeAngulo(gx, gy);
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

bool Turtlebot::isClear(int k) {
    
    bool clear = true;
    if (data_scan.ranges[k] <= 10)
        clear = false;
    return clear;
}

int Turtlebot::findClearRegion(int k) {
    
    if (isClear(k + 1) == isClear(k + 2) && isClear(k) == isClear(k + 1) && isClear(k + 1) == true)
        return 1;
    else
        return -1;
}

/*
 *   findAltGoal finds a pair of alternatives coordenates when an
 *   obstacle is detected
 */

std::vector<float> Turtlebot::findAltGoal() {
    
    
    
    
    
    /*
     *
     *      Afecta a la trayectoria???
     *
     */
    
    
    std::vector<float> vector(2);
    
    //std::cout << "Angle min " << data_scan.angle_min << " rad" << ", " << data_scan.angle_min * (180/M_PI) <<" degrees" << std::endl;
    //std::cout << "Angle max " << data_scan.angle_max << " rad" << ", " << data_scan.angle_max * (180/M_PI) <<" degrees" << std::endl;
    
    // Get read #
    
    bool encontrado = false;
    bool obstcl_init; // true -> osbtacle; false -> clear
    bool obstcl_read; // true -> osbtacle; false -> clear
    int k = 0; // laserScan read number
    
    obstcl_init = isClear(1);
    
    std::cout << "isClear inicial: " << obstcl_init << std::endl;
    std::cout << "Numero de lecturas : " << data_scan.ranges.size() << std::endl;
    
    int i = 1;
    bool drc;
    
    while (encontrado == false && i < data_scan.ranges.size()) {
        
        obstcl_read = isClear(i);
        //std::cout << "Lectura #" << i << " Resultado: " << data_scan.ranges[i] << std::endl;
        if (obstcl_init != obstcl_read) {
            
            std::cout << "Posible lectura! " << std::endl;
            
            bool read_plus1 = isClear(i + 1);
            bool read_plus2 = isClear(i + 2);
            
            if (read_plus1 == read_plus2 && read_plus1 == obstcl_read) {
                
                if (isClear(i)) {
                    k = i - 1;
                    drc = false; //clear region left
                } else {
                    k = i;
                    drc = true; // clear region right
                }
                
                encontrado = true;
                std::cout << "Lectura encontrada: " << i << " isClear:  " << isClear(i) << std::endl;
            }
        }
        i++;
    }
    
    std::cout << "Lectura #" << k << " Salida: " << data_scan.ranges[k] << std::endl;
    
    // Get angle
    
    double gamma = data_scan.angle_min + (data_scan.angle_increment * k);
    
    std::cout << "Angulo Gamma" << gamma << " rad" << ", " << gamma * (180 / M_PI) << " degrees" << std::endl;
    
    // Get X point coordenates
    
    double w = sin(gamma) * data_scan.ranges[k];
    double h = cos(gamma) * data_scan.ranges[k];
    
    std::cout << "R coordenates: " << x << ", " << y << std::endl;
    std::cout << "(h, w): " << h << ", " << w << std::endl;
    std::cout << "X coordenates: " << x + h << ", " << y + w << std::endl;
    
    // Generate Alt Goal
    
    double ax = x + h;
    double ay;
    
    if (!drc) {
        ay = y + w + 1;
        std::cout << "findClearRegion > 0: IZQ" << std::endl;
        
    } else {
        ay = y - w - 1;
        std::cout << "findClearRegion < 0: DRC" << std::endl;
    }
    
    
    std::cout << "Alt Goal coordenates: " << ax << ", " << ay << std::endl;
    //std::cout << "Alt Goal coordenates: " << vector << std::endl;
    
    vector[0] = ax;
    vector[1] = ay;
    
    return vector;
}

/*
 *   isPathClear returns true if there is not any obstacle
 */

bool Turtlebot::isPathClear() {
    
    receiveKinect(data_scan);
    bool is_clear = true;
    bool osctl_free = true;
    float min = 10.0;
    int i = 0;
    
    while (i < data_scan.ranges.size() && osctl_free == true) {
        if (data_scan.ranges[i] < min)
            osctl_free = false;
        i++;
    }
    
    
    
    
    
    return osctl_free;
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
    
    data_scan = msg;
}


std::vector<geometry_msgs::Pose> loadPlan(const char *filename);
void visualizePlan(const std::vector<geometry_msgs::Pose> &plan, ros::Publisher &marker_pub);

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_control");
    Turtlebot robot;
    ros::NodeHandle n;
    
    if (argc < 2) {
        std::cout << "Insufficient number of parameters" << std::endl;
        std::cout << "Usage: robot_control <filename>" << std::endl;
        return 0;
    }
    
    std::vector<geometry_msgs::Pose> plan = loadPlan(argv[1]);
    unsigned int cont_wp = 0;
    std::vector<float> alt_goal;
    bool alt = false;
    bool new_dest = true;
    bool align = false;
    double dest[] = {-1, plan[0].position.x, plan[0].position.y};
    
    ros::Rate loop_rate(20);
     
    while (ros::ok() && cont_wp < plan.size()) {
        
        ros::spinOnce();
        
        if (align == false) {
            
            std::cout << "Alineando Turtlebot!" << std::endl;
            align = robot.alignToGoal(dest[1], dest[2]);
        
        } else {
        
                
            if (robot.isPathClear() == true && new_dest == true) {

                std::cout << "Camino hacia el objetivo despejado!" << std::endl;
                dest[0] = -1;
                dest[1] = plan[cont_wp].position.x;
                dest[2] = plan[cont_wp].position.y;

                new_dest == false;

            } else if (robot.isPathClear() == false && new_dest == true){

                std::cout << "El obstaculo obliga a cambiar la trayectoria!" << std::endl;

                if (alt == false){
                    alt_goal = robot.findAltGoal();
                    alt = true;
                }

                std::cout << "Reconduciendo a las coordenadas: " << alt_goal[0] << ", " << alt_goal[1] << std::endl;

                dest[0] = 1;
                dest[1] = (double) alt_goal[0];
                dest[2] = (double) alt_goal[1];

                align = false;
                new_dest = false;
            }
            
            bool ret_value = robot.command(dest[1], dest[2]);
            if (ret_value == true){
                
                if (dest[0] > 0)
                    alt = false;
                else
                    cont_wp++;
                
                align = false;
                new_dest = true;
            }
        }
        
        //usleep(1*500000);
        std::cout << "------------------------------------------\n" << std::endl;
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
