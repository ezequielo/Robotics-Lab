

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/tf.h>

#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Odometry.h>


#include <stdio.h>

#include <math.h>
#include <vector>
#include <fstream>

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

private:


    ros::NodeHandle nh_;

    //2D robot pose
    double x, y, theta;

    //Publisher and subscribers
    ros::Publisher vel_pub_;
    ros::Subscriber pose_sub_;

    // helper methods
    double angleDiff(double dest_angle);
    double computeW(double dest_angle);
    double computeV(double gx, double gy);

    //!Publish the command to the turtlebot
    void publish(double angular_vel, double linear_vel);

    //!Callback for robot position
    void receivePose(const nav_msgs::OdometryConstPtr & pose);

};

Turtlebot::Turtlebot() {
    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);

    pose_sub_ = nh_.subscribe("/odom", 1, &Turtlebot::receivePose, this);

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

double Turtlebot::computeV(double gx, double gy) {

    // distancia euclidea
    double dx = gx - x;
    double dy = gy - y;
    double dist = sqrt((dx * dx) + (dy * dy));

    // calculo de la velocidad
    double v_base = 0.1;
    return v_base * dist > v_base ? v_base * dist : v_base;

}

bool Turtlebot::command(double gx, double gy) {

    bool ret_val;
    double linear_vel;
    double angular_vel;
    double margen = 0.05;

    std::cout << "Dif X: " << gx << " - " << x << " = " << gx - x << std::endl;
    std::cout << "Dif Y: " << gy << " - " << y << " = " << gy - y << std::endl;

    if (fabs(gx - x) < margen && fabs(gy - y) < margen) {

        linear_vel = 0.0;
        angular_vel = 0.0;
        std::cout << "Hemos llegado! " << std::endl;

    } else {

        // calcular angulo
        double angulo = atan2((gy - y), (gx - x));
        std::cout << "Theta: " << theta << std::endl;
        std::cout << "Angulo: " << angulo << " Margen:  " << margen << std::endl;

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




//Publish the command to the turtlebot

void Turtlebot::publish(double angular, double linear) {
    geometry_msgs::Twist vel;
    vel.angular.z = angular;
    vel.linear.x = linear;

    //std::cout << "Velocidades: " << vel.linear.x << ", " << vel.angular.z << std::endl;

    vel_pub_.publish(vel);


    return;
}


//Callback for robot position

void Turtlebot::receivePose(const nav_msgs::OdometryConstPtr & msg) {

    x = msg->pose.pose.position.x;
    y = msg->pose.pose.position.y;

    theta = tf::getYaw(msg->pose.pose.orientation);

    //std::cout << "Pose: " << x << ", " << y << " ," << theta << std::endl;


}

std::vector<geometry_msgs::Pose> loadPlan(const char *filename);

int main(int argc, char** argv) {
    ros::init(argc, argv, "robot_control");
    Turtlebot robot;
    ros::NodeHandle n;

    if (argc < 2) {
        std::cout << "Insufficient number of parameters" << std::endl;
        std::cout << "Usage: robot_control <filename>" << std::endl;
        return 0;
    }

    // vector
    std::vector<geometry_msgs::Pose> plan = loadPlan(argv[1]);
    unsigned int cont_wp = 0;
    ros::Rate loop_rate(20);

    while (ros::ok() && cont_wp < plan.size()) {

        ros::spinOnce();
        bool ret_value = robot.command(plan[cont_wp].position.x, plan[cont_wp].position.y);

        if (ret_value == true) {

            cont_wp++;
            std::cout << "\tGoal # " << cont_wp << " Reached !" << std::endl;
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


