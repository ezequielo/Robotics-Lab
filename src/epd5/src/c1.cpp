

#include <ros/ros.h>

#include <tf/tf.h>
#include <nav_msgs/Odometry.h>

#include <math.h>
#include <stdio.h>


/**
* Our class to control the robot
* It has members to store the robot pose, and
* methods to control the robot by publishing data
*/
class Turtlebot
{
public:
  Turtlebot();
 

  /**
   * This function should command the robot to reach the goal
   * It should compute the commands to the robot by knowing the current position
   * and the goal position
   */
  void command(double goal_x, double goal_y);

private:

  
  ros::NodeHandle nh_;
  
  //2D robot pose
  double x,y,theta;

  //Publisher and subscribers
  ros::Publisher vel_pub_;
  ros::Subscriber pose_sub_;

  //!Publish the command to the turtlebot
  void publish(double angular_vel, double linear_vel);

  //!Callback for robot position
  void receivePose(const nav_msgs::OdometryConstPtr & pose);

};

Turtlebot::Turtlebot()
{
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 1);

  pose_sub_ = nh_.subscribe("/odom",1,&Turtlebot::receivePose,this);

}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "robot_control");
  Turtlebot robot;
  ros::NodeHandle n;

  if(argc<3)
  {
	std::cout << "Insuficient number of parameters" << std::endl;
	std::cout << "Usage: c1 Goal_coordinate_x Goal_coordinate_y" << std::endl;
	return 0;
  }

  double xGoal=5.0, yGoal=3.0;

  xGoal=std::atof(argv[1]);
  yGoal=std::atof(argv[2]);

  ros::Rate loop_rate(20);

  while (ros::ok())
  {
   
    ros::spinOnce();

    robot.command(xGoal,yGoal);

    loop_rate.sleep();
   
  }


  return 0;

}


void Turtlebot::command(double gx, double gy)  
{

	double linear_vel=0.0;
	double angular_vel=0.0;

	double margen = 0.05;
	double v_base = 0.1;
	
	std::cout << "Dif X: " << gx << " - " << x << " = " << gx-x << std::endl;
	std::cout << "Dif Y: " << gy << " - " << y << " = " << gy-y << std::endl;

	if (fabs(gx-x)<margen && fabs(gy-y)<margen){
		
		std::cout << "Hemos llegado! " << std::endl;

		linear_vel = 0.0;
		angular_vel = 0.0;

	} else {

		// calcular angulo
		
		double angulo = atan2((gy-y),(gx-x));

		std::cout << "Tan: " << tan << std::endl;
		std::cout << "Theta: " << theta << std::endl;
		std::cout << "Angulo: " << angulo << " Margen:  " << margen << std::endl;
		
		if (fabs(angulo-theta) < margen){			
			
			// desplazamiento mejorado
			// la velocidad depende de la distancia al goal

			
			// distancia euclidea

			double dx = gx - x;
			double dy = gy - y;
			double dist = sqrt((dx*dx) + (dy*dy));

			// calculo de la velocidad

			if (v_base * dist > v_base) {

				linear_vel = v_base * dist;
 
			} else {

				linear_vel = v_base;

			}
			
			std::cout << "Distancia al objetivo: " << dist << std::endl;
			std::cout << "Desplazando a  " << linear_vel << " m/s" << std::endl;

		} else {

			// rotacion mejorada
			// gira en sentido del angulo menor
			std::cout << "Theta: " << theta << std::endl;
			std::cout << "Angulo: " << angulo << " Margen:  " << margen << std::endl;
			
			if ( angulo < 0) {
				
				// transform angulo
				angulo = (2 * M_PI) + angulo;
			
			}

			if (theta < -0.1) {
				
				// transform theta
				theta = (2 * M_PI) + theta;

			}

			std::cout << "Theta Trans: " << theta << std::endl;
			std::cout << "Angulo Trans: " << angulo << " Margen:  " << margen << std::endl;

			if ((angulo-theta) > 0 ) {

				angular_vel = 0.1;
				std::cout << "Rotando IZQ! " << std::endl;

			} else {

				angular_vel = -0.1;
				std::cout << "Rotando DRA! " << std::endl;

			}
			
		}
	}

   	publish(angular_vel,linear_vel);    


  return;
}



//Publish the command to the turtlebot
void Turtlebot::publish(double angular, double linear)  
{
    geometry_msgs::Twist vel;
    vel.angular.z = angular;
    vel.linear.x = linear;

    vel_pub_.publish(vel);    


  return;
}


//Callback for robot position
void Turtlebot::receivePose(const nav_msgs::OdometryConstPtr& msg)
{

	x=msg->pose.pose.position.x;
	y=msg->pose.pose.position.y;

	theta=tf::getYaw(msg->pose.pose.orientation);

	std::cout << "Pose: " << x << ", " << y << " ," << theta << std::endl;


}



