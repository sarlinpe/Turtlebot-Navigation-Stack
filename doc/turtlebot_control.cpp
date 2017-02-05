/* 
 * Title:			EE4308 Lab 2
 * File:			turtlebot_control.cpp
 * Date:			2017-01-27
 * Authors:			Rebecca Thorburn (A0164110Y) and Paul-Edouard Sarlin (A0153124U)
 * Description:		Path controller for the Turlebot using waypoints and PI controllers
 *					for the orientation and the distance. 
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

#include <iostream>
#include <tf/tfMessage.h>
#include <tf/transform_datatypes.h>
#include <cmath>

/* Number of waypoints */
#define NB_PTS			5
/* Correction for a bug in the /odom topic */
#define	YAW_FACTOR		2.19

/* PI coefficients */
#define K_p_orient		1
#define K_p_dist		0.1
#define K_i_orient		5e-4
#define K_i_dist		2e-5

/* Tolerances on the controllers */
#define TOL_ORIENT		0.001
#define TOL_DIST		0.05

using namespace std;
enum CTRL_STATE {ORIENT, MOVE, GOAL};

/* Waypoints, first one being the initial position */
double pts_x[NB_PTS] = {4.0, 7.5, 7.5, 7.5, 4.0};
double pts_y[NB_PTS] = {-5.0, -5, -1.5, 1.0, 1.0};

class vdpgtController{
    
    private:
        ros::Subscriber pos_sub;
        ros::Publisher cmd_vel_pub;

		/* Positions (x, y) and orientation Theta */
        double init_x, init_y, init_theta, pos_y, pos_x, theta;
		int pts_cnt;
		double sum_theta, sum_dist;
		CTRL_STATE ctrl_state;
    
    public:
        vdpgtController(ros::NodeHandle &nh){
			init_x = pts_x[0];
			init_y = pts_y[0];
			init_theta = 0; 	// assume initial orientation is 0 (x axis)
	    	pts_cnt = 1; 		// counter for the waypoints
			sum_theta = 0.0;
			sum_dist = 0.0;
	    	ctrl_state = ORIENT;
            pos_sub = nh.subscribe("/odom",1,&vdpgtController::callback, this);
            cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop",1);
        }

        void callback( const nav_msgs::OdometryConstPtr& poseMsg){
            geometry_msgs::Twist base_cmd;
			double vel_x, vel_z;
			double err_theta = 0.0;
			double err_dist = 0.0;

			/* Get position and orientation in the Gazebo absolute frame */
            pos_x = poseMsg->pose.pose.position.x + init_x;
            pos_y = poseMsg->pose.pose.position.y + init_y;
			theta = poseMsg->pose.pose.orientation.z*YAW_FACTOR + init_theta;

			switch(ctrl_state){

				/* Control the orientation when switching waypoints */
				case ORIENT:
				{
					/* Compute orientation error */
					double goal_theta = atan2(pts_y[pts_cnt]-pos_y,pts_x[pts_cnt]-pos_x);
					err_theta = (goal_theta - theta);

					/* Dirty hack due to incorrect yaw value of /odom in quadrant II */ 
					if (pts_cnt == (NB_PTS - 1)){
						err_theta = YAW_FACTOR - theta;
					}

					/* Check if orientation is satisfactory => now move to it */
					if (err_theta < TOL_ORIENT){
						vel_z = 0.0;
						ctrl_state = MOVE;
						sum_theta = 0.0;
						sum_dist = 0.0;
						cout << "From ORIENT to MOVE" << endl;
					}
					else{
						sum_theta += err_theta; // integrator
						vel_z = K_p_orient*err_theta + K_i_orient*sum_theta;
					}					
					vel_x = 0.0;
					break;
				}

				/* Move towards the next waypoints */
				case MOVE:
				{
					/* Compute orientation and distance errors */
					double goal_theta = atan2(pts_y[pts_cnt]-pos_y,pts_x[pts_cnt]-pos_x);
					err_theta = (goal_theta - theta);
					err_dist = sqrt( pow(pts_x[pts_cnt]-pos_x,2) + pow(pts_y[pts_cnt]-pos_y,2) );

					/* Same dirty hack */
					if (pts_cnt == (NB_PTS - 1)){
						err_theta = YAW_FACTOR - theta;
						/* If we missed the goal: consider it OK */
						if (pos_x < pts_x[NB_PTS-1]){
							err_dist = 0;
						}
					}

					/* Check if distance is satisfactory */
					if (err_dist < TOL_DIST){
						vel_z = 0.0;
						vel_x = 0.0;
						if (pts_cnt == (NB_PTS - 1)){
							ctrl_state = GOAL;
							cout << "GOAL reached!" << endl;
						}
						else{
							pts_cnt++;
							ctrl_state = ORIENT;
							sum_theta = 0.0;
							sum_dist = 0.0;
							cout << "Finished ORIENT, next point: " << pts_cnt << endl;
						}
					}
					else{
						sum_theta += err_theta;
						sum_dist += err_dist;

						vel_z = K_p_orient*err_theta 	+ K_i_orient*sum_theta;
						vel_x = K_p_dist*err_dist 		+ K_i_dist*sum_dist;
					}
					break;
				}

				/* Goal has been reached */
				default:
				{
					vel_x = 0.0;
					vel_z = 0.0;
					sum_theta = 0.0;
					sum_dist = 0.0;
				}
			}
            
			/* Finally send velocity commands */
			base_cmd.linear.x = vel_x;
			base_cmd.angular.z = vel_z;
            cmd_vel_pub.publish(base_cmd);

			if (ctrl_state != GOAL){
				cout << "---" << endl;
           		cout << setprecision(4) << "Vel_x: " << vel_x << "; \t\tVel_z: " << vel_z << endl;
				cout << setprecision(4) << "Err_dist: " << err_dist << "; \tErr_theta: " << err_theta << endl;
				cout << setprecision(4) << "Sum_dist: " << sum_dist << "; \tSum_theta: " << sum_theta << endl;
			}
        }
};

int main(int argc, char** argv){
    ros::init(argc, argv, "curvature_vdpgt");
    ros::NodeHandle nh;
    vdpgtController vc(nh);

    ros::spin();
    return 0;
}
