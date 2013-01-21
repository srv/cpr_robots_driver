/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Commonplace Robotics GmbH
 *  http://www.commonplacerobotics.com
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name Commonplace Robotics nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
// Created on: 	Jan 12th, 2013
// Last Update:	Jan 21st, 2013


#ifndef cpr_robots_driver_kin_slider_H
#define cpr_robots_driver_kin_slider_H

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>

#include <cpr_comm_rs232.h>


namespace cpr_robots{



//* Kinematic for mobile Platform Slider
/**
* This class holds the kinematic for the mobile platform Slider from Commonplace Robotics
* It subscribes to twist messages, transforms 
* these messages to joint speeds for the four mecanum
* wheels and then forwards these joint speeds to CPRCommRS232 to
* put them on the CAN bus.
*/
class CPRKinSlider{

	private:

		int jointID_[4];				/**< the CAN ids of the joint controller */
		double length_;					/**< the length of the platform (distance of the axles in m) */
		double width_;					/**< the width of the platform (distance of the wheel centers in m) */
		double diameter_;				/**< the diameter of the wheels in m */

		double scale_translation_;			/**< factor to compute the velocity command */
		double scale_rotation_;				/**< factor to compute the velocity command */

		bool flag_sending_;				/**< flag if the main loop shall send vel command to the hardware */
			
		geometry_msgs::Twist cmd_twist_;		/**< the current twist command */
		geometry_msgs::Pose pos_current_;		/**< the current position in world coordinates */

	  	ros::NodeHandle n_;
		ros::Subscriber sub_twist_slider_; 		/**< .. */
		ros::Publisher odom_slider_pub;			/**< publishes the current position  */		
		
		/*!
    		* \brief	Callback for the twist messges 
		* scales the velocitites and forwards them to the hardware
		*/
		void twistCallback(const geometry_msgs::Twist::ConstPtr& msg);

		/*!
    		* \brief	Check velocities and accelerations 
		* Limits the velocities and accelerations to max values
		*/
		void checkVelocities(double * vel);

		/*!
	    	* \brief	Forwards the velocities to the CAN hardware 
		*/
		int setVelocities(double * vel);
		
		/*!
	    	* \brief	Computes the joint values of the mecanum wheels out of the cartesions x-y-rz velocites
		*	x and y velocity in m/s, rz velocity in rad/s 
		*/
		void invKin(const geometry_msgs::Twist::ConstPtr& vel_cart, double vel_joint[]);


		/*!
	    	* \brief	Tracks the position in world coordinates (odometry)
		*/
		void updatePosition(const geometry_msgs::Twist::ConstPtr& vel_cart);

		CPRCommRS232 itf_;			/**< the interface to the hardware  */

	public:
		CPRKinSlider();
		~CPRKinSlider();
		/*!
	    	* \brief	General initialization
		*/		
		int init();

		/*!
	    	* \brief	Resets the errors of all four joint controllers
		*/
		int resetErrors();
		
		/*!
	    	* \brief	Resets the joint controllers position to a mid position 
		*/
		int resetPosition();

		/*!
	    	* \brief	Enables the motors in the controller hardware 
		*/
		int enableMotors();

		/*!
	    	* \brief	Disables the controllers in the motor hardware
		*/
		int disableMotors();
		

};



}



#endif
