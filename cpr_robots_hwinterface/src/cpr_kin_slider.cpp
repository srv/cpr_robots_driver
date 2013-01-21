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



#include <cpr_kin_slider.h>



void quit(int sig)
{
  ros::shutdown();
  exit(0);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "CPR-Robots KinSlider");

	cpr_robots::CPRKinSlider hw;
	hw.init();

	ros::spin();
  	signal(SIGINT,quit);	
	return(0);
}



namespace cpr_robots{


	//*************************************************************************************
	CPRKinSlider::CPRKinSlider(){

		jointID_[0] = 0x02;				// standard configuration
		jointID_[1] = 0x04;
		jointID_[2] = 0x06;
		jointID_[3] = 0x08;

		length_ = 0.3;				// values for Slider 100 and 150
		width_ = 0.270;
		diameter_ = 0.154;				// value for Slider 150 only

		double max_vel = 123.0;				// velocity from -123 to 123
		double rpm_at_max_vel = 0.535;			// due to gear ratio and motor characteristics 
		double mps_at_max_vel = rpm_at_max_vel * diameter_ * 3.141;
		scale_translation_ =  max_vel / mps_at_max_vel;

		double complete_rotation = width_ * 3.141;	// Rotational speed given in rad/s
		scale_rotation_ = complete_rotation / (2.0 * 3.141) * scale_translation_;		
		

		flag_sending_ = false;

	}


	//*************************************************************************************
	CPRKinSlider::~CPRKinSlider(){

	}



	//*************************************************************************************
	int CPRKinSlider::init(){
		
		pos_current_.position.x = 0.0;
		pos_current_.position.y = 0.0;
		pos_current_.orientation.z = 0.0;

		itf_.connect();

		ros::Duration(1.0).sleep();
		resetErrors();
		ros::Duration(0.02).sleep();
		enableMotors();
		ros::Duration(0.02).sleep();

		sub_twist_slider_ = n_.subscribe<geometry_msgs::Twist>("/twistSlider", 1, &CPRKinSlider::twistCallback, this);
		odom_slider_pub = n_.advertise<geometry_msgs::Pose>("/poseSlider", 1);

		flag_sending_ = true;
			
		return 0;
	}




	//*************************************************************************************
	int CPRKinSlider::resetErrors(){

		int id = 2;
		int len = 2;
		char data[8];
		data[0] = 0x01;	data[1] = 0x06;		// Code for error reset
		int res = 0;
		
		flag_sending_ = false;
		for(int i=0; i<4; i++){
			id = jointID_[i];
			res = itf_.sendMsg(id, len, data);
			ros::Duration(0.01).sleep();
		}	
		flag_sending_ = true;
		return res;	
	}

	//*************************************************************************************
	int CPRKinSlider::resetPosition(){

		int id = 2;
		int len = 2;
		char data[8];
		data[0] = 0x01;	data[1] = 0x08;		// Code for position reset
		int res = 0;
		flag_sending_ = false;
		for(int i=0; i<4; i++){
			id = jointID_[i];
			res = itf_.sendMsg(id, len, data);
			ros::Duration(0.01).sleep();
		}	

		pos_current_.position.x = 0.0;
		pos_current_.position.y = 0.0;
		pos_current_.orientation.z = 0.0;

		flag_sending_ = true;

		return res;	
	}


	
	//*************************************************************************************
	int CPRKinSlider::enableMotors(){
		
		int id = 2;
		int len = 2;
		char data[8];
		data[0] = 0x01;	data[1] = 0x09;		// Code for enable motors
		int res = 0;
		flag_sending_ = false;
		for(int i=0; i<4; i++){
			id = jointID_[i];
			res = itf_.sendMsg(id, len, data);
			ros::Duration(0.01).sleep();
		}	
		flag_sending_ = true;
		return res;	
	}


	
	//*************************************************************************************
	int CPRKinSlider::disableMotors(){
		int id = 2;
		int len = 2;
		char data[8];
		data[0] = 0x01;	data[1] = 0x0A;		// Code for disable motors
		int res = 0;
		flag_sending_ = false;
		for(int i=0; i<4; i++){
			id = jointID_[i];
			res = itf_.sendMsg(id, len, data);
			ros::Duration(0.05).sleep();
		}	
		flag_sending_ = true;
		return res;	
	}
		


	//*************************************************************************************
	int CPRKinSlider::setVelocities(double * vel){

		if(!flag_sending_)
			return 1;

		int id = 2;
		int len = 3;
		char data[8];
		data[0] = 0x05;	
		data[1] = (char)127.0; 
		data[2] = 0x23;
		int res = 0;
		double v = 0;

		for(int i=0; i<4; i++){
			id = jointID_[i];
			v = 127.0 + vel[i];
			if(v > 250.0) v = 250.0;
			if(v < 7.0) v = 7.0;
			data[1] = (char)((int)v); 
			res = itf_.sendMsg(id, len, data);
			ros::Duration(0.005).sleep();
		}	
		return res;
	}


	//*************************************************************************************
	void CPRKinSlider::twistCallback(const geometry_msgs::Twist::ConstPtr& msg){
		
		double vel_joint[4];
		std::string s;
		double x = msg->linear.x;
		double y = msg->linear.y;
		double a = msg->angular.z;
		//ROS_INFO("Received: %s ", s.c_str()) ;
		//ROS_INFO("Received vx=%.3lf vy=%.3lf rz=%.3lf", x, y, a);

		invKin(msg, &vel_joint[0]);

		if(itf_.getConnectionStatus()){			// send only, if the interface is ok
			setVelocities(&vel_joint[0]);
			int l = 0;
			char d[8];
			int err = 0;
			int curr = 0;		
			for(int i=0; i<4; i++){
				itf_.getLastMessage(jointID_[0]+1, &l, d);
				err =  err | d[0];
			 	curr += d[4];
			}
			updatePosition(msg);
			ROS_INFO("Vel: %.3lf %.3lf %.3lf - Err: %d Curr: %d mA - Pos: %.3lf %.3lf %.3lf", 
				x, y, a, err, curr, 
				pos_current_.position.x, 
				pos_current_.position.y, 
				pos_current_.orientation.z);
				
		}

		odom_slider_pub.publish(pos_current_);
				
	}	

	
	//**************************************************************************************
	void CPRKinSlider::invKin(const geometry_msgs::Twist::ConstPtr& vel_cart, double *vel_joint){

		
		// naming:
		// the front of the slider is the empty side, the back is the side with the main switch
		// front left: vel[2] inverted		front right vel[3]	(looking from behind)		
		// back left: vel[0] inverted		back left: vel[1]

		// forward: all four wheels 
		vel_joint[0] = -scale_translation_ * vel_cart->linear.x;
		vel_joint[1] =  scale_translation_ * vel_cart->linear.x;
		vel_joint[2] = -scale_translation_ * vel_cart->linear.x;
		vel_joint[3] =  scale_translation_ * vel_cart->linear.x;

		// traverse: 
		vel_joint[0] +=  scale_translation_ * vel_cart->linear.y;
		vel_joint[1] +=  scale_translation_ * vel_cart->linear.y;
		vel_joint[2] += -scale_translation_ * vel_cart->linear.y;
		vel_joint[3] += -scale_translation_ * vel_cart->linear.y;

		// rotate: 
		vel_joint[0] +=  scale_rotation_ * vel_cart->angular.z;
		vel_joint[1] +=  scale_rotation_ * vel_cart->angular.z;
		vel_joint[2] +=  scale_rotation_ * vel_cart->angular.z;
		vel_joint[3] +=  scale_rotation_ * vel_cart->angular.z;

		return;

	} 

	
	//*************************************************************************************
	void CPRKinSlider::checkVelocities(double * vel){

		
		return;
	}

	//*************************************************************************************
	void CPRKinSlider::updatePosition(const geometry_msgs::Twist::ConstPtr& vel_cart){

		double duration = 0.1;				// preset at 10 Hz. But this could be more precise...		

		double x_local = vel_cart->linear.x;		// the velocities in the local coordinate system		
		double y_local = vel_cart->linear.y;		
		double rz_local = vel_cart->angular.z;		

		// rotate x and y to the current direction of the platform
		double rz_current = pos_current_.orientation.z;			// not correct! this is a quaternion orientation! to be changed!
		
		double x_global = std::cos(rz_current) * x_local - std::sin(rz_current) * y_local; 	// rotation matrix for xy-plane
		double y_global = std::sin(rz_current) * x_local + std::cos(rz_current) * y_local; 

		pos_current_.position.x += x_global * duration;						// add the global velocites with the according time factor
		pos_current_.position.y += y_global * duration;
		pos_current_.orientation.z += rz_local * duration;
		
		return;
	}






}




