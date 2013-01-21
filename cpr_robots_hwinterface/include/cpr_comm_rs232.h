/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Commonplace Robotics GmbH
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
// Last Update:	


#ifndef cpr_robots_driver_comm_rs232_H
#define cpr_robots_driver_comm_rs232_H

#include <termios.h>
#include <signal.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <boost/asio.hpp>
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>



namespace cpr_robots{

//* Struct to store a CAN message with ID, time stamp and data array
/**
*/
struct msg{
	int id;				// message id
	int length;			// length of data part
	char data[8];			// data
	long time;			// receive time
};


//* Interface to read / write the CAN bus with the Commonplace Robotics USB2CAN bridge
/**
* The USB2CAN bridge is an interface to connect a PC via USB to a CAN field bus.
* The bridge connects via the FTDI virtual com port as a serial port.
* It sends received CAN messages as 10 char messages to the PC.
* The PC sends CAN messages as 10 char messages to the bridge, the bridge forwards it to the CAN bus.
*
* The standard port is defined as /dev/ttyUSB0
* If the CAN bridge connects on another port this has to be corrected in the .cpp file
* Baud rates: 115.200 in the serial line, 500 kb/s on the CAN 
*/
class CPRCommRS232{

	private:

		msg msg_buffer_[256];				/**< Storage for the last CAN message of IDs 0 to 255  */			
		


	public:

		boost::asio::serial_port *port_; 		/**< the serial port this instance is connected to */
		bool flag_connected_;				/**< flag regarding the connection status  */
	
		CPRCommRS232();
		~CPRCommRS232();

		/*!
	    	* \brief	Connects the USB2CAN bridge 
		*/
		int connect();

		/*!
	    	* \brief	Disconnects the USB2CAN bridge 
		*/		
		int disconnect();
		
		/*!
	    	* \brief	Returns the connection status
		*/
		int getConnectionStatus();

		/*!
	    	* \brief	Sends a CAN message to the bus
		*/
		int sendMsg(int id, int length, char data[]);
		
		/*!
	    	* \brief	Provides the last message that was received with this CAN ID
		*/	
		int getLastMessage(int id, int *length, char *data);
		
		/*!
	    	* \brief	Evaluates a 1 char test array to a msg structure
		*/
		int evaluateBuffer(char* buf);


};



}



#endif
