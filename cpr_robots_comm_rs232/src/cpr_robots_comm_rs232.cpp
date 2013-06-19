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


#include <cpr_robots_comm_rs232.h>



using namespace boost::asio;
using namespace boost::posix_time;
using namespace std;


namespace cpr_robots{


#define COMPORT "/dev/ttyUSB0"



//***************************************************************
// constantly reads the serial line for incoming messages
void readLoop(void * context )
{
	ptime start, now;
	time_duration passed;

	CPRCommRS232 *ctx;
	ctx = (CPRCommRS232*)context;
    	char bu[11];
    	char buffer[11];
    	int bufferCnt = 0;

	for(int i=0; i<11; i++)
		buffer[i] = 0x00;

    	while (true)
    	{

    	/*
    	 * Main Read loop: read one byte in a time
    	 * When a known sender is found the complete CAN message of 10 bytes is read
    	 * Still missing: resync when too much time has passed between the single bytes
    	 */

    	    	if(ctx->flag_connected_){
    			start = microsec_clock::universal_time();
			boost::asio::read(*(ctx->port_), boost::asio::buffer(bu, 1));	// read one byte
			now = microsec_clock::universal_time();
			passed = now - start;

			if(passed.total_milliseconds() > 2){				// we are out of sync!!!
				bufferCnt = 0;						// restart
			}

				buffer[bufferCnt] = bu[0];				// store the char
				bufferCnt++;

				if(bufferCnt == 11){					// when there are 10 chars
					ctx->evaluateBuffer(buffer);			// the message is complete
					bufferCnt = 0;

					for(int i=0; i<11; i++)
						buffer[i] = 0x00;
				}

	    	}
	    }
	    std::cout << "readLoop: finished" << std::endl;
	}


	//*******************************************************************
	CPRCommRS232::CPRCommRS232(){
		flag_connected_ = 0;
	}

	//*******************************************************************
	CPRCommRS232::~CPRCommRS232(){

	}

	//*******************************************************************
	int CPRCommRS232::connect()
	{ 
		io_service io;
		const char *PORT = COMPORT;
		serial_port_base::baud_rate baud_option(460800);
		flag_connected_ = false;
		string s;
		try{
			port_ = new serial_port(io);
			port_->open(PORT);				// open the com port_
			port_->set_option(baud_option); 			// set the baud rate

			flag_connected_ = true;
			std::cerr << "Port " << PORT << " opened\n";
			s = "Port" + string(PORT) + "opened successfully";

		}catch (boost::system::system_error &e){		// description of the failure
			boost::system::error_code ec = e.code();
			s = "Could not open port_ " + string(PORT) + "! "; // + ec.category().name();
			std::cerr << "Cannot open port_ " << PORT << " - error code: " << ec.category().name() << std::endl;
		}catch(std::exception e){
			std::cerr << "Cannot open port_ " << PORT << " - error code: " << e.what() << endl;
			s = "Could not open port_ " + string(PORT) + "! "; // + string(e.what());
		}

		boost::thread readThread(readLoop, (void*)this);	// start the reading loop

		return true;
	}

	//*******************************************************************
	int CPRCommRS232::disconnect()
	{ 
		if(port_->is_open()){
			flag_connected_ = false;
			port_->close();
			std::cout << "Port closed";
		}
		return 0;
	}

	//*******************************************************************
	int CPRCommRS232::getConnectionStatus()
	{ 
		if(port_->is_open())
			return 1;
		else 
			return 0;
	}



	//*******************************************************************
	// send a CAN message
	int CPRCommRS232::sendMsg(int id, int length, char data[])
	{
		unsigned char commands[11] = {16, 4, 4, 125, 125, 0,0,0,0,0,18};
		int sum = 0;

		commands[0] = id;
		commands[1] = length;
		for(int i=0; i<8; i++)
			commands[2+i] = data[i];

		// Build Testbyte
		for(int i=0; i<10; i++)
			sum += commands[i];
		sum = sum % 256;
		commands[10] = sum; 

		if(flag_connected_)
			boost::asio::write(*port_, boost::asio::buffer(commands, 11));

		//ROS_INFO("SendMsg: %d %d - %d %d %d %d %d %d %d %d", (int)commands[0], (int)commands[1], (int)commands[2], (int)commands[3],(int)commands[4] ,
		//	(int)commands[5],(int)commands[6],(int)commands[7],(int)commands[8],(int)commands[9]) ;
		return 0;
	}


	//*******************************************************************
	// Returns the last message received for this ID
	int CPRCommRS232::getLastMessage(int id, int *length, char *data){
		if(id>255)
			throw std::string("invalid message id!");


		length[0] = msg_buffer_[id].length;
		for(int i=0; i<8; i++)
			data[i] = msg_buffer_[id].data[i];
		//m.time = msg_buffer_[id].time;

		return 0;
	}


	//***************************************************************
	/*
	 * Checks for validity of the messages and stores them in a buffer for later access
	 * Validity check here is poor; For performing applications further checks should be incorporated.
	 */
	int CPRCommRS232::evaluateBuffer(char* buf){

		int i = 0;
		int mid = (int)buf[0];
		int length = (int)buf[1];

		//ROS_INFO("GetMsg: %d %d - %d %d %d %d %d %d %d %d", (int)buf[0], (int)buf[1], (int)buf[2], (int)buf[3],(int)buf[4] ,
		//	(int)buf[5],(int)buf[6],(int)buf[7],(int)buf[8],(int)buf[9]) ;

		if(length <= 8){
			msg_buffer_[mid].length = length;
			msg_buffer_[mid].id = mid;
			for(i=0; i<8; i++)
				msg_buffer_[mid].data[i] = buf[i+2];
		}else{
			;
		}

		return 0;
	}








}




