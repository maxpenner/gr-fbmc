/* -*- c++ -*- */
/* 
 * Copyright 2014 <+YOU OR YOUR COMPANY+>.
 * 
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street,
 * Boston, MA 02110-1301, USA.
 */

#include <fstream>
#include <sstream>
#include <algorithm>
#include <ctime>
 
#include "../utils/debug.h"
#include "scheduler.h"
#include "subroutines.h"
#include "sensor.h"
#include "extractor.h"

// state machine
static enum SENSOR_SCHEDULER_INTERNAL_STATE 
{
	FORCE_IDLE = 0,
    SEND_INITIAL_MESSAGE = 1,
    AWAIT_INITIAL_ANSWER = 2,
    SEND_MODE = 3,
    AWAIT_MODE_ANSWER = 4,
    UPDATE_SENSORS = 5,
    SEND_RESET_REQUEST = 6,
    AWAIT_RESET_REQUEST = 7,   
    GENERATE_PAYLOADS = 8,
    SLEEP = 9,
    SEND_LOG_REQUEST = 10,
    AWAIT_LOG_REQUEST = 11
} INT_STATE;

// variables for time
static uint64_t module_time = 0;
static uint64_t module_rel_time = 0;
static uint64_t module_rel_time_offset = 0;

// other variables
extern std::vector<sensor> sensor_array;
static std::string msg2send;
static extractor extrac;

namespace FBMC_SENSOR_SCHEDU
{
	//_________________________________________________
	void init_sensor_module(int spf, std::vector<int> logical_channels)
	{
		if(init_sensor_list(spf, logical_channels) == false)
		{
			PRINT("Unable to init sensor list. Sensor module idles.");
			INT_STATE = FORCE_IDLE;
			return;
		}
		else
		{
			INT_STATE = SEND_INITIAL_MESSAGE;
			
			// set seed for 'real' randomness
			srand(time(0));
		}
	}
	
	//_________________________________________________
	void clean_sensor_module()
	{
	}
	
	//_________________________________________________
	void update_time(uint64_t latest_time)
	{
		module_time = latest_time;
		module_rel_time = module_time - module_rel_time_offset;
	}
	
	//_________________________________________________
	SENSOR_SCHEDULER_ACTION get_sensor_module_state()
	{
		enum SENSOR_SCHEDULER_ACTION ACTION = SENSOR_IDLE;
			
		// depending on internal state of sensor module, perform a certain action
		switch(INT_STATE)
		{
			case FORCE_IDLE:
				break;
			
			case SEND_INITIAL_MESSAGE:
			
				msg2send.assign("TRANSMITTER: HELLO RECEIVER");
				ACTION = SENSOR_SEND_CONTROL_MESSAGE;
				INT_STATE = AWAIT_INITIAL_ANSWER;
				break;
				
			case AWAIT_INITIAL_ANSWER:
			
				// check if message from receiver received
				if(extrac.count_msgs()>0)
				{
					std::string msg;
					extrac.get_oldest_message(msg);
					extrac.del_oldest_message();
					
					if(msg.compare("RECEIVER: HELLO TRANSMITTER") == 0)
					{
						PRINT("Transmitter and Receiver connected.");
						INT_STATE = SEND_MODE;
					}
				}
				break;				
				
			case SEND_MODE:
			
				msg2send.assign("TRANSMITTER: MODE");
				prepare_mode_data(msg2send);
				ACTION = SENSOR_SEND_CONTROL_MESSAGE;
				INT_STATE = AWAIT_MODE_ANSWER;
				break;
				
			case AWAIT_MODE_ANSWER:
			
				// check if message from receiver received
				if(extrac.count_msgs()>0)
				{
					std::string msg;
					extrac.get_oldest_message(msg);
					extrac.del_oldest_message();
					
					if(msg.compare("RECEIVER: MODE INFO RECEIVED") == 0)
						INT_STATE = UPDATE_SENSORS;
				}
				break;				
			
			case UPDATE_SENSORS:

				if(update_sensor_list() == false)
				{
					PRINT("Update sensors ended. Sensor module idles.");
					INT_STATE = FORCE_IDLE;
				}
				else 
					INT_STATE = SEND_RESET_REQUEST;
				break;
			
			case SEND_RESET_REQUEST:

				msg2send.assign("TRANSMITTER: RESET REQUEST");
				ACTION = SENSOR_SEND_CONTROL_MESSAGE;
				INT_STATE = AWAIT_RESET_REQUEST;
				break;
			
			case AWAIT_RESET_REQUEST:
			
				// check if message from receiver received
				if(extrac.count_msgs()>0)
				{
					std::string msg;
					extrac.get_oldest_message(msg);
					extrac.del_oldest_message();
					
					if(msg.compare("RECEIVER: RESET EXECUTED") == 0)
					{
						// reset relative time
						module_rel_time_offset = module_time;
						module_rel_time = 0;
						INT_STATE = GENERATE_PAYLOADS;
					}
				}
				break;
				
			case GENERATE_PAYLOADS:

				if(module_rel_time >= get_runtime())
				{
					module_rel_time_offset = module_time;
					module_rel_time = 0;
					INT_STATE = SLEEP;
				}
				else
					ACTION = SENSOR_GENERATE_PAYLOADS;
				break;
			
			case SLEEP:
			
				if(module_rel_time >= get_sleeptime())
					INT_STATE = SEND_LOG_REQUEST;
				break;
			
			case SEND_LOG_REQUEST:
			
				msg2send.assign("TRANSMITTER: LOG REQUEST");
				ACTION = SENSOR_SEND_CONTROL_MESSAGE;
				INT_STATE = AWAIT_LOG_REQUEST;
				break;
			
			case AWAIT_LOG_REQUEST:
			
				// check if message from receiver received
				if(extrac.count_msgs()>0)
				{
					std::string msg;
					extrac.get_oldest_message(msg);
					extrac.del_oldest_message();
					
					if(msg.compare(0, 21, "RECEIVER: LOGGED DATA") == 0)
					{								
						if(log_data(std::string(msg, 21, msg.length() - 21)) == false)
						{
							PRINT("Unable to save data.");
							PRINT("Loop indices not increased. Retransmitting case.");
						}
						else	
						{	
							increase_loop_indices();
						}
						
						INT_STATE = UPDATE_SENSORS;
					}
				}
				break;
		}
		
		return ACTION;
	}
	
	//_________________________________________________
	void feed_control_data(const char *input, int input_length)
	{
		extrac.append_bytes(input, input_length);
	}
	
	//_________________________________________________
	void read_control_data(char *&out_control_stream, int &out_control_stream_length)
	{	
		extrac.add_length_field(msg2send, out_control_stream, out_control_stream_length);
	}

	//_________________________________________________
	void get_max_new_payloads(int &n_payload, int &n_payload_bytes)
	{
		n_payload = 0;
		n_payload_bytes = 0;
		
		// check how many payloads each single sensor will create until module_time (!) at a max
		for(int i=0; i<sensor_array.size(); i++)
		{	
			if(sensor_array[i].get_airtime() < module_rel_time)
			{
				int total_span = module_rel_time - sensor_array[i].get_airtime_f2f();
				int new_payloads = (total_span - 1)/sensor_array[i].get_f2f_time() + 1 + 1;
			
				n_payload += new_payloads;
				n_payload_bytes += new_payloads*sensor_array[i].get_payload_len();
			}
			else
				break;
		}
	}
	
	//_________________________________________________
	bool get_payload_param(struct metadataPayload *metadata)
	{
		// neccessary since we don't know the exact random_back_off_time
		if(sensor_array[0].get_airtime() >= module_rel_time)
			return false;	
			
		metadata->scale 		= sensor_array[0].get_scale();
		metadata->encoding		= sensor_array[0].get_encoding();
		metadata->sender		= sensor_array[0].get_sensor_ID();
		metadata->log_chann		= sensor_array[0].get_log_chann();
		metadata->seq_no		= sensor_array[0].get_seq_no();
		metadata->payload_len 	= sensor_array[0].get_payload_len();
		metadata->airtime		= sensor_array[0].get_airtime() + module_rel_time_offset;
		metadata->interl_type 	= sensor_array[0].get_interleaving_type();
		metadata->interl_type 	= sensor_array[0].get_interleaving_type();
		metadata->altern_pream  = sensor_array[0].get_alternate_preamble();;
		
		return true;
	}
	
	//_________________________________________________
	void generate_payload(char *payload)
	{
		sensor_array[0].generate_payload(payload);
		
		// update sensor data
		sensor_array[0].inc_seq_no();
		sensor_array[0].inc_frames_send();
		sensor_array[0].inc_airtime();
		
		// sort array by transmission time
		std::sort(sensor_array.begin(), sensor_array.end());
	}
}
