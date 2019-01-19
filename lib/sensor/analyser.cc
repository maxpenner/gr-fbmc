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

#include <vector>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <string.h>
 
#include "../utils/debug.h"
#include "analyser.h"
#include "extractor.h"

// mode data
ANY_MODE_DATA_TYPE mode_analyser;
struct mode_freerun_data mode_fre_data;
struct mode_referen_data mode_ref_data;

// container for the sensors in free run mode
static std::vector<sensor_cnt> sensor_cnts_array;

// container for statics in reference mode (extern to access it easily)
struct ReferenceFrameStatistics rfs;

// container for the received bytes of the reference frame
char *reference_bytes = NULL;

// other variables
static extractor extrac;
static std::string msg2send;

namespace FBMC_SENSOR_ANALYSER
{
	//_________________________________________________
	void init_sensor_module()
	{
	}
	
	//_________________________________________________
	void clean_sensor_module()
	{
		free(reference_bytes);
	}
	
	//_________________________________________________
	SENSOR_ANALYSER_ACTION get_sensor_module_state()
	{
		enum SENSOR_ANALYSER_ACTION ACTION = ANALYSER_IDLE;
		
		// check if message from receiver received
		if(extrac.count_msgs()>0)
		{
			std::string msg;

			extrac.get_oldest_message(msg);
			extrac.del_oldest_message();
			
			if(msg.compare("TRANSMITTER: HELLO RECEIVER") == 0)
			{
				msg2send.assign("RECEIVER: HELLO TRANSMITTER");
			
				ACTION = ANALYSER_SEND_CONTROL_MESSAGE;
			}
			else if(msg.compare(0, 17, "TRANSMITTER: MODE") == 0)
			{	
				const char *cstr = msg.c_str();				
				memcpy(&mode_analyser, cstr + 17, sizeof(ANY_MODE_DATA_TYPE));
				
				// extract mode information
				switch(mode_analyser)
				{
					case MODE_SENSORS_FREERUN:
						memcpy(&mode_fre_data, cstr + 17 + sizeof(ANY_MODE_DATA_TYPE), sizeof(mode_freerun_data));
						break;
						
					case MODE_SENSORS_REFEREN:
						memcpy(&mode_ref_data, cstr + 17 + sizeof(ANY_MODE_DATA_TYPE), sizeof(mode_referen_data));
						reference_bytes = (char*) malloc(mode_ref_data.payload_len);
						break;
				}
				
				ACTION = ANALYSER_SEND_VALVE_MESSAGE;
							
				msg2send.assign("RECEIVER: MODE INFO RECEIVED");
			}			
			else if(msg.compare("TRANSMITTER: RESET REQUEST") == 0)
			{
				// reset logging data container
				switch(mode_analyser)
				{
					case MODE_SENSORS_FREERUN:
						sensor_cnts_array.clear();
						break;
						
					case MODE_SENSORS_REFEREN:
						rfs = ReferenceFrameStatistics();
						break;
				}
				
				ACTION = ANALYSER_SEND_VALVE_MESSAGE;
				
				msg2send.assign("RECEIVER: RESET EXECUTED");
			}
			else if(msg.compare("TRANSMITTER: LOG REQUEST") == 0)
			{
				std::string msg("RECEIVER: LOGGED DATA");

				// append logged data
				switch(mode_analyser)
				{
					case MODE_SENSORS_FREERUN:
						for(int i=0; i<sensor_cnts_array.size(); i++)
							msg.append((const char*) &sensor_cnts_array[i], sizeof(sensor_cnt));
						break;
						
					case MODE_SENSORS_REFEREN:								
						msg.append((const char*) &rfs, sizeof(ReferenceFrameStatistics));
						break;
				}				
				
				ACTION = ANALYSER_SEND_CONTROL_MESSAGE;
				
				msg2send.assign(msg);
			}
		}
		
		return ACTION;
	}
	
	//_________________________________________________
	void feed_control_data(const char *input, int input_length)
	{
		extrac.append_bytes(input, input_length);
	}
	
	//_________________________________________________
	void add_sensor_data_freerun_mode(uint16_t sensor_ID, uint16_t seq_no, uint32_t bytes_received, const char *mpdu)
	{
		// in case we use custom mac header, we first have to extract all necessary data from mac header (for IEEE it's done in an earlier stage)
		if(mode_fre_data.submode == FREERUN_N_SENSOR_CUSTOM)
		{
			custom_sensor_mac_header header_temp;
			memcpy(&header_temp, mpdu, sizeof(custom_sensor_mac_header));
			
			sensor_ID = header_temp.sender_id;
			seq_no = header_temp.seq;
			bytes_received = mode_fre_data.payload_len;
		}
		
		int index = -1;
		
		// check if sensor ID already exists
		for(int i=0; i<sensor_cnts_array.size(); i++)
		{
			if(sensor_cnts_array[i].get_sensor_ID() == sensor_ID)
			{
				index = i;
				break;
			}
		}
		
		// if new sensor add it to container
		if(index == -1)
		{
			sensor_cnts_array.push_back(sensor_cnt(sensor_ID));
			index = sensor_cnts_array.size()-1;
		}
		
		// only if new packet
		if(sensor_cnts_array[index].check_seq_no(seq_no) == true)
		{
			// add data
			sensor_cnts_array[index].inc_frames_received();
			sensor_cnts_array[index].inc_bytes_received(bytes_received);
		}
	}
	
	//_________________________________________________
	void read_control_data(char *&out_control_stream, int &out_control_stream_length)
	{
		extrac.add_length_field(msg2send, out_control_stream, out_control_stream_length);
	}
}
