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

#ifndef INCLUDED_FBMC_SENSOR_SCHEDULDER_H
#define INCLUDED_FBMC_SENSOR_SCHEDULDER_H

#include <stdint.h>
#include <string>
#include <vector>

#include "utils/metadata.h"

namespace FBMC_SENSOR_SCHEDU
{
	enum SENSOR_SCHEDULER_ACTION
	{
		SENSOR_IDLE = 0,
		SENSOR_SEND_CONTROL_MESSAGE = 1,
		SENSOR_GENERATE_PAYLOADS = 2
	};
	
	void init_sensor_module(int spf, std::vector<int> logical_channels);
	
	void clean_sensor_module();
	
	void update_time(uint64_t latest_time);
	
	SENSOR_SCHEDULER_ACTION get_sensor_module_state();
	
	void feed_control_data(const char *input, int input_length);
	
	void read_control_data(char *&out_control_stream, int &out_control_stream_length);
	
	/*_________________________________________________
	 * In State "SENSOR_SEND_CONTROL_MESSAGE":
	 * 
	 * 		Functions to create payloads of sensors up to the latest module_time.
	 */
	void get_max_new_payloads(int &n_payload, int &n_payload_bytes);
	
	bool get_payload_param(struct metadataPayload *metadata);
	
	void generate_payload(char *payload);
}

#endif /* INCLUDED_FBMC_SENSOR_SENSOR_SCHEDULDER_H */
