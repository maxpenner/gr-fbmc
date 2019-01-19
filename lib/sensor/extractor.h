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

#ifndef INCLUDED_FBMC_SENSOR_EXTRACTOR_H
#define INCLUDED_FBMC_SENSOR_EXTRACTOR_H

#include <vector>
#include <string>

class extractor
{
	public:
		
		extractor();
		~extractor();
		
		// convert string to byte stream (with prepended length field, 4 bytes)
		void add_length_field(std::string &msg, char *&out_control_stream, int &out_control_stream_length);
		
		// receive byte stream and extract string messages
		void append_bytes(const char* in_stream, int in_stream_length);
		
		// access to msg container
		void get_oldest_message(std::string &msg);
		void del_oldest_message();
		int count_msgs();
	
	private:
		
		std::vector<std::string> messages;
	
		char *residual_bytes;
		int residual_bytes_length;
};

#endif /* INCLUDED_FBMC_SENSOR_EXTRACTOR_H */
