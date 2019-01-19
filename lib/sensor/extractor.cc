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
 
#include <stdlib.h>
#include <string.h>

#include "../utils/debug.h"
#include "extractor.h"

#define LENGTH_FIELD_TYPE int

//_________________________________________________
extractor::extractor()
{
	residual_bytes = NULL;
	residual_bytes_length = 0;
}

//_________________________________________________
extractor::~extractor()
{
	if(residual_bytes != NULL)
		free(residual_bytes);
}

//_________________________________________________
void extractor::add_length_field(std::string &msg, char *&out_control_stream, int &out_control_stream_length)
{
	LENGTH_FIELD_TYPE msg_length = msg.length();
	out_control_stream_length = sizeof(LENGTH_FIELD_TYPE) + msg_length;
	out_control_stream = (char*) malloc(out_control_stream_length);
	memcpy(out_control_stream, &msg_length, sizeof(LENGTH_FIELD_TYPE));
	memcpy(out_control_stream + sizeof(LENGTH_FIELD_TYPE), msg.c_str(), msg_length);
}

//_________________________________________________
void extractor::append_bytes(const char* in_stream, int in_stream_length)
{	
	// STEP 1: append new bytes to byte container
	char *temp = (char*) realloc(residual_bytes, residual_bytes_length + in_stream_length);
	
	residual_bytes = temp;
	
	memcpy(residual_bytes + residual_bytes_length, in_stream, in_stream_length);
	
	residual_bytes_length += in_stream_length;
	
	// STEP 2: extract discrete string messages from byte stream

    int offset = 0;

    while(1==1)
    {
        // length available?
        if(residual_bytes_length - offset < sizeof(LENGTH_FIELD_TYPE))
            break;

        // extract length of message
        LENGTH_FIELD_TYPE msg_len;
        memcpy(&msg_len, residual_bytes + offset, sizeof(LENGTH_FIELD_TYPE));

        // entire message available?
        if(residual_bytes_length - offset < sizeof(LENGTH_FIELD_TYPE) + msg_len)
            break;
        
        // extract message
        messages.push_back(std::string(residual_bytes + offset + sizeof(LENGTH_FIELD_TYPE), msg_len));

        // increase offset
        offset += sizeof(LENGTH_FIELD_TYPE) + msg_len;
    }

    // save residual message part
    if(residual_bytes_length > 0)
    {
        residual_bytes_length -= offset;
        
        char old_bytes[residual_bytes_length];
        memcpy(old_bytes, residual_bytes + offset, residual_bytes_length);
        
        char *new_bytes = (char*) realloc(residual_bytes, residual_bytes_length);
        
        residual_bytes = new_bytes;
        
        memcpy(residual_bytes, old_bytes, residual_bytes_length);
    }
    else
    {
        free(residual_bytes);
        residual_bytes = NULL;
    }
}

//_________________________________________________
void extractor::get_oldest_message(std::string &msg)
{
	if(messages.size() > 0)
		msg.assign(messages[0]);
}

//_________________________________________________
void extractor::del_oldest_message()
{
	if(messages.size() > 0)
		messages.erase(messages.begin());
}

//_________________________________________________
int extractor::count_msgs()
{
	return messages.size();
}
