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

#include "../utils/debug.h"
#include "logical_channel.h"

//#define LOGICAL_CHANNELS_SORT

#ifdef LOGICAL_CHANNELS_SORT
#include <algorithm>
#endif

//______________________________________________________
logical_channel::logical_channel()
{
}

//______________________________________________________
int logical_channel::get_size()
{
	return this->subcarrier.size();
}

//______________________________________________________
int logical_channel::get_min()
{
	return this->subcarrier[0];
}

//______________________________________________________
int logical_channel::get_max()
{
	return this->subcarrier[subcarrier.size()-1];
}

//______________________________________________________
void logical_channels_widths(int32_t *coded_input, int input_length, std::vector<int> &channel_widths)
{
	int index = 0; 
	
	// in each loop extract one width information
	while(index != input_length)
	{
		// subcarriers in logical_channel
		int number_subcarriers = coded_input[index];
		index++;
		
		// append width information
		channel_widths.push_back(number_subcarriers);
		index += number_subcarriers;
	}
}

//______________________________________________________
void logical_channel_convert(int32_t *coded_input, int input_length, std::vector<logical_channel> &logical_channels_array)
{
	int index = 0;
	
	// in each loop create a new entry in "logical_channels_array" 
	while(index != input_length)
	{
		// subcarriers in logical_channel
		int number_subcarriers = coded_input[index];
		index++;
		
		// append new logical channel
		logical_channels_array.push_back(logical_channel());
		int end = logical_channels_array.size()-1;
		
		// fill new logical channel with subcarrier information
		for(int i=0; i<number_subcarriers; i++)
		{
			logical_channels_array[end].subcarrier.push_back(coded_input[index]);
			index++;
		}
		
		// sort logical channels
#ifdef LOGICAL_CHANNELS_SORT
		std::sort (logical_channels_array[end].subcarrier.begin(), logical_channels_array[end].subcarrier.end());
#endif
	}
}

//______________________________________________________
void logical_channel_convert_indices(logical_channel log_chann, std::vector<int> &indices)
{
	bool search = true;
	int curr_index;
	int i = 0;

	while(i < log_chann.get_size())
	{
		// find an even channel index
		if(search == true)
		{
			if(log_chann.subcarrier[i] % 2 == 0)
			{
				indices.push_back(log_chann.subcarrier[i]);
				indices.push_back(1);	
				curr_index = log_chann.subcarrier[i];
				search = false;
			}
			
			i++;
		}
		else
		{
			// check if next channel is consecutive, if not restart search for next even channel
			if(curr_index + 1 == log_chann.subcarrier[i])
			{
				indices[indices.size() - 1]++;
				curr_index++;
				i++;
			}
			else
			{
				search = true;
			}
		}
	}
}