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

#ifndef INCLUDED_FBMC_LOGICAL_CHANNEL_LOGICAL_CHANNEL_H
#define INCLUDED_FBMC_LOGICAL_CHANNEL_LOGICAL_CHANNEL_H

#include <stdint.h>
#include <vector>

#define LOG_CHANN_SUBCCARRIER_DEACT		0
#define LOG_CHANN_SUBCCARRIER_ACTIV		1

class logical_channel
{
	public:
	
		logical_channel();
		std::vector<int> subcarrier;

		int get_size();
		int get_min();
		int get_max();
};

/*
 *  Take coded logical channels and extract channelwidths.
 */
void logical_channels_widths(int32_t *coded_input, int input_length, std::vector<int> &channel_widths);

/*
 *  Take coded logical channels and extract vector which contains the logical channels and their subcarrier.
 */
void logical_channel_convert(int32_t *coded_input, int input_length, std::vector<logical_channel> &logical_channels_array);

/*
 * Take decoded logical channels and write list of continious subcarriers.
 */
void logical_channel_convert_indices(logical_channel log_chann, std::vector<int> &indices);

#endif /* INCLUDED_FBMC_LOGICAL_CHANNEL_LOGICAL_CHANNEL_H */

