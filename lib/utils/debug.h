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

#ifndef INCLUDED_FBMC_UTILS_DEBUG_H
#define INCLUDED_FBMC_UTILS_DEBUG_H

#include <iostream>

#define PRINT(x) (std::cout << (x) << std::endl)

//______________________________________________________
#define SAMPLE_COLLECTOR_PRINT_RATE 10
#define VALVE_TD_PRINT_RATE 10
//#define VALVE_FD_PRINT_RATE 10
#define PHY_DECODER_SEND_RATE 2
#define MAC_DECODER_PRINT_RATE 5

//______________________________________________________
class fbmc_timer
{
	public:
		fbmc_timer(int update_rate_argument);
		bool new_update();
		uint32_t get_passed_time();
		
	private:
		int update_rate;
		uint32_t start_time;
		uint32_t latest_time;
		uint32_t passed_time;
};

#endif /* INCLUDED_FBMC_UTILS_DEBUG_H */

