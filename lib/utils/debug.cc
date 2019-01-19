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
 
#include <chrono>

#include "debug.h"

//_________________________________________________
fbmc_timer::fbmc_timer(int update_rate_argument)
{
	update_rate = update_rate_argument;
	start_time = std::chrono::system_clock::now().time_since_epoch()/std::chrono::seconds(1);
	latest_time = start_time;
}

//_________________________________________________
bool fbmc_timer::new_update()
{
	uint32_t now = std::chrono::system_clock::now().time_since_epoch()/std::chrono::seconds(1);
	
	if(now - latest_time >= update_rate)
	{	
		latest_time = now;
		passed_time = now - start_time;
	
		return true;
	}
	
	return false;
}

//_________________________________________________
uint32_t fbmc_timer::get_passed_time()
{
	return passed_time;
}
