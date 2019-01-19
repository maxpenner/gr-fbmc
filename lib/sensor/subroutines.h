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

#ifndef INCLUDED_FBMC_SENSOR_SUBROUTINES_H
#define INCLUDED_FBMC_SENSOR_SUBROUTINES_H

#include <vector>
#include <stdint.h>
#include <string>

#include "sensor.h"

// container for the sensors
extern std::vector<sensor> sensor_array;

bool init_sensor_list(int spf, std::vector<int> logical_channels);

void prepare_mode_data(std::string &mode_string);

bool update_sensor_list();

uint64_t get_runtime();

uint64_t get_sleeptime();

bool log_data(std::string data);

void increase_loop_indices();

#endif /* INCLUDED_FBMC_SENSOR_SUBROUTINES_H */
