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

#ifndef INCLUDED_FBMC_SENSOR_ANALYSER_H
#define INCLUDED_FBMC_SENSOR_ANALYSER_H

#include "mode.h"
#include "sensor.h"

// can be accessed by sensor block and by analyser
extern ANY_MODE_DATA_TYPE mode_analyser;
extern struct mode_freerun_data mode_fre_data;
extern struct mode_referen_data mode_ref_data;

// container for statics in reference mode (extern to access it easily)
extern struct ReferenceFrameStatistics rfs;

// container for the received bytes of the reference frame
extern char *reference_bytes;

namespace FBMC_SENSOR_ANALYSER
{	
	enum SENSOR_ANALYSER_ACTION
	{
		ANALYSER_IDLE = 0,
		ANALYSER_SEND_CONTROL_MESSAGE = 1,
		ANALYSER_SEND_VALVE_MESSAGE = 2
	};
	
	/*
	 * Initialize internal variables.
	 */	
	void init_sensor_module();
	
	/*
	 * Delete internal variables.
	 */
	void clean_sensor_module();
	
	/*
	 * Return the state of the sensor module.
	 */
	SENSOR_ANALYSER_ACTION get_sensor_module_state();

	//_________________________________________________
	// In either State:

	/*
	 * For messages at control input: Feed module with received bytes.
	 */
	void feed_control_data(const char *input, int input_length);
	
	/*
	 * Add new data about received sensors in freerun mode.
	 */
	void add_sensor_data_freerun_mode(uint16_t sensor_ID, uint16_t seq_no, uint32_t bytes_received, const char *mpdu);
	
	/*
	 * Add new data about received sensors in reference mode.
	 */
	// variable is extern so it can be accessed easily
	
	//_________________________________________________
	// In State ANALYSER_SEND_CONTROL_MESSAGE:
	
	/*
	 * Read control message from module to send it to the transmitter.
	 */
	void read_control_data(char *&out_control_stream, int &out_control_stream_length);
}

#endif /* INCLUDED_FBMC_SENSOR_ANALYSER_H */
