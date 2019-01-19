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

#ifndef INCLUDED_FBMC_SENSOR_MODE_H
#define INCLUDED_FBMC_SENSOR_MODE_H

#define MODE_SENSORS_FREERUN 0
#define MODE_SENSORS_REFEREN 1

#define ANY_MODE_DATA_TYPE int

#define FREERUN_N_SENSOR_CUSTOM 0
#define FREERUN_N_SENSORS 1

struct mode_freerun_data
{
	ANY_MODE_DATA_TYPE submode;
	uint8_t encoding_family;
	uint8_t encoding;
	uint16_t payload_len;
	int hard_soft_decoding;
	uint16_t interl_type;
};

struct mode_referen_data
{
	uint8_t ref_enco;		// can be the encoding or the encoding family
	uint8_t encoding;
	uint16_t payload_len;
	uint16_t payload_dis;
	int hard_soft_decoding;
	uint16_t interl_type;
	
	int ref2ref_samples;
	int ref_delay_max;
	int case_samples;
	int known_offset;
	int found_unsecu;
	float refer_thresh;
};


#endif /* INCLUDED_FBMC_SENSOR_MODE_H */
