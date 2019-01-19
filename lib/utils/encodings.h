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

#ifndef INCLUDED_FBMC_UTILS_ENCODINGS_H
#define INCLUDED_FBMC_UTILS_ENCODINGS_H

// PHY + MAC coupled
#define	IEEE_BPSK_1_2 	0
#define	IEEE_BPSK_3_4  	1
#define	IEEE_QPSK_1_2  	2
#define	IEEE_QPSK_3_4  	3
#define	IEEE_QAM16_1_2 	4
#define	IEEE_QAM16_3_4 	5
#define	IEEE_QAM64_2_3 	6
#define	IEEE_QAM64_3_4 	7

// SENSOR_MINIMAL: 
// The number of subchannels is fixed to a multiple of 48.
// 	1. no MAC bytes in MAC-encoder
// 	2. raw bits -> convolutional coder -> interleaving as in IEEE (therfore a multiple 48)
#define SENSOR_MINIMAL_BPSK_1_2		8
#define SENSOR_MINIMAL_BPSK_2_3		9
#define SENSOR_MINIMAL_BPSK_3_4		10
#define	SENSOR_MINIMAL_QPSK_1_2  	11
#define	SENSOR_MINIMAL_QPSK_2_3  	12
#define	SENSOR_MINIMAL_QPSK_3_4  	13
#define	SENSOR_MINIMAL_QPSK_7_8  	14
#define	SENSOR_MINIMAL_QAM16_1_2 	15
#define	SENSOR_MINIMAL_QAM16_3_4 	16
#define	SENSOR_MINIMAL_QAM64_2_3 	17
#define	SENSOR_MINIMAL_QAM64_3_4 	18

#endif /* INCLUDED_FBMC_UTILS_ENCODINGS_H */
