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

#include "debug.h"
#include "encodings.h"
#include "constellations.h"

//_________________________________________________ (normalized to avg power 1)
static const std::complex<float> BPSK_D[2] = 
{
	std::complex<float>(-1.0, 0.0), std::complex<float>(1.0, 0.0)
};

//_________________________________________________ (normalized to avg power 1)
static const std::complex<float> QPSK_D[4] = 
{
	std::complex<float>(-0.7071, -0.7071), std::complex<float>(-0.7071, 0.7071),
	std::complex<float>(+0.7071, -0.7071), std::complex<float>(+0.7071, 0.7071)
};

//_________________________________________________ (normalized to avg power 1)
static const std::complex<float> QAM16_D[16] = 
{
	std::complex<float>(-0.9487, -0.9487), std::complex<float>(-0.9487, -0.3162),
	std::complex<float>(-0.9487, 0.9487), std::complex<float>(-0.9487, 0.3162),
	std::complex<float>(-0.3162, -0.9487), std::complex<float>(-0.3162, -0.3162),
	std::complex<float>(-0.3162, 0.9487), std::complex<float>(-0.3162, 0.3162),
	std::complex<float>(0.9487, -0.9487), std::complex<float>(0.9487, -0.3162),
	std::complex<float>(0.9487, 0.9487), std::complex<float>(0.9487, 0.3162),
	std::complex<float>(0.3162, -0.9487), std::complex<float>(0.3162, -0.3162),
	std::complex<float>(0.3162, 0.9487), std::complex<float>(0.3162, 0.3162)
};

//_________________________________________________ (normalized to avg power 1)
static const std::complex<float> QAM64_D[64] = 
{
	std::complex<float>(-1.0801, -1.0801), std::complex<float>(-1.0801, -0.7715),
	std::complex<float>(-1.0801, -0.1543), std::complex<float>(-1.0801, -0.4629),
	std::complex<float>(-1.0801, 1.0801), std::complex<float>(-1.0801, 0.7715),
	std::complex<float>(-1.0801, 0.1543), std::complex<float>(-1.0801, 0.4629),
	std::complex<float>(-0.7715, -1.0801), std::complex<float>(-0.7715, -0.7715),
	std::complex<float>(-0.7715, -0.1543), std::complex<float>(-0.7715, -0.4629),
	std::complex<float>(-0.7715, 1.0801), std::complex<float>(-0.7715, 0.7715),
	std::complex<float>(-0.7715, 0.1543), std::complex<float>(-0.7715, 0.4629),
	std::complex<float>(-0.1543, -1.0801), std::complex<float>(-0.1543, -0.7715),
	std::complex<float>(-0.1543, -0.1543), std::complex<float>(-0.1543, -0.4629),
	std::complex<float>(-0.1543, 1.0801), std::complex<float>(-0.1543, 0.7715),
	std::complex<float>(-0.1543, 0.1543), std::complex<float>(-0.1543, 0.4629),
	std::complex<float>(-0.4629, -1.0801), std::complex<float>(-0.4629, -0.7715),
	std::complex<float>(-0.4629, -0.1543), std::complex<float>(-0.4629, -0.4629),
	std::complex<float>(-0.4629, 1.0801), std::complex<float>(-0.4629, 0.7715),
	std::complex<float>(-0.4629, 0.1543), std::complex<float>(-0.4629, 0.4629),
	std::complex<float>(1.0801, -1.0801), std::complex<float>(1.0801, -0.7715),
	std::complex<float>(1.0801, -0.1543), std::complex<float>(1.0801, -0.4629),
	std::complex<float>(1.0801, 1.0801), std::complex<float>(1.0801, 0.7715),
	std::complex<float>(1.0801, 0.1543), std::complex<float>(1.0801, 0.4629),
	std::complex<float>(0.7715, -1.0801), std::complex<float>(0.7715, -0.7715),
	std::complex<float>(0.7715, -0.1543), std::complex<float>(0.7715, -0.4629),
	std::complex<float>(0.7715, 1.0801), std::complex<float>(0.7715, 0.7715),
	std::complex<float>(0.7715, 0.1543), std::complex<float>(0.7715, 0.4629),
	std::complex<float>(0.1543, -1.0801), std::complex<float>(0.1543, -0.7715),
	std::complex<float>(0.1543, -0.1543), std::complex<float>(0.1543, -0.4629),
	std::complex<float>(0.1543, 1.0801), std::complex<float>(0.1543, 0.7715),
	std::complex<float>(0.1543, 0.1543), std::complex<float>(0.1543, 0.4629),
	std::complex<float>(0.4629, -1.0801), std::complex<float>(0.4629, -0.7715),
	std::complex<float>(0.4629, -0.1543), std::complex<float>(0.4629, -0.4629),
	std::complex<float>(0.4629, 1.0801), std::complex<float>(0.4629, 0.7715),
	std::complex<float>(0.4629, 0.1543), std::complex<float>(0.4629, 0.4629)
};

//_________________________________________________
void get_scaled_constellation(int encoding, float scale, std::vector<std::complex<float>> &constellation)
{	
	switch(encoding) 
	{
		case IEEE_BPSK_1_2:
		case IEEE_BPSK_3_4:	
		case SENSOR_MINIMAL_BPSK_1_2:
		case SENSOR_MINIMAL_BPSK_2_3:
		case SENSOR_MINIMAL_BPSK_3_4:
		
			for(int i=0; i<2; i++)
				constellation.push_back(scale*BPSK_D[i]);
			constellation.push_back(std::complex<float>(0.0, 0.0));				
			break;
			
		case IEEE_QPSK_1_2:
		case IEEE_QPSK_3_4:	
		case SENSOR_MINIMAL_QPSK_1_2:
		case SENSOR_MINIMAL_QPSK_2_3:
		case SENSOR_MINIMAL_QPSK_3_4:			
		case SENSOR_MINIMAL_QPSK_7_8:
			for(int i=0; i<4; i++)
				constellation.push_back(scale*QPSK_D[i]);
			constellation.push_back(scale*BPSK_D[0]);
			constellation.push_back(scale*BPSK_D[1]);
			constellation.push_back(std::complex<float>(0.0, 0.0));			
			break;
			
		case IEEE_QAM16_1_2:
		case IEEE_QAM16_3_4:
		case SENSOR_MINIMAL_QAM16_1_2:
		case SENSOR_MINIMAL_QAM16_3_4:
		
			for(int i=0; i<16; i++)
				constellation.push_back(scale*QAM16_D[i]);
			constellation.push_back(scale*BPSK_D[0]);
			constellation.push_back(scale*BPSK_D[1]);
			constellation.push_back(std::complex<float>(0.0, 0.0));			
			break;
			
		case IEEE_QAM64_2_3:
		case IEEE_QAM64_3_4:	
		case SENSOR_MINIMAL_QAM64_2_3:
		case SENSOR_MINIMAL_QAM64_3_4:
		
			for(int i=0; i<64; i++)
				constellation.push_back(scale*QAM64_D[i]);
			constellation.push_back(scale*BPSK_D[0]);
			constellation.push_back(scale*BPSK_D[1]);
			constellation.push_back(std::complex<float>(0.0, 0.0));			
			break;
			
		default:
			PRINT("get_scaled_constellation error: Unknown encoding. Setting to maximum constellation of QAM-64 to avoid segmentation fault. But frame is nonsense.");
			for(int i=0; i<64; i++)
				constellation.push_back(scale*QAM64_D[i]);
			constellation.push_back(scale*BPSK_D[0]);
			constellation.push_back(scale*BPSK_D[1]);
			constellation.push_back(std::complex<float>(0.0, 0.0));			
			break;
		
	}
}
