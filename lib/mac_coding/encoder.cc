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
 
#include <math.h>
#include <string.h>
#include <boost/crc.hpp>

#include "../utils/debug.h"
#include "../utils/encodings.h"

#include "encoder.h"
#include "ieee/ieee.h"

// general module variables
static enum {IEEE, SENSOR_MIN} d_MODULE_STATE;
static struct metadataPayload *d_metadata;
static int d_channel_width;

// IEEE MAC-encoding
static void IEEE_fragmentation(std::vector<int> &fragment_lengths);			
static int IEEE_get_mpdu_len(int fragment_len);
static void IEEE_generate_mpdu(char *mpdu, const char *fragment, int fragment_len, int frag_no);

// SENSOR_MIN MAC-encoding
static void SENSOR_MIN_fragmentation(std::vector<int> &fragment_lengths);			
static int SENSOR_MIN_get_mpdu_len(int fragment_len);
static void SENSOR_MIN_generate_mpdu(char *mpdu, const char *fragment, int fragment_len);

namespace FBMC_MAC
{	
	//______________________________________________________		
	void load_encoder(struct metadataPayload *metadata, int channel_width)
	{
		d_metadata = metadata;
		d_channel_width	= channel_width;
		
		switch(d_metadata->encoding)
		{
			case IEEE_BPSK_1_2:
			case IEEE_BPSK_3_4:
			case IEEE_QPSK_1_2:
			case IEEE_QPSK_3_4:
			case IEEE_QAM16_1_2:
			case IEEE_QAM16_3_4:
			case IEEE_QAM64_2_3:
			case IEEE_QAM64_3_4:
				d_MODULE_STATE = IEEE;
				break;
		
			case SENSOR_MINIMAL_BPSK_1_2:
			case SENSOR_MINIMAL_BPSK_2_3:
			case SENSOR_MINIMAL_BPSK_3_4:
			case SENSOR_MINIMAL_QPSK_1_2:
			case SENSOR_MINIMAL_QPSK_2_3:
			case SENSOR_MINIMAL_QPSK_3_4:
			case SENSOR_MINIMAL_QPSK_7_8:
			case SENSOR_MINIMAL_QAM16_1_2:
			case SENSOR_MINIMAL_QAM16_3_4:
			case SENSOR_MINIMAL_QAM64_2_3:
			case SENSOR_MINIMAL_QAM64_3_4:
				d_MODULE_STATE = SENSOR_MIN;
				break;
				
			default:
				PRINT("load_encoder MAC stage error: Unknown encoding.");			
		}
	}
	
	//______________________________________________________
	void fragmentation(std::vector<int> &fragment_lengths)
	{
		switch(d_MODULE_STATE)
		{
			case IEEE:
				IEEE_fragmentation(fragment_lengths);
				break;

			case SENSOR_MIN:
				SENSOR_MIN_fragmentation(fragment_lengths);
				break;
		}
	}
	
	//______________________________________________________
	int get_mpdu_len(int fragment_len)
	{
		int mpdu_len = 0;
		
		switch(d_MODULE_STATE)
		{
			case IEEE:
				mpdu_len = IEEE_get_mpdu_len(fragment_len);
				break;
				
			case SENSOR_MIN:
				mpdu_len = SENSOR_MIN_get_mpdu_len(fragment_len);
				break;
		}
		
		return mpdu_len;
	}
		
	//______________________________________________________				 
	void generate_mpdu(char *mpdu, const char *fragment, int fragment_len, int frag_no)
	{
		switch(d_MODULE_STATE)
		{
			case IEEE:
				IEEE_generate_mpdu(mpdu, fragment, fragment_len, frag_no);
				break;
				
			case SENSOR_MIN:
				SENSOR_MIN_generate_mpdu(mpdu, fragment, fragment_len);
				break;
		}
	}
}
 
// IEEE MAC-encoding
//______________________________________________________
void IEEE_fragmentation(std::vector<int> &fragment_lengths)
{
	int n_bpsc;
	int n_cbps;
	int n_dbps;
	
	switch(d_metadata->encoding)
	{
		case IEEE_BPSK_1_2:
			n_bpsc = 1;
			n_cbps = 48;
			n_dbps = 24;
			break;
		case IEEE_BPSK_3_4:
			n_bpsc = 1;
			n_cbps = 48;
			n_dbps = 36;
			break;
		case IEEE_QPSK_1_2:
			n_bpsc = 2;
			n_cbps = 96;
			n_dbps = 48;
			break;
		case IEEE_QPSK_3_4:
			n_bpsc = 2;
			n_cbps = 96;
			n_dbps = 72;
			break;
		case IEEE_QAM16_1_2:
			n_bpsc = 4;
			n_cbps = 192;
			n_dbps = 96;
			break;
		case IEEE_QAM16_3_4:
			n_bpsc = 4;
			n_cbps = 192;
			n_dbps = 144;
			break;
		case IEEE_QAM64_2_3:
			n_bpsc = 6;
			n_cbps = 288;
			n_dbps = 192;
			break;
		case IEEE_QAM64_3_4:
			n_bpsc = 6;
			n_cbps = 288;
			n_dbps = 216;
			break;
	}
	
	// 48 -> PLCP header frequency samples
	// 16 -> service field bits at IEEE PHY-encoding
	// 6  -> tail bits bits at IEEE PHY-encoding
	// fs -> frequency constellations point
	int mac_bytes	= sizeof(IEEE_mac_header) + IEEE_CHECKSUM;
	int total_fs  	= d_metadata->spf*d_channel_width;
	int usable_fs  	= total_fs-48;
	int vec48 		= floor(usable_fs/48);
	int max_bytes 	= floor((vec48*n_dbps-16-6)/8) - mac_bytes;
	
	// security mechanism
	if(max_bytes <= 0)
	{
		PRINT("IEEE_fragmentation error: No net data fits in frame. Frame too small. Dropping data.");
		return;
	}
	
	int max_frames 	= floor(d_metadata->payload_len / max_bytes);
	int left_bytes 	= d_metadata->payload_len % max_bytes;
	
	for(int i=0; i<max_frames; i++)
		fragment_lengths.push_back(max_bytes);
		
	if(left_bytes > 0)
		fragment_lengths.push_back(left_bytes);
}

//______________________________________________________
int IEEE_get_mpdu_len(int fragment_len)
{
	return sizeof(IEEE_mac_header) + fragment_len + IEEE_CHECKSUM;
}

//______________________________________________________
void IEEE_generate_mpdu(char *mpdu, const char *fragment, int fragment_len, int frag_no)
{
	// create mac header
	IEEE_mac_header header;
		
	// fill mac header
	header.frame_control = 0x08;				// random value
	header.duration = 0x2e;					// random value
	memset(header.addr1, 0x00, sizeof(header.addr1));
	memset(header.addr2, 0x00, sizeof(header.addr2));
	memset(header.addr3, 0x00, sizeof(header.addr3));
	memset(header.addr4, 0x00, sizeof(header.addr4));
	header.addr1[4] = d_metadata->sender >> 8;
	header.addr1[5] = d_metadata->sender & 0xFF;
	header.seq = d_metadata->seq_no % 4096;			// seq has to be 12 bit
	header.seq = header.seq << 4;
	header.seq |= ((frag_no % 16) & 0x0F);			// fragment_no has to be 4 bit

	// copy MAC-header into mpdu
	memcpy(mpdu, &header, sizeof(IEEE_mac_header));

	// copy msdu into mpdu
	memcpy(mpdu + sizeof(IEEE_mac_header), fragment, fragment_len);

	// copy checksum into mpdu
	boost::crc_32_type result;
	result.process_bytes(mpdu, sizeof(IEEE_mac_header)+fragment_len);
	unsigned int fcs = result.checksum();
	memcpy(mpdu+sizeof(IEEE_mac_header)+fragment_len, &fcs, IEEE_CHECKSUM);
}

// SENSOR_MIN MAC-encoding
//______________________________________________________
void SENSOR_MIN_fragmentation(std::vector<int> &fragment_lengths)
{	
	int M;
	int numerator;
	int denominator;
	
	switch(d_metadata->encoding)
	{
		case SENSOR_MINIMAL_BPSK_1_2:
			M = 1;
			numerator = 1;
			denominator = 2;
			break;
		case SENSOR_MINIMAL_BPSK_2_3:
			M = 1;
			numerator = 2;
			denominator = 3;
			break;			
		case SENSOR_MINIMAL_BPSK_3_4:
			M = 1;
			numerator = 3;
			denominator = 4;
			break;
		case SENSOR_MINIMAL_QPSK_1_2:
			M = 2;
			numerator = 1;
			denominator = 2;
			break;
		case SENSOR_MINIMAL_QPSK_2_3:
			M = 2;
			numerator = 2;
			denominator = 3;
			break;			
		case SENSOR_MINIMAL_QPSK_3_4:
			M = 2;
			numerator = 3;
			denominator = 4;
			break;
		case SENSOR_MINIMAL_QPSK_7_8:
			M = 2;
			numerator = 7;
			denominator = 8;
			break;
		case SENSOR_MINIMAL_QAM16_1_2:
			M = 4;
			numerator = 1;
			denominator = 2;
			break;
		case SENSOR_MINIMAL_QAM16_3_4:
			M = 4;
			numerator = 3;
			denominator = 4;
			break;
		case SENSOR_MINIMAL_QAM64_2_3:
			M = 6;
			numerator = 2;
			denominator = 3;
			break;
		case SENSOR_MINIMAL_QAM64_3_4:
			M = 6;
			numerator = 3;
			denominator = 4;
			break;
		default:
			PRINT("SENSOR_MIN_fragmentation error: Unknown encoding. Dropping payload.");
			return;			
	}
	
	// check if frame is big enough
	int nBitsGross = M*d_metadata->spf*d_channel_width;	// total number of bits on the air
	int nBitsNetMax = nBitsGross/denominator;		// maximum number of application bits that fit in one frame
	nBitsNetMax *= numerator;				// "
	//int nBytesNetMax = nBitsNetMax/8;			// maximum number of application bytes that fit in one frame
	int nBytesAppNetMax = nBitsNetMax - 6;			// maximum number of application bytes that fit in one frame, 6 trailing zero bits for viterbi decoder
	nBytesAppNetMax /= 8;	
	
	// security mechanism
	if(nBytesAppNetMax <= 0)
	{
		PRINT("SENSOR_MIN_fragmentation error: No net data fits in frame. Frame too small. Dropping payload.");
		return;
	}
	
	// security mechanism
	if(d_metadata->payload_len > nBytesAppNetMax)
	{
		PRINT("SENSOR_MIN_fragmentation error: Too much data for a single frame. Dropping payload.");
		return;
	}	
	
	fragment_lengths.push_back(d_metadata->payload_len);
}

//______________________________________________________
int SENSOR_MIN_get_mpdu_len(int fragment_len)
{
	return fragment_len;
}

//______________________________________________________
void SENSOR_MIN_generate_mpdu(char *mpdu, const char *fragment, int fragment_len)
{
	memcpy(mpdu, fragment, fragment_len);
}
