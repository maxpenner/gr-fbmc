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
 
#include <string.h>
#include <boost/crc.hpp>
#include <stdint.h>

#include "../utils/debug.h"
#include "../utils/encodings.h"

#include "decoder.h"
#include "ieee/ieee.h"

// general module variables
static enum {IEEE} d_MODULE_STATE;		
static int d_mpdu_len;
static int d_encoding_family;

// IEEE	MAC-decoding
static int IEEE_get_payload_len(const char *mpdu);
static uint16_t IEEE_get_sender_id(const char *mpdu);
static uint16_t IEEE_get_seq_no(const char *mpdu);
static void IEEE_extract_payload(char *payload, const char* mpdu);

namespace FBMC_MAC
{
	//______________________________________________________
	void load_decoder(int mpdu_len, int encoding_family)
	{
		d_mpdu_len = mpdu_len;
		d_encoding_family = encoding_family;
		
		switch(d_encoding_family)
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
		}
	}
	
	//______________________________________________________
	int get_payload_len(const char *mpdu)
	{
		int payload_length = 0;
		
		switch(d_MODULE_STATE)
		{
			case IEEE:
				payload_length = IEEE_get_payload_len(mpdu);
				break;
		}
		
		return payload_length;
	}
	
	//______________________________________________________
	uint16_t get_sender_id(const char *mpdu)
	{
		uint16_t sender_id = 0;
		
		switch(d_MODULE_STATE)
		{
			case IEEE:
				sender_id = IEEE_get_sender_id(mpdu);
				break;
		}
		
		return sender_id;
	}
	
	//______________________________________________________
	uint16_t get_seq_no(const char *mpdu)
	{
		uint16_t seq_no = 0;
		
		switch(d_MODULE_STATE)
		{
			case IEEE:
				seq_no = IEEE_get_seq_no(mpdu);
				break;
		}
		
		return seq_no;
	}	
	
	//______________________________________________________
	void extract_payload(char *payload, const char* mpdu)
	{
		switch(d_MODULE_STATE)
		{
			case IEEE:
				IEEE_extract_payload(payload, mpdu);
				break;
		}
	}
}

// IEEE	MAC-decoding
//______________________________________________________
int IEEE_get_payload_len(const char *mpdu)
{
	// checksum
	boost::crc_32_type result;
	result.process_bytes(mpdu, d_mpdu_len);
	
	if(result.checksum() != 558161692)
		return 0;
		
	return d_mpdu_len - sizeof(IEEE_mac_header) - IEEE_CHECKSUM;
}

//______________________________________________________
uint16_t IEEE_get_sender_id(const char *mpdu)
{
	IEEE_mac_header *mh = const_cast<IEEE_mac_header*>(reinterpret_cast<const IEEE_mac_header*> (mpdu));
	
	uint16_t sender_id = 0;
	
	sender_id = mh->addr1[4];
	sender_id = sender_id << 8;
	sender_id |= mh->addr1[5] & 0xFF;
	
	return sender_id;
}

//______________________________________________________
uint16_t IEEE_get_seq_no(const char *mpdu)
{
	IEEE_mac_header *mh = const_cast<IEEE_mac_header*>(reinterpret_cast<const IEEE_mac_header*> (mpdu));
	
	uint16_t seq_no = 0;
	
	seq_no = mh->seq >> 4;
	
	return seq_no;
}

//______________________________________________________
void IEEE_extract_payload(char *payload, const char* mpdu)
{
	memcpy(payload, mpdu + sizeof(IEEE_mac_header), d_mpdu_len - sizeof(IEEE_mac_header) - IEEE_CHECKSUM);
}
