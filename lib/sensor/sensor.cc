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

#include <stdlib.h>
#include <cstdlib>
#include <string.h>
#include <boost/crc.hpp>

#include "../utils/debug.h"
#include "sensor.h"
	
// variables
static const char alphanum[] = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz";
static std::string locked_payload;
static const uint8_t randomIntegers[1013] = {	161,18,3,239,115,111,107,144,159,132,63,183,214,24,35,254,230,226,238,219,
												91,240,216,32,246,121,248,101,178,37,201,46,248,66,172,218,110,101,213,253,
												255,5,236,187,11,143,252,102,184,143,19,86,15,219,146,246,86,46,229,238,
												58,175,197,165,249,69,221,132,89,2,101,48,115,233,44,174,153,162,149,105,
												167,200,11,213,49,191,100,168,132,196,1,83,149,179,38,103,231,160,149,1,
												155,137,97,102,171,129,116,243,142,217,57,103,176,185,122,239,180,84,219,138,
												99,226,180,192,150,165,63,185,40,234,1,140,184,59,232,123,97,205,9,53,
												37,230,187,213,173,79,201,103,144,252,49,13,166,147,141,60,188,255,226,56,
												7,160,58,83,93,36,226,213,178,129,37,141,186,207,9,102,244,251,104,84,
												108,36,69,141,159,122,233,35,109,70,69,71,96,103,14,180,139,33,10,25,
												47,56,172,250,207,93,143,16,214,26,45,62,149,253,206,143,138,60,191,191,
												186,47,122,141,5,201,153,22,187,130,36,63,30,23,116,193,80,7,40,191,
												95,11,160,85,22,199,38,42,92,150,153,40,15,82,98,254,19,155,37,139,
												215,28,61,158,249,171,230,83,116,195,230,244,250,131,202,246,47,29,155,87,
												188,158,46,92,167,35,115,77,99,22,240,239,73,16,93,137,24,104,65,213,
												146,240,212,193,59,6,53,3,198,169,57,209,152,186,136,246,119,226,83,141,
												46,91,196,106,103,143,230,138,188,132,124,186,246,142,182,30,3,140,103,245,
												245,105,239,126,32,77,195,25,110,44,96,45,48,135,223,14,234,95,104,228,
												68,252,28,178,69,240,226,38,177,240,134,92,50,212,84,78,164,54,99,91,
												236,242,47,249,66,251,73,161,242,151,68,49,88,219,166,148,1,113,156,140,
												58,173,26,84,85,74,172,220,200,113,18,1,238,202,249,136,8,244,43,89,
												96,253,125,214,39,205,170,249,180,241,112,81,100,82,20,230,54,32,48,159,
												23,196,50,189,89,229,32,146,28,168,45,179,70,116,150,138,117,121,25,204,
												164,86,229,244,32,153,104,255,100,116,7,160,39,192,106,111,33,219,83,191,
												47,169,26,9,226,226,71,142,151,215,244,139,102,29,79,139,3,12,197,33,
												12,63,12,92,25,241,183,132,118,136,110,112,96,28,68,182,186,177,60,210,
												185,76,143,169,238,141,73,85,13,142,246,74,34,69,194,5,91,250,189,7,
												89,174,197,72,102,186,229,110,59,174,195,128,111,134,254,205,200,73,177,106,
												169,135,80,59,13,148,211,126,213,94,2,195,24,249,48,37,112,35,211,187,
												98,105,46,182,88,7,100,19,4,62,174,195,157,182,200,123,112,175,212,200,
												160,154,87,160,188,114,127,227,221,148,181,147,3,80,50,66,13,232,75,94,
												6,192,54,151,203,165,178,17,20,222,165,191,147,75,254,95,13,156,79,190,
												233,149,131,169,139,133,213,112,171,104,141,165,202,217,205,169,24,230,134,228,
												205,141,232,13,175,95,156,211,128,29,56,32,61,233,109,44,152,50,83,62,
												191,149,161,89,228,239,147,101,56,114,197,129,115,164,186,72,62,193,213,34,
												208,131,26,116,14,212,84,157,146,77,59,89,146,66,128,51,215,89,207,58,
												98,22,100,62,78,217,117,209,32,103,95,24,16,23,207,50,219,107,143,90,
												158,184,14,183,12,95,183,226,172,213,228,38,127,214,91,84,246,40,68,74,
												147,255,128,219,50,152,226,238,153,103,219,51,164,150,19,225,125,209,174,237,
												136,91,111,72,184,223,184,49,15,100,103,172,57,242,151,121,57,160,60,14,
												71,248,255,187,185,239,5,66,15,92,82,111,214,16,233,104,58,189,117,152,
												64,89,207,36,224,171,173,229,39,36,156,178,248,150,50,46,137,206,99,42,
												116,227,124,214,149,53,112,152,6,67,233,66,216,73,111,248,195,18,138,214,
												59,116,229,36,19,167,112,129,111,23,232,3,130,212,192,104,89,243,253,43,
												206,106,36,150,236,19,30,157,213,110,151,136,72,49,10,216,118,127,193,129,
												252,56,100,94,205,104,208,209,158,66,85,72,191,113,242,157,164,193,251,255,
												8,33,237,206,73,171,83,136,133,76,218,242,61,101,195,97,234,190,60,91,
												137,79,234,129,139,239,63,52,221,109,113,232,245,211,143,224,42,212,135,13,
												23,186,182,125,252,207,88,116,25,31,231,101,159,173,47,201,183,40,14,55,
												205,200,158,224,183,124,96,110,232,127,9,7,154,53,127,119,148,117,194,102,
												253,177,88,91,214,193,140,247,28,181,136,172,45};
												
// functions
static void ByteRandomGenerator(char *target, int type, int nBytes, int &seedInx);

//_________________________________________________
uint16_t sensor::n_sensor = 0;
int sensor::locked_payload_sensor_ID = -1;

//_________________________________________________
sensor::sensor(float scl, uint8_t enc, uint16_t lc, uint16_t pl, uint32_t st, uint32_t f2f, uint32_t rbo)
{
	scale 			 = scl;
	encoding		 = enc;
	sensor_ID		 = n_sensor++;
	log_chann 		 = lc;
	
	seq_no			 = 0;
	payload_len		 = pl;
	frames_send		 = 0;
	
	start_time 		 = st;
	frame2frame_time = f2f;
	randbackoff_time = rbo;
	
	// security mechanism
	if(randbackoff_time > frame2frame_time)
	{
		PRINT("sensor error: Random backoff time bigger than frame to frame time. Setting to frame to frame time.");
		randbackoff_time = frame2frame_time;
	}
	
	airtime_f2f		 = start_time;
	airtime			 = start_time;
	
	rand_payload 	 = true;	
	lock_exception 	 = false;
	random_gen_type  = RANDOM_GEN_CPP_CH;
	random_gen_seed_idx = 0;
	
	insert_custom_header = false;
	
	interleaving_type = 0;
	
	alternate_preamble = false;
}

//_________________________________________________
sensor::~sensor()
{
}

//_________________________________________________
void sensor::generate_payload(char *payload)
{
	// check if frame has a MAC-Part or is random
	if(insert_custom_header == true)
	{
		int p_temp = (int) payload_len;
		int residual_payload = p_temp - sizeof(custom_sensor_mac_header) - 4;
		
		if(residual_payload <= 0)
		{
			PRINT("sensor error: Payload size is smaller or equal to the length of the header + the check sequence. Returning random data as allocated by c++.");
			return;
		}			
		
		// insert header
		custom_sensor_mac_header header_tmp;
		header_tmp.seq = seq_no;
		header_tmp.sender_id = sensor_ID;
		memcpy(payload, &header_tmp, sizeof(custom_sensor_mac_header));
		
		// insert random data
		ByteRandomGenerator(payload + sizeof(custom_sensor_mac_header), random_gen_type, residual_payload, random_gen_seed_idx);
		
		// insert check sum
		boost::crc_32_type result;
		result.process_bytes(payload, sizeof(custom_sensor_mac_header) + residual_payload);
		unsigned int fcs = result.checksum();
		memcpy(payload + sizeof(custom_sensor_mac_header) + residual_payload, &fcs, sizeof(unsigned int));
	}
	else
	{
		// first check if payload is fixed
		if(locked_payload_sensor_ID == -1 || lock_exception == true)	
		{
			if(rand_payload == true)
				ByteRandomGenerator(payload, random_gen_type, payload_len, random_gen_seed_idx);
			else
				memcpy(payload, fixed_payload.c_str(), payload_len);
		}
		else
		{
			// check if you are the locking point
			if(locked_payload_sensor_ID == (int) sensor_ID)
			{
				ByteRandomGenerator(payload, random_gen_type, payload_len, random_gen_seed_idx);
				
				// save if as a reference for other sensors
				locked_payload.clear();
				locked_payload = std::string((const char*) payload, payload_len);
			}
			else
			{
				// copy reference payload into your payload
				if(locked_payload.size() == payload_len)
					memcpy(payload, locked_payload.c_str(), payload_len);
				else
					PRINT("sensor error: locked payload is not set, returning unlocked bytes as payload.");
			}
		}
	}
	
	/*
	// DEBUG
	PRINT(" ");
	PRINT("CREATED BYTES:");
	PRINT(" ");
	for(int i=0; i<payload_len; i++)
	{
		PRINT((int) payload[i]);
	}
	*/
}

//_________________________________________________
void sensor::set_rand_payload(bool rp)
{
	rand_payload = rp;
	
	if(rand_payload == false)
	{
		char payload_temp[payload_len];
		
		ByteRandomGenerator(payload_temp, random_gen_type, payload_len, random_gen_seed_idx);
		
		fixed_payload.clear();
		fixed_payload = std::string((const char*) payload_temp, payload_len);
	}
}

//_________________________________________________
void sensor::inc_airtime()
{
	airtime_f2f += frame2frame_time;
	
	if(randbackoff_time > 0)
	{
		// causes bug, don't know why
		//int curr_randback_off = rand() % (2*randbackoff_time - 1);
		//curr_randback_off -= randbackoff_time - 1;
		
		int curr_randback_off = rand() % randbackoff_time;
		airtime = airtime_f2f + curr_randback_off;
	}
	else
	{
		airtime = airtime_f2f;
	}
}

//_________________________________________________
void sensor::set_random_seq_no()
{
	seq_no = rand() % 65536; //UINT16_MAX;
}

//_________________________________________________
sensor_cnt::sensor_cnt(uint16_t sensor_id)
{
	sensor_ID = sensor_id;
	seq_no_init = false;
	last_seq_no = 0;
	bytes_received = 0;
	frames_received = 0;
}

//_________________________________________________
sensor_cnt::~sensor_cnt()
{
}

//_________________________________________________
bool sensor_cnt::check_seq_no(uint16_t seq_no)
{
	bool ret_val;
	
	if(seq_no_init == false)
	{
		last_seq_no = seq_no;
		seq_no_init = true;
		ret_val = true;
	}
	else
	{
		if(last_seq_no == seq_no)
		{
			ret_val = false;
		}
		else
		{
			last_seq_no = seq_no;
			ret_val = true;
		}
	}
	
	return ret_val;
}

//______________________________________________________
ReferenceFrameStatistics::ReferenceFrameStatistics()
{
	reference_frame_not_init = false;
	reference_frame_missed = false;
	reference_frame_undecodable = false;
	reference_frame_delayed = false;
	known_frame_missed = false;	

	recv_ref_frame = 0;
	decoded_ref_frame = 0;

	recv_found_frames = 0;
	decoded_found_frames = 0;

	recv_known_frames = 0;
	decoded_known_frames = 0;	
}

//______________________________________________________
ReferenceFrameStatistics::~ReferenceFrameStatistics()
{
}

//______________________________________________________
void ByteRandomGenerator(char *target, int type, int nBytes, int &seedInx)
{

	switch(type)
	{
		case RANDOM_GEN_CPP_CH:
		{			
			char random_bytes[nBytes];

			for(int i=0; i < nBytes; i++)
				random_bytes[i] = alphanum[rand() % (sizeof(alphanum) - 1)];
			
			memcpy(target, random_bytes, nBytes);			
			
			break;
		}
		
		case RANDOM_GEN_CPP_U8:
		{
			uint8_t random_bytes[nBytes];

			for(int i=0; i < nBytes; i++)
				random_bytes[i] = (uint8_t) (rand() % 256);
			
			memcpy(target, random_bytes, nBytes);
			
			break;
		}		
		
		case RANDOM_GEN_CUSTOM:
		{
			int array_len = strlen((char*) randomIntegers);
			
			// security mechanism
			if(seedInx >= array_len)
			{
				PRINT("ByteRandomGenerator error: Seed is too high. Returning random bytes as allocated by c++, seed not adjusted.");
				std::cout << "Seed can be between 0 and " << array_len << " . But it is: " << seedInx << std::endl;
				break;
			}
			
			if(seedInx + nBytes - 1 < array_len)
			{
				memcpy(target, &randomIntegers[seedInx], nBytes);
			}
			else
			{
				int residual_copy = array_len - seedInx;
				memcpy(target, &randomIntegers[seedInx], residual_copy);
				memcpy(&target[residual_copy], &randomIntegers[0], nBytes - residual_copy);
			}
		
			seedInx += nBytes;
			
			seedInx = seedInx % array_len;
			
			/*
			// DEBUG
			for(int i=0; i<nBytes; i++)
				PRINT((unsigned) (uint8_t) target[i]);
			PRINT(" ");
			PRINT(seedInx);
			*/
			
			break;
		}
		
		default:
			PRINT("ByteRandomGenerator error: Undefinded random generator, returning random bytes as allocated by c++.");
	}	
}
