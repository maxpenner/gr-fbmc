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
 
#include <algorithm>
#include <string.h>
#include <sys/types.h>
#include <dirent.h>
#include <fstream>
#include <chrono>
#include <math.h>

#include "../utils/debug.h"
#include "../utils/logical_channel.h"
#include "../utils/encodings.h"
#include "../utils/frame_param.h"
#include "subroutines.h"
#include "mode.h"

#define SLEEP_TIME 4500000

// timing variable
static const uint64_t runtime = 3000000;

// mode data
static ANY_MODE_DATA_TYPE d_MODE;
static struct mode_freerun_data mode_fre_data;
static struct mode_referen_data mode_ref_data;

// sensors at transmitter (read by the scheduler)
std::vector<sensor> sensor_array;

// general variables
static int d_spf;
static std::vector<int> d_log_chann;

// hardcoded variables for sanity check
static int d_veclength = 64;
static int d_filterlength = 4;
static int d_spacereducer = FRAME_SPACER_LENGTH_REDUCE;
static bool overwrite_framing = true;

// loop variables with initial values which can be overwritten
static int loop0_start 	= -1;
static int loop0_step 	= -1;
static int loop0_end 	= -1;
static int loop0_index 	= -1;
static int loop1_start 	= -1;
static int loop1_step 	= -1;
static int loop1_end 	= -1;
static int loop1_index 	= -1;

// log destination folder
//static std::string log_dest("/home/mpenner/Desktop/grc/log_files/");
static std::string log_dest("/home/mpenner/log_files/");
//static std::string log_dest("/home/default19ubuntu/log_files/");

// buffer for function 'remaining_compute_time()'
static uint32_t old_time = 0;

// functions
static bool isDirEmpty();
static bool sanity_check(bool overwrite_values);
static void show_loop_info();

//_________________________________________________
bool init_sensor_list(int spf, std::vector<int> logical_channels)
{
	// FIXED: check if logging destination exist and is empty
	if(isDirEmpty() == false)
		return false;
		
	// FIXED: set variables
	d_spf = spf;
	d_log_chann = logical_channels;
	
	//::::::::::::::::::::::::::::::::::::::::::::
	
	// VARIABLE: set mode
	//d_MODE = MODE_SENSORS_FREERUN;
	d_MODE = MODE_SENSORS_REFEREN;
	
	// VARIABLE: set mode data for receiver
	switch(d_MODE)
	{
		case MODE_SENSORS_FREERUN:
		
			loop0_start 	= 0;					// SNR -> change
			loop0_step 		= 1;					// SNR -> change
			loop0_end 		= 1;					// SNR -> change
			loop0_index 	= loop0_start;
			loop1_start 	= 100;					// SNR -> 0
			loop1_step 		= 100;					// SNR -> 1
			loop1_end 		= 1501;					// SNR -> 1
			loop1_index 	= loop1_start;		
		
			mode_fre_data.submode = FREERUN_N_SENSOR_CUSTOM;
			//mode_fre_data.submode = FREERUN_N_SENSORS;
			mode_fre_data.encoding_family = 0;
			mode_fre_data.encoding = 11;
			mode_fre_data.payload_len = 10;			
			mode_fre_data.hard_soft_decoding = 0;
			mode_fre_data.interl_type = 0;
			
			break;
			
		case MODE_SENSORS_REFEREN:

			loop0_start 	= 1024;			// overwritable
			loop0_step 		= 4;
			loop0_end 		= 2176;			// overwritable
			loop0_index 	= loop0_start;	// overwritable
			loop1_start 	= 0;
			loop1_step 		= 1;
			loop1_end 		= 11;
			loop1_index 	= loop1_start;		
		
			mode_ref_data.ref_enco = 5;
			mode_ref_data.encoding = 11;
			mode_ref_data.payload_len = 10;
			mode_ref_data.payload_dis = 10;
			mode_ref_data.hard_soft_decoding = 1;
			mode_ref_data.interl_type = 3;
			mode_ref_data.ref2ref_samples = 4000;	// overwritable
			mode_ref_data.ref_delay_max = 1;
			mode_ref_data.case_samples = 3680;		// overwritable
			mode_ref_data.known_offset = 1600;		// overwritable
			mode_ref_data.found_unsecu = 1;
			mode_ref_data.refer_thresh = 0.7f;
			
			break;
	}
	
	// FIXED: sanity check to see if input data makes sense
	if(sanity_check(overwrite_framing) == false)
		return false;
	
	// VARIABLE: create initial sensor setup
	switch(d_MODE)
	{
		case MODE_SENSORS_FREERUN:		
		
			switch(mode_fre_data.submode)
			{
				case FREERUN_N_SENSOR_CUSTOM:
					break;
				
				case FREERUN_N_SENSORS:					
					break;	
			}			
			break;
			
		case MODE_SENSORS_REFEREN:
		
			// DEBUG
			//sensor_array.push_back(sensor(1.0f, mode_ref_data.encoding, 0, mode_ref_data.payload_len, mode_ref_data.known_offset, 100000000, 0));	// Reference
			//sensor_array[0].set_random_gen_type(RANDOM_GEN_CUSTOM);
			//sensor_array[0].set_random_gen_seed_idx(1012);
		
			// three sensors are needed in reference mode
			sensor_array.push_back(sensor(1.0f, mode_ref_data.ref_enco, 0, mode_ref_data.payload_len, 0, mode_ref_data.ref2ref_samples, 0));							// Reference
			sensor_array.push_back(sensor(0.5f, mode_ref_data.encoding, 0, mode_ref_data.payload_len, mode_ref_data.known_offset, mode_ref_data.ref2ref_samples, 0));	// Targetframe
			sensor_array.push_back(sensor(0.5f, mode_ref_data.encoding, 0, mode_ref_data.payload_dis, loop0_start, mode_ref_data.ref2ref_samples, 0));					// Disturber
			
			// adjust reference
			sensor_array[0].set_sensor_to_lock_point();
			
			// adjust targetframe
			sensor_array[1].set_interleaving_type(mode_ref_data.interl_type);		
			
			// adjust disturber
			sensor_array[2].set_lock_exception();
			//sensor_array[2].set_alternate_preamble(true);
			
			break;
	}	
	
	//::::::::::::::::::::::::::::::::::::::::::::
	
	return true;
}

//_________________________________________________
void prepare_mode_data(std::string &mode_string)
{
	mode_string.append((const char*) &d_MODE, sizeof(ANY_MODE_DATA_TYPE));
	
	switch(d_MODE)
	{
		case MODE_SENSORS_FREERUN:
			mode_string.append((const char*) &mode_fre_data, sizeof(mode_freerun_data));
			break;
			
		case MODE_SENSORS_REFEREN:
			mode_string.append((const char*) &mode_ref_data, sizeof(mode_referen_data));
			break;
	}
}

//_________________________________________________
bool update_sensor_list()
{	
	// FIXED: check for stop condition
	if(loop0_index >= loop0_end)
	{
		PRINT("Loop ended in subroutines.");
		return false;
	}
	
	// FIXED
	show_loop_info();
	
	//::::::::::::::::::::::::::::::::::::::::::::
	
	// VARIABLE: change sensor list as wished
	switch(d_MODE)
	{
		case MODE_SENSORS_FREERUN:		
		{
			switch(mode_fre_data.submode)
			{
				case FREERUN_N_SENSOR_CUSTOM:					
				{
					sensor_array.clear();
					
					/*
					// SNR measurement
					sensor_array.push_back(sensor(1.0f, mode_fre_data.encoding, 0, mode_fre_data.payload_len, 5000000, 5000, 0));
					sensor_array[sensor_array.size()-1].set_random_seq_no();
					sensor_array[sensor_array.size()-1].set_insert_custom_header(true);						
					sensor_array[sensor_array.size()-1].set_interleaving_type(mode_fre_data.interl_type);
					
					// SNR measurement -> dB change
					float db_decrease = -1;
					float factor = (float) loop0_index;
					factor /= 10.0f;
					factor += 10.0f;
					factor = db_decrease * factor;
					factor = pow(10.0f, factor/20.0f);
					sensor_array[0].set_scale(1.0f*factor);
					*/
					
					float near = 1.0f;
					float far = 5.0f;
					
					float prescale = 0.2f;
					
					for(int i=0; i<loop1_index; i++)
					{						
						uint32_t curr_start_time = rand() % (1000000-1);
						float distance = static_cast <float>(rand()) / (static_cast <float> (RAND_MAX/(far-near)));
						distance += near;
						
						// equal distance
						distance = 1.0f;
						
						sensor_array.push_back(sensor(1.0f/(distance*distance) * prescale, mode_fre_data.encoding, 0, mode_fre_data.payload_len, curr_start_time, 1000000, 1000000));
						
						sensor_array[sensor_array.size()-1].set_random_seq_no();
						sensor_array[sensor_array.size()-1].set_insert_custom_header(true);						
						sensor_array[sensor_array.size()-1].set_interleaving_type(mode_fre_data.interl_type);
					}
					
					// delete all sensor's internal counters
					for(int i=0; i<sensor_array.size(); i++)
					{
						sensor_array[i].reset_frames_send();
						sensor_array[i].reset_airtime();
					}
								
					break;
				}
				
				case FREERUN_N_SENSORS:
				{	
					sensor_array.clear();
					
					float near = 1.0f;
					float far = 5.0f;
					
					float prescale = 0.1f;
					
					for(int i=0; i<loop1_index; i++)
					{						
						uint32_t curr_start_time = rand() % (5000000 - 1);
						float distance = static_cast <float>(rand()) / (static_cast <float> (RAND_MAX/(far-near)));
						distance += near;
						
						// equal distance
						distance = 1.0f;
						
						sensor_array.push_back(sensor(1.0f/(distance*distance) * prescale, 2, 0, 1, curr_start_time, 5000000, 2450000));
					}
					
					// delete all sensor's internal counters
					for(int i=0; i<sensor_array.size(); i++)
					{
						sensor_array[i].reset_seq_no();
						sensor_array[i].reset_frames_send();
						sensor_array[i].reset_airtime();
					}					
					
					break;	
				}
			}	
				
			break;
		}
			
		case MODE_SENSORS_REFEREN:
		{
			// sort sensors by sensor ID, so they can be easily accessed
			std::sort(sensor_array.begin(), sensor_array.end(), [](sensor a, sensor b) { return a.get_sensor_ID() < b.get_sensor_ID(); });
			
			float db_decrease = -1;
			float factor = (float) loop1_index;
			factor = db_decrease * factor;
			factor = pow(10.0f, factor/20.0f);
			sensor_array[2].set_start_time(loop0_index);
			sensor_array[2].set_scale(0.5f*factor);
		
			// delete all sensor's internal counters
			for(int i=0; i<sensor_array.size(); i++)
			{
				sensor_array[i].reset_seq_no();
				sensor_array[i].reset_frames_send();
				sensor_array[i].reset_airtime();
			}			
			
			break;
		}
	}	
	
	//::::::::::::::::::::::::::::::::::::::::::::
	
	// FIXED: sort sensors by transmission time
	std::sort(sensor_array.begin(), sensor_array.end());
	
	return true;
}

//_________________________________________________
uint64_t get_runtime()
{
	return runtime;
}

//_________________________________________________
uint64_t get_sleeptime()
{
	return SLEEP_TIME;
}

//_________________________________________________
bool log_data(std::string data)
{
	// VARIABLE: log data as mode needs it
	switch(d_MODE)
	{
		case MODE_SENSORS_FREERUN:		
		{
			// FIXED: prepare list of sensors for both submodes
			std::vector<sensor_cnt> sensor_cnts_array;
			
			// FIXED: convert data into easy format
			if(data.length() > 0)
			{
				int instances = data.length() / sizeof(sensor_cnt);
				sensor_cnts_array.resize(instances, sensor_cnt(0));
				memcpy(&sensor_cnts_array[0], data.c_str(), data.length());
				
				// FIXED: sort receiver array by sender ID
				std::sort(sensor_cnts_array.begin(), sensor_cnts_array.end(), [](sensor_cnt a, sensor_cnt b) { return a.get_sensor_ID() < b.get_sensor_ID(); });
			}
			
			// FIXED: sort transmitter array by sender IDs
			std::sort(sensor_array.begin(), sensor_array.end(), [](sensor a, sensor b) { return a.get_sensor_ID() < b.get_sensor_ID(); });
			
			//::::::::::::::::::::::::::::::::::::::::::::
			
			switch(mode_fre_data.submode)
			{				
				case FREERUN_N_SENSOR_CUSTOM:
				case FREERUN_N_SENSORS:
				{
					uint32_t send_frames = 0;
					uint32_t received_frames = 0;
					int max_received = 0;
					int min_received = 0;
					
					// all send frames
					for(int i=0; i<sensor_array.size(); i++)
						send_frames += sensor_array[i].get_frames_send();	
					
					if(sensor_cnts_array.size() > 0)
					{
						received_frames = sensor_cnts_array[0].get_frames_received();
						max_received = sensor_cnts_array[0].get_frames_received();
						min_received = sensor_cnts_array[0].get_frames_received();
					
						// all received frames
						for(int i=1; i<sensor_cnts_array.size(); i++)
						{
							received_frames += sensor_cnts_array[i].get_frames_received();
							
							if(sensor_cnts_array[i].get_frames_received() > max_received)
								max_received = sensor_cnts_array[i].get_frames_received();
								
							if(sensor_cnts_array[i].get_frames_received() < min_received)
								min_received = sensor_cnts_array[i].get_frames_received();				
						}
					}
					
					std::string sensor_found_file_dest(log_dest);
					std::string send_packets_file_dest(log_dest);
					std::string recv_packets_file_dest(log_dest);
					std::string max_packets_file_dest(log_dest);
					std::string min_packets_file_dest(log_dest);
					
					sensor_found_file_dest.append("sensors_found");
					send_packets_file_dest.append("send_packets");
					recv_packets_file_dest.append("recv_packets");
					max_packets_file_dest.append("max_packets");
					min_packets_file_dest.append("min_packets");
					
					std::ofstream outfiles;
					
					outfiles.open(sensor_found_file_dest.c_str(), std::ios::app);
					outfiles << sensor_cnts_array.size() << "\n";
					outfiles.close();
					
					outfiles.open(send_packets_file_dest.c_str(), std::ios::app);
					outfiles << send_frames << "\n";
					outfiles.close();
					
					outfiles.open(recv_packets_file_dest.c_str(), std::ios::app);
					outfiles << received_frames << "\n";
					outfiles.close();
					
					outfiles.open(max_packets_file_dest.c_str(), std::ios::app);
					outfiles << max_received << "\n";
					outfiles.close();
					
					outfiles.open(min_packets_file_dest.c_str(), std::ios::app);
					outfiles << min_received << "\n";
					outfiles.close();				
					
					break;	
				}

				//::::::::::::::::::::::::::::::::::::::::::::
				
			}		
			break;
		}
			
		case MODE_SENSORS_REFEREN:
		{
			struct ReferenceFrameStatistics rfs;
			
			memcpy(&rfs, data.c_str(), data.length());
			
			bool error_occured = false;
			
			// check conditions too make sure, case frames were received correctly
			if(rfs.reference_frame_not_init == true)
			{
				PRINT(" ");
				PRINT("//////////////////////////////////");
				PRINT("log_data() in subroutines error: First received frame was not a reference frame.");
				error_occured = true;
			}			
			
			if(rfs.reference_frame_missed == true)
			{
				PRINT(" ");
				PRINT("//////////////////////////////////");
				PRINT("log_data() in subroutines error: Missed a reference frame.");
				error_occured = true;
			}
			
			if(rfs.reference_frame_undecodable == true)
			{
				PRINT(" ");
				PRINT("//////////////////////////////////");
				PRINT("log_data() in subroutines error: Reference frame is undecodable.");
				error_occured = true;
			}			
			
			if(rfs.reference_frame_delayed == true)
			{
				PRINT(" ");
				PRINT("//////////////////////////////////");
				PRINT("log_data() in subroutines error: Reference frame is delayed.");
				error_occured = true;
			}	
			
			if(rfs.known_frame_missed == true)
			{
				PRINT(" ");
				PRINT("//////////////////////////////////");
				PRINT("log_data() in subroutines error: Missed a known frame.");
				error_occured = true;
			}	
			
			if(error_occured == true)
			{
				PRINT("Loop error information:");
				std::cout << "Loop 0: " << " start: " << loop0_start << " end: " << loop0_end << " step: " << loop0_step << " now: " << loop0_index << std::endl;
				std::cout << "Loop 1: " << " start: " << loop1_start << " end: " << loop1_end << " step: " << loop1_step << " now: " << loop1_index << std::endl;
				PRINT("//////////////////////////////////");
				return false;
			}
			
			// data at transmitter
			std::string transmitted_refere_frames_file_dest(log_dest);
			std::string transmitted_target_frames_file_dest(log_dest);
			std::string transmitted_distur_frames_file_dest(log_dest);
			transmitted_refere_frames_file_dest.append("transmitted_refere");
			transmitted_target_frames_file_dest.append("transmitted_target");
			transmitted_distur_frames_file_dest.append("transmitted_distur");
			
			std::ofstream outfiles;
			
			outfiles.open(transmitted_refere_frames_file_dest.c_str(), std::ios::app);
			outfiles << sensor_array[0].get_frames_send() << "\n";
			outfiles.close();
			
			outfiles.open(transmitted_target_frames_file_dest.c_str(), std::ios::app);
			outfiles << sensor_array[1].get_frames_send() << "\n";
			outfiles.close();
			
			outfiles.open(transmitted_distur_frames_file_dest.c_str(), std::ios::app);
			outfiles << sensor_array[2].get_frames_send() << "\n";
			outfiles.close();			
			
			// data from receiver
			std::string rref_file_dest(log_dest);
			std::string dref_file_dest(log_dest);
			std::string rfou_file_dest(log_dest);
			std::string dfou_file_dest(log_dest);
			std::string rkno_file_dest(log_dest);
			std::string dkno_file_dest(log_dest);		
			
			rref_file_dest.append("recv_ref_frames");
			dref_file_dest.append("decoded_ref_frames");
			rfou_file_dest.append("recv_found_frames");
			dfou_file_dest.append("decoded_found_frames");
			rkno_file_dest.append("recv_known_frames");
			dkno_file_dest.append("decoded_known_frames");
			
			outfiles.open(rref_file_dest.c_str(), std::ios::app);
			outfiles << rfs.recv_ref_frame << "\n";
			outfiles.close();
			
			outfiles.open(dref_file_dest.c_str(), std::ios::app);
			outfiles << rfs.decoded_ref_frame << "\n";
			outfiles.close();
			
			outfiles.open(rfou_file_dest.c_str(), std::ios::app);
			outfiles << rfs.recv_found_frames << "\n";
			outfiles.close();
			
			outfiles.open(dfou_file_dest.c_str(), std::ios::app);
			outfiles << rfs.decoded_found_frames << "\n";
			outfiles.close();
			
			outfiles.open(rkno_file_dest.c_str(), std::ios::app);
			outfiles << rfs.recv_known_frames << "\n";
			outfiles.close();	
			
			outfiles.open(dkno_file_dest.c_str(), std::ios::app);
			outfiles << rfs.decoded_known_frames << "\n";
			outfiles.close();
		}
	}	
	
	//::::::::::::::::::::::::::::::::::::::::::::
	
	return true;
}

void increase_loop_indices()
{
	loop1_index += loop1_step;
	if(loop1_index >= loop1_end)
	{
		loop1_index = loop1_start;
		loop0_index += loop0_step;
	}	
}

//_________________________________________________
bool isDirEmpty()
{
	int n = 0;
	struct dirent *d;
	
	DIR *dir = opendir(log_dest.c_str());
	
	if (dir == NULL)
	{
		PRINT("Logging destination does not exist.");
		closedir(dir);
		return false;
	}
	
	while((d = readdir(dir)) != NULL) 
	{
		if(++n > 2)
		break;
	}
	
	closedir(dir);
	
	if (n > 2)
	{
		PRINT("Logging destination is not empty.");
		return false;
	}
	
	return true;
}

//_________________________________________________
bool sanity_check(bool overwrite_values)
{
	bool return_value_sc = true;
	
	PRINT(" ");
	PRINT("/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_");
	PRINT(" ");
	PRINT("SANITY CHECK RESULTS: ");
	PRINT(" ");	
			
	if(d_MODE == MODE_SENSORS_FREERUN)
	{
		if(mode_fre_data.submode == FREERUN_N_SENSOR_CUSTOM)
		{			
			PRINT(" ");
			PRINT("CURRENT MODE: FREERUN WITH SUBMODE FREERUN_N_SENSOR_CUSTOM.");
			PRINT(" ");
			PRINT("No sanity check performed yet.");
			PRINT(" ");
		}
		else if(mode_fre_data.submode == FREERUN_N_SENSORS)
		{
			PRINT(" ");
			PRINT("CURRENT MODE: FREERUN WITH SUBMODE FREERUN_N_SENSORS.");
			PRINT(" ");
			PRINT("No sanity check performed yet.");
			PRINT(" ");			
		}
	}
	else if(d_MODE == MODE_SENSORS_REFEREN)
	{	
		// FIRST STEP: Calculate minimum number of symbols needed to contain reference bytes.
		int n_bpsc, n_cbps, n_dbps;
		
		switch(mode_ref_data.ref_enco)
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

		int d_channel_width = d_log_chann[0];	// only in time domain -> first number is channel width of channel 0
		int mac_bytes = 30 + 4;					// sizeof(IEEE_mac_header) + IEEE_CHECKSUM
			
		// calculate the minimum number of symbols per frame needed for the reference frame
		int d_spf_min_r = 1;
		while(1 == 1)
		{
			int total_fs = d_spf_min_r*d_channel_width;
			int usable_fs = total_fs-48;
			int vec48 = floor(usable_fs/48);
			int max_bytes = floor((vec48*n_dbps-16-6)/8) - mac_bytes;
			int max_frames = floor(mode_ref_data.payload_len / max_bytes);
			int left_bytes = mode_ref_data.payload_len % max_bytes;		
			
			if((max_bytes <= 0) || (max_frames > 1) || (max_frames == 1 && left_bytes > 0))
			{
				d_spf_min_r++;
				continue;
			}
			else
				break;
		}			
		
		// SECOND STEP: Calculate minimum number of symbols needed to contain target and disturber bytes.
		int M, numerator, denominator;
		
		switch(mode_ref_data.encoding)
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
				PRINT("sanity_check error: Unknown encoding.");
				return false;			
		}
		
		// target
		int nApplicationBits = mode_ref_data.payload_len*8;
		int nBitsNetUser = nApplicationBits + 6;	
		int nBitsEncodedMin = (nBitsNetUser + numerator - 1)/numerator;
		nBitsEncodedMin *= denominator;
		int nBitsSymbGross = M*d_channel_width;
		int d_spf_min_t = (nBitsEncodedMin + nBitsSymbGross - 1)/nBitsSymbGross;		

		// disturber
		nApplicationBits = mode_ref_data.payload_dis*8;
		nBitsNetUser = nApplicationBits + 6;	
		nBitsEncodedMin = (nBitsNetUser + numerator - 1)/numerator;
		nBitsEncodedMin *= denominator;
		nBitsSymbGross = M*d_channel_width;
		int d_spf_min_d = (nBitsEncodedMin + nBitsSymbGross - 1)/nBitsSymbGross;			
		
		// THIRD STEP: Print all relevant data about the frames.	

		PRINT(" ");
		PRINT("CURRENT MODE: REFERENCE.");
		PRINT(" ");
		
		std::cout << "Hardcoded Parameters: Veclength:     " << d_veclength << std::endl 
				  << "                      Filterlength:  " << d_filterlength << std::endl 
				  << "                      Spacerreducer: " << d_spacereducer << std::endl
				  << "                      Channelwidth:  " << d_channel_width << std::endl << std::endl;
		
		std::cout << "REFERE FRAME SPF: Needed at least: " << d_spf_min_r << " Is: " << d_spf << std::endl;
		std::cout << "TARGET FRAME SPF: Needed at least: " << d_spf_min_t << " Is: " << d_spf << std::endl;
		std::cout << "DISTUR FRAME SPF: Needed at least: " << d_spf_min_d << " Is: " << d_spf << std::endl << std::endl;
		
		int effec_len_ref = (2*d_filterlength + d_spf_min_r - d_spacereducer)*d_veclength;
		int effec_len_tar = (2*d_filterlength + d_spf_min_t - d_spacereducer)*d_veclength;
		int effec_len_dis = (2*d_filterlength + d_spf_min_d - d_spacereducer)*d_veclength;
		
		std::cout << "REFERE FRAME LEN: " << effec_len_ref << " Samples   " <<  30+4+mode_ref_data.payload_len << " Bytes" << std::endl;
		std::cout << "TARGET FRAME LEN: " << effec_len_tar << " Samples   " <<  mode_ref_data.payload_len << " Bytes" << std::endl;
		std::cout << "DISTUR FRAME LEN: " << effec_len_dis << " Samples   " <<  mode_ref_data.payload_dis << " Bytes" << std::endl << std::endl;
		
		if(overwrite_values == true)
		{
			PRINT("OVERWRITING FRAMING PARAMETERS!");
			PRINT(" ");	
							
			int preamble_len = (d_filterlength + 1 - d_spacereducer)*d_veclength;
			int max_preamble_len = (d_filterlength + 1)*d_veclength;
			int input_collect = max_preamble_len + preamble_len + (d_spf + d_filterlength - 1)*d_veclength;
			
			int mult_1000 = (input_collect + effec_len_dis + max_preamble_len + d_veclength + d_veclength + input_collect + max_preamble_len + 1000 - 1) / 1000;
			
			mode_ref_data.ref2ref_samples = mult_1000 * 1000;
			mode_ref_data.case_samples = mode_ref_data.ref2ref_samples - max_preamble_len;
			mode_ref_data.known_offset = effec_len_ref + max_preamble_len + effec_len_dis;
			
			loop0_start = effec_len_ref + max_preamble_len;
			loop0_end = loop0_start + effec_len_dis + effec_len_tar;
			loop0_index = loop0_start;	
		}

		std::cout << "REF2REF SAMPLES: " << mode_ref_data.ref2ref_samples << " Samples" << std::endl;
		std::cout << "CASE SAMPLES:    " << mode_ref_data.case_samples << " Samples" << std::endl;
		std::cout << "KNOWN OFFSET:    " << mode_ref_data.known_offset << " Samples" << std::endl << std::endl;
		
		std::cout << "Loop 0: " << " start: " << loop0_start << " end: " << loop0_end << " step: " << loop0_step << " now: " << loop0_index << std::endl;
		std::cout << "Loop 1: " << " start: " << loop1_start << " end: " << loop1_end << " step: " << loop1_step << " now: " << loop1_index << std::endl << std::endl;
		
		// FOURTH STEP: Check for any errors
		if(d_spf_min_r > d_spf)
		{
			PRINT("--> ERROR DETECTED <--");
			PRINT("Reference frame is too short.");
			PRINT(" ");
			return_value_sc = false;
		}	
		if(d_spf_min_t > d_spf)
		{
			PRINT("--> ERROR DETECTED <--");
			PRINT("Target frame is too short.");
			PRINT(" ");
			return_value_sc = false;
		}
		if(d_spf_min_d > d_spf)
		{
			PRINT("--> ERROR DETECTED <--");
			PRINT("Disturber frame is too short.");
			PRINT(" ");
			return_value_sc = false;
		}
	}	
	
	PRINT(" ");		
	PRINT("/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_");
	PRINT(" ");		
	
	return return_value_sc;
}

//_________________________________________________
void show_loop_info()
{
	PRINT(" ");
	PRINT("---------------------------");
	PRINT("Loop information:");
	std::cout << "Roundtime: " << runtime << " Samples" << std::endl;
	std::cout << "Sleeptime: " << SLEEP_TIME << " Samples" << std::endl;
	std::cout << "Loop 0: " << " start: " << loop0_start << " end: " << loop0_end << " step: " << loop0_step << " now: " << loop0_index << std::endl;
	std::cout << "Loop 1: " << " start: " << loop1_start << " end: " << loop1_end << " step: " << loop1_step << " now: " << loop1_index << std::endl;
	
	uint32_t now = std::chrono::system_clock::now().time_since_epoch()/std::chrono::milliseconds(1);
	
	if(old_time == 0)
	{
		old_time = now;
		
		PRINT(" ");
		PRINT("First loop.");
	}
	else
	{
		// UTC time
		time_t now_ = time(0);
		char* dt_ = ctime(&now_);
		tm *gmtm_ = gmtime(&now_);
		dt_ = asctime(gmtm_);
		
		PRINT(" ");
		PRINT("Time:");
		std::cout << dt_ << std::endl;
		
		// remaining simulation time
		uint32_t passed_time = now - old_time;
		int left_loops0 = (loop0_end - loop0_index - 1) / loop0_step + 1;
		int all_loops1 = (loop1_end - loop1_start - 1) / loop1_step + 1;
		int loops = left_loops0*all_loops1 - (loop1_index-loop1_start) / loop1_step;
		uint32_t left_time = passed_time*loops/1000;

		PRINT("Roundtime:");
		std::cout << passed_time/(float)1000 << " sec" << std::endl << std::endl;
		
		PRINT("Remaining time:");
		std::cout 	<< floor(left_time/60/60) << " h  " 
					<< floor((left_time/60)%60) << " min  "
					<< left_time%60 << " sec " << std::endl;
					
		old_time = now;
	}
	PRINT("---------------------------");
}
