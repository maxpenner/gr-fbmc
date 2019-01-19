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

#ifndef INCLUDED_FBMC_SENSOR_SENSOR_H
#define INCLUDED_FBMC_SENSOR_SENSOR_H

#include <stdint.h>
#include <vector>

#define RANDOM_GEN_CPP_CH 0
#define RANDOM_GEN_CPP_U8 1
#define RANDOM_GEN_CUSTOM 2

class sensor
{
	public:
		
		sensor(float scl, uint8_t enc, uint16_t lc, uint16_t pl, uint32_t st, uint32_t f2f, uint32_t rbo);
		~sensor();
		
		void generate_payload(char *payload);
		
		// getter
		float get_scale(){ return scale; }
		uint8_t get_encoding(){ return encoding; }
		uint16_t get_sensor_ID(){ return sensor_ID; }
		uint16_t get_log_chann(){ return log_chann; }
		uint16_t get_seq_no(){ return seq_no; }
		uint16_t get_payload_len(){ return payload_len; }
		uint32_t get_frames_send(){ return frames_send; }
		uint32_t get_start_time(){ return start_time; }
		uint32_t get_f2f_time(){ return frame2frame_time; }
		uint32_t get_rbo_time(){ return randbackoff_time; }
		uint64_t get_airtime_f2f(){ return airtime_f2f; }
		uint64_t get_airtime(){ return airtime; }
		uint16_t get_interleaving_type(){ return interleaving_type; };
		bool get_alternate_preamble(){ return alternate_preamble; };
		
		// setter
		void set_scale(float scl){ scale = scl; };
		void set_payload_len(uint16_t pl){ payload_len = pl; };
		void set_start_time(uint32_t st){ start_time = st; };
		void set_f2f_time(uint32_t f2f){ frame2frame_time = f2f; };
		void set_rbo_time(uint32_t rbo){ randbackoff_time = rbo; };
		void set_rand_payload(bool rp);
		void set_lock_exception(){ lock_exception = true; };
		void set_random_gen_type(int rgt){ random_gen_type = rgt; };
		void set_random_gen_seed_idx(int rgsi){ random_gen_seed_idx = rgsi; };
		void set_interleaving_type(uint16_t it){ interleaving_type = it; };
		void set_alternate_preamble(bool ap){ alternate_preamble = ap; };
		
		// incremeter
		void inc_seq_no(){ seq_no++; } 
		void inc_frames_send(){ frames_send++; } 
		void inc_airtime();
		
		// resetter
		void reset_seq_no(){ seq_no = 0; } 
		void reset_frames_send(){ frames_send = 0; } 
		void reset_airtime(){ airtime = start_time; airtime_f2f = start_time; }
		
		// for sort
		bool operator<(const sensor &rhs) const { return airtime < rhs.airtime; }
		
		// to lock payload of sensors
		void set_sensor_to_lock_point(){ locked_payload_sensor_ID = (int) sensor_ID; };
		
		// full packet encoding
		void set_insert_custom_header(bool ich) { insert_custom_header = ich; };
		void set_random_seq_no();
		
	private:
	
		// number of sensors
		static uint16_t n_sensor;
				
		// for locking the payload
		static int locked_payload_sensor_ID;
	
		// variables sensor
		float scale;
		uint8_t encoding;
		uint16_t sensor_ID;
		uint16_t log_chann;
	
		// variables frame
		uint16_t seq_no;
		uint16_t payload_len;
		uint32_t frames_send;
		
		// variables timing
		uint32_t start_time;
		uint32_t frame2frame_time;
		uint32_t randbackoff_time;
		uint64_t airtime_f2f;
		uint64_t airtime;
		
		// variables payload
		bool rand_payload;
		std::string fixed_payload;		
		bool lock_exception;
		int random_gen_type;
		int random_gen_seed_idx;
		
		// header insertion
		bool insert_custom_header;
		
		// coding parameters
		uint16_t interleaving_type;
		
		// preamble parameters
		bool alternate_preamble;
};

class sensor_cnt
{
	public:
		
		sensor_cnt(uint16_t sensor_id);
		~sensor_cnt();
		
		bool check_seq_no(uint16_t seq_no);
		
		// getter
		uint16_t get_sensor_ID(){ return sensor_ID; }
		uint32_t get_bytes_received(){ return bytes_received; }
		uint32_t get_frames_received(){ return frames_received; }
		
		// incremeter
		void inc_bytes_received(uint32_t new_bytes){ bytes_received += new_bytes; }
		void inc_frames_received(){ frames_received++; }
		
	private:

		uint16_t sensor_ID;
		bool seq_no_init;
		uint16_t last_seq_no;
		uint32_t bytes_received;
		uint32_t frames_received;
};

struct ReferenceFrameStatistics
{
	ReferenceFrameStatistics();
	~ReferenceFrameStatistics();
	
	bool reference_frame_not_init;
	bool reference_frame_missed;
	bool reference_frame_undecodable;
	bool reference_frame_delayed;
	bool known_frame_missed;
	
	uint32_t recv_ref_frame;
	uint32_t decoded_ref_frame;
	
	uint32_t recv_found_frames;
	uint32_t decoded_found_frames;
	
	uint32_t recv_known_frames;
	uint32_t decoded_known_frames;
};

struct custom_sensor_mac_header
{
	uint16_t seq;
	uint16_t sender_id;
}__attribute__((packed));

#endif /* INCLUDED_FBMC_SENSOR_SENSOR_H */
