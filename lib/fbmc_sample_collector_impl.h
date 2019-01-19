/* -*- c++ -*- */
/*
 * Copyright 2015 <+YOU OR YOUR COMPANY+>.
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

#ifndef INCLUDED_FBMC1_FBMC_SAMPLE_COLLECTOR_IMPL_H
#define INCLUDED_FBMC1_FBMC_SAMPLE_COLLECTOR_IMPL_H

#include <fbmc1/fbmc_sample_collector.h>

#include "utils/tx_delay_rx_measure.h"
#include "utils/debug.h"

#include <chrono>

namespace gr {
  namespace fbmc1 {

	struct chunk
	{
		chunk();
		~chunk();

		void chunk_release();

		gr_complex *data;

		static int chunk_len;
		static uint64_t low_time;
		static uint64_t high_time;
	};

    class fbmc_sample_collector_impl : public fbmc_sample_collector
    {
     private:

		// variables
		enum {VIDEO_ZEROS_BURST_START, VIDEO_INPUT, VIDEO_COPY, VIDEO_RESIDUAL, VIDEO_ZEROS_BURST_END, VIDEO_BLOCK, SENSOR_PRE_PUSH, SENSOR_CHOOSE, SENSOR_COPY} d_STATE;
		int d_counter;

		// variables video mode
		gr_complex *d_video_frame;
		int d_video_frame_len;

		// variables sensor mode
		chunk *d_sensor_chunk;
		int d_sensor_chunk_len;
		int d_sensor_pre_push;
		int d_sensor_skip;
		uint64_t d_sensor_chunk_time;
		std::vector<chunk*> d_sensor_chunk_vec;
		
		// the transmission can be interrupted, to measure this we need a time tag
		std::chrono::time_point<std::chrono::system_clock> d_start_time;

		// functions
		bool check_input_video();
		void check_input_sensor(pmt::pmt_t msg);

#ifdef SAMPLE_COLLECTOR_PRINT_RATE
		fbmc_timer *d_sample_coll_timer;
		uint64_t d_frames_found;
		uint64_t d_frames_send;

		void display();
#endif

     public:
      fbmc_sample_collector_impl(int mode, int sensor_chunk_len, int sensor_pre_push, int sensor_skip);
      ~fbmc_sample_collector_impl();

      // Where all the action really happens
      int work(int noutput_items,
	       gr_vector_const_void_star &input_items,
	       gr_vector_void_star &output_items);
    };

  } // namespace fbmc1
} // namespace gr

#endif /* INCLUDED_FBMC1_FBMC_SAMPLE_COLLECTOR_IMPL_H */
