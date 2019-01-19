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

#ifndef INCLUDED_FBMC1_FBMC_PAYLOAD_GENERATOR_IMPL_H
#define INCLUDED_FBMC1_FBMC_PAYLOAD_GENERATOR_IMPL_H

#include <fbmc1/fbmc_payload_generator.h>

namespace gr {
  namespace fbmc1 {

    class fbmc_payload_generator_impl : public fbmc_payload_generator
    {
     private:

		// variables
		enum {VIDEO, SENSOR} d_MODE;
		int d_spf;
		std::vector<int> d_logical_channels;
		
		// variables video mode
		int d_video_encoding;
		int d_channel2use;
		
		// variables sensor mode
		int d_sensor_chunk_len;
		int d_sensor_skip;
		uint64_t d_old_time;

		// functions
		void control_in(pmt::pmt_t msg);
		void app_in(pmt::pmt_t msg);
		void ch_state_in(pmt::pmt_t msg);
		void mode_video(const char *APP_blob, size_t APP_blob_len);
		void mode_sensor(uint64_t latest_time);
		void publish_blob(const char *MAC_blob, size_t MAC_blob_len);

     public:
      fbmc_payload_generator_impl(int mode, int symbols_per_frame, std::vector<int> logical_channels, int video_encoding, int sensor_chunk_len, int sensor_skip);
      ~fbmc_payload_generator_impl();

      // Where all the action really happens
      int work(int noutput_items,
	       gr_vector_const_void_star &input_items,
	       gr_vector_void_star &output_items);
    };

  } // namespace fbmc1
} // namespace gr

#endif /* INCLUDED_FBMC1_FBMC_PAYLOAD_GENERATOR_IMPL_H */

