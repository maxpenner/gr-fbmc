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

#ifndef INCLUDED_FBMC1_FBMC_SENSOR_IMPL_H
#define INCLUDED_FBMC1_FBMC_SENSOR_IMPL_H

#include <fbmc1/fbmc_sensor.h>

#include "utils/debug.h"
#include "utils/logical_channel.h"

namespace gr {
  namespace fbmc1 {

    class fbmc_sensor_impl : public fbmc_sensor
    {
     private:

		// variables
		enum {DATA_ACQUISITION, DECODE} d_STATE;
		int d_veclength;			
		int d_spf;
		int d_encoding_fam;
		uint64_t d_receiver_tag;
		int d_log_chann;
		int d_samples_index;
		int d_counter;
		
		std::vector<logical_channel> d_log_channs;
		std::vector<int> d_widths;
		gr_complex *d_samples;
		
		// variables for submode reference
		enum {EXPECT_REF_FRAME_INIT, EXPECT_REF_FRAME, EXPECT_FOUND_FRAME, EXPECT_KNOWN_FRAME, IDLE_DUE_TO_ERROR} d_REFERENCE_SUBMODE;
		uint64_t ref_frame_last_start;		

		// functions
		void send_control_msg_2_transmitter();
		void send_valve_msg();
		void valve_in(pmt::pmt_t msg);
		void control_in(pmt::pmt_t msg);
		
		// functions for decoding
		bool decode_reference_frame();

     public:
      fbmc_sensor_impl(int veclength, int symbols_per_frame, std::vector<int> logical_channels);
      ~fbmc_sensor_impl();

      // Where all the action really happens
      int work(int noutput_items,
	       gr_vector_const_void_star &input_items,
	       gr_vector_void_star &output_items);
    };

  } // namespace fbmc1
} // namespace gr

#endif /* INCLUDED_FBMC1_FBMC_SENSOR_IMPL_H */

