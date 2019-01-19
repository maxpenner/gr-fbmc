/* -*- c++ -*- */
/* 
 * Copyright 2018 <+YOU OR YOUR COMPANY+>.
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

#ifndef INCLUDED_FBMC1_FBMC_LOG_CHANN_STATE_IMPL_H
#define INCLUDED_FBMC1_FBMC_LOG_CHANN_STATE_IMPL_H

#include <fbmc1/fbmc_log_chann_state.h>

#include "utils/logical_channel.h"

#include <chrono>

namespace gr {
  namespace fbmc1 {

    class fbmc_log_chann_state_impl : public fbmc_log_chann_state
    {
     private:

		enum {INIT_TIME, WAIT, COLLECT_SAMPLES} d_STATE;
		int d_counter;
		
		int d_veclength;
		int d_fft_length;
		int d_avg;	
		std::vector<logical_channel> d_log_channs;
		
		// the power in each subcarrier
		std::vector<float> d_carrier_power;
		
		// time
		std::chrono::time_point<std::chrono::system_clock> d_start_time;
		
		// functions
		void estimate_channels_send_message();

     public:
      fbmc_log_chann_state_impl(int veclength, int fft_length, int avg, std::vector<int> logical_channels);
      ~fbmc_log_chann_state_impl();

      // Where all the action really happens
      int work(int noutput_items,
         gr_vector_const_void_star &input_items,
         gr_vector_void_star &output_items);
    };

  } // namespace fbmc1
} // namespace gr

#endif /* INCLUDED_FBMC1_FBMC_LOG_CHANN_STATE_IMPL_H */

