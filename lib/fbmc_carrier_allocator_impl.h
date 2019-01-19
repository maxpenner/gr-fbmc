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

#ifndef INCLUDED_FBMC1_FBMC_CARRIER_ALLOCATOR_IMPL_H
#define INCLUDED_FBMC1_FBMC_CARRIER_ALLOCATOR_IMPL_H

#include <fbmc1/fbmc_carrier_allocator.h>

#include "utils/debug.h"
#include "utils/logical_channel.h"

namespace gr {
  namespace fbmc1 {
	  
	struct ca_ppdu
	{
		ca_ppdu();
		~ca_ppdu();
		
		uint8_t *d_ppdu;
		int d_ppdu_len;
		int d_ppdu_index;
		int d_residual_reset;
		
		int d_spf;
		int d_log_chann;
		float d_scale;
		bool d_altern_pream;
		uint8_t d_encoding;
		uint64_t d_airtime;
	};

    class fbmc_carrier_allocator_impl : public fbmc_carrier_allocator
    {
     private:

		// variables
		enum {INPUT, PREAMBLE, SPACER, PPDU, APPEND_ZEROS} d_STATE;
		int d_veclength;
		int d_filterlength;
		int d_preamble_set;
		int d_microseconds;
		int d_counter;
		std::vector<gr_complex> d_preamble;
		std::vector<gr_complex> d_constellation;
		std::vector<logical_channel> d_log_channs;
		std::vector<ca_ppdu*> d_ca_ppdu_vec;
		
		// ###################################
		// DEBUG: pilots
		int d_pilot_space;
		int d_pilot_space_cnt;
		std::vector<gr_complex> d_pilotFollowUp;
		int d_pilotWidth;
		std::vector<float> d_pilotSum;
		// ###################################
		
		// functions
		void caa_in(pmt::pmt_t msg);
		void add_length_airtime_tag(int produced);

     public:
      fbmc_carrier_allocator_impl(int veclength, int filterlength, int preamble_set, int microseconds);
      ~fbmc_carrier_allocator_impl();

      // Where all the action really happens
      int work(int noutput_items,
	       gr_vector_const_void_star &input_items,
	       gr_vector_void_star &output_items);
    };

  } // namespace fbmc1
} // namespace gr

#endif /* INCLUDED_FBMC1_FBMC_CARRIER_ALLOCATOR_IMPL_H */

