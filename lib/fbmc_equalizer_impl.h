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

#ifndef INCLUDED_FBMC1_FBMC_EQUALIZER_IMPL_H
#define INCLUDED_FBMC1_FBMC_EQUALIZER_IMPL_H

#include <fbmc1/fbmc_equalizer.h>

#include "utils/logical_channel.h"

namespace gr {
  namespace fbmc1 {

    class fbmc_equalizer_impl : public fbmc_equalizer
    {
     private:

		// variables 
		enum {TRANSIENT_PREAMBLE, PAYLOAD_SAMPLES} d_STATE;
		int d_veclength;
		int d_trans_symb;
		int d_first_symb;
		int d_frame_symb;
		uint64_t d_receiver_tag;
		int d_log_chann;
		int d_counter;
		
		gr_complex *d_chan_coef;
		std::vector<gr_complex> d_preamble;
		std::vector<logical_channel> d_log_channs;
		
		// ###################################
		// DEBUG: pilots
		int d_pilot_space;
		int d_pilot_space_cnt;
		std::vector<float> d_measuredPilotsPhase;
		gr_complex *d_chan_coef_default;
		// ###################################		

		// functions
		void channel_estimation(const gr_complex *in);
		void add_log_chann_tag(int produced);

     public:
      fbmc_equalizer_impl(int veclength, int filterlength, int symbols_per_frame, int preamble_set, std::vector<int> logical_channels);
      ~fbmc_equalizer_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
		       gr_vector_int &ninput_items,
		       gr_vector_const_void_star &input_items,
		       gr_vector_void_star &output_items);
    };

  } // namespace fbmc1
} // namespace gr

#endif /* INCLUDED_FBMC1_FBMC_EQUALIZER_IMPL_H */