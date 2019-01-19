/* -*- c++ -*- */
/* 
 * Copyright 2016 <+YOU OR YOUR COMPANY+>.
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

#ifndef INCLUDED_FBMC1_FBMC_EQU_OQAM_PHY_IMPL_H
#define INCLUDED_FBMC1_FBMC_EQU_OQAM_PHY_IMPL_H

#include <fbmc1/fbmc_equ_oqam_phy.h>

#include "utils/debug.h"
#include "utils/logical_channel.h"

namespace gr {
  namespace fbmc1 {

    class fbmc_equ_oqam_phy_impl : public fbmc_equ_oqam_phy
    {
     private:

		// input variables
		int d_veclength;
		int d_filterlength;
		int d_spf;
		int d_preamble_set;
		int d_encoding_fam;
		std::vector<logical_channel> d_log_channs;
		
		// streaming
		uint64_t d_receiver_tag;
		
		// frame vector counter
		int d_counter;
		
		// frame collection
		int d_trans_symbols;
		int d_frame_symbols;
		gr_complex *d_frame;
		
		// equalizer
		std::vector<gr_complex> d_preamble;
		void channel_estimation();
		void phase_tracking();
		
		// oqam
		gr_complex *d_mask;
		void oqamization();
		
		// phy decoding
		std::vector<int> d_widths;
		gr_complex *d_samples;
		int d_samples_index;	
		void phy_decode();
		
		// statistics send to mac block
#ifdef MAC_DECODER_PRINT_RATE
		uint64_t d_phy_frames;
		uint64_t d_phy_errors;
#ifdef VALVE_FD_DOUBLE_FRAMING
		uint64_t d_phy_double;
#endif
		boost::thread *d_thread;
		gr::thread::mutex d_mutex;
		void send_stats();
#endif

     public:
      fbmc_equ_oqam_phy_impl(int veclength, int filterlength, int symbols_per_frame, int preamble_set, int encoding_family, std::vector<int> logical_channels);
      ~fbmc_equ_oqam_phy_impl();

      // Where all the action really happens
      int work(int noutput_items,
	       gr_vector_const_void_star &input_items,
	       gr_vector_void_star &output_items);
    };

  } // namespace fbmc1
} // namespace gr

#endif /* INCLUDED_FBMC1_FBMC_EQU_OQAM_PHY_IMPL_H */