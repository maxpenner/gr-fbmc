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

#ifndef INCLUDED_FBMC1_FBMC_VALVE_TD_IMPL_H
#define INCLUDED_FBMC1_FBMC_VALVE_TD_IMPL_H

#include <fbmc1/fbmc_valve_td.h>

#include <gnuradio/filter/fir_filter.h>

#include "utils/debug.h"

namespace gr {
  namespace fbmc1 {

    class fbmc_valve_td_impl : public fbmc_valve_td
    {
     private:

		// variables
		enum {FREERUN, REFERENCE} d_MODE;
		enum {	COARSE_SYNC, 
				COLLECT_FRAME, 
				COPY_DATA, 
				CHECK_RESIDUAL, 
				COPY_RESIDUAL} d_FREERUN_STATE;
		enum {	COARSE_SYNC_REF, 
				COLLECT_FRAME_REF, 
				COPY_DATA_REF,
				
				COARSE_SYNC_FREE, 
				COLLECT_FRAME_FREE, 
				COPY_DATA_FREE, 
				CHECK_RESIDUAL_FREE, 
				COPY_RESIDUAL_FREE, 
				
				COPY_KNOWN} d_REFEREN_STATE;
		
		// general variables
		int d_veclength;
		int d_preamble_len;
		int d_samp_frame;
		int d_input_collect;
		int d_counter;
		bool d_init_lock;
		std::vector<gr_complex> d_complex_peaks;
		
		// general variables for cfo correction
		float d_cfo_angular;

		// general variables for synchronisation
		float d_threshold;	
		int d_plateau;
		int d_peak_index;
		int d_type;	
		int d_len;
		int d_offset;
		int d_symm;	
		int d_refresh;
		uint64_t d_sync_counter;
		uint64_t d_cut_start;
		uint64_t d_frame_start;

		gr_complex *d_frame_in0;
		gr_complex *d_frame_in1;
		gr_complex *d_preamble;

		gr::filter::kernel::fir_filter_ccc *d_fir;
		
		// variables for mode REFERENCE
		int d_case_samples;
		int d_known_offset;
		int d_found_unsecu;	
		float d_refer_thresh;
		uint64_t d_refer_start;
		uint64_t d_known_start;
		gr_complex *d_known_frame;	
		int d_known_frame_cnt;
		int d_free_search_len;
		bool d_free_lock;

		// functions
		void preload_fir_filter();
		void cfo_fine_sync();
		void add_receiver_tag(int produced, int frame_type);
		void valve_in(pmt::pmt_t msg);

#ifdef VALVE_TD_PRINT_RATE
		fbmc_timer *d_valve_timer;
		float d_snr_avg;
		std::vector<float> d_snr_shift_reg;
		
		void snr_estimation();
		void display();
#endif

     public:
      fbmc_valve_td_impl(int veclength, int filterlength, int symbols_per_frame, float threshold, int plateau, int preamble_set, int sync_type, int sync_len, int sync_symm, int sync_refresh, int init_lock);
      ~fbmc_valve_td_impl();

      // Where all the action really happens
      void forecast (int noutput_items, gr_vector_int &ninput_items_required);

      int general_work(int noutput_items,
		       gr_vector_int &ninput_items,
		       gr_vector_const_void_star &input_items,
		       gr_vector_void_star &output_items);
    };

  } // namespace fbmc1
} // namespace gr

#endif /* INCLUDED_FBMC1_FBMC_VALVE_TD_IMPL_H */

