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

#ifndef INCLUDED_FBMC1_FBMC_VALVE_FD_IMPL_H
#define INCLUDED_FBMC1_FBMC_VALVE_FD_IMPL_H

#include <fbmc1/fbmc_valve_fd.h>

#include "utils/debug.h"
#include "utils/freq_sync_double_framing.h"

namespace gr {
  namespace fbmc1 {
	  
	struct frame
	{
		frame(uint64_t sti, uint64_t rt, float cfo);
		~frame();
		
		gr_complex *data;
		uint64_t start_index;
		uint64_t recv_tag;
		float cfo_angular;
		int data_cnt;
		
		static int frame_len;
	};

    class fbmc_valve_fd_impl : public fbmc_valve_fd
    {
     private:
		
		// variables
		int d_veclength;
		int d_filterlength;
		int d_frame_len;
		int d_n_log_chann;
		int d_counter;
		std::vector<frame*> frame_vec;
		
#ifdef VALVE_FD_DOUBLE_FRAMING
		int boundary_low;
		int boundary_high;
#endif
		
#ifdef VALVE_FD_PRINT_RATE
		fbmc_timer *d_valve_timer;
		uint64_t d_frames_received;
		
		void display();
#endif

     public:
      fbmc_valve_fd_impl(int veclength, int filterlength, int symbols_per_frame, std::vector<int> logical_channels);
      ~fbmc_valve_fd_impl();

      // Where all the action really happens
      int work(int noutput_items,
	       gr_vector_const_void_star &input_items,
	       gr_vector_void_star &output_items);
    };

  } // namespace fbmc1
} // namespace gr

#endif /* INCLUDED_FBMC1_FBMC_VALVE_FD_IMPL_H */

