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

#ifndef INCLUDED_FBMC1_FBMC_NORM_AKF_TD_IMPL_H
#define INCLUDED_FBMC1_FBMC_NORM_AKF_TD_IMPL_H

#include <fbmc1/fbmc_norm_akf_td.h>

namespace gr {
  namespace fbmc1 {

    class fbmc_norm_akf_td_impl : public fbmc_norm_akf_td
    {
     private:

		// variables
		int d_nwin; 
		int d_akf_offset; 
		float d_security;
		int d_flush;
		int d_curr_flush;
		int d_flush_rbo;
		int d_counter;
		gr_complex d_akf_window;
		gr_complex d_power_window0;
		gr_complex d_power_window1;
		
		gr_complex *d_akf_history;
		gr_complex *d_power_history;
		
		void new_rbo();

     public:
      fbmc_norm_akf_td_impl(int nwin, int akf_offset, float security, int flush, int flush_rbo);
      ~fbmc_norm_akf_td_impl();

      // Where all the action really happens
      int work(int noutput_items,
	       gr_vector_const_void_star &input_items,
	       gr_vector_void_star &output_items);
    };

  } // namespace fbmc1
} // namespace gr

#endif /* INCLUDED_FBMC1_FBMC_NORM_AKF_TD_IMPL_H */

