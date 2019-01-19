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

#ifndef INCLUDED_FBMC1_FBMC_NORM_AKF_FD_IMPL_H
#define INCLUDED_FBMC1_FBMC_NORM_AKF_FD_IMPL_H

#include <fbmc1/fbmc_norm_akf_fd.h>

namespace gr {
  namespace fbmc1 {
	  
	struct log_chann_akf
	{
		std::vector<int> indices;
		std::vector<float> power_vals;
		std::vector<gr_complex> akf_vals;
		
		void add_power_val(float power_val);
		void add_akf_val(gr_complex akf_val);
		void reset_akf_val();
		float get_mean_power();
		int get_akf_vals_max_index();
	};

    class fbmc_norm_akf_fd_impl : public fbmc_norm_akf_fd
    {
     private:

		// variables 
		int d_veclength;
		float d_threshold;
		float d_security;
		std::vector<gr_complex> d_preamble;
		std::vector<log_chann_akf> log_chann_akf_vec;
		std::vector<gr_complex> d_cfo_estimations;
		
		int d_dbg_log_chann;
		
		// blocking parameters:
		//
		//	When there is high correlation in a logical channel, the receiver gets flooded with wrong detection.
		//
		//	Here, the blocking length defines a minimum length between two frames for each logical channel.
		//
		//	This way the receiver does not get flooded.
		//
		//	The length refers to vectors. So the sample-distance is "number of vectors" x "d_veclength".
		//
		unsigned int d_blocking_length;
		std::vector<uint64_t> d_block_logical_channel_last;		
		
		// functions
		void load_log_chann_akf_vec(std::vector<int> logical_channels);
		void calculate_tau(int &tau, int &tau_offset, float &cfo_angular, log_chann_akf lca, const gr_complex *in);

     public:
      fbmc_norm_akf_fd_impl(int veclength, float threshold, float security, int preamble_set, std::vector<int> logical_channels, int dbg_log_chann);
      ~fbmc_norm_akf_fd_impl();

      // Where all the action really happens
      int work(int noutput_items,
	       gr_vector_const_void_star &input_items,
	       gr_vector_void_star &output_items);
    };

  } // namespace fbmc1
} // namespace gr

#endif /* INCLUDED_FBMC1_FBMC_NORM_AKF_FD_IMPL_H */

