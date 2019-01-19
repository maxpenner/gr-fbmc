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

#ifndef INCLUDED_FBMC1_FBMC_PHY_ENCODER_IMPL_H
#define INCLUDED_FBMC1_FBMC_PHY_ENCODER_IMPL_H

#include <fbmc1/fbmc_phy_encoder.h>

namespace gr {
  namespace fbmc1 {

    class fbmc_phy_encoder_impl : public fbmc_phy_encoder
    {
     private:

		// functions
		void phy_in(pmt::pmt_t msg);
		void phy2caa_blob(const char *PHY_blob, size_t PHY_blob_len, int32_t *log_chann, size_t log_chann_len);
		void publish_blob(const char *CAA_blob, size_t CAA_blob_len, int32_t *log_chann, size_t log_chann_len);

     public:
      fbmc_phy_encoder_impl();
      ~fbmc_phy_encoder_impl();

      // Where all the action really happens
      int work(int noutput_items,
	       gr_vector_const_void_star &input_items,
	       gr_vector_void_star &output_items);
    };

  } // namespace fbmc1
} // namespace gr

#endif /* INCLUDED_FBMC1_FBMC_PHY_ENCODER_IMPL_H */

