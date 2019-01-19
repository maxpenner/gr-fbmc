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

#ifndef INCLUDED_FBMC1_FBMC_MAC_DECODER_IMPL_H
#define INCLUDED_FBMC1_FBMC_MAC_DECODER_IMPL_H

#include <fbmc1/fbmc_mac_decoder.h>

#include "utils/debug.h"
#include "utils/freq_sync_double_framing.h"

namespace gr {
  namespace fbmc1 {

    class fbmc_mac_decoder_impl : public fbmc_mac_decoder
    {
     private:

		// variables
		int d_encoding_family;
		
		// functions
		void mac_in(pmt::pmt_t msg);
		void mac2app_blob(const char *MAC_blob, size_t MAC_blob_len, uint64_t receiver_tag);
		void publish_blob(const char *APP_blob, size_t APP_blob_len);
				
#ifdef MAC_DECODER_PRINT_RATE
		fbmc_timer *d_mac_timer;
		uint64_t d_phy_frames;
		uint64_t d_phy_errors;
		uint64_t d_mac_frames;
		uint64_t d_mac_frames_ok;
		uint64_t d_mac_bytes;
		uint64_t d_mac_bytes_ok;
		
#ifdef VALVE_FD_DOUBLE_FRAMING
		uint64_t d_phy_double;
		uint64_t d_mac_double;
#endif

		boost::thread *d_thread;
		gr::thread::mutex d_mutex;
		
		void display();
#endif

     public:
      fbmc_mac_decoder_impl(int encoding_family);
      ~fbmc_mac_decoder_impl();

      // Where all the action really happens
      int work(int noutput_items,
	       gr_vector_const_void_star &input_items,
	       gr_vector_void_star &output_items);
    };

  } // namespace fbmc1
} // namespace gr

#endif /* INCLUDED_FBMC1_FBMC_MAC_DECODER_IMPL_H */

