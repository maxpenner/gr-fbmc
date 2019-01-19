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

#ifndef INCLUDED_FBMC1_FBMC_TAGGED_STREAM_TO_PDU_IMPL_H
#define INCLUDED_FBMC1_FBMC_TAGGED_STREAM_TO_PDU_IMPL_H

#include <fbmc1/fbmc_tagged_stream_to_pdu.h>

namespace gr {
  namespace fbmc1 {

    class fbmc_tagged_stream_to_pdu_impl : public fbmc_tagged_stream_to_pdu
    {
     private:

		enum {GET_DATA, COPY_DATA, DROP_TAIL} d_STATE;

		int d_veclength, d_length, d_counter;
		uint64_t d_airtime;
		gr_complex *d_frame;

     public:
      fbmc_tagged_stream_to_pdu_impl(int veclength);
      ~fbmc_tagged_stream_to_pdu_impl();

      // Where all the action really happens
      int work(int noutput_items,
	       gr_vector_const_void_star &input_items,
	       gr_vector_void_star &output_items);
    };

  } // namespace fbmc1
} // namespace gr

#endif /* INCLUDED_FBMC1_FBMC_TAGGED_STREAM_TO_PDU_IMPL_H */

