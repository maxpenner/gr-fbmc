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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <gnuradio/io_signature.h>
#include "fbmc_tagged_stream_to_pdu_impl.h"

#include "utils/debug.h"
#include "utils/frame_param.h"

namespace gr {
  namespace fbmc1 {

    fbmc_tagged_stream_to_pdu::sptr
    fbmc_tagged_stream_to_pdu::make(int veclength)
    {
      return gnuradio::get_initial_sptr
        (new fbmc_tagged_stream_to_pdu_impl(veclength));
    }

    /*
     * The private constructor
     */
    fbmc_tagged_stream_to_pdu_impl::fbmc_tagged_stream_to_pdu_impl(int veclength)
      : gr::sync_block("fbmc_tagged_stream_to_pdu",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(0, 0, 0))
    {
		d_STATE = GET_DATA;
		d_veclength = veclength;
		d_length = 0;
		d_counter = 0;
		d_frame = NULL;
		
		// output port
		message_port_register_out(pmt::mp("frame out"));
	}

    /*
     * Our virtual destructor.
     */
    fbmc_tagged_stream_to_pdu_impl::~fbmc_tagged_stream_to_pdu_impl()
    {
		free(d_frame);
    }

    int
    fbmc_tagged_stream_to_pdu_impl::work(int noutput_items,
			  gr_vector_const_void_star &input_items,
			  gr_vector_void_star &output_items)
    {
		const gr_complex *in = (const gr_complex *) input_items[0];

		int consumed  = 0;
		int buffer = 0;

		while(consumed < noutput_items)
		{
			switch(d_STATE)
			{
				//_______________________
				case GET_DATA:
				{
					std::vector<gr::tag_t> length_tag;
					std::vector<gr::tag_t> airtime_tag;
					uint64_t offset = nitems_read(0)+consumed;
					
					// read tags
					get_tags_in_range(length_tag, 0, offset, offset+1, pmt::string_to_symbol("packet_len"));
					get_tags_in_range(airtime_tag, 0, offset, offset+1, pmt::string_to_symbol("airtime"));
					
					if(length_tag.size() == 1)
					{
						if(airtime_tag.size() == 1)
						{
							d_length = (int) pmt::to_long(length_tag[0].value);
							d_airtime = pmt::to_uint64(airtime_tag[0].value);
						}
						else if(airtime_tag.size() == 0)
						{
							PRINT("fbmc_tagged_stream_to_pdu_impl error: No airtime tag.");
							return -1;
						}
						else
						{	
							PRINT("fbmc_tagged_stream_to_pdu_impl error: More than one airtime tag.");
							return -1;
						}
					}
					else if(length_tag.size() == 0)
					{
						PRINT("fbmc_tagged_stream_to_pdu_impl error: No length tag.");
						return -1;
					}
					else
					{
						PRINT("fbmc_tagged_stream_to_pdu_impl error: More than one length tag.");
						return -1;
					}
				
					gr_complex *temp;
					temp = (gr_complex*) realloc(d_frame, d_length*sizeof(gr_complex));
					d_frame = temp;
					d_counter = 0;
					d_STATE = COPY_DATA;
					
					break;
				}
				
				//_______________________
				case COPY_DATA:
				{
					int new_length = d_length - FRAME_APPEND_ZEROS_DROP*d_veclength;
					
					buffer = new_length - d_counter;
					buffer = std::min(buffer, noutput_items-consumed);

					// copy to output
					memcpy(&d_frame[d_counter], &in[consumed], buffer*sizeof(gr_complex));

					d_counter += buffer;
					consumed += buffer;
					
					if(d_counter == new_length)
					{
						// publish blob
						pmt::pmt_t PMT_airtime = pmt::from_uint64(d_airtime);
						pmt::pmt_t dict = pmt::make_dict();
						dict = pmt::dict_add(dict, pmt::mp("airtime"), PMT_airtime);
						pmt::pmt_t PHY_frame_blob = pmt::make_blob(d_frame, new_length*sizeof(gr_complex));
						message_port_pub(pmt::mp("frame out"), pmt::cons(dict, PHY_frame_blob));
						
						d_counter = 0;
						d_STATE = DROP_TAIL;
					}
					
					break;
				}
				
				//_______________________
				case DROP_TAIL:
				
					buffer = FRAME_APPEND_ZEROS_DROP*d_veclength - d_counter;
					buffer = std::min(buffer, noutput_items-consumed);

					// do nothing

					d_counter += buffer;
					consumed += buffer;
					
					if(d_counter == FRAME_APPEND_ZEROS_DROP*d_veclength)
					{
						d_counter = 0;
						d_STATE = GET_DATA;
					}
					
					break;

			}
		}

        return noutput_items;
    }

  } /* namespace fbmc1 */
} /* namespace gr */

