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
#include "fbmc_pdu_to_tagged_stream_impl.h"

#include "utils/debug.h"

namespace gr {
  namespace fbmc1 {

    fbmc_pdu_to_tagged_stream::sptr
    fbmc_pdu_to_tagged_stream::make(int veclength, int microseconds)
    {
      return gnuradio::get_initial_sptr
        (new fbmc_pdu_to_tagged_stream_impl(veclength, microseconds));
    }

    /*
     * The private constructor
     */
    fbmc_pdu_to_tagged_stream_impl::fbmc_pdu_to_tagged_stream_impl(int veclength, int microseconds)
      : gr::sync_block("fbmc_pdu_to_tagged_stream",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make2(2, 2, sizeof(gr_complex), sizeof(gr_complex)))
    {
		// init variables
		d_STATE 			= INPUT;
		d_veclength			= veclength;
		d_microseconds		= microseconds;
		d_receiver_tag		= 0;
		d_counter			= 0;
		
		d_FRAME_blob 		= NULL;
		d_FRAME_blob_len 	= 0;
		
		// input port
		message_port_register_in(pmt::mp("frame in"));
	}

    /*
     * Our virtual destructor.
     */
    fbmc_pdu_to_tagged_stream_impl::~fbmc_pdu_to_tagged_stream_impl()
    {
		free(d_FRAME_blob);
    }

    //______________________________________________________
    bool fbmc_pdu_to_tagged_stream_impl::check_input()
    {
		pmt::pmt_t msg(delete_head_nowait(pmt::intern("frame in")));
		if (msg.get() == NULL)
			return false;
	
		const gr_complex *FRAME_blob;

		// input blob extraction
		if(pmt::is_pair(msg))
		{
			// cdr = blob
			FRAME_blob = reinterpret_cast<const gr_complex *>(pmt::blob_data(pmt::cdr(msg)));
			d_FRAME_blob_len = pmt::blob_length(pmt::cdr(msg));
			d_FRAME_blob_len /= sizeof(gr_complex);
			
			// car == dictionary = keys + values
			d_receiver_tag = pmt::to_uint64(pmt::dict_ref(pmt::car(msg), pmt::mp("receiver_tag"), pmt::PMT_NIL));
			
		}  
		else
		{
			throw std::invalid_argument("fbmc_pdu_to_tagged_stream_impl error: Expect PDUs as input. Dropping data.");
			return false;
		}
		
		// make local copy of frame
		d_FRAME_blob = (gr_complex*) malloc(d_FRAME_blob_len*sizeof(gr_complex));
		memcpy(d_FRAME_blob, FRAME_blob, d_FRAME_blob_len*sizeof(gr_complex));
		
		// outputs are delayed, actual frame is smaller that 'd_FRAME_blob_len'
		d_FRAME_blob_len -= d_veclength/2;
	
		return true;
    }
    
    //______________________________________________________
    void fbmc_pdu_to_tagged_stream_impl::add_receiver_tag(int produced)
    {
		pmt::pmt_t key = pmt::string_to_symbol("receiver_tag");
		pmt::pmt_t value = pmt::from_uint64(d_receiver_tag);
		add_item_tag(0, nitems_written(0)+produced, key, value, pmt::PMT_NIL);
    }
    
    //______________________________________________________
    int
    fbmc_pdu_to_tagged_stream_impl::work(int noutput_items,
			  gr_vector_const_void_star &input_items,
			  gr_vector_void_star &output_items)
    {
		gr_complex *out0 = (gr_complex *) output_items[0];
		gr_complex *out1 = (gr_complex *) output_items[1];
		
		int produced  = 0;
		bool token = false;

		while(produced < noutput_items && token == false)
		{
			switch(d_STATE)
			{
				//_______________________
				case INPUT:
				
					// check input for new FRAME_blob
					if(check_input() == false)
					{
						if(d_microseconds > 0)
							boost::this_thread::sleep(boost::posix_time::microseconds(d_microseconds));
						token = true;
						break;
					}
					
					d_counter = 0;
					d_STATE = COPY;
				
				//_______________________
				case COPY:
				{
					int buffer = d_FRAME_blob_len - d_counter;
					buffer = std::min(noutput_items-produced, buffer);
					
					if(d_counter == 0)
						add_receiver_tag(produced);
				
					memcpy(&out0[produced], &d_FRAME_blob[d_counter], buffer*sizeof(gr_complex));
					memcpy(&out1[produced], &d_FRAME_blob[d_counter + d_veclength/2], buffer*sizeof(gr_complex));
					
					d_counter += buffer;
					produced  += buffer;
					
					if(d_counter == d_FRAME_blob_len)
					{
						free(d_FRAME_blob);
						d_FRAME_blob = NULL;
						d_STATE = INPUT;
					}
					
					break;
				}	
	    	}
		}
		
        return produced;
    }

  } /* namespace fbmc1 */
} /* namespace gr */

