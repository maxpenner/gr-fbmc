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
#include "fbmc_p2s_impl.h"

namespace gr {
  namespace fbmc1 {

    fbmc_p2s::sptr
    fbmc_p2s::make(int veclength, int delayed)
    {
      return gnuradio::get_initial_sptr
        (new fbmc_p2s_impl(veclength, delayed));
    }

    /*
     * The private constructor
     */
    fbmc_p2s_impl::fbmc_p2s_impl(int veclength, int delayed)
      : gr::sync_decimator("fbmc_p2s",
              gr::io_signature::make(1, 1, sizeof(gr_complex)*veclength),
              gr::io_signature::make(1, 1, sizeof(gr_complex)*veclength), 2)
    {
		// init variables
		d_veclength = veclength;
		d_delayed = delayed;

		// init history and propagation policy for tags
		if(d_delayed==1)
		{
			set_history(2);
			set_tag_propagation_policy(TPP_DONT);
		}
	}

    /*
     * Our virtual destructor.
     */
    fbmc_p2s_impl::~fbmc_p2s_impl()
    {
    }

	//______________________________________________________
    int
    fbmc_p2s_impl::work(int noutput_items,
			  gr_vector_const_void_star &input_items,
			  gr_vector_void_star &output_items)
    {
        const gr_complex *in = (const gr_complex *) input_items[0];
        gr_complex *out = (gr_complex *) output_items[0];

		if(d_delayed == 0)
		{
			// copy each seconds item
			for(int i=0; i<noutput_items; i++)
				memcpy(&out[i*d_veclength], &in[2*i*d_veclength], d_veclength*sizeof(gr_complex));	
		}
		else
		{
			int vec_1_2 = d_veclength/2;
			
			// for first element copy second half only
			memcpy(out, &in[vec_1_2], vec_1_2*sizeof(gr_complex));
			
			for(int i=1; i<noutput_items; i++)
				memcpy(&out[(i-1)*d_veclength+vec_1_2], &in[2*i*d_veclength], d_veclength*sizeof(gr_complex));
							
			// for last element copy first half only
			memcpy(&out[(noutput_items-1)*d_veclength+vec_1_2], &in[2*noutput_items*d_veclength], vec_1_2*sizeof(gr_complex));
		}
        
        return noutput_items;
    }

  } /* namespace fbmc1 */
} /* namespace gr */

