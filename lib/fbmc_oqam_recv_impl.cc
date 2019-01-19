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
#include "fbmc_oqam_recv_impl.h"

#include <volk/volk.h>

namespace gr {
  namespace fbmc1 {

    fbmc_oqam_recv::sptr
    fbmc_oqam_recv::make(int veclength)
    {
      return gnuradio::get_initial_sptr
        (new fbmc_oqam_recv_impl(veclength));
    }

    /*
     * The private constructor
     */
    fbmc_oqam_recv_impl::fbmc_oqam_recv_impl(int veclength)
      : gr::sync_decimator("fbmc_oqam_recv",
              gr::io_signature::make(1, 1, sizeof(gr_complex)*veclength),
              gr::io_signature::make(1, 1, sizeof(gr_complex)*veclength), 2)
    {
		// init variables
		d_veclength = veclength;
		
		size_t alig = volk_get_alignment();
		d_mask = (gr_complex*) volk_malloc(2*d_veclength*sizeof(gr_complex), alig);	
		
		// fill mask vector
		for(int i=0; i<d_veclength; i=i+2)
		{
			d_mask[i]   = gr_complex(1,0);
			d_mask[i+1] = gr_complex(0,-1);
		}
		for(int i=d_veclength; i<d_veclength*2; i=i+2)
		{
			d_mask[i]   = gr_complex(0,-1);
			d_mask[i+1] = gr_complex(1,0);
		}

		// init VOLK
		const int alignment_multiple = volk_get_alignment()/sizeof(gr_complex);
		set_alignment(std::max(1, alignment_multiple));
    }

    /*
     * Our virtual destructor.
     */
    fbmc_oqam_recv_impl::~fbmc_oqam_recv_impl()
    {
		volk_free(d_mask);
    }

	//______________________________________________________
    int
    fbmc_oqam_recv_impl::work(int noutput_items,
			  gr_vector_const_void_star &input_items,
			  gr_vector_void_star &output_items)
    {
        const gr_complex *in = (const gr_complex *) input_items[0];
        gr_complex *out = (gr_complex *) output_items[0];
        
        size_t alig = volk_get_alignment();
        gr_complex *buffer = (gr_complex*) volk_malloc(2*d_veclength*sizeof(gr_complex), alig);
        float *real0 = (float*) volk_malloc(d_veclength*sizeof(float), alig);
        float *real1 = (float*) volk_malloc(d_veclength*sizeof(float), alig);

		// number of input items always a multiple of 2
        for(int i=0; i<noutput_items; i++)
        {
			// multiply with mask
			volk_32fc_x2_multiply_32fc(buffer, &in[i*2*d_veclength], d_mask, d_veclength*2);
			
			// extract two real parts
			volk_32fc_deinterleave_real_32f(real0, buffer, d_veclength);
			volk_32fc_deinterleave_real_32f(real1, &buffer[d_veclength], d_veclength);

			// combine two real parts two one complex vector
			volk_32f_x2_interleave_32fc(&out[i*d_veclength], real0, real1, d_veclength);
		}
		
		volk_free(buffer);
		volk_free(real0);
		volk_free(real1);

        return noutput_items;
    }

  } /* namespace fbmc1 */
} /* namespace gr */

