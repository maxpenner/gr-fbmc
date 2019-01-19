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
#include "fbmc_oqam_impl.h"

#include <volk/volk.h>

namespace gr {
  namespace fbmc1 {

    fbmc_oqam::sptr
    fbmc_oqam::make(int veclength)
    {
      return gnuradio::get_initial_sptr
        (new fbmc_oqam_impl(veclength));
    }

    /*
     * The private constructor
     */
    fbmc_oqam_impl::fbmc_oqam_impl(int veclength)
      : gr::sync_interpolator("fbmc_oqam",
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
			d_mask[i+1] = gr_complex(0,1);
		}
		for(int i=d_veclength; i<d_veclength*2; i=i+2)
		{
			d_mask[i]   = gr_complex(0,1);
			d_mask[i+1] = gr_complex(1,0);
		}

		// init VOLK
		const int alignment_multiple = volk_get_alignment()/sizeof(gr_complex);
		set_alignment(std::max(1, alignment_multiple));
    }

    /*
     * Our virtual destructor.
     */
    fbmc_oqam_impl::~fbmc_oqam_impl()
    {
		volk_free(d_mask);
    }

	//______________________________________________________
    int
    fbmc_oqam_impl::work(int noutput_items,
			  gr_vector_const_void_star &input_items,
			  gr_vector_void_star &output_items)
    {
        const gr_complex *in = (const gr_complex *) input_items[0];
        gr_complex *out = (gr_complex *) output_items[0];
        
        size_t alig = volk_get_alignment();
        float *real = (float*) volk_malloc(d_veclength*sizeof(float), alig);
        float *imag = (float*) volk_malloc(d_veclength*sizeof(float), alig);
        float *zero = (float*) volk_malloc(d_veclength*sizeof(float), alig);
       
        memset(zero, 0x00, d_veclength*sizeof(float));

		// noutput_items always a multiple of 2
		for(int i=0; i<noutput_items/2; i++)
		{
			// seperate real and imag of input
			volk_32fc_deinterleave_32f_x2(real, imag, &in[i*d_veclength], d_veclength);
			
			// create new vectors with real/imag + zero-imag
			volk_32f_x2_interleave_32fc(&out[2*i*d_veclength], real, zero, d_veclength);
			volk_32f_x2_interleave_32fc(&out[(2*i+1)*d_veclength], imag, zero, d_veclength);
			
			// multiply with mask
			volk_32fc_x2_multiply_32fc(&out[2*i*d_veclength], &out[2*i*d_veclength], d_mask, 2*d_veclength);
		}
		
		// clean up
		volk_free(real);
		volk_free(imag);
		volk_free(zero);

        return noutput_items;
    }

  } /* namespace fbmc1 */
} /* namespace gr */