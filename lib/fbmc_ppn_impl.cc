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
#include "fbmc_ppn_impl.h"

#include <math.h>
#include <volk/volk.h>

namespace gr {
  namespace fbmc1 {

    fbmc_ppn::sptr
    fbmc_ppn::make(int veclength, int filterlength, float normalize)
    {
      return gnuradio::get_initial_sptr
        (new fbmc_ppn_impl(veclength, filterlength, normalize));
    }

    /*
     * The private constructor
     */
    fbmc_ppn_impl::fbmc_ppn_impl(int veclength, int filterlength, float normalize)
      : gr::sync_block("fbmc_ppn",
              gr::io_signature::make(1, 1, sizeof(gr_complex)*veclength),
              gr::io_signature::make2(2, 2, sizeof(gr_complex)*veclength, sizeof(gr_complex)*veclength))
    {
		// init variables
		d_veclength 	= veclength;
		d_filterlength 	= filterlength;
		
		size_t alig = volk_get_alignment();
		d_filter = (gr_complex*) volk_malloc(d_filterlength*d_veclength*sizeof(gr_complex), alig);

		// init filter
		calculate_filter_fbmc(d_filter, d_veclength, d_filterlength, normalize, 0);

		// init history
		set_history(d_filterlength*2-1);

		// init VOLK
		const int alignment_multiple = volk_get_alignment()/sizeof(gr_complex);
		set_alignment(std::max(1, alignment_multiple));
    }

    /*
     * Our virtual destructor.
     */
    fbmc_ppn_impl::~fbmc_ppn_impl()
    {
		volk_free(d_filter);
    }

	//______________________________________________________
    int
    fbmc_ppn_impl::work(int noutput_items,
			  gr_vector_const_void_star &input_items,
			  gr_vector_void_star &output_items)
    {
        const gr_complex *in = (const gr_complex *) input_items[0];
        gr_complex *out0 = (gr_complex *) output_items[0];
        gr_complex *out1 = (gr_complex *) output_items[1];
        
        size_t alig = volk_get_alignment();
		gr_complex *buffer = (gr_complex*) volk_malloc(d_filterlength*d_veclength*sizeof(gr_complex), alig);

		for(int i=0; i<noutput_items; i++)
		{
			// multiply input with filter into buffer
			for(int j=0; j<d_filterlength; j++)
				volk_32fc_x2_multiply_32fc(&buffer[j*d_veclength], &in[(i+2*j)*d_veclength], &d_filter[j*d_veclength], d_veclength);
			
			// sum buffer (misuse of volk float addition for complex)
			for(int k=1; k<d_filterlength; k++)
				volk_32f_x2_add_32f((float*) buffer, (const float*) buffer, (const float*) &buffer[k*d_veclength], 2*d_veclength);
				
			// copy sum into outputs
			memcpy(&out0[i*d_veclength], buffer, d_veclength*sizeof(gr_complex));
			memcpy(&out1[i*d_veclength], buffer, d_veclength*sizeof(gr_complex));
		}
		
		volk_free(buffer);

		return noutput_items;
    }

  } /* namespace fbmc1 */
} /* namespace gr */

