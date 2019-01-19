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
#include "fbmc_add_impl.h"

#include <volk/volk.h>

namespace gr {
  namespace fbmc1 {

    fbmc_add::sptr
    fbmc_add::make(int veclength)
    {
      return gnuradio::get_initial_sptr
        (new fbmc_add_impl(veclength));
    }

    /*
     * The private constructor
     */
    fbmc_add_impl::fbmc_add_impl(int veclength)
      : gr::sync_block("fbmc_add",
              gr::io_signature::make2(2, 2, veclength*sizeof(gr_complex), veclength*sizeof(gr_complex)),
              gr::io_signature::make(1, 1, veclength*sizeof(gr_complex)))
    {
		// init variables
		d_veclength = veclength;
	}

    /*
     * Our virtual destructor.
     */
    fbmc_add_impl::~fbmc_add_impl()
    {
    }

	//______________________________________________________
    int
    fbmc_add_impl::work(int noutput_items,
			  gr_vector_const_void_star &input_items,
			  gr_vector_void_star &output_items)
    {
        const gr_complex *in0 = (const gr_complex *) input_items[0];
        const gr_complex *in1 = (const gr_complex *) input_items[1];
        gr_complex *out = (gr_complex *) output_items[0];

		// misuse of volk float addition for complex
        volk_32f_x2_add_32f((float*) out, (const float*) in0, (const float*) in1, 2*noutput_items*d_veclength);

        // Tell runtime system how many output items we produced.
        return noutput_items;
    }

  } /* namespace fbmc1 */
} /* namespace gr */

