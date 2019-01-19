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
#include "fbmc_norm_akf_td_impl.h"

#include <volk/volk.h>

#include "utils/debug.h"

namespace gr {
  namespace fbmc1 {

    fbmc_norm_akf_td::sptr
    fbmc_norm_akf_td::make(int nwin, int akf_offset, float security, int flush, int flush_rbo)
    {
      return gnuradio::get_initial_sptr
        (new fbmc_norm_akf_td_impl(nwin, akf_offset, security, flush, flush_rbo));
    }

    /*
     * The private constructor
     */
    fbmc_norm_akf_td_impl::fbmc_norm_akf_td_impl(int nwin, int akf_offset, float security, int flush, int flush_rbo)
      : gr::sync_block("fbmc_norm_akf_td",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(1, 1, sizeof(gr_complex)))
    {
		// init variables
		d_nwin 			= nwin; 
		d_akf_offset 	= akf_offset; 
		d_security		= security;
		d_flush			= flush;
		d_flush_rbo		= (flush_rbo < d_flush/2 - 1) ? flush_rbo : d_flush/2 - 1;
		d_counter		= 0;
		d_akf_window 	= gr_complex(0.0f,0.0f);
		d_power_window0 = gr_complex(0.0f,0.0f);
		d_power_window1 = gr_complex(0.0f,0.0f);
		
		//new_rbo();
		
		size_t alig 	= volk_get_alignment();
		d_akf_history 	= (gr_complex*) volk_malloc(d_nwin*sizeof(gr_complex), alig);
		d_power_history = (gr_complex*) volk_malloc((d_akf_offset + d_nwin)*sizeof(gr_complex), alig);
		
		std::fill(d_akf_history, d_akf_history + d_nwin, gr_complex(0.0f, 0.0f));
		std::fill(d_power_history, d_power_history + d_akf_offset + d_nwin, gr_complex(0.0f, 0.0f));

		// init history
		set_history(nwin + akf_offset + 1);
	}

    /*
     * Our virtual destructor.
     */
    fbmc_norm_akf_td_impl::~fbmc_norm_akf_td_impl()
    {
		volk_free(d_akf_history);
		volk_free(d_power_history);
    }
    
    //______________________________________________________
    void fbmc_norm_akf_td_impl::new_rbo()
    {
		d_curr_flush = rand() % (2*d_flush_rbo - 1);
		d_curr_flush -= d_flush_rbo - 1;
		d_curr_flush += d_flush;
    }

	//______________________________________________________
    int
    fbmc_norm_akf_td_impl::work(int noutput_items,
			  gr_vector_const_void_star &input_items,
			  gr_vector_void_star &output_items)
    {
        const gr_complex *in = (const gr_complex *) input_items[0];
        gr_complex *out = (gr_complex *) output_items[0];

		int a = d_nwin;
		int b = d_akf_offset;
		int c = d_akf_offset + d_nwin;
		float power;
		
		// allocate memory
		size_t alig = volk_get_alignment();
		gr_complex *akf_buf = (gr_complex*) volk_malloc((d_nwin + noutput_items)*sizeof(gr_complex), alig);
		gr_complex *power_buf = (gr_complex*) volk_malloc((d_akf_offset + d_nwin + noutput_items)*sizeof(gr_complex), alig);
		
		// copy values that are know from last call of work
		memcpy(akf_buf, d_akf_history, d_nwin*sizeof(gr_complex));
		memcpy(power_buf, d_power_history, (d_akf_offset + d_nwin)*sizeof(gr_complex));
		
		// calculate values
		volk_32fc_x2_multiply_conjugate_32fc(&akf_buf[a], &in[a], &in[c], noutput_items);
		volk_32fc_x2_multiply_conjugate_32fc(&power_buf[c], &in[c], &in[c], noutput_items);

		// slide windows over values
		for(int i=0; i<noutput_items; i++)
		{	
			d_counter++;
			
			// security mechanism against numerical instability
			if(d_counter >= d_flush)
			{
				// DEBUG: use this to print noise level -> adjust power accordingly
				//PRINT(" ");
				//PRINT(d_akf_window);
				//PRINT(d_power_window0);
				//PRINT(d_power_window1);
				
				d_akf_window = gr_complex(0.0f,0.0f);
				d_power_window0 = gr_complex(0.0f,0.0f);
				d_power_window1 = gr_complex(0.0f,0.0f);
				
				for(int j=0; j<d_nwin; j++)
				{
					d_akf_window += akf_buf[i+j];
					d_power_window0 += power_buf[b+j];
					d_power_window1 += power_buf[i+j];
				}
				
				new_rbo();
				d_counter = 0;
			}
			
			// refresh akf window
			d_akf_window += akf_buf[a];
			d_akf_window -= akf_buf[i];
			
			// refresh power windows
			d_power_window0 += power_buf[c];
			d_power_window0 -= power_buf[b];
			d_power_window1 += power_buf[a];
			d_power_window1 -= power_buf[i];
			
			// security mechanism against two low noise
			power = std::max(abs(d_power_window0), abs(d_power_window1));

			if(power < d_security)
				power = d_security;

			out[i] = d_akf_window/power;
			
			a++;
			b++;
			c++;
		}
		
		// save values for next call
		memcpy(d_akf_history, &akf_buf[noutput_items], d_nwin*sizeof(gr_complex));
		memcpy(d_power_history, &power_buf[noutput_items], (d_akf_offset + d_nwin)*sizeof(gr_complex));
		
		// clean up
		volk_free(akf_buf);
		volk_free(power_buf);

        return noutput_items;
    }

  } /* namespace fbmc1 */
} /* namespace gr */

