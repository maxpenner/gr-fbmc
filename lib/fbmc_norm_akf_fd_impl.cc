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
#include "fbmc_norm_akf_fd_impl.h"

#include "utils/debug.h"
#include "utils/logical_channel.h"
#include "utils/preamble_set_freq_domain.h"

/* history for this block:
 * 
 * 		4 old vectors: two old vectors for akf and two old vectors if akf_vals[0] is the main peak in 'calculate_tau()')
 * 
 */
#define AKF_HISTORY 4
#define AKF_SPAN 3
#define CFO_FRAMES_ESTIM_LENGTH 50
#define VOLK_FAST_MULTIPLICATION_OVER_NOISE

#ifdef VOLK_FAST_MULTIPLICATION_OVER_NOISE
#include <volk/volk.h>
#endif

namespace gr {
  namespace fbmc1 {

	//______________________________________________________
	void log_chann_akf::add_power_val(float power_val)
	{	
		power_vals.erase(power_vals.begin());
		power_vals.push_back(power_val);
	}
	
	//______________________________________________________
	void log_chann_akf::add_akf_val(gr_complex akf_val)
	{	
		akf_vals.erase(akf_vals.begin());
		akf_vals.push_back(akf_val);
	}
	
	//______________________________________________________
	void log_chann_akf::reset_akf_val()
	{	
		std::fill(akf_vals.begin(), akf_vals.end(), gr_complex(0.0f, 0.0f));
	}
	
	//______________________________________________________
	float log_chann_akf::get_mean_power()
	{	
		return (power_vals[0] + power_vals[2])/2;
	}
	
	//______________________________________________________
	int log_chann_akf::get_akf_vals_max_index()
	{	
		float max = abs(akf_vals[0]);
		int max_idx = 0;
		
		for(int i=1; i<akf_vals.size(); i++)
		{
			float temp = abs(akf_vals[i]);
			
			if(temp > max)
			{
				max = temp;
				max_idx = i;
			}
		}
		
		return max_idx;
	}

	//______________________________________________________
    fbmc_norm_akf_fd::sptr
    fbmc_norm_akf_fd::make(int veclength, float threshold, float security, int preamble_set, std::vector<int> logical_channels, int dbg_log_chann)
    {
      return gnuradio::get_initial_sptr
        (new fbmc_norm_akf_fd_impl(veclength, threshold, security, preamble_set, logical_channels, dbg_log_chann));
    }

    /*
     * The private constructor
     */
    fbmc_norm_akf_fd_impl::fbmc_norm_akf_fd_impl(int veclength, float threshold, float security, int preamble_set, std::vector<int> logical_channels, int dbg_log_chann)
      : gr::sync_block("fbmc_norm_akf_fd",
              gr::io_signature::make(1, 1, sizeof(gr_complex)*veclength),
              gr::io_signature::make2(2, 2, sizeof(float)*veclength, sizeof(int)*veclength))
    {
		// init variables
		d_veclength	= veclength;
		d_threshold	= threshold;
		d_security	= security;
		
		// init preamble
		get_scaled_preamble_set_freq_domain(preamble_set, d_veclength, 1.0, d_preamble);
		
		// init structures for logical channels
		load_log_chann_akf_vec(logical_channels);
		
		// init index for debugging logical channel
		d_dbg_log_chann = (dbg_log_chann >= 0) ? dbg_log_chann : 0;
		d_dbg_log_chann = (dbg_log_chann < log_chann_akf_vec.size()) ? dbg_log_chann : log_chann_akf_vec.size() - 1;

		// init history
		set_history(AKF_HISTORY + 1);
		
		// set blocking parameters
		d_blocking_length = 30;
		for (int i=0; i<log_chann_akf_vec.size(); i++)
			d_block_logical_channel_last.push_back(0);

#ifdef VOLK_FAST_MULTIPLICATION_OVER_NOISE
		// init VOLK
		const int alignment_multiple = volk_get_alignment()/sizeof(gr_complex);
		set_alignment(std::max(1, alignment_multiple));
#endif
    }

    /*
     * Our virtual destructor.
     */
    fbmc_norm_akf_fd_impl::~fbmc_norm_akf_fd_impl()
    {
    }

    //______________________________________________________
    void fbmc_norm_akf_fd_impl::load_log_chann_akf_vec(std::vector<int> logical_channels)
    {
		// first extract the logical channels from coded input
		std::vector<logical_channel> extracted_log_channs;
		logical_channel_convert(&logical_channels[0], logical_channels.size(), extracted_log_channs);
		
		// for each logical channel add new structure to 'log_chann_akf_vec'
		for(int i=0; i<extracted_log_channs.size(); i++)
		{
			log_chann_akf_vec.push_back(log_chann_akf());
			
			// set indices
			logical_channel_convert_indices(extracted_log_channs[i], log_chann_akf_vec[i].indices);
			
			// init structure vectors
			log_chann_akf_vec[i].power_vals.resize(AKF_SPAN, 0.0f);
			log_chann_akf_vec[i].akf_vals.resize(AKF_SPAN, gr_complex(0.0f, 0.0f));
		}
	}
	
    //______________________________________________________
    void fbmc_norm_akf_fd_impl::calculate_tau(int &tau, int &tau_offset, float &cfo_angular, log_chann_akf lca, const gr_complex *in)
	{		
		// We have detected a peak above the threshold.
		// But we don't know, if there isn't a higher peak. Therefore we check the next two peaks as well.
		int max_idx = lca.get_akf_vals_max_index();
		
		// whereever we found the global peak, jump to it
		in = &in[max_idx*d_veclength];
		
		// jump to the second preamble vector
		const gr_complex *in_delay = &in[(AKF_SPAN-1)*d_veclength];

		// tau and cfo: estimate within two symbols
		gr_complex temp0 = gr_complex(0.0f,0.0f);
		gr_complex temp1 = gr_complex(0.0f,0.0f);		
		for(int i=0; i<lca.indices.size(); i=i+2)
		{
			// extract at which subchannel index logical channel starts and how long it is
			int idx = lca.indices[i];
			int len = lca.indices[i+1];

			// formula (30) from Thein Eurasip
			for(int j = idx + 2; j<idx + len; j=j+2)
			{
				temp0 += in[j-2] / d_preamble[j-2] * conj(in[j]) / d_preamble[j];
				temp0 += in_delay[j-2] / d_preamble[j-2] * conj(in_delay[j]) / d_preamble[j];
			}
			
			// formula (22) from Thein Eurasip
			for(int j = idx; j<idx + len; j=j+2)
				temp1 += conj(in[j])*in_delay[j];		
		}

		// tau: calculate in samples
		int k = 2;
		float tau_f = arg(temp0)/(6.283185f*k)*d_veclength;
		
		// tau: set output
		tau = (tau_f >= 0) ? int(tau_f + 0.5f) : int(tau_f - 0.5f);
		
		// tau_offset: depends on where we found the global maximum
		tau_offset = max_idx*d_veclength/2;
		
		// cfo: append cfo estimation to moving average container
		if(d_cfo_estimations.size() == CFO_FRAMES_ESTIM_LENGTH)
			d_cfo_estimations.erase(d_cfo_estimations.begin());
		d_cfo_estimations.push_back(temp1);
		
		// cfo: average value
		gr_complex average_peak = gr_complex(0.0f, 0.0f);
		for(int i=0; i<d_cfo_estimations.size(); i++)
			average_peak += d_cfo_estimations[i];
			
		// cfo: set output as cfo from samples to sample
		cfo_angular = arg(average_peak)/(float) d_veclength;
		
		// DEBUG: show cfo in Hertz (sampling rate has to be set manually)
		//float samp_rate = 1e6;
		//float cfo_hz = cfo_angular/6.283185f*samp_rate;
		//PRINT(cfo_hz);
    }

	//______________________________________________________
    int
    fbmc_norm_akf_fd_impl::work(int noutput_items,
			  gr_vector_const_void_star &input_items,
			  gr_vector_void_star &output_items)
    {
		const gr_complex *in = (const gr_complex *) input_items[0];	
		float *out_dbg = (float *) output_items[0];
		int *out = (int *) output_items[1];
		
		// for each input vector ...
		for(int i=AKF_HISTORY; i<AKF_HISTORY+noutput_items; i++)
		{					
			// first int in out-vector indicates if at least one frame was found
			*out = 0;
			
			// ... and for each logical channel ...
			for(int j=0; j<log_chann_akf_vec.size(); j++)
			{
				gr_complex temp_pow = gr_complex(0,0);
				gr_complex temp_akf = gr_complex(0,0);

				// ... and for each continious subcarrier-chunk
				for(int k=0; k<log_chann_akf_vec[j].indices.size(); k=k+2)
				{
					int idx = i*d_veclength + log_chann_akf_vec[j].indices[k];
					int len = log_chann_akf_vec[j].indices[k+1];

#ifdef VOLK_FAST_MULTIPLICATION_OVER_NOISE
					gr_complex temp;
				
					volk_32fc_x2_conjugate_dot_prod_32fc(&temp, &in[idx], &in[idx], len);
					temp_pow += temp;
					
					volk_32fc_x2_conjugate_dot_prod_32fc(&temp, &in[idx], &in[idx-2*d_veclength], len);
					temp_akf += temp;
#else
					for(int l=0; l<len; l=l+2)
					{
						temp_pow += in[idx+l]*conj(in[idx+l]); 
						temp_akf += in[idx+l]*conj(in[idx-2*d_veclength+l]); 
					}
#endif
				}

				// swap energies
				log_chann_akf_vec[j].add_power_val(temp_pow.real());

				// make sure division is working properly
				float power = log_chann_akf_vec[j].get_mean_power();
				if(power < d_security)
					power = d_security;
					
				// swap akf values
				log_chann_akf_vec[j].add_akf_val(temp_akf/ gr_complex(power, 0.0f));
				
				// 'd_veclength' indicates that no frame was found for this logical channel
				int tau = d_veclength;
				int tau_offset = 0;
				float cfo_angular = 0;
				
				// check for frame condition: above threshold
				if(abs(log_chann_akf_vec[j].akf_vals[0]) > d_threshold)
				{
					// determine distance between this frame detection and the last one
					uint64_t last_starting_vector = d_block_logical_channel_last[j];
					uint64_t this_starting_vector = nitems_read(0) + i;					
					uint64_t distance = this_starting_vector - last_starting_vector;
					
					// if the distance is large enough, assume that a new frame was detected
					if (distance >= d_blocking_length)
					{						
						calculate_tau(tau, tau_offset, cfo_angular, log_chann_akf_vec[j], &in[(i-AKF_SPAN-1)*d_veclength]);
						log_chann_akf_vec[j].reset_akf_val();
						
						// at least one frame found
						*out = 1;
					}
					
					// remember when the current frame was detected
					d_block_logical_channel_last[j] = this_starting_vector;					
				}
				
				// set 3 output information
				int *out_temp = &out[1+j*3];
				*(out_temp++) = tau;
				*(out_temp++) = tau_offset;
				*(out_temp++) = 10e6*cfo_angular;		// multiplied with 10e6 to keep precision, 10e6 is more or less random
				
				// set output for debugging
				if(j == d_dbg_log_chann)
					*out_dbg = abs(temp_akf/gr_complex(power, 0.0f));
			}
			
			out += d_veclength;
			out_dbg += d_veclength;
		}

		return noutput_items;
    }

  } /* namespace fbmc1 */
} /* namespace gr */

