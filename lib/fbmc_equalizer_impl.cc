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
#include "fbmc_equalizer_impl.h"

#include <volk/volk.h>

#include "utils/debug.h"
#include "utils/frame_param.h"
#include "utils/preamble_set_freq_domain.h"
#include "utils/receiver_tag.h"

namespace gr {
  namespace fbmc1 {

    fbmc_equalizer::sptr
    fbmc_equalizer::make(int veclength, int filterlength, int symbols_per_frame, int preamble_set, std::vector<int> logical_channels)
    {
      return gnuradio::get_initial_sptr
        (new fbmc_equalizer_impl(veclength, filterlength, symbols_per_frame, preamble_set, logical_channels));
    }

    /*
     * The private constructor
     */
    fbmc_equalizer_impl::fbmc_equalizer_impl(int veclength, int filterlength, int symbols_per_frame, int preamble_set, std::vector<int> logical_channels)
      : gr::block("fbmc_equalizer",
              gr::io_signature::make(1, 1, sizeof(gr_complex)*veclength),
              gr::io_signature::make(1, 1, sizeof(gr_complex)*veclength))
    {
		// init variables
		d_STATE        	 = TRANSIENT_PREAMBLE;
		d_veclength    	 = veclength;
		d_trans_symb 	 = (2*filterlength - FRAME_SPACER_LENGTH_REDUCE)*2;
		d_first_symb	 = 2*filterlength;
		d_frame_symb	 = d_trans_symb + 2*symbols_per_frame;
		d_receiver_tag   = 0;
		d_log_chann 	 = 0;
		d_counter      	 = 0;
		d_chan_coef 	 = NULL;

		// init preamble
		get_scaled_preamble_set_freq_domain(preamble_set, d_veclength, FRAME_PREAMBLE_PRESCALE, d_preamble);

		// init logical channels
		logical_channel_convert(&logical_channels[0], logical_channels.size(), d_log_channs);

		// remove tags
		set_tag_propagation_policy(TPP_DONT);

		// init VOLK
		const int alignment_multiple = volk_get_alignment() / sizeof(gr_complex);
		set_alignment(std::max(1, alignment_multiple));
		
		// ###################################
		// DEBUG: pilots
		d_pilot_space = 2;
		d_pilot_space_cnt = d_pilot_space*2+1;
		for(int i=0; i<4; i++)
		{
			d_measuredPilotsPhase.push_back(0.0f);
		}
		d_chan_coef_default = NULL;
		// ###################################		
		
		/*
		PRINT(" ");
		PRINT("EQUALIZER INIT");
		PRINT(d_frame_symb);
		PRINT(" ");
		*/
	}

    /*
     * Our virtual destructor.
     */
    fbmc_equalizer_impl::~fbmc_equalizer_impl()
    {
		volk_free(d_chan_coef);
		
		// ###################################
		// DEBUG: pilots
		volk_free(d_chan_coef_default);
		// ###################################		
    }
    
    //______________________________________________________
    void fbmc_equalizer_impl::channel_estimation(const gr_complex *in)
    {

		/*
		// DEBUG
		PRINT(" ");
		PRINT("EQUALIZER");
		PRINT("INPUT");
		PRINT(d_first_symb);
		PRINT(" ");
		for(int i=0; i<d_veclength; i++)
			std::cout << "i: " << i+1 << " " << in[i] << std::endl;
		*/
				
		/*
		// DEBUG
		PRINT(" ");
		PRINT("EQUALIZER");
		PRINT("PREAMBLE");
		PRINT(d_first_symb);
		PRINT(" ");
		for(int i=0; i<d_veclength; i++)
			std::cout << "i: " << i+1 << " " << d_preamble[i] << std::endl;
		*/
		
		volk_free(d_chan_coef);
		d_chan_coef = NULL;
		
		// allocate d_chann_coef
		size_t alig = volk_get_alignment();
		d_chan_coef = (gr_complex*) volk_malloc(d_veclength*sizeof(gr_complex), alig);
		memset(d_chan_coef, 0x00, d_veclength*sizeof(gr_complex));
		
		// create allocation vector
		std::vector<int> alloc_vec(d_veclength, LOG_CHANN_SUBCCARRIER_DEACT);
		for(int i=0; i<d_log_channs[d_log_chann].get_size(); i++)
		{
			int index = d_log_channs[d_log_chann].subcarrier[i];
			alloc_vec[index] = LOG_CHANN_SUBCCARRIER_ACTIV;
		}

		// estimate channels with even index
		for(int i=0; i<d_veclength; i=i+2)
		{
			if(alloc_vec[i] == LOG_CHANN_SUBCCARRIER_ACTIV && in[i] != gr_complex(0.0f,0.0f))
				d_chan_coef[i] = d_preamble[i] / in[i];
		}

		// estimate channels with odd index
		for(int i=1; i<d_veclength; i=i+2)
		{
			if(alloc_vec[i] == LOG_CHANN_SUBCCARRIER_ACTIV)
			{
				if(alloc_vec[i-1] == LOG_CHANN_SUBCCARRIER_ACTIV && alloc_vec[i+1] == LOG_CHANN_SUBCCARRIER_ACTIV)
					d_chan_coef[i] = (d_chan_coef[i-1]+d_chan_coef[i+1])/gr_complex(2.0f,0.0f);
				else
					d_chan_coef[i] = d_chan_coef[i-1];
			}
		}

		// special case: carrier next to DC
		if(alloc_vec[d_veclength/2+1] == LOG_CHANN_SUBCCARRIER_ACTIV)
			d_chan_coef[d_veclength/2+1] = d_chan_coef[d_veclength/2+2];
	
		/*
		// ###################################
		// DEBUG: pilots
		d_chan_coef[11] = (d_chan_coef[10]+d_chan_coef[12])/gr_complex(2.0f, 0.0f);
		d_chan_coef[25] = (d_chan_coef[24]+d_chan_coef[26])/gr_complex(2.0f, 0.0f);
		d_chan_coef[39] = (d_chan_coef[38]+d_chan_coef[40])/gr_complex(2.0f, 0.0f);
		d_chan_coef[53] = (d_chan_coef[52]+d_chan_coef[54])/gr_complex(2.0f, 0.0f);
		
		volk_free(d_chan_coef_default);
		d_chan_coef_default = NULL;
		size_t alig2 = volk_get_alignment();
		d_chan_coef_default = (gr_complex*) volk_malloc(d_veclength*sizeof(gr_complex), alig2);	
		memcpy(d_chan_coef_default, d_chan_coef, d_veclength*sizeof(gr_complex));
		// ###################################
		*/
		
		/*	
		// DEBUG
		PRINT(" ");
		PRINT("EQUALIZER  2");
		PRINT(d_first_symb);
		PRINT(" ");
		for(int i=0; i<d_veclength; i++)
			std::cout << "i: " << i+1 << " " << d_chan_coef[i] << std::endl;
		*/			
	}
	
    //______________________________________________________
    void fbmc_equalizer_impl::add_log_chann_tag(int produced)
    {
		pmt::pmt_t key = pmt::string_to_symbol("receiver_tag");
		pmt::pmt_t value = pmt::from_uint64(d_receiver_tag);
		add_item_tag(0, nitems_written(0)+produced, key, value, pmt::PMT_NIL);
    }

	//______________________________________________________
    void
    fbmc_equalizer_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
		// always excepts as many samples as needed to complete frame
		int missing_samples = d_frame_symb - d_counter;
		ninput_items_required[0] = std::min(missing_samples, (int) (8191/d_veclength));
    }

	//______________________________________________________
    int
    fbmc_equalizer_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
        const gr_complex *in = (const gr_complex *) input_items[0];
        gr_complex *out = (gr_complex *) output_items[0];

		int consumed = 0;
		int produced = 0;

		while(consumed < ninput_items[0] && produced < noutput_items){

			switch(d_STATE)
			{
				//______________________________________________________
				case TRANSIENT_PREAMBLE:
				{
					int buffer = d_trans_symb - d_counter;
					buffer = std::min(ninput_items[0]-consumed, buffer);
					
					// extract stream tag
					if(d_counter == 0)
					{
						std::vector<gr::tag_t> tags;
						uint64_t offset = nitems_read(0)+consumed;
						get_tags_in_range(tags, 0, offset, offset+1, pmt::string_to_symbol("receiver_tag"));
						
						if(tags.size() == 0)
						{
							PRINT("fbmc_equalizer_impl error: Tag for logical channel not found.");
						}
						else
						{
							d_receiver_tag = pmt::to_uint64(tags[0].value);
							d_log_chann = (int) (d_receiver_tag & RECEIVER_TAG_LOGIC_CHAN_MASK);
						}
					}

					// check for offset for channel estimation
					for(int i=0; i<buffer;i++)
					{
						if(d_counter++ == d_first_symb)
							channel_estimation(&in[(consumed+i)*d_veclength]);
					}
					
					consumed += buffer;
					
					if(d_counter == d_trans_symb)
					{
						add_log_chann_tag(produced);
						d_STATE = PAYLOAD_SAMPLES;
						
						/*
						// ###################################
						// DEBUG: pilots
						d_pilot_space_cnt = d_pilot_space*2+1;
						for(int i=0; i<4; i++)
						{
							d_measuredPilotsPhase[i] = 0.0f;
						}
						// ###################################
						*/
						
					}
						
					break;
				}
				
				//______________________________________________________
				case PAYLOAD_SAMPLES:
				{
					int buffer = d_frame_symb - d_counter;
					buffer = std::min(ninput_items[0]-consumed, buffer);
					buffer = std::min(noutput_items-produced, buffer);

					int min = d_log_channs[d_log_chann].get_min();
					int max = d_log_channs[d_log_chann].get_max();
						
					// correct phase of samples
					for(int i=0; i<buffer; i++)
					{						
						int j = consumed*d_veclength + min;
						int k = produced*d_veclength + min;
						volk_32fc_x2_multiply_32fc(&out[k], &in[j], &d_chan_coef[min], max-min+1);
						
						/*
						// ###################################
						// DEBUG: pilots
						// adjust phase error
						if(d_pilot_space_cnt == 3)
						{
							// extract phases
							d_measuredPilotsPhase[0] = arg(gr_complex(out[produced*d_veclength+11].imag(), out[produced*d_veclength+11].real()));
							d_measuredPilotsPhase[1] = arg(gr_complex(out[produced*d_veclength+25].imag(), out[produced*d_veclength+25].real()));
							d_measuredPilotsPhase[2] = arg(gr_complex(out[produced*d_veclength+39].imag(), out[produced*d_veclength+39].real()));
							d_measuredPilotsPhase[3] = arg(gr_complex(out[produced*d_veclength+53].imag(), out[produced*d_veclength+53].real()));
							float errorSum = d_measuredPilotsPhase[0] + d_measuredPilotsPhase[1] + d_measuredPilotsPhase[2] + d_measuredPilotsPhase[3];
							errorSum /= 4.0f;
							
							// correct phase error vector
							gr_complex blablub = exp(gr_complex(0, errorSum));
							for(int ii=min; ii<max; ii++)
							{
								//d_chan_coef[ii] = d_chan_coef_default[ii]*blablub;
							}
														
							// DEBUG print
							errorSum *= 360.0f/(2.0f*3.14159);
							//gr_complex absSum = out[produced*d_veclength+11] + out[produced*d_veclength+25] + out[produced*d_veclength+39] + out[produced*d_veclength+53];
							//if(fabs(errorSum) > 10.0f && (fabs(absSum.imag())>0.9f))
							{
								PRINT(d_measuredPilotsPhase[0]/(2.0f*3.14159)*360.0f);
								PRINT(d_measuredPilotsPhase[1]/(2.0f*3.14159)*360.0f);
								PRINT(d_measuredPilotsPhase[2]/(2.0f*3.14159)*360.0f);
								PRINT(d_measuredPilotsPhase[3]/(2.0f*3.14159)*360.0f);
								
								PRINT(out[produced*d_veclength+11]);
								PRINT(out[produced*d_veclength+25]);
								PRINT(out[produced*d_veclength+39]);
								PRINT(out[produced*d_veclength+53]);								
								
								PRINT(errorSum);
								PRINT(arg(blablub)*360.0f/(2.0f*3.14159));
								PRINT(" ");
							}
						}
						
						if(d_pilot_space_cnt == 0)
						{
							d_pilot_space_cnt = d_pilot_space*2+1;
						}
						else
						{
							d_pilot_space_cnt--;
						}
						// ###################################
						*/
						
						consumed++;
						produced++;
					}

					d_counter += buffer;
					
					if(d_counter == d_frame_symb)
					{
						d_counter = 0;
						d_STATE = TRANSIENT_PREAMBLE;
					}
					
					break;
				}
			}
		}

        consume(0, consumed);
        return produced;
    }

  } /* namespace fbmc1 */
} /* namespace gr */
