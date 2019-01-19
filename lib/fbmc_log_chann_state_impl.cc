/* -*- c++ -*- */
/* 
 * Copyright 2018 <+YOU OR YOUR COMPANY+>.
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
#include "fbmc_log_chann_state_impl.h"

#include "utils/debug.h"
#include "utils/tx_delay_rx_measure.h"

namespace gr {
  namespace fbmc1 {

    fbmc_log_chann_state::sptr
    fbmc_log_chann_state::make(int veclength, int fft_length, int avg, std::vector<int> logical_channels)
    {
      return gnuradio::get_initial_sptr
        (new fbmc_log_chann_state_impl(veclength, fft_length, avg, logical_channels));
    }

    /*
     * The private constructor
     */
    fbmc_log_chann_state_impl::fbmc_log_chann_state_impl(int veclength, int fft_length, int avg, std::vector<int> logical_channels)
      : gr::sync_block("fbmc_log_chann_state",
              gr::io_signature::make(1, 1, sizeof(gr_complex)*veclength),
              gr::io_signature::make(0, 0, 0))
    {
		// init variables
		d_STATE			= INIT_TIME;
		d_veclength		= veclength;	// number of subcarriers
		d_fft_length 	= fft_length;	// the fft length can be higher than the number of subcarriers
		d_avg	 		= avg;			// number of vectors to average over
		d_counter 		= 0;

		// init logical channels
		logical_channel_convert(&logical_channels[0], logical_channels.size(), d_log_channs);		
		
		// init the channel state
		for (int i=0;i<d_veclength;i++)
			d_carrier_power.push_back(0.0f);
		
		// output ports
		message_port_register_out(pmt::mp("channel control"));	
	}

    /*
     * Our virtual destructor.
     */
    fbmc_log_chann_state_impl::~fbmc_log_chann_state_impl()
    {
    }
    
    void 
    fbmc_log_chann_state_impl::estimate_channels_send_message()
    {	
		// divide power by the length of the averaging filter
		for (int i=0;i<d_veclength;i++)
			d_carrier_power[i] = d_carrier_power[i]/float(d_avg);
			
		// convert power from each subcarrier to dB
		std::vector<float> d_carrier_power_dB;
		for (int i=0;i<d_veclength;i++)
			d_carrier_power_dB.push_back(10.0f*log10(d_carrier_power[i]));
			
			
		// ###########################################################################################
		// ############ HAS TO BE HARDCODED FOR EACH LOGICAL CHANNEL CONFIGURATION ###################
		// ############ HARDCODED FOR A CONFIGURATION WITH 64 SUBCARRIER FOR DEMO PURPOSES ###########
		// ###########################################################################################
			
		// STEP 1: CHECK IF WE HAVEN'T RECEIVED OUR OWN PACKET:
		
		// count carriers with power above the predefined threshold in dB		
		int channels_above = 0;
		for (int i=0; i<d_veclength; i++)
		{
			if (d_carrier_power_dB[i] >= -20.f)
			{
				channels_above++;
			}
		}
		
		// drop measurement if too many subcarrier are above the threshold -> we received our own frame
		if (channels_above >= 15)
		{

#ifdef RX_DELAY_DEBUG
			PRINT("fbmc_log_chann_state_impl debug: Too many channels above threshold. Probably received my own frame.");
#endif			
			
			return;
		}		
		
		// STEP 2: DETERMINE SUBCARRIER WITH THE MOST POWER:
		
		// find the index
		int carrier_most_power_index = 34;
		float carrier_most_power = d_carrier_power_dB[0];
		for (int i=1; i<63; i++)
		{
			if (d_carrier_power_dB[i] >= carrier_most_power)
			{
				carrier_most_power = d_carrier_power_dB[i];
				carrier_most_power_index = i;
			}
		}
		
		// finals channel that will be used
		int channel2use = 0;
		
		// check if the power is above a certain threshold
		if (carrier_most_power >= -19.0f)
		{
			if (carrier_most_power_index >= 36 && carrier_most_power_index <= 39)
				channel2use = 1;
				
			if (carrier_most_power_index >= 40 && carrier_most_power_index <= 45)
				channel2use = 2;
				
			if (carrier_most_power_index >= 46 && carrier_most_power_index <= 51)
				channel2use = 3;
				
			if (carrier_most_power_index >= 52 && carrier_most_power_index <= 60)
				channel2use = 4;
			
#ifdef RX_DELAY_DEBUG
			PRINT("fbmc_log_chann_state_impl debug: Subcarrier with the most power is:");
			PRINT(carrier_most_power_index);
			PRINT("fbmc_log_chann_state_impl debug: Channel in use:");
			PRINT(channel2use);			
#endif		
	
		}
		{
			
#ifdef RX_DELAY_DEBUG
			PRINT("fbmc_log_chann_state_impl debug: No subarrier found that has enough power to be a disturber.");
#endif	
		
		}
		
		// ###########################################################################################
		// ###########################################################################################
		// ###########################################################################################
		
		// send message to the payload generator
		// publish PDU
		pmt::pmt_t channel2use_pmt = pmt::from_long((long) channel2use);
		pmt::pmt_t dict = pmt::make_dict();
		dict = pmt::dict_add(dict, pmt::mp("channel2use"), channel2use_pmt);
		pmt::pmt_t PMT_dummy_blob = pmt::make_blob(nullptr, 0);
		message_port_pub(pmt::mp("channel control"), pmt::cons(dict, PMT_dummy_blob));

		
#ifdef RX_DELAY_DEBUG
		// print each subcarrier power
		for (int i=0;i<d_veclength;i++)
			std::cout << "i: " << i << " " << d_carrier_power_dB[i] << std::endl;
#endif
		
	}    

    int
    fbmc_log_chann_state_impl::work(int noutput_items,
        gr_vector_const_void_star &input_items,
        gr_vector_void_star &output_items)
    {
		const gr_complex *in = (const gr_complex *) input_items[0];
		
		int consumed  = 0;

		while(consumed < noutput_items)
		{
			switch(d_STATE)
			{
				//_______________________	
				case INIT_TIME:
				{					
					// determine current time for the first time
					d_start_time = std::chrono::high_resolution_clock::now();
					
					d_STATE = WAIT;
					
					break;
				}
				
				//_______________________
				case WAIT:
				{
					// determine time elapsed since last measurement
					std::chrono::time_point<std::chrono::system_clock> now_ms = std::chrono::high_resolution_clock::now();
					int64_t duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now_ms - d_start_time).count();
					
					// check time elapsed condition
					if (duration_ms >= TX_DELAY_TRANSMITTING_MS)
					{
						d_start_time = now_ms;
						d_STATE = COLLECT_SAMPLES;
					}
					else
					{
						// skip all samples of thir work function call
						consumed = noutput_items;
					}					

					break;
				}

				//_______________________
				case COLLECT_SAMPLES:
				{
					int buffer = d_avg - d_counter;
					buffer = std::min(noutput_items-consumed, buffer);
					
					// average over the input vectors
					for (int i=0; i<buffer; i++)
					{
						// sum up the energy for each subcarrier
						for (int j=0;j<d_veclength;j++)
						{
							d_carrier_power[j] += abs(in[consumed*d_veclength + j]);
						}
					}					

					consumed += buffer;
					d_counter += buffer;
					
					if(d_counter == d_avg)
					{
						// estimate channels state and send message to payload generator
						estimate_channels_send_message();
						
						// reset the channel state
						for (int i=0;i<d_veclength;i++)
							d_carrier_power[i] = 0.0f;
						
						// go back to waiting state
						d_STATE = WAIT;
						
						// reset counter for the next time
						d_counter = 0;						
					}

					break;
				}				
			}
		}

		return consumed;
    }

  } /* namespace fbmc1 */
} /* namespace gr */

