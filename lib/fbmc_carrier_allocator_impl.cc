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
#include "fbmc_carrier_allocator_impl.h"

#include <math.h>

#include "utils/metadata.h"
#include "utils/encodings.h"
#include "utils/constellations.h"
#include "utils/frame_param.h"
#include "utils/preamble_set_freq_domain.h"

#define MAX_CA_FRAME 		500
#define ALTERNATE_PREAMBLE 	2

namespace gr {
  namespace fbmc1 {
	  
	//______________________________________________________
	ca_ppdu::ca_ppdu()
	{
		d_ppdu = NULL;
		d_ppdu_index = 0;
	}
	
	//______________________________________________________
	ca_ppdu::~ca_ppdu()
	{
		free(d_ppdu);
	}

    fbmc_carrier_allocator::sptr
    fbmc_carrier_allocator::make(int veclength, int filterlength, int preamble_set, int microseconds)
    {
      return gnuradio::get_initial_sptr
        (new fbmc_carrier_allocator_impl(veclength, filterlength, preamble_set, microseconds));
    }

    /*
     * The private constructor
     */
    fbmc_carrier_allocator_impl::fbmc_carrier_allocator_impl(int veclength, int filterlength, int preamble_set, int microseconds)
      : gr::sync_block("fbmc_carrier_allocator",
              gr::io_signature::make(0,0,0),
              gr::io_signature::make(1, 1, sizeof(gr_complex)*veclength))
    {
		// init variables
		d_STATE 			= INPUT;
		d_veclength			= veclength;
		d_filterlength		= filterlength;
		d_preamble_set		= preamble_set;
		d_microseconds		= microseconds;
		d_counter			= 0;
		
		// ###################################
		// DEBUG: pilots
		d_pilot_space = 2;
		d_pilot_space_cnt = d_pilot_space;
		d_pilotWidth = 2;
		for(int i=0; i<4; i++)
		{
			d_pilotFollowUp.push_back(gr_complex(0.0f, 0.0f));
			d_pilotSum.push_back(0.0f);
		}
		// ###################################
				
		// input port
		message_port_register_in(pmt::mp("caa in"));
		set_msg_handler(pmt::mp("caa in"), boost::bind(&fbmc_carrier_allocator_impl::caa_in, this, _1));
	}

    /*
     * Our virtual destructor.
     */
    fbmc_carrier_allocator_impl::~fbmc_carrier_allocator_impl()
    {
		for(int i=0; i<d_ca_ppdu_vec.size(); i++)
			delete d_ca_ppdu_vec[i];
    }

    //______________________________________________________
    void fbmc_carrier_allocator_impl::caa_in(pmt::pmt_t msg)
    {
		const char *CAA_blob;
		size_t CAA_blob_len;
		int32_t *log_chann;
		size_t log_chann_len;

		// input blob extraction
		if(pmt::is_pair(msg))
		{
			CAA_blob = reinterpret_cast<const char *>(pmt::blob_data(pmt::cdr(msg)));
			CAA_blob_len = pmt::blob_length(pmt::cdr(msg));
			pmt::pmt_t meta_dict = pmt::dict_values(pmt::car(msg));
			log_chann = pmt::s32vector_writable_elements(pmt::nth(0, meta_dict), log_chann_len);
		}  
		else
		{
			throw std::invalid_argument("fbmc_carrier_allocator_impl error: Expect PDUs as input. Dropping data.");
			return;
		}
		
		// logical channels
		d_log_channs.clear();
		logical_channel_convert(log_chann, log_chann_len, d_log_channs);
		
		int CAA_blob_index = 0;
		
		// extract each single ppdu
		while(CAA_blob_index != CAA_blob_len)
		{
			// add new element to container
			if(d_ca_ppdu_vec.size() < MAX_CA_FRAME)
			{
				d_ca_ppdu_vec.push_back(new ca_ppdu());
			}
			else
			{
				PRINT("fbmc_carrier_allocator_impl error: Too many ppdus to buffer. Dropping data.");
				break;
			}
			
			int index = d_ca_ppdu_vec.size() - 1;
			
			// mpdu information
			metadataPpdu *mp = const_cast<metadataPpdu*>(reinterpret_cast<const metadataPpdu*> (CAA_blob + CAA_blob_index));
			CAA_blob_index += sizeof(metadataPpdu);

			d_ca_ppdu_vec[index]->d_log_chann = mp->log_chann;
			d_ca_ppdu_vec[index]->d_scale = mp->scale;
			d_ca_ppdu_vec[index]->d_encoding = mp->encoding;
			d_ca_ppdu_vec[index]->d_airtime = mp->airtime;
			d_ca_ppdu_vec[index]->d_altern_pream = mp->altern_pream;
			
			// determine frame length in symbols
			int channel_width = d_log_channs[mp->log_chann].get_size();
			d_ca_ppdu_vec[index]->d_spf = (mp->ppdu_len + channel_width - 1) / channel_width;
			int total_cp = d_ca_ppdu_vec[index]->d_spf*channel_width;
			
			// fill ppdu
			d_ca_ppdu_vec[index]->d_ppdu = (uint8_t*) malloc(total_cp);
			memcpy(d_ca_ppdu_vec[index]->d_ppdu, CAA_blob + CAA_blob_index, mp->ppdu_len);
			d_ca_ppdu_vec[index]->d_ppdu_len = total_cp;
			d_ca_ppdu_vec[index]->d_residual_reset = total_cp - mp->ppdu_len;
			
			CAA_blob_index += mp->ppdu_len;
		}
    }
    
    //______________________________________________________
    void fbmc_carrier_allocator_impl::add_length_airtime_tag(int produced)
    {
		int length = (2*d_filterlength + d_ca_ppdu_vec[0]->d_spf - FRAME_SPACER_LENGTH_REDUCE + FRAME_APPEND_ZEROS_ADDITIONAL)*d_veclength;
		pmt::pmt_t key = pmt::string_to_symbol("packet_len");
		pmt::pmt_t value = pmt::from_long(length);
		pmt::pmt_t srcid = pmt::string_to_symbol(alias());
		add_item_tag(0, nitems_written(0)+produced, key, value, srcid);
		
		key = pmt::string_to_symbol("airtime");
		value = pmt::from_uint64(d_ca_ppdu_vec[0]->d_airtime);
		srcid = pmt::string_to_symbol(alias());
		add_item_tag(0, nitems_written(0)+produced, key, value, srcid);
    }

	//______________________________________________________
    int
    fbmc_carrier_allocator_impl::work(int noutput_items,
			  gr_vector_const_void_star &input_items,
			  gr_vector_void_star &output_items)
    {
		gr_complex *out = (gr_complex *) output_items[0];
		
		int produced  = 0;
		bool token = false;

		while(produced < noutput_items && token == false)
		{
			switch(d_STATE)
			{
				//_______________________
				case INPUT:

					if(d_ca_ppdu_vec.size() == 0)
					{
						if(d_microseconds > 0)
							boost::this_thread::sleep(boost::posix_time::microseconds(d_microseconds));
						token = true;
					}
					else
					{
						// update symbol table
						d_constellation.clear();
						get_scaled_constellation(d_ca_ppdu_vec[0]->d_encoding, d_ca_ppdu_vec[0]->d_scale, d_constellation);
						
						// update preamble
						d_preamble.clear();
						if(d_ca_ppdu_vec[0]->d_altern_pream == false)
							get_scaled_preamble_set_freq_domain(d_preamble_set, d_veclength, d_ca_ppdu_vec[0]->d_scale*FRAME_PREAMBLE_PRESCALE, d_preamble);
						else
							get_scaled_preamble_set_freq_domain(ALTERNATE_PREAMBLE, d_veclength, d_ca_ppdu_vec[0]->d_scale*FRAME_PREAMBLE_PRESCALE, d_preamble);
						
						// reset pad cp to zero
						int zero_index = d_constellation.size() - 1;
						for(int i = d_ca_ppdu_vec[0]->d_ppdu_len - d_ca_ppdu_vec[0]->d_residual_reset; i<d_ca_ppdu_vec[0]->d_ppdu_len; i++)
							d_ca_ppdu_vec[0]->d_ppdu[i] = zero_index;
								
						d_STATE = PREAMBLE;
						
						/*
						// ###################################
						// DEBUG: pilots
						d_pilot_space_cnt = d_pilot_space;
						// ###################################
						*/
					}
					
					break;
				
				//_______________________
				case PREAMBLE:
				{
					int buffer = 2 - d_counter;
					buffer = std::min(noutput_items-produced, buffer);
					
					// tag stream with length of this frame
					if(d_counter == 0)
						add_length_airtime_tag(produced);

					// copy preamble
					for(int i=produced; i<produced+buffer; i++)
					{
						memset(&out[i*d_veclength], 0x00, d_veclength*sizeof(gr_complex));
						
						for(int j=0; j<d_log_channs[d_ca_ppdu_vec[0]->d_log_chann].get_size(); j++)
						{
							int index = d_log_channs[d_ca_ppdu_vec[0]->d_log_chann].subcarrier[j];
							out[i*d_veclength + index] = d_preamble[index];
						}
					}
					
					d_counter += buffer;
					produced  += buffer;
					
					if(d_counter == 2)
					{
						d_counter = 0;
						d_STATE = SPACER;
					}
					
					break;
				}

				//_______________________
				case SPACER:
				{
					int buffer = d_filterlength - 1 - FRAME_SPACER_LENGTH_REDUCE - d_counter;
					buffer = std::min(noutput_items-produced, buffer);
					
					memset(&out[produced*d_veclength], 0x00, buffer*d_veclength*sizeof(gr_complex));
					
					produced  += buffer;
					d_counter += buffer;
					
					if(d_counter == d_filterlength - 1 - FRAME_SPACER_LENGTH_REDUCE)
					{
						d_counter = 0;
						d_STATE = PPDU;
					}
					
					break;
				}
		
				//_______________________
				case PPDU:
				{
					int buffer = d_ca_ppdu_vec[0]->d_spf - d_counter;
					buffer = std::min(noutput_items-produced, buffer);

					for(int i=produced; i<produced+buffer; i++)
					{
						memset(&out[i*d_veclength], 0x00, d_veclength*sizeof(gr_complex));
						
						for(int j=0; j<d_log_channs[d_ca_ppdu_vec[0]->d_log_chann].get_size(); j++)
						{
							int sub_ind = d_log_channs[d_ca_ppdu_vec[0]->d_log_chann].subcarrier[j];
							out[i*d_veclength + sub_ind] = d_constellation[d_ca_ppdu_vec[0]->d_ppdu[d_ca_ppdu_vec[0]->d_ppdu_index++]];
							
							//out[i*d_veclength + d_log_channs[d_ca_ppdu_vec[0]->d_log_chann].subcarrier[j]] = d_constellation[d_ca_ppdu_vec[0]->d_ppdu[d_ca_ppdu_vec[0]->d_ppdu_index++]];
						}
						
						/*
						// ###################################
						// DEBUG: pilots
						
						// set random values in pilots
						unsigned int test = std::rand() % (d_constellation.size()-3);
						out[i*d_veclength + 11] = d_constellation[test];
						test = std::rand() % (d_constellation.size()-3);
						out[i*d_veclength + 25] = d_constellation[test];
						test = std::rand() % (d_constellation.size()-3);
						out[i*d_veclength + 39] = d_constellation[test];
						test = std::rand() % (d_constellation.size()-3);
						out[i*d_veclength + 53] = d_constellation[test];
						
						//float grid00 = 1.0f;
						//float grid10 = 1.0f;
						//float grid20 = 1.0f;
						
						//float grid01 = 1.0f;
						//float grid11 = 1.0f;
						//float grid21 = 1.0f;
						
						//float grid02 = 1.0f;
						//float grid12 = 1.0f;
						//float grid22 = 1.0f;
						
						//float grid03 = 1.0f;
						//float grid13 = 1.0f;
						//float grid23 = 1.0f;
						
						//float grid04 = 1.0f;
						//float grid14 = 1.0f;
						//float grid24 = 1.0f;
						
						float grid00 = -0.1250f;
						float grid10 =  0.0002f;
						float grid20 = -0.1250f;
						
						float grid01 = -0.2058f;
						float grid11 = -0.5644f;
						float grid21 =  0.2058f;
						
						float grid02 =  0.2393f;
						float grid12 =  1.0000f;
						float grid22 =  0.2393f;
						
						float grid03 =  0.2058f;
						float grid13 = -0.5644f;
						float grid23 = -0.2058f;
						
						float grid04 = -0.1250f;
						float grid14 =  0.0002f;
						float grid24 = -0.1250f;
						
						// first symbol of unit
						if(d_pilot_space_cnt == d_pilot_space)
						{							
							// reset follow up symbol and sum
							for(int i=0; i<4; i++)
							{
								d_pilotFollowUp[i] = gr_complex(0.0f, 0.0f);
								d_pilotSum[i] = 0.0f;
							}
						}
						
						// symbol before pilot symbol
						if(d_pilot_space_cnt == 2)
						{
							d_pilotSum[0] += out[i*d_veclength + 10].imag()*grid01;
							d_pilotSum[0] += out[i*d_veclength + 11].imag()*grid11;
							d_pilotSum[0] += out[i*d_veclength + 12].imag()*grid21;
															
							d_pilotSum[1] += out[i*d_veclength + 24].imag()*grid01;
							d_pilotSum[1] += out[i*d_veclength + 25].imag()*grid11;
							d_pilotSum[1] += out[i*d_veclength + 26].imag()*grid21;
							
							d_pilotSum[2] += out[i*d_veclength + 38].imag()*grid01;
							d_pilotSum[2] += out[i*d_veclength + 39].imag()*grid11;
							d_pilotSum[2] += out[i*d_veclength + 40].imag()*grid21;
							
							d_pilotSum[3] += out[i*d_veclength + 52].imag()*grid01;
							d_pilotSum[3] += out[i*d_veclength + 53].imag()*grid11;
							d_pilotSum[3] += out[i*d_veclength + 54].imag()*grid21;					
							
							if(d_pilotWidth == 2)
							{
								d_pilotSum[0] += out[i*d_veclength + 10].real()*grid00;
								d_pilotSum[0] += out[i*d_veclength + 11].real()*grid10;
								d_pilotSum[0] += out[i*d_veclength + 12].real()*grid20;
																
								d_pilotSum[1] += out[i*d_veclength + 24].real()*grid00;
								d_pilotSum[1] += out[i*d_veclength + 25].real()*grid10;
								d_pilotSum[1] += out[i*d_veclength + 26].real()*grid20;
								
								d_pilotSum[2] += out[i*d_veclength + 38].real()*grid00;
								d_pilotSum[2] += out[i*d_veclength + 39].real()*grid10;
								d_pilotSum[2] += out[i*d_veclength + 40].real()*grid20;
							
								d_pilotSum[3] += out[i*d_veclength + 52].real()*grid00;
								d_pilotSum[3] += out[i*d_veclength + 53].real()*grid10;
								d_pilotSum[3] += out[i*d_veclength + 54].real()*grid20;								
							}						
						}
												
						// pilot symbol of unit
						if(d_pilot_space_cnt == 1)
						{
							d_pilotSum[0] += out[i*d_veclength + 10].real()*grid02 + out[i*d_veclength + 10].imag()*grid03;
							d_pilotSum[0] += out[i*d_veclength + 12].real()*grid22 + out[i*d_veclength + 12].imag()*grid23;
															
							d_pilotSum[1] += out[i*d_veclength + 24].real()*grid02 + out[i*d_veclength + 24].imag()*grid03;
							d_pilotSum[1] += out[i*d_veclength + 26].real()*grid22 + out[i*d_veclength + 26].imag()*grid23;
							
							d_pilotSum[2] += out[i*d_veclength + 38].real()*grid02 + out[i*d_veclength + 38].imag()*grid03;
							d_pilotSum[2] += out[i*d_veclength + 40].real()*grid22 + out[i*d_veclength + 40].imag()*grid23;
						
							d_pilotSum[3] += out[i*d_veclength + 52].real()*grid02 + out[i*d_veclength + 52].imag()*grid03;
							d_pilotSum[3] += out[i*d_veclength + 54].real()*grid22 + out[i*d_veclength + 54].imag()*grid23;
							
							// check if last symbol is a zero symbol
							if(((d_counter+buffer) == d_ca_ppdu_vec[0]->d_spf) && (i==(produced+buffer-1)))
							{
								// the last symbol will be a zero symbol, therefore pilotsums stay as they are
							}
							else
							{
								// create random followup symbols
								for(int i=0; i<4; i++)
								{
									unsigned int test_rand_followup = std::rand() % (d_constellation.size()-3);
									d_pilotFollowUp[i] = d_constellation[test_rand_followup];
								}
								
								// calculate sum for last symbols if used
								if(d_pilotWidth == 2)
								{
									d_pilotSum[0] += d_constellation[d_ca_ppdu_vec[0]->d_ppdu[d_ca_ppdu_vec[0]->d_ppdu_index+4]].real()*grid04;
									d_pilotSum[0] += d_pilotFollowUp[0].real()*grid14;
									d_pilotSum[0] += d_constellation[d_ca_ppdu_vec[0]->d_ppdu[d_ca_ppdu_vec[0]->d_ppdu_index+5]].real()*grid24;
																	
									d_pilotSum[1] += d_constellation[d_ca_ppdu_vec[0]->d_ppdu[d_ca_ppdu_vec[0]->d_ppdu_index+17]].real()*grid04;
									d_pilotSum[1] += d_pilotFollowUp[1].real()*grid14;
									d_pilotSum[1] += d_constellation[d_ca_ppdu_vec[0]->d_ppdu[d_ca_ppdu_vec[0]->d_ppdu_index+18]].real()*grid24;
									
									d_pilotSum[2] += d_constellation[d_ca_ppdu_vec[0]->d_ppdu[d_ca_ppdu_vec[0]->d_ppdu_index+29]].real()*grid04;
									d_pilotSum[2] += d_pilotFollowUp[2].real()*grid14;
									d_pilotSum[2] += d_constellation[d_ca_ppdu_vec[0]->d_ppdu[d_ca_ppdu_vec[0]->d_ppdu_index+30]].real()*grid24;
								
									d_pilotSum[3] += d_constellation[d_ca_ppdu_vec[0]->d_ppdu[d_ca_ppdu_vec[0]->d_ppdu_index+42]].real()*grid04;
									d_pilotSum[3] += d_pilotFollowUp[3].real()*grid14;
									d_pilotSum[3] += d_constellation[d_ca_ppdu_vec[0]->d_ppdu[d_ca_ppdu_vec[0]->d_ppdu_index+43]].real()*grid24;
								}
							}
							
							// set pilots
							out[i*d_veclength + 11] = gr_complex(1.0f, -d_pilotSum[0]/grid13);
							out[i*d_veclength + 25] = gr_complex(1.0f, -d_pilotSum[1]/grid13);
							out[i*d_veclength + 39] = gr_complex(1.0f, -d_pilotSum[2]/grid13);
							out[i*d_veclength + 53] = gr_complex(1.0f, -d_pilotSum[3]/grid13);
						}
						
						// last symbol of unit
						if(d_pilot_space_cnt == 0)
						{							
							d_pilot_space_cnt = d_pilot_space;
							
							// set follow up symbols
							out[i*d_veclength + 11] = d_pilotFollowUp[0];
							out[i*d_veclength + 25] = d_pilotFollowUp[1];
							out[i*d_veclength + 39] = d_pilotFollowUp[2];
							out[i*d_veclength + 53] = d_pilotFollowUp[3];
						}
						else
						{
							d_pilot_space_cnt--;
						}
						// ###################################
						*/
					}
		
					produced  += buffer;
					d_counter += buffer;
					
					if(d_counter == d_ca_ppdu_vec[0]->d_spf)
					{
						d_counter = 0;
						d_STATE = APPEND_ZEROS;
					}
					
					break;
				}
				
				//_______________________
				case APPEND_ZEROS:
				{
					int buffer = d_filterlength - 1 + FRAME_APPEND_ZEROS_ADDITIONAL - d_counter;
					buffer = std::min(noutput_items-produced, buffer);
					
					memset(&out[produced*d_veclength], 0x00, buffer*d_veclength*sizeof(gr_complex));
					
					produced  += buffer;
					d_counter += buffer;
					
					if(d_counter == d_filterlength - 1 + FRAME_APPEND_ZEROS_ADDITIONAL)
					{
						d_counter = 0;
						d_STATE = INPUT;
						
						// delete first frame
						delete d_ca_ppdu_vec[0];
						d_ca_ppdu_vec.erase(d_ca_ppdu_vec.begin());
					}
					
					break;
				}	
	    	}
		}
		
        return produced;
    }

  } /* namespace fbmc1 */
} /* namespace gr */
