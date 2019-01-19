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
#include "fbmc_valve_fd_impl.h"

#include <volk/volk.h>

#include "utils/logical_channel.h"
#include "utils/frame_param.h"
#include "utils/preamble_set_time_domain.h"
#include "utils/receiver_tag.h"

#define MAX_FRAME_VEC_MULTIPLE_N_LOG_CHANN 20

namespace gr {
  namespace fbmc1 {

	//______________________________________________________
	frame::frame(uint64_t sti, uint64_t rt, float ca)
	{	
		size_t alig = volk_get_alignment();
		data = (gr_complex*) volk_malloc(frame_len*sizeof(gr_complex), alig);
		
		start_index	= sti;
		recv_tag 	= rt;
		cfo_angular = ca;
		data_cnt 	= 0;
	}
	
	//______________________________________________________
	frame::~frame()
	{
		volk_free(data);
	}
	
	//______________________________________________________
	int frame::frame_len = 0;
	
	//______________________________________________________
    fbmc_valve_fd::sptr
    fbmc_valve_fd::make(int veclength, int filterlength, int symbols_per_frame, std::vector<int> logical_channels)
    {
      return gnuradio::get_initial_sptr
        (new fbmc_valve_fd_impl(veclength, filterlength, symbols_per_frame, logical_channels));
    }

    /*
     * The private constructor
     */
    fbmc_valve_fd_impl::fbmc_valve_fd_impl(int veclength, int filterlength, int symbols_per_frame, std::vector<int> logical_channels)
      : gr::sync_block("fbmc_valve_fd",
              gr::io_signature::make2(2, 2, sizeof(int)*veclength, sizeof(gr_complex)*veclength/2),
              gr::io_signature::make(0, 0, 0))
    {
		// init variables
		d_veclength 	= veclength;
		d_filterlength 	= filterlength;
		d_frame_len		= (2*d_filterlength + symbols_per_frame - FRAME_SPACER_LENGTH_REDUCE)*d_veclength + d_veclength/2;
		
		// set frame length for structure
		frame::frame_len = d_frame_len;
		
		// calculate number of logical channels
		std::vector<logical_channel> log_channs;
		logical_channel_convert(&logical_channels[0], logical_channels.size(), log_channs);
		d_n_log_chann = log_channs.size();

#ifdef VALVE_FD_DOUBLE_FRAMING
		switch(d_veclength)
		{
			case 64:
				boundary_low  = BOUNDARY_64_LOW;
				boundary_high = BOUNDARY_64_HIGH;
				break;			
			case 128:
				boundary_low  = BOUNDARY_128_LOW;
				boundary_high = BOUNDARY_128_HIGH;
				break;
			case 256:
				boundary_low  = BOUNDARY_256_LOW;
				boundary_high = BOUNDARY_256_HIGH;
				break;
		}
#endif
		
#ifdef VALVE_FD_PRINT_RATE
		d_valve_timer = new fbmc_timer(VALVE_FD_PRINT_RATE);
		d_frames_received = 0;
#endif
		
		// output port
		message_port_register_out(pmt::mp("frame out"));

		// init VOLK
		const int alignment_multiple = volk_get_alignment() / sizeof(gr_complex);
		set_alignment(std::max(1, alignment_multiple));
	}

    /*
     * Our virtual destructor.
     */
    fbmc_valve_fd_impl::~fbmc_valve_fd_impl()
    {
		
#ifdef VALVE_FD_PRINT_RATE
		delete d_valve_timer;
#endif

    }
    
#ifdef VALVE_FD_PRINT_RATE
    //______________________________________________________
    void fbmc_valve_fd_impl::display()
    {
		if(d_valve_timer->new_update())
		{
			std::cout << std::endl << std::endl
			<< "---------------- Valve FD ----------------" << std::endl
			<< "frames detected: " << d_frames_received << std::endl
			<< "------------------------------------------" << std::endl;
		}
    }
#endif

	//______________________________________________________
    int
    fbmc_valve_fd_impl::work(int noutput_items,
			  gr_vector_const_void_star &input_items,
			  gr_vector_void_star &output_items)
    {
        const int *in0 = (const int *) input_items[0];
        const gr_complex *in1 = (const gr_complex *) input_items[1];
        gr_complex *out = (gr_complex *) output_items[0];
        
#ifdef VALVE_FD_PRINT_RATE	
		display();
#endif
        
        // STEP 1: check input items for new frames
        for(int i=0; i<noutput_items; i++)
        {
			// check for any frame
			if(in0[i*d_veclength] == 0)
				continue;
			
			// iterate though logical channels
			for(int j=0; j<d_n_log_chann; j++)
			{
				int tau = in0[i*d_veclength + j*3 + 1];
				
				if(tau != d_veclength)
				{
					
#ifdef VALVE_FD_PRINT_RATE   
					d_frames_received++;
#endif

					if(frame_vec.size() < MAX_FRAME_VEC_MULTIPLE_N_LOG_CHANN*d_n_log_chann)
					{
						int tau_offset = in0[i*d_veclength + j*3 + 2];
						float cfo_angular = (float) in0[i*d_veclength + j*3 + 3];
						cfo_angular /= 10e6;	// was multiplied with 10e6 to keep precision
						
#ifdef VALVE_FD_DOUBLE_FRAMING
						int tau_abs = abs(tau);

						// check if delay lies in critical area
						if(tau_abs >= boundary_low && tau_abs <= boundary_high)
						{
							uint64_t receiver_tag = 0;
							// TODO: set sample number
							receiver_tag |= uint64_t(1) << RECEIVER_TAG_DOUBL_FRAM_OFFS;
							receiver_tag |= (uint64_t(j) & RECEIVER_TAG_LOGIC_CHAN_MASK);
						
							uint64_t start_index = (nitems_read(0)+i)*d_veclength/2 + d_veclength/4 + tau_offset - tau_abs;
							frame_vec.push_back(new frame(start_index, receiver_tag, cfo_angular));
							frame_vec.push_back(new frame(start_index + 2*tau_abs, receiver_tag, cfo_angular));
						}
						else
						{
							uint64_t receiver_tag = 0;
							// TODO: set sample number
							receiver_tag |= (uint64_t(j) & RECEIVER_TAG_LOGIC_CHAN_MASK);
							
							uint64_t start_index = (nitems_read(0)+i)*d_veclength/2 + d_veclength/4 + tau_offset + tau;
							frame_vec.push_back(new frame(start_index, receiver_tag, cfo_angular));
						}
#else
						uint64_t start_index = (nitems_read(0)+i)*d_veclength/2 + d_veclength/4 + tau_offset + tau;
						frame_vec.push_back(new frame(start_index, j, cfo_angular));
#endif
					}
					else
					{
						PRINT("fbmc_valve_fd_impl error: Too many frames to buffer. Dropping frame.");
					}
				}
			}
		}
		
		// STEP 2: collect frame samples from input items and - if neccessary - publish frames
		
		// global samples indices in this call of work
		uint64_t min_samp = nitems_read(0)*d_veclength/2;
		uint64_t max_samp = (nitems_read(0) + noutput_items)*d_veclength/2;
			
		// copy samples into frame structures
		for(int i=0; i<frame_vec.size(); i++)
		{
			int rel_idx, buffer;
			
			// if frame already started
			if(frame_vec[i]->data_cnt > 0)
			{
				rel_idx = 0;
				buffer = frame::frame_len - frame_vec[i]->data_cnt;
				buffer = std::min(buffer, noutput_items*d_veclength/2);	
			}
			// if frame beginning lies in this work-call's samples
			else if(frame_vec[i]->start_index >= min_samp && frame_vec[i]->start_index < max_samp)
			{
				rel_idx = frame_vec[i]->start_index - min_samp;
				buffer = frame::frame_len;
				buffer = std::min(buffer, (int) (max_samp - frame_vec[i]->start_index));	
			}
			else
			{
				continue;
			}
			
			memcpy(&frame_vec[i]->data[frame_vec[i]->data_cnt], &in1[rel_idx], buffer*sizeof(gr_complex));
			frame_vec[i]->data_cnt += buffer;

			// if entire frame collected
			if(frame_vec[i]->data_cnt == frame::frame_len)
			{
				// correct cfo
				//gr_complex phase_incre = exp(gr_complex(0, frame_vec[i]->cfo_angular));
				//gr_complex phase_start = gr_complex(1.0f, 0.0f);
				//volk_32fc_s32fc_x2_rotator_32fc(frame_vec[i]->data, frame_vec[i]->data, phase_incre, &phase_start, frame_vec[i]->data_cnt);
				
				// correct cfo
				gr_complex phase_incre = exp(gr_complex(0, -frame_vec[i]->cfo_angular));
				gr_complex phase_start = gr_complex(1.0f, 0.0f);
				//volk_32fc_s32fc_x2_rotator_32fc(frame_vec[i]->data, frame_vec[i]->data, phase_incre, &phase_start, frame_vec[i]->data_cnt);				
				
				// frame length in bytes
				unsigned int frame_byte_length = frame_vec[i]->data_cnt*sizeof(gr_complex);
			
				// publish PDU
				pmt::pmt_t PMT_md = pmt::from_uint64(frame_vec[i]->recv_tag);
				pmt::pmt_t dict = pmt::make_dict();
				dict = pmt::dict_add(dict, pmt::mp("receiver_tag"), PMT_md);
				pmt::pmt_t PMT_FRAME_blob = pmt::make_blob((char*) frame_vec[i]->data, frame_byte_length);
				message_port_pub(pmt::mp("frame out"), pmt::cons(dict, PMT_FRAME_blob));
			}
		}
		
		// STEP 3: delete frames
		for(int i=frame_vec.size()-1; i>=0; i--)
		{			
			// if entire frame collected
			if(frame_vec[i]->data_cnt == frame::frame_len)
			{
				// delete frame	
				delete frame_vec[i];
				frame_vec.erase(frame_vec.begin() + i);
			}			
		}

		consume_each(noutput_items);   
		
        return 0;
    }

  } /* namespace fbmc1 */
} /* namespace gr */

