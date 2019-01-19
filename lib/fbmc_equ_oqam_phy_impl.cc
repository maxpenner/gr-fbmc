/* -*- c++ -*- */
/* 
 * Copyright 2016 <+YOU OR YOUR COMPANY+>.
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
#include "fbmc_equ_oqam_phy_impl.h"

#include <volk/volk.h>

#include "utils/frame_param.h"
#include "utils/preamble_set_freq_domain.h"
#include "utils/receiver_tag.h"
#include "phy_coding/decoder.h"

static bool test = false;

#define PI_1_2 1.570796327f
#define PI_1 3.141592654f
#define PI_2 6.283185307f

#define wrap_pi(x) (x < -PI_1 ? x+PI_2 : (x > PI_1 ? x - PI_2: x))

namespace gr {
  namespace fbmc1 {

    fbmc_equ_oqam_phy::sptr
    fbmc_equ_oqam_phy::make(int veclength, int filterlength, int symbols_per_frame, int preamble_set, int encoding_family, std::vector<int> logical_channels)
    {
      return gnuradio::get_initial_sptr
        (new fbmc_equ_oqam_phy_impl(veclength, filterlength, symbols_per_frame, preamble_set, encoding_family, logical_channels));
    }

    /*
     * The private constructor
     */
    fbmc_equ_oqam_phy_impl::fbmc_equ_oqam_phy_impl(int veclength, int filterlength, int symbols_per_frame, int preamble_set, int encoding_family, std::vector<int> logical_channels)
      : gr::sync_block("fbmc_equ_oqam_phy",
              gr::io_signature::make(1, 1, sizeof(gr_complex)*veclength),
              gr::io_signature::make(0,0,0))
    {
		// input variables
		d_veclength = veclength;
		d_filterlength = filterlength;
		d_spf = symbols_per_frame;
		d_preamble_set = preamble_set;
		d_encoding_fam = encoding_family;	
		logical_channel_convert(&logical_channels[0], logical_channels.size(), d_log_channs);
		
		// streaming
		d_receiver_tag = 0;
		
		// state machine
		d_counter = 0;
		
		// frame collection
		d_trans_symbols = (2*d_filterlength-FRAME_SPACER_LENGTH_REDUCE)*2;
		d_frame_symbols = d_trans_symbols + 2*d_spf;
		d_frame = (gr_complex*) volk_malloc(d_veclength*d_frame_symbols*sizeof(gr_complex), volk_get_alignment());	
		
		// equalizer
		get_scaled_preamble_set_freq_domain(preamble_set, d_veclength, FRAME_PREAMBLE_PRESCALE, d_preamble);
		
		// oqam
		d_mask = (gr_complex*) volk_malloc(2*d_veclength*sizeof(gr_complex), volk_get_alignment());	
		for(int i=0; i<d_veclength; i=i+2)
		{
			d_mask[i] = gr_complex(1,0);
			d_mask[i+1] = gr_complex(0,-1);
		}
		for(int i=d_veclength; i<d_veclength*2; i=i+2)
		{
			d_mask[i] = gr_complex(0,-1);
			d_mask[i+1] = gr_complex(1,0);
		}
		
		// phy decoding
		logical_channels_widths(&logical_channels[0], logical_channels.size(), d_widths);
		int widest_channel = *max_element(d_widths.begin(), d_widths.end());
		d_samples = (gr_complex*) malloc(widest_channel*d_spf*sizeof(gr_complex));		

		// output port
		message_port_register_out(pmt::mp("phy out"));
		
		// VOLK
		const int alignment_multiple = volk_get_alignment()/sizeof(gr_complex);
		set_alignment(std::max(1, alignment_multiple));
		
		// statistics
#ifdef MAC_DECODER_PRINT_RATE
		d_phy_frames = 0;
		d_phy_errors = 0;
#ifdef VALVE_FD_DOUBLE_FRAMING
		d_phy_double = 0;
#endif
		d_thread = new boost::thread(boost::bind(&fbmc_equ_oqam_phy_impl::send_stats, this));
#endif
	}

    /*
     * Our virtual destructor.
     */
    fbmc_equ_oqam_phy_impl::~fbmc_equ_oqam_phy_impl()
    {
		volk_free(d_frame);
		volk_free(d_mask);
		free(d_samples);
		
#ifdef MAC_DECODER_PRINT_RATE
		{
			gr::thread::scoped_lock(d_mutex);
			d_thread->interrupt();
			d_thread->join();
			delete d_thread;
		}
#endif
    }
    
    //______________________________________________________
    void fbmc_equ_oqam_phy_impl::channel_estimation()
    {
		// vector position in preamble for channel estimation
		const gr_complex *in = &d_frame[2*d_filterlength*d_veclength];
		
		// channel coefficients
		gr_complex *chan_coef = (gr_complex*) volk_malloc(d_veclength*sizeof(gr_complex), volk_get_alignment());
		memset(chan_coef, 0x00, d_veclength*sizeof(gr_complex));
		
		// which channel?
		int logical_channel = receiver_tag_logical_channel(d_receiver_tag);
		
		// create allocation vector
		std::vector<int> alloc_vec(d_veclength, LOG_CHANN_SUBCCARRIER_DEACT);
		for(int i=0; i<d_log_channs[logical_channel].get_size(); i++)
		{
			int index = d_log_channs[logical_channel].subcarrier[i];
			alloc_vec[index] = LOG_CHANN_SUBCCARRIER_ACTIV;
		}

		// estimate channels with even index
		for(int i=0; i<d_veclength; i=i+2)
		{
			if((alloc_vec[i] == LOG_CHANN_SUBCCARRIER_ACTIV) && (in[i] != gr_complex(0.0f,0.0f)))
				chan_coef[i] = d_preamble[i]/in[i];
		}

		// estimate channels with odd index
		for(int i=1; i<d_veclength; i=i+2)
		{
			if(alloc_vec[i] == LOG_CHANN_SUBCCARRIER_ACTIV)
			{
				if((alloc_vec[i-1] == LOG_CHANN_SUBCCARRIER_ACTIV) && (alloc_vec[i+1] == LOG_CHANN_SUBCCARRIER_ACTIV))
					chan_coef[i] = (chan_coef[i-1]+chan_coef[i+1])/gr_complex(2.0f,0.0f);
				else
					chan_coef[i] = chan_coef[i-1];
			}
		}

		// special case: carrier next to DC
		if(alloc_vec[d_veclength/2+1] == LOG_CHANN_SUBCCARRIER_ACTIV)
			chan_coef[d_veclength/2+1] = chan_coef[d_veclength/2+2];
			
		// ###################################
		// DEBUG: pilots
		chan_coef[11] = (chan_coef[10]+chan_coef[12])/gr_complex(2.0f, 0.0f);
		chan_coef[25] = (chan_coef[24]+chan_coef[26])/gr_complex(2.0f, 0.0f);
		chan_coef[39] = (chan_coef[38]+chan_coef[40])/gr_complex(2.0f, 0.0f);
		chan_coef[53] = (chan_coef[52]+chan_coef[54])/gr_complex(2.0f, 0.0f);
		// ###################################			
			
		// apply channel correction
		int minn = d_log_channs[logical_channel].get_min();
		int maxx = d_log_channs[logical_channel].get_max();
		for(int i=d_trans_symbols; i<d_frame_symbols; i++)
		{
			int idx = i*d_veclength + minn;
			volk_32fc_x2_multiply_32fc(&d_frame[idx], &d_frame[idx], &chan_coef[minn], maxx-minn+1);
		}

		// clean up
		volk_free(chan_coef);
	}
	
	//______________________________________________________
    void fbmc_equ_oqam_phy_impl::phase_tracking()
    {
		// ###################################
		// DEBUG: pilots
				
		// vector position in preamble for phase tracking
		const gr_complex *in = &d_frame[d_trans_symbols*d_veclength];
		
		int d_pilot_space = 2;
		int d_pilot_space_cnt = (d_pilot_space+1)*2;
		int cnt = 0;
		int cnt2 = 0;
		std::vector<float> d_measuredPilotsPhase(4, 0.0f);
				
		if(!test)
		{
			test = true;
			
			cnt = d_pilot_space_cnt;
			
			for(int i=d_pilot_space_cnt; i<d_frame_symbols - d_trans_symbols; i++)
			{
				cnt = (cnt == d_pilot_space_cnt) ? 0 : cnt;
				
				if(cnt == d_pilot_space_cnt - 4)
				{
					cnt2++;
					
					float arg0 = wrap_pi(arg(in[i*d_veclength+11]))-wrap_pi(arg(in[(i-d_pilot_space_cnt)*d_veclength+11]));
					float arg1 = wrap_pi(arg(in[i*d_veclength+25]))-wrap_pi(arg(in[(i-d_pilot_space_cnt)*d_veclength+25]));
					float arg2 = wrap_pi(arg(in[i*d_veclength+39]))-wrap_pi(arg(in[(i-d_pilot_space_cnt)*d_veclength+39]));
					float arg3 = wrap_pi(arg(in[i*d_veclength+53]))-wrap_pi(arg(in[(i-d_pilot_space_cnt)*d_veclength+53]));
					
					arg0 = wrap_pi(arg0);
					arg1 = wrap_pi(arg1);
					arg2 = wrap_pi(arg2);
					arg3 = wrap_pi(arg3);					
					
					d_measuredPilotsPhase[0] += arg0;
					d_measuredPilotsPhase[1] += arg1;
					d_measuredPilotsPhase[2] += arg2;
					d_measuredPilotsPhase[3] += arg3;
					
					std::cout << "i: " << i << " " << in[i*d_veclength+11] << std::endl;
					std::cout << "i: " << i << " " << arg(in[i*d_veclength+11])6.283185307f*360.0f << std::endl;
					std::cout << "i: " << i << " " << arg(in[(i-d_pilot_space_cnt)*d_veclength+11])/6.283185307f*360.0f << std::endl;
					std::cout << "i: " << i << " " << arg0/6.283185307f*360.0f << std::endl;
				}
				
				cnt++;
			}
			
			// total phase freq
			float blablub = 0.0f;
			for(int k=0; k<4; k++)
				blablub += d_measuredPilotsPhase[k];
			blablub /= 4.0f;
			
			// divide by cases
			d_measuredPilotsPhase[0] /= (float) cnt2;
			d_measuredPilotsPhase[1] /= (float) cnt2;
			d_measuredPilotsPhase[2] /= (float) cnt2;
			d_measuredPilotsPhase[3] /= (float) cnt2;
			blablub /= (float) cnt2;
			
			// measured offsets
			std::cout << "0 f: " << d_measuredPilotsPhase[0]/6.283185307f*360.0f << std::endl;
			std::cout << "1 f: " << d_measuredPilotsPhase[1]/6.283185307f*360.0f << std::endl;
			std::cout << "2 f: " << d_measuredPilotsPhase[2]/6.283185307f*360.0f << std::endl;
			std::cout << "3 f: " << d_measuredPilotsPhase[3]/6.283185307f*360.0f << std::endl;		
			std::cout << "bla: " << blablub/6.283185307f*360.0f << std::endl;
			
			// estimated cfo
			float factor = 5e6/6.283185307f;
			factor /= (float) d_veclength;
			factor /= (float) (d_pilot_space+1);
			
			std::cout << "cfo: " << blablub*factor << std::endl;
		}		

		// ###################################
	}
	
	//______________________________________________________
    void fbmc_equ_oqam_phy_impl::oqamization()
    {	
        gr_complex *buffer = (gr_complex*) volk_malloc(2*d_veclength*sizeof(gr_complex), volk_get_alignment());
        float *real0 = (float*) volk_malloc(d_veclength*sizeof(float), volk_get_alignment());
        float *real1 = (float*) volk_malloc(d_veclength*sizeof(float), volk_get_alignment());

		// number of input items must be a multiple of 2
        for(int i=0; i<d_frame_symbols - d_trans_symbols; i++)
        {
			// multiply with mask
			volk_32fc_x2_multiply_32fc(buffer, &d_frame[(d_trans_symbols+i*2)*d_veclength], d_mask, d_veclength*2);
			
			// extract two real parts
			volk_32fc_deinterleave_real_32f(real0, buffer, d_veclength);
			volk_32fc_deinterleave_real_32f(real1, &buffer[d_veclength], d_veclength);

			// combine two real parts two one complex vector
			// write to "d_frame" to where the transient part ends
			volk_32f_x2_interleave_32fc(&d_frame[(d_trans_symbols+i)*d_veclength], real0, real1, d_veclength);
		}	
		
		volk_free(buffer);
		volk_free(real0);
		volk_free(real1);
	}
	
	//______________________________________________________
    void fbmc_equ_oqam_phy_impl::phy_decode()
    {
		d_samples_index = 0;
		
		// which channel?
		int logical_channel = receiver_tag_logical_channel(d_receiver_tag);
		
		// collect frequency samples
		for(int i=0; i<d_spf; i++)
		{
			int idx = (d_trans_symbols+i)*d_veclength;
			
			for(int j=0; j<d_log_channs[logical_channel].get_size(); j++)
				d_samples[d_samples_index++] = d_frame[idx + d_log_channs[logical_channel].subcarrier[j]];
		}
		
#ifdef MAC_DECODER_PRINT_RATE
		{
			gr::thread::scoped_lock(d_mutex);
			d_phy_frames++;
#ifdef VALVE_FD_DOUBLE_FRAMING		
			int doubleFrameBit = (int)((d_receiver_tag >> RECEIVER_TAG_DOUBL_FRAM_OFFS) & 1ULL);
			if(doubleFrameBit == 1)
				d_phy_double++;
#endif

		}
#endif		
	
		// load relevant data into decoder
		FBMC_PHY::load_decoder(d_samples_index, d_encoding_fam, -1, d_widths[logical_channel], FBMC_PHY_DECODER_DONT_CARE, FBMC_PHY_DECODER_DONT_CARE);
		
		int mpdu_len = 0;
		
		// check if any data can be extracted
		if(FBMC_PHY::get_mpdu_len(mpdu_len, d_samples) != true)
		{
#ifdef MAC_DECODER_PRINT_RATE
			{
				gr::thread::scoped_lock(d_mutex);
				d_phy_errors++;
			}
#endif			
			return;
		}
		
		// allocate space for mpdu
		char mpdu[mpdu_len];
		
		// extract mpdu
		FBMC_PHY::extract_mpdu(mpdu, d_samples);
		
		// publish PDU
		pmt::pmt_t recv_tag_pmt = pmt::from_uint64(d_receiver_tag);
		pmt::pmt_t dict = pmt::make_dict();
		dict = pmt::dict_add(dict, pmt::mp("receiver_tag"), recv_tag_pmt);
		pmt::pmt_t PMT_MAC_blob = pmt::make_blob(mpdu, mpdu_len);
		message_port_pub(pmt::mp("phy out"), pmt::cons(dict, PMT_MAC_blob));
	}
	
#ifdef MAC_DECODER_PRINT_RATE
    //______________________________________________________
    void fbmc_equ_oqam_phy_impl::send_stats()
    {	
		try 
		{
			while(1)
			{
				boost::this_thread::sleep(boost::posix_time::milliseconds(PHY_DECODER_SEND_RATE*1000));
				{
					gr::thread::scoped_lock(d_mutex);

					// publish PDU
					pmt::pmt_t PMT_phy_frames = pmt::from_uint64(d_phy_frames);
					pmt::pmt_t PMT_phy_errors = pmt::from_uint64(d_phy_errors);
					pmt::pmt_t dict = pmt::make_dict();
					dict = pmt::dict_add(dict, pmt::mp("phy_frames"), PMT_phy_frames);
					dict = pmt::dict_add(dict, pmt::mp("phy_errors"), PMT_phy_errors);
#ifdef VALVE_FD_DOUBLE_FRAMING
					pmt::pmt_t PMT_phy_double = pmt::from_uint64(d_phy_double);
					dict = pmt::dict_add(dict, pmt::mp("phy_double"), PMT_phy_double);
#endif
					pmt::pmt_t PMT_dummy_blob = pmt::make_blob(nullptr, 0);
					message_port_pub(pmt::mp("phy out"), pmt::cons(dict, PMT_dummy_blob));
				}
			}
		}
		catch(boost::thread_interrupted) 
		{
			//PRINT("fbmc_phy_decoder_impl: Thread for sending statistics properly interrupted.");
		}
	}
#endif	
    
    //______________________________________________________
    int
    fbmc_equ_oqam_phy_impl::work(int noutput_items,
			  gr_vector_const_void_star &input_items,
			  gr_vector_void_star &output_items)
    {
		const gr_complex *in = (const gr_complex *) input_items[0];

		int consumed = 0;

		while(consumed < noutput_items)
		{
			int buffer = d_frame_symbols - d_counter;
			buffer = std::min(noutput_items-consumed, buffer);
			
			// extract stream tag
			if(d_counter == 0)
			{
				std::vector<gr::tag_t> tags;
				uint64_t offset = nitems_read(0)+consumed;
				get_tags_in_range(tags, 0, offset, offset+1, pmt::string_to_symbol("receiver_tag"));
				
				if(tags.size() == 0)
					PRINT("fbmc_equ_oqam_phy_impl error: Tag for logical channel not found.");
				else
					d_receiver_tag = pmt::to_uint64(tags[0].value);
			}						
					
			memcpy(&d_frame[d_counter*d_veclength], &in[consumed*d_veclength], buffer*d_veclength*sizeof(gr_complex));				
			
			d_counter += buffer;
			consumed += buffer;
			
			if(d_counter == d_frame_symbols)
			{	
				d_counter = 0;
				channel_estimation();
				phase_tracking();
				oqamization();
				phy_decode();
			}			
		}

        consume_each(noutput_items);
        
        return 0;
    }

  } /* namespace fbmc1 */
} /* namespace gr */
