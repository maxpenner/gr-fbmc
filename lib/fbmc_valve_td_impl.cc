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
#include "fbmc_valve_td_impl.h"

#include <math.h>
#include <volk/volk.h>

#include "utils/frame_param.h"
#include "utils/preamble_set_time_domain.h"
#include "utils/receiver_tag.h"

#define TIME_SYNC_LOG_CHANN			0

//#define CFO_CORRECTION

#ifdef CFO_CORRECTION
#define CFO_FRAME_PEAKS_ESTIM_LENGTH 		1
#endif

#ifdef VALVE_TD_PRINT_RATE
#define SNR_SAMPLES_ESTIM_LENGTH		100
#define SNR_FRAMES_ESTIM_LENGTH			50
#endif

//#define DEBUG_VALVE_TD_REFERENCE_MODE

#ifdef DEBUG_VALVE_TD_REFERENCE_MODE
static uint64_t debug_start = 0;
static int length_an = 20;
static uint64_t old_ref_time = 0;
static std::vector<uint64_t> old_times(length_an, 0);
static std::vector<uint64_t> old_frame_types(length_an, 0);
#endif

namespace gr {
  namespace fbmc1 {

    fbmc_valve_td::sptr
    fbmc_valve_td::make(int veclength, int filterlength, int symbols_per_frame, float threshold, int plateau, int preamble_set, int sync_type, int sync_len, int sync_symm, int sync_refresh, int init_lock)
    {
      return gnuradio::get_initial_sptr
        (new fbmc_valve_td_impl(veclength, filterlength, symbols_per_frame, threshold, plateau, preamble_set, sync_type, sync_len, sync_symm, sync_refresh, init_lock));
    }

    /*
     * The private constructor
     */
    fbmc_valve_td_impl::fbmc_valve_td_impl(int veclength, int filterlength, int symbols_per_frame, float threshold, int plateau, int preamble_set, int sync_type, int sync_len, int sync_symm, int sync_refresh, int init_lock)
      : gr::block("fbmc_valve_td",
              gr::io_signature::make2(2, 2, sizeof(gr_complex), sizeof(gr_complex)),
              gr::io_signature::make(1, 2, sizeof(gr_complex)))
    {
		// init variables
		d_MODE 			= FREERUN;
		d_FREERUN_STATE	= COARSE_SYNC;
		d_REFEREN_STATE	= COARSE_SYNC_REF;
		d_veclength 	= veclength;
		d_preamble_len  = (filterlength + 1 - FRAME_SPACER_LENGTH_REDUCE)*d_veclength;
		d_samp_frame	= d_preamble_len + (symbols_per_frame + filterlength - 1)*d_veclength;
		d_input_collect = (filterlength + 1)*d_veclength +  d_samp_frame;
		d_counter 		= 0;
		d_init_lock 	= (init_lock == 1) ? true : false;

#ifdef VALVE_TD_PRINT_RATE
		d_valve_timer 	= new fbmc_timer(VALVE_TD_PRINT_RATE);
		d_snr_avg		= 0.0;
		d_input_collect += SNR_SAMPLES_ESTIM_LENGTH;
#endif

		d_threshold		= threshold;
		d_plateau		= plateau;
		d_peak_index	= 0;
		d_type			= (sync_type == 0 || sync_type == 1) ? sync_type : 1;
		d_len   		= (sync_len <= d_preamble_len) ? sync_len : d_preamble_len;
		d_offset 		= ((filterlength + 1)*d_veclength - d_len)/2;
		d_offset 		= (d_offset + d_len > d_preamble_len) ? d_preamble_len - d_len: d_offset;
		d_symm   		= (sync_symm < 0) ? (filterlength + 1)*d_veclength : sync_symm;
		d_symm   		= (sync_symm > (filterlength + 1)*d_veclength) ? (filterlength + 1)*d_veclength : sync_symm;
		d_refresh		= (sync_refresh == 0) ? 1 : sync_refresh;
		d_sync_counter  = 0;
		d_cut_start 	= 0;
		d_frame_start	= 0;

		size_t alig = volk_get_alignment();
		d_frame_in0 = (gr_complex*) volk_malloc(d_input_collect*sizeof(gr_complex), alig);
		d_frame_in1 = (gr_complex*) volk_malloc(d_input_collect*sizeof(gr_complex), alig);
		d_preamble  = (gr_complex*) volk_malloc(d_len*sizeof(gr_complex), alig);
		
		d_fir = NULL;		
		
		// load preamble
		get_preamble_set_conj_time_domain(preamble_set, d_veclength, filterlength, d_offset, d_len, d_preamble);
		
		// preload filter if neccessary
		if(d_type == 1)
			preload_fir_filter();		
			
		// init variables for mode REFERENCE
		d_case_samples = 0;
		d_known_offset = 0;
		d_found_unsecu = 0;
		d_refer_thresh = 0.0f;		
		d_refer_start = 0;
		d_known_start = 0;
		d_free_search_len = 0;
		d_known_frame = NULL;
		
		// for simulation remove tags 
		set_tag_propagation_policy(TPP_DONT);

		// init VOLK
		const int alignment_multiple = volk_get_alignment() / sizeof(gr_complex);
		set_alignment(std::max(1, alignment_multiple));
		
		// input ports
		message_port_register_in(pmt::mp("valve in"));
		set_msg_handler(pmt::mp("valve in"), boost::bind(&fbmc_valve_td_impl::valve_in, this, _1));
		
		// output port
		message_port_register_out(pmt::mp("valve out"));		
	}

    /*
     * Our virtual destructor.
     */
    fbmc_valve_td_impl::~fbmc_valve_td_impl()
    {
		volk_free(d_frame_in0);
		volk_free(d_frame_in1);
		volk_free(d_preamble);
		
		delete d_fir;
		
		volk_free(d_known_frame);
		
#ifdef VALVE_TD_PRINT_RATE
		delete d_valve_timer;
#endif

    }

	//_________________________________________________
    void fbmc_valve_td_impl::preload_fir_filter()
    {
		std::vector<gr_complex> temp;

		// matched filter in reverse order
		for(int i=d_len-1; i>=0; i--) 
			temp.push_back(d_preamble[i]);
		
		d_fir = new gr::filter::kernel::fir_filter_ccc(1,temp);
			
		volk_free(d_preamble);
		d_preamble = NULL;
    }

    //______________________________________________________
    void fbmc_valve_td_impl::cfo_fine_sync()
    {
		
#ifdef CFO_CORRECTION
		// find absolute peak
		unsigned int complex_peak_index;
		volk_32fc_index_max_16u(&complex_peak_index, d_frame_in0, d_preamble_len);

		// append to moving average container
		if(d_complex_peaks.size() == CFO_FRAME_PEAKS_ESTIM_LENGTH)
			d_complex_peaks.erase(d_complex_peaks.begin());
		d_complex_peaks.push_back(d_frame_in0[complex_peak_index]);
		
		// average value of peak from past frames
		gr_complex average_peak = gr_complex(0.0f,0.0f);
		for(int i=0;i<d_complex_peaks.size(); i++)
			average_peak += d_complex_peaks[i];

		// correct cfo
		d_cfo_angular = arg(average_peak)/d_veclength;
		gr_complex phase_incre = exp(gr_complex(0.0f, d_cfo_angular));
		gr_complex phase_start = gr_complex(1.0f, 0.0f);
		volk_32fc_s32fc_x2_rotator_32fc(d_frame_in1, d_frame_in1, phase_incre, &phase_start, d_input_collect);
		
		// DEBUG: show cfo in Hertz (sampling rate has to be set manually)
		//float samp_rate = 5e6;
		//float cfo_hz = d_cfo_angular*(-1)*samp_rate/(2*3.14159);
		//PRINT(cfo_hz);
#endif

		// set borders for matched filter
		int low, high;
		int full_preamble_len = FRAME_SPACER_LENGTH_REDUCE*d_veclength + d_preamble_len;
		if(d_refresh > 0)
		{
			// maximum search area
			if(d_sync_counter++ % d_refresh == 0)
			{
				low  = 0;
				high = full_preamble_len;
			}
			// symmetric search area around last synchronization point
			else
			{
				low  = (d_peak_index-d_symm > 0) ? d_peak_index-d_symm : 0;
				high = (d_peak_index+d_symm <= full_preamble_len) ? d_peak_index + d_symm : full_preamble_len;
			}
		}
		else
		{
			low  = 0;
			high = full_preamble_len - d_symm;			
		}
		
		// allocate memory
		size_t alig = volk_get_alignment();
		gr_complex *convol = (gr_complex*) volk_malloc(sizeof(gr_complex)*(high-low), alig);

		// convolute with preamble in time domain or with preamble in frequency domain via fir-filter
		if(d_type == 0)
		{
			for(int i=low; i<high; i++)
				volk_32fc_x2_dot_prod_32fc(&convol[i-low], &d_frame_in1[i+d_offset], d_preamble, d_len);	
		}
		else if(d_type == 1)
			d_fir->filterN(convol, &d_frame_in1[d_offset+low], high-low);
		
		// find fine synchronisation point
		uint16_t temp;
		unsigned int temp2 = 0;
		volk_32fc_index_max_16u(&temp, convol, high-low);
		temp2 += (unsigned int) temp;
		temp2 += (unsigned int) low;
		d_peak_index = (int) temp2;
		
		// set start index of frame
		d_frame_start = d_cut_start + d_peak_index;
		
		// clean up
		volk_free(convol);
    }
    
    //______________________________________________________
    void fbmc_valve_td_impl::add_receiver_tag(int produced, int frame_type)
    {
		uint64_t receiver_tag = 0;
		
		// set sample index
		receiver_tag = d_frame_start;
		receiver_tag = receiver_tag << RECEIVER_TAG_SAMPL_NUMB_OFFS;
		
		// set frame type
		receiver_tag |= (frame_type & 1) << RECEIVER_TAG_FRAME_TYPE_OFFS;
		receiver_tag |= ((frame_type >> 1) & 1) << (RECEIVER_TAG_FRAME_TYPE_OFFS + 1);
		
		// set double framing
		// time domain synchronization has no frame doubling
		
		// set logical channel
		receiver_tag |= TIME_SYNC_LOG_CHANN & RECEIVER_TAG_LOGIC_CHAN_MASK;
		
		pmt::pmt_t key = pmt::string_to_symbol("receiver_tag");
		pmt::pmt_t value = pmt::from_uint64(receiver_tag);
		pmt::pmt_t srcid = pmt::string_to_symbol(alias());
		add_item_tag(0, nitems_written(0)+produced, key, value, srcid);
		
#ifdef DEBUG_VALVE_TD_REFERENCE_MODE					
		if(debug_start++ > 0)
		{
			bool error = false;
			
			if(frame_type == VALVE_FRAME_TYPE_REFER)
			{
				uint64_t diff = d_frame_start - old_ref_time;
								
				if(diff != d_case_samples + d_preamble_len + FRAME_SPACER_LENGTH_REDUCE*d_veclength)
				{
					PRINT(" ");
					PRINT(" ERROR 0 ");
					PRINT(old_ref_time);
					PRINT(diff);
					error = true;
				}
			}
			else if(old_frame_types[length_an-1] == VALVE_FRAME_TYPE_REFER)
			{				
				if(frame_type != VALVE_FRAME_TYPE_FOUND && frame_type != VALVE_FRAME_TYPE_KNOWN)
				{
					PRINT(" ");
					PRINT(" ERROR 1 ");
					error = true;
				}
			}
			else if(old_frame_types[length_an-1] == VALVE_FRAME_TYPE_FOUND)
			{
				if(frame_type != VALVE_FRAME_TYPE_KNOWN)
				{
					PRINT(" ");
					PRINT(" ERROR 2 ");
					error = true;
				}				
			}
			else if(old_frame_types[length_an-1] == VALVE_FRAME_TYPE_KNOWN)
			{
				if(frame_type != VALVE_FRAME_TYPE_REFER)
				{
					PRINT(" ");
					PRINT(" ERROR 3 ");
					error = true;
				}				
			}
			
			if(error == true)
			{
				PRINT(d_frame_start);
				PRINT(frame_type);				
				for(int i=0; i<length_an; i++)
					std::cout << "i: " << i << " " << old_times[i] << std::endl;
				for(int i=0; i<length_an; i++)
					std::cout << "i: " << i << " " << old_frame_types[i] << std::endl;
			}
		}	
		
		if(frame_type == VALVE_FRAME_TYPE_REFER)
			old_ref_time = d_frame_start;
				
		old_frame_types.erase(old_frame_types.begin());
		old_frame_types.push_back(frame_type);
		old_times.erase(old_times.begin());
		old_times.push_back(d_frame_start);
#endif

    }
    
    //______________________________________________________
    void fbmc_valve_td_impl::valve_in(pmt::pmt_t msg) 
    {
		// mode reference
		if(pmt::dict_has_key(pmt::car(msg), pmt::mp("case_samples")))
		{			
			// extract information
			d_case_samples = (int) pmt::to_long(pmt::dict_ref(pmt::car(msg), pmt::mp("case_samples"), pmt::PMT_NIL));
			d_known_offset = (int) pmt::to_long(pmt::dict_ref(pmt::car(msg), pmt::mp("known_offset"), pmt::PMT_NIL));
			d_found_unsecu = (int) pmt::to_long(pmt::dict_ref(pmt::car(msg), pmt::mp("found_unsecu"), pmt::PMT_NIL));
			d_refer_thresh = (float) pmt::to_double(pmt::dict_ref(pmt::car(msg), pmt::mp("refer_thresh"), pmt::PMT_NIL));
			
			// set and reset variables
			d_MODE = REFERENCE;
			d_refer_start = 0;
			d_known_start = 0;	
			d_free_search_len = 0;	
			d_counter = 0;		
			d_known_frame_cnt = 0;
			
			if(d_REFEREN_STATE == COPY_DATA_REF || d_REFEREN_STATE == COPY_KNOWN || d_REFEREN_STATE == COPY_DATA_FREE)
			{
				PRINT("fbmc_valve_td_impl error: Reset command received, but still in copy mode.");
				PRINT(d_REFEREN_STATE);
			}
			else
				d_REFEREN_STATE = COARSE_SYNC_REF;	
			
			// allocate memory for known frame
			volk_free(d_known_frame);
			size_t alig = volk_get_alignment();		
			d_known_frame = (gr_complex*) volk_malloc((d_samp_frame+d_veclength/2)*sizeof(gr_complex), alig);
		}
		
		// unblock the input
		d_init_lock = false;
			
		// answer with an empty reply
		pmt::pmt_t PMT_dummy_val = pmt::from_long(0);
		pmt::pmt_t dict = pmt::make_dict();
		dict = pmt::dict_add(dict, pmt::mp("VALVE_DUMMY_ANSWER"), PMT_dummy_val);			
		pmt::pmt_t PMT_dummy_blob = pmt::make_blob(nullptr, 0);
		message_port_pub(pmt::mp("valve out"), pmt::cons(dict, PMT_dummy_blob));	
	}    

#ifdef VALVE_TD_PRINT_RATE
    //______________________________________________________
    void fbmc_valve_td_impl::snr_estimation()
    {
		gr_complex temp;
		
		// signal + noise power from payload
		int payload_offset = d_peak_index + 2*d_preamble_len + (FRAME_SPACER_LENGTH_REDUCE-2)*d_veclength;
		volk_32fc_x2_conjugate_dot_prod_32fc(&temp, &d_frame_in1[payload_offset], &d_frame_in1[payload_offset], SNR_SAMPLES_ESTIM_LENGTH);
		float payload_plus_noise = temp.real();
		
		// noise power from appended zeros
		int noise_offset = d_peak_index + d_samp_frame;
		volk_32fc_x2_conjugate_dot_prod_32fc(&temp, &d_frame_in1[noise_offset], &d_frame_in1[noise_offset], SNR_SAMPLES_ESTIM_LENGTH);
		float snr = (payload_plus_noise-temp.real())/temp.real();

		// moving average
		if(d_snr_shift_reg.size() == SNR_FRAMES_ESTIM_LENGTH)
			d_snr_shift_reg.erase(d_snr_shift_reg.begin());
		if(snr>0)
			d_snr_shift_reg.push_back(snr);
			
		d_snr_avg = 0;
		for(int i=0; i<d_snr_shift_reg.size(); i++)
			d_snr_avg += d_snr_shift_reg[i];
			
		d_snr_avg = d_snr_avg/d_snr_shift_reg.size();
		d_snr_avg = 10*log10(d_snr_avg);
    }

    //______________________________________________________
    void fbmc_valve_td_impl::display()
    {
		if(d_valve_timer->new_update())
		{
			std::cout << std::endl << std::endl
			<< "---------------- Valve TD ----------------" << std::endl
			<< "frames detected: " << d_sync_counter << std::endl
			<< "S/N: " << d_snr_avg << " dB" << std::endl
			<< "------------------------------------------" << std::endl;
		}
    }
#endif    

	//_________________________________________________
    void
    fbmc_valve_td_impl::forecast (int noutput_items, gr_vector_int &ninput_items_required)
    {
		// demand the maximum
		ninput_items_required[0] = 8191; 
		ninput_items_required[1] = 8191;
    }

    int
    fbmc_valve_td_impl::general_work (int noutput_items,
                       gr_vector_int &ninput_items,
                       gr_vector_const_void_star &input_items,
                       gr_vector_void_star &output_items)
    {
		const gr_complex *in0 = (const gr_complex *) input_items[0];
		const gr_complex *in1 = (const gr_complex *) input_items[1];
		gr_complex *out0 = (gr_complex *) output_items[0];
		gr_complex *out1 = (gr_complex *) output_items[1];
		
		int consumed  = 0;
		int produced  = 0;
		int buffer = 0;
		
		int n_input  = std::min(ninput_items[0], ninput_items[1]);
		
		// if block is locked drop input samples and produce no output
		if(d_init_lock == true)
		{
			consume(0, n_input);
			consume(1, n_input);
			return 0;			
		}

		while(consumed < n_input && produced < noutput_items)
		{			
			// ################################################
			// ################################################
			// ################################################
			if(d_MODE == FREERUN)
			{			
				switch(d_FREERUN_STATE)
				{
					//_______________________
					case COARSE_SYNC:

#ifdef VALVE_TD_PRINT_RATE	
						display();
#endif
					
						for(consumed; consumed<n_input; consumed++)
						{
							if(abs(in0[consumed]) > d_threshold)
							{
								d_counter++;
								if(d_counter == d_plateau)
								{
									d_counter = 0;
									d_cut_start = nitems_read(0) + consumed;
									d_FREERUN_STATE = COLLECT_FRAME;
									break;
								}
							}
							else
								d_counter = 0;
						}
						
						break;

					//_______________________
					case COLLECT_FRAME:
					
						buffer = d_input_collect - d_counter;
						buffer = std::min(n_input-consumed, buffer);
						
						// copy stream-samples to buffer
						memcpy(&d_frame_in0[d_counter], &in0[consumed], buffer*sizeof(gr_complex));
						memcpy(&d_frame_in1[d_counter], &in1[consumed], buffer*sizeof(gr_complex));

						d_counter += buffer;
						consumed += buffer;
						
						if(d_counter == d_input_collect)
						{
							cfo_fine_sync();
							
#ifdef VALVE_TD_PRINT_RATE	
							snr_estimation();
#endif

							d_counter = 0;
							d_FREERUN_STATE = COPY_DATA;
						}
						
						break;

					//_______________________
					case COPY_DATA:
					
						buffer = d_samp_frame - d_counter;
						buffer = std::min(buffer, noutput_items-produced);
						
						if(d_counter == 0)
							add_receiver_tag(produced, VALVE_FRAME_TYPE_FREER);

						// fill output, out1 has to be delayed
						memcpy(&out0[produced], &d_frame_in1[d_peak_index+d_counter], buffer*sizeof(gr_complex));
						memcpy(&out1[produced], &d_frame_in1[d_peak_index+d_counter+d_veclength/2], buffer*sizeof(gr_complex));

						d_counter += buffer;
						produced  += buffer;
						
						if(d_counter == d_samp_frame)
						{
							d_counter = 0;
							d_FREERUN_STATE = CHECK_RESIDUAL;
						}
						
						break;

					//_______________________
					case CHECK_RESIDUAL:
					{
						// skip preamble
						buffer = d_peak_index;		 // search through payload as well
						buffer = 100*d_veclength/64; // width of sync peak at 0.2
						
						// calculate magnitude to search for second frame in buffered input
						size_t alig = volk_get_alignment();
						float *temp = (float*) volk_malloc((d_input_collect-buffer)*sizeof(float), alig);
						volk_32fc_magnitude_32f(temp, &d_frame_in0[buffer], d_input_collect-buffer);

						for(int i=0; i<d_input_collect-buffer; i++)
						{
							if(temp[i] > d_threshold)
							{
								d_counter++;
								if(d_counter == d_plateau)
								{
									d_counter = 0;
									d_FREERUN_STATE = COPY_RESIDUAL;
									buffer += i;
									volk_free(temp);
									temp = NULL;
									break;
								}
							}
							else
								d_counter = 0;
						}
						
						volk_free(temp);
						
						if(d_FREERUN_STATE == CHECK_RESIDUAL)
							d_FREERUN_STATE = COARSE_SYNC;
							
						break;
					}
					
					//_______________________
					case COPY_RESIDUAL:
					{
						d_counter = d_input_collect - buffer;
						
#ifdef CFO_CORRECTION
						// restore buffered samples
						gr_complex phase_incre = exp(gr_complex(0, -d_cfo_angular));
						gr_complex phase_start = exp(gr_complex(0, -d_cfo_angular*d_counter));
						volk_32fc_s32fc_x2_rotator_32fc(&d_frame_in1[d_counter], &d_frame_in1[d_counter], phase_incre, &phase_start, d_input_collect-d_counter);
#endif

						d_cut_start += buffer;
						
						// copy stream-samples to buffer
						memmove(d_frame_in0, &d_frame_in0[buffer], d_counter*sizeof(gr_complex));
						memmove(d_frame_in1, &d_frame_in1[buffer], d_counter*sizeof(gr_complex));

						d_FREERUN_STATE = COLLECT_FRAME;
						
						break;
					}
				}
				
			}
			// ################################################
			// ################################################
			// ################################################
			
			// ################################################
			// ################################################
			// ################################################			
			else if(d_MODE = REFERENCE)
			{					
				switch(d_REFEREN_STATE)
				{
					//_______________________
					case COARSE_SYNC_REF:
					
						for(consumed; consumed<n_input; consumed++)
						{
							if(abs(in0[consumed]) > d_refer_thresh)
							{
								d_counter++;
								if(d_counter == d_plateau)
								{																	
									d_counter = 0;
									d_cut_start = nitems_read(0) + consumed;							
									d_REFEREN_STATE = COLLECT_FRAME_REF;									
									d_free_lock = false;
									break;
								}
							}
							else
								d_counter = 0;
						}
						
						break;

					//_______________________
					case COLLECT_FRAME_REF:
					
						buffer = d_input_collect - d_counter;
						buffer = std::min(n_input-consumed, buffer);
						
						// copy stream-samples to buffer
						memcpy(&d_frame_in0[d_counter], &in0[consumed], buffer*sizeof(gr_complex));
						memcpy(&d_frame_in1[d_counter], &in1[consumed], buffer*sizeof(gr_complex));

						d_counter += buffer;
						consumed += buffer;
						
						if(d_counter == d_input_collect)
						{
							cfo_fine_sync();
							
							// set start indices for this case
							d_refer_start = d_frame_start;
							d_known_start = d_refer_start + d_known_offset;							
							d_counter = 0;
							d_REFEREN_STATE = COPY_DATA_REF;					
						}
							
						break;
						
					//_______________________
					case COPY_DATA_REF:
														
						buffer = d_samp_frame - d_counter;
						buffer = std::min(buffer, noutput_items-produced);
						
						if(d_counter == 0)
							add_receiver_tag(produced, VALVE_FRAME_TYPE_REFER);

						// fill output, out1 has to be delayed
						memcpy(&out0[produced], &d_frame_in1[d_peak_index+d_counter], buffer*sizeof(gr_complex));
						memcpy(&out1[produced], &d_frame_in1[d_peak_index+d_counter+d_veclength/2], buffer*sizeof(gr_complex));

						d_counter += buffer;
						produced  += buffer;
						
						if(d_counter == d_samp_frame)
						{																					
							// bound input for free search
							d_free_search_len = d_case_samples - (d_input_collect - d_peak_index);
							d_counter = 0;				
							d_REFEREN_STATE = COARSE_SYNC_FREE;
						}
						
						break;										
						
					//_______________________
					case COARSE_SYNC_FREE:
					{										
						// bound input for free search
						buffer = std::min(d_free_search_len, n_input - consumed);
						int temp_consumed = consumed;
					
						for(consumed; consumed < temp_consumed + buffer; consumed++)
						{
							d_free_search_len--;
							
							if(abs(in0[consumed]) > d_threshold)
							{
								d_counter++;
								if(d_counter == d_plateau)
								{																
									d_counter = 0;
									d_cut_start = nitems_read(0) + consumed;
																		
									// Check if possible frame lies within reasonable area.
									// Reasonable is an index after the known start + the full preamble length + a security area of veclength.
									// If not reasonable, ignore coarse sync frame -> target frame won't be found anyway, it's too late.
									uint64_t max_start = d_known_start + d_preamble_len + FRAME_SPACER_LENGTH_REDUCE*d_veclength + d_veclength;
									if(d_cut_start <= max_start)
									{							
										d_REFEREN_STATE = COLLECT_FRAME_FREE;
										break;
									}
									else
										d_counter = 0;
								}
							}
							else
								d_counter = 0;
						}
						
						// if case samples processed copy known frame to output
						if(d_free_search_len == 0)
						{
							d_counter = 0;
							d_REFEREN_STATE = COPY_KNOWN;
						}
						
						break;		 
					}
					
					//_______________________
					case COLLECT_FRAME_FREE:
					
						buffer = d_input_collect - d_counter;
						buffer = std::min(n_input-consumed, buffer);
						
						// copy stream-samples to buffer
						memcpy(&d_frame_in0[d_counter], &in0[consumed], buffer*sizeof(gr_complex));
						memcpy(&d_frame_in1[d_counter], &in1[consumed], buffer*sizeof(gr_complex));

						d_counter += buffer;
						consumed += buffer;
						
						if(d_counter == d_input_collect)
						{
							cfo_fine_sync();
							
							// calculate offset between found start and known start
							uint64_t delay;
							delay = (d_frame_start > d_known_start) ? d_frame_start - d_known_start : d_known_start - d_frame_start;
														
							// check if frame lies within boundaries
							if(delay <= d_found_unsecu && d_free_lock == false)
							{
								d_REFEREN_STATE = COPY_DATA_FREE;
								d_free_lock = true;
							}
							else
								d_REFEREN_STATE = CHECK_RESIDUAL_FREE;	

							d_counter = 0;							
						}
							
						break;

					//_______________________
					case COPY_DATA_FREE:
					
						buffer = d_samp_frame - d_counter;
						buffer = std::min(buffer, noutput_items-produced);
						
						if(d_counter == 0)
							add_receiver_tag(produced, VALVE_FRAME_TYPE_FOUND);

						// fill output, out1 has to be delayed
						memcpy(&out0[produced], &d_frame_in1[d_peak_index+d_counter], buffer*sizeof(gr_complex));
						memcpy(&out1[produced], &d_frame_in1[d_peak_index+d_counter+d_veclength/2], buffer*sizeof(gr_complex));

						d_counter += buffer;
						produced  += buffer;
						
						if(d_counter == d_samp_frame)
						{
							d_counter = 0;
							d_REFEREN_STATE = CHECK_RESIDUAL_FREE;
						}
						
						break;	
				
					//_______________________
					case CHECK_RESIDUAL_FREE:
					{
						// skip preamble
						buffer = d_peak_index;		 // search through payload as well
						buffer = 100*d_veclength/64; // width of sync peak at 0.2
						
						// check if case samples already processed
						d_free_search_len -= buffer;						
						if(d_free_search_len <= 0)
						{
							d_REFEREN_STATE = COPY_KNOWN;
							break;
						}
						
						// bound input for free search
						int max_samples = std::min(d_free_search_len, d_input_collect-buffer);					
						
						// calculate magnitude to search for second frame in buffered input
						size_t alig = volk_get_alignment();
						float *temp = (float*) volk_malloc(max_samples*sizeof(float), alig);
						volk_32fc_magnitude_32f(temp, &d_frame_in0[buffer], max_samples);

						for(int i=0; i<max_samples; i++)
						{
							d_free_search_len--;
							
							if(temp[i] > d_threshold)
							{
								d_counter++;
								if(d_counter == d_plateau)
								{									
									d_counter = 0;
												
									// Check if possible frame lies within reasonable area.
									uint64_t cut_start_tmp = d_cut_start + buffer + i;
									uint64_t max_start = d_known_start + d_preamble_len + FRAME_SPACER_LENGTH_REDUCE*d_veclength + d_veclength;
									if(cut_start_tmp <= max_start)
									{					
										d_REFEREN_STATE = COPY_RESIDUAL_FREE;
										buffer += i;
										volk_free(temp);
										temp = NULL;
										break;
									}									
								}
							}
							else
								d_counter = 0;
						}
						
						volk_free(temp);					
						
						if(d_REFEREN_STATE == CHECK_RESIDUAL_FREE)
							d_REFEREN_STATE = COARSE_SYNC_FREE;
							
						break;
					}
					
					//_______________________
					case COPY_RESIDUAL_FREE:
					{
						d_counter = d_input_collect - buffer;
						
#ifdef CFO_CORRECTION
						// restore buffered samples
						gr_complex phase_incre = exp(gr_complex(0, -d_cfo_angular));
						gr_complex phase_start = exp(gr_complex(0, -d_cfo_angular*d_counter));
						volk_32fc_s32fc_x2_rotator_32fc(&d_frame_in1[d_counter], &d_frame_in1[d_counter], phase_incre, &phase_start, d_input_collect-d_counter);
#endif

						d_cut_start += buffer;						
						
						// copy stream-samples to buffer
						memmove(d_frame_in0, &d_frame_in0[buffer], d_counter*sizeof(gr_complex));
						memmove(d_frame_in1, &d_frame_in1[buffer], d_counter*sizeof(gr_complex));

						d_REFEREN_STATE = COLLECT_FRAME_FREE;
						
						break;
					}
					
					//_______________________
					case COPY_KNOWN:
					{											
						// security mechanism
						if(d_known_frame_cnt != d_samp_frame + d_veclength/2)
						{
							PRINT("valve_td error: Less samples than needed collected for known frame. Copying nonsense to output.");
							std::cout << "Collected: " << d_known_frame_cnt << "    needed: " << d_samp_frame + d_veclength/2 << std::endl;
						}
						
						buffer = d_samp_frame - d_counter;
						buffer = std::min(buffer, noutput_items-produced);
						
						if(d_counter == 0)
						{
							d_frame_start = d_known_start;
							add_receiver_tag(produced, VALVE_FRAME_TYPE_KNOWN);
						}
							
						// correct cfo as if was calculated for the reference frame
						// TODO

						// fill output, out1 has to be delayed
						memcpy(&out0[produced], &d_known_frame[d_counter], buffer*sizeof(gr_complex));
						memcpy(&out1[produced], &d_known_frame[d_counter+d_veclength/2], buffer*sizeof(gr_complex));

						d_counter += buffer;
						produced  += buffer;
						
						if(d_counter == d_samp_frame)
						{
							d_counter = 0;
							d_REFEREN_STATE = COARSE_SYNC_REF;
							d_known_frame_cnt = 0;
							d_known_start = 0;
						}
						
						break;
					}						
				}
								
				// collect samples of known frame (it will be copied later automatically in the last state)					
				if(d_known_frame_cnt < d_samp_frame + d_veclength/2)
				{
					uint64_t start = nitems_read(0);
					uint64_t end = nitems_read(0) + n_input;					
					uint64_t curr_index = d_known_start + d_known_frame_cnt;

					if(curr_index >= start && curr_index < end)
					{
						int new_samples = end - curr_index;	
						int new_samples_index = curr_index - start;			
									
						buffer = std::min(d_samp_frame + d_veclength/2 - d_known_frame_cnt, new_samples);	
												
						memcpy(&d_known_frame[d_known_frame_cnt], &in1[new_samples_index], buffer*sizeof(gr_complex));
						
						d_known_frame_cnt += buffer;							
					}
				}												
			}
			// ################################################
			// ################################################
			// ################################################				
		}

		consume(0, consumed);
		consume(1, consumed);

        return produced;
    }

  } /* namespace fbmc1 */
} /* namespace gr */

