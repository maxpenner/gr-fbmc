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
#include "fbmc_sample_collector_impl.h"

#include <volk/volk.h>

namespace gr {
  namespace fbmc1 {

	//______________________________________________________
	chunk::chunk()
	{
		size_t alig = volk_get_alignment();
		data = (gr_complex*) volk_malloc(chunk_len*sizeof(gr_complex), alig);

		memset(data, 0x00, chunk_len*sizeof(gr_complex));

		high_time += (uint64_t) chunk_len;
	}

	//______________________________________________________
	chunk::~chunk()
	{
		volk_free(data);
	}

	//______________________________________________________
	void chunk::chunk_release()
	{
		low_time += (uint64_t) chunk_len;
	}

	//______________________________________________________
	int chunk::chunk_len = 0;
	uint64_t chunk::low_time = 0;
	uint64_t chunk::high_time = 0;

	//______________________________________________________
    fbmc_sample_collector::sptr
    fbmc_sample_collector::make(int mode, int sensor_chunk_len, int sensor_pre_push, int sensor_skip)
    {
      return gnuradio::get_initial_sptr
        (new fbmc_sample_collector_impl(mode, sensor_chunk_len, sensor_pre_push, sensor_skip));
    }

    /*
     * The private constructor
     */
    fbmc_sample_collector_impl::fbmc_sample_collector_impl(int mode, int sensor_chunk_len, int sensor_pre_push, int sensor_skip)
      : gr::sync_block("fbmc_sample_collector",
              gr::io_signature::make(0, 0, 0),
              gr::io_signature::make(1, 1, sizeof(gr_complex)))
    {
		// init variables
		if(mode == 0)
		{
			d_STATE = VIDEO_ZEROS_BURST_START;
			message_port_register_in(pmt::mp("scl in"));
		}
		else if(mode == 1)
		{
			d_STATE = SENSOR_PRE_PUSH;
			message_port_register_in(pmt::mp("scl in"));
			set_msg_handler(pmt::mp("scl in"), boost::bind(&fbmc_sample_collector_impl::check_input_sensor, this, _1));
		}

		d_counter			= 0;
		d_video_frame		= NULL;
		d_video_frame_len	= 0;
		d_sensor_chunk		= NULL;
		d_sensor_chunk_len 	= sensor_chunk_len;
		d_sensor_pre_push 	= sensor_pre_push;
		d_sensor_skip		= sensor_skip;
		d_sensor_chunk_time = 0;

		chunk::chunk_len = sensor_chunk_len;

#ifdef SAMPLE_COLLECTOR_PRINT_RATE
		d_sample_coll_timer = new fbmc_timer(SAMPLE_COLLECTOR_PRINT_RATE);
		d_frames_found = 0;
		d_frames_send = 0;
#endif

		// init VOLK
		const int alignment_multiple = volk_get_alignment() / sizeof(gr_complex);
		set_alignment(std::max(1, alignment_multiple));
	}

    /*
     * Our virtual destructor.
     */
    fbmc_sample_collector_impl::~fbmc_sample_collector_impl()
    {
		volk_free(d_video_frame);

		delete d_sensor_chunk;

		for(int i=0; i<d_sensor_chunk_vec.size(); i++)
			delete d_sensor_chunk_vec[i];

#ifdef SAMPLE_COLLECTOR_PRINT_RATE
		delete d_sample_coll_timer;
#endif

    }

    //______________________________________________________
    bool fbmc_sample_collector_impl::check_input_video()
    {
		pmt::pmt_t msg(delete_head_nowait(pmt::intern("scl in")));
		if (msg.get() == NULL)
			return false;

		const gr_complex *FRAME_blob;
		size_t FRAME_blob_len;

		// input blob extraction
		if(pmt::is_pair(msg))
		{
			// cdr = blob
			FRAME_blob = reinterpret_cast<const gr_complex *>(pmt::blob_data(pmt::cdr(msg)));
			FRAME_blob_len = pmt::blob_length(pmt::cdr(msg));
			FRAME_blob_len /= sizeof(gr_complex);
		}
		else
		{
			throw std::invalid_argument("fbmc_sample_collector_impl error: Mode Video: Expect PDUs as input. Dropping data.");
			return false;
		}

		// make local copy of frame
		size_t alig 		= volk_get_alignment();
		d_video_frame 		= (gr_complex*) volk_malloc(FRAME_blob_len*sizeof(gr_complex), alig);
		d_video_frame_len 	= FRAME_blob_len;
		d_counter 			= 0;
		memcpy(d_video_frame, FRAME_blob, FRAME_blob_len*sizeof(gr_complex));

		return true;
    }

    //______________________________________________________
    void fbmc_sample_collector_impl::check_input_sensor(pmt::pmt_t msg)
    {
		const gr_complex *FRAME_blob;
		size_t FRAME_blob_len;
		uint64_t airtime;

		// input blob extraction
		if(pmt::is_pair(msg))
		{
			// cdr = blob
			FRAME_blob = reinterpret_cast<const gr_complex *>(pmt::blob_data(pmt::cdr(msg)));
			FRAME_blob_len = pmt::blob_length(pmt::cdr(msg));
			FRAME_blob_len /= sizeof(gr_complex);

			// car == dictionary = keys + values
			airtime = pmt::to_uint64(pmt::dict_ref(pmt::car(msg), pmt::mp("airtime"), pmt::PMT_NIL));
		}
		else
		{
			throw std::invalid_argument("fbmc_sample_collector_impl error: Mode Sensor: Expect PDUs as input. Dropping data.");
			return;
		}

#ifdef SAMPLE_COLLECTOR_PRINT_RATE
		d_frames_found++;
#endif

		// first check if new frame is not too late
		if(airtime < chunk::low_time)
		{
			PRINT("fbmc_sample_collector_impl error: Airtime of frame smaller than actual time. Dropping frame.");
			PRINT(chunk::low_time);
			PRINT(airtime);
			PRINT(chunk::low_time - airtime);
			return;
		}

#ifdef SAMPLE_COLLECTOR_PRINT_RATE
		d_frames_send++;
#endif

		// check difference between airtime and chunk::high_time
		int new_samples_needed = 0;
		uint64_t max_time = airtime + (uint64_t) FRAME_blob_len;
		if(max_time > chunk::high_time)
			new_samples_needed = max_time - chunk::high_time;

		// debug mechanism
		if(new_samples_needed > d_sensor_pre_push*d_sensor_chunk_len)
		{
			PRINT("fbmc_sample_collector_impl error: Too many new samples needed.");
			PRINT(new_samples_needed);
			PRINT(d_sensor_pre_push*d_sensor_chunk_len);
		}

		// check if new chunks have to be appended
		if(new_samples_needed > 0)
		{
			int new_chunks_needed = (new_samples_needed + chunk::chunk_len - 1) / chunk::chunk_len;

			for(int i=0; i<new_chunks_needed; i++)
				d_sensor_chunk_vec.push_back(new chunk());
		}

		// outer and inner index
		uint64_t diff = airtime - chunk::low_time;
		int chunk_o_idx = ((int) diff) / chunk::chunk_len;
		int chunk_i_idx = airtime % chunk::chunk_len;
		int cnt = 0;

		// superimpose new frame and chunks
		while(cnt != FRAME_blob_len)
		{
			int buffer = FRAME_blob_len - cnt;
			buffer = std::min(chunk::chunk_len - chunk_i_idx, buffer);

			gr_complex *target = (gr_complex*) &d_sensor_chunk_vec[chunk_o_idx]->data[chunk_i_idx];

			// misuse of volk float addition for complex
			volk_32f_x2_add_32f((float*) target, (const float*) target, (const float*) &FRAME_blob[cnt], 2*buffer);

			cnt += buffer;
			chunk_o_idx++;
			chunk_i_idx = 0;
		}
    }

#ifdef SAMPLE_COLLECTOR_PRINT_RATE
    //______________________________________________________
    void fbmc_sample_collector_impl::display()
    {
		if(d_sample_coll_timer->new_update())
		{
			std::cout << std::endl 	<< std::endl
			<< "------------- SAMPLE COLLECTOR -----------" << std::endl
			<< "frames found: " << d_frames_found << std::endl
			<< "frames sent:  " << d_frames_send << std::endl
			<< "------------------------------------------" << std::endl;
		}
    }
#endif

	//______________________________________________________
    int
    fbmc_sample_collector_impl::work(int noutput_items,
			  gr_vector_const_void_star &input_items,
			  gr_vector_void_star &output_items)
    {
		gr_complex *out0 = (gr_complex *) output_items[0];

		int produced  = 0;
		bool token = false;

		while(produced < noutput_items && token == false)
		{
			switch(d_STATE)
			{
				//_______________________
				case VIDEO_ZEROS_BURST_START:
				{
					int buffer = TX_DELAY_ADD_ZEROS_BURST - d_counter;
					buffer = std::min(noutput_items-produced, buffer);
					
					// we start a burst with zeros, so set a tag
					if(d_counter == 0)
					{
						static const pmt::pmt_t sob_key = pmt::string_to_symbol("tx_sob");
						static const pmt::pmt_t value = pmt::PMT_T;
						static const pmt::pmt_t srcid = pmt::string_to_symbol(alias());
						add_item_tag(0, nitems_written(0)+produced, sob_key, value, srcid);
						
						// save time when burst started
						d_start_time = std::chrono::high_resolution_clock::now();
					}					

					memset(&out0[produced], 0x00, buffer*sizeof(gr_complex));				

					produced += buffer;
					d_counter += buffer;
					
					if(d_counter == TX_DELAY_ADD_ZEROS_BURST)
					{
						d_STATE = VIDEO_INPUT;
						d_counter = 0;						
					}

					break;
				}				
				
				//_______________________
				case VIDEO_INPUT:
				
					// are we even using transmission delaying?
					if(TX_DELAY_TRANSMITTING_MS > 0)
					{
						std::chrono::time_point<std::chrono::system_clock> now_ms = std::chrono::high_resolution_clock::now();
						int64_t duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now_ms - d_start_time).count();
						
						// determine if enough time has passed
						if (duration_ms >= TX_DELAY_TRANSMITTING_MS)
						{
							
#ifdef TX_DELAY_DEBUG
							PRINT("STARTING BLOCKING");
#endif							
							
							d_start_time = now_ms;
							
							// the burst has to end, so append zeros at the end
							d_STATE = VIDEO_ZEROS_BURST_END;
							d_counter = 0;
							
							// leave the switch statement
							break;
						}
					}
					
					d_STATE = (check_input_video() == true) ? VIDEO_COPY : VIDEO_RESIDUAL;

#ifdef SAMPLE_COLLECTOR_PRINT_RATE
					display();
#endif

					break;

				//_______________________
				case VIDEO_COPY:
				{
					int buffer = d_video_frame_len - d_counter;
					buffer = std::min(noutput_items-produced, buffer);

					memcpy(&out0[produced], &d_video_frame[d_counter], buffer*sizeof(gr_complex));					

					d_counter += buffer;
					produced  += buffer;

					if(d_counter == d_video_frame_len)
					{
						volk_free(d_video_frame);
						d_video_frame = NULL;
						d_STATE = VIDEO_INPUT;

#ifdef SAMPLE_COLLECTOR_PRINT_RATE
		d_frames_send++;
#endif

					}

					break;
				}

				//_______________________
				case VIDEO_RESIDUAL:
				{
					int buffer = noutput_items - produced;

					memset(&out0[produced], 0x00, buffer*sizeof(gr_complex));

					produced += buffer;

					d_STATE = VIDEO_INPUT;

					break;
				}
				
				//_______________________
				case VIDEO_ZEROS_BURST_END:
				{
					int buffer = TX_DELAY_ADD_ZEROS_BURST - d_counter;
					buffer = std::min(noutput_items-produced, buffer);

					memset(&out0[produced], 0x00, buffer*sizeof(gr_complex));


					produced += buffer;
					d_counter += buffer;
					
					if(d_counter == TX_DELAY_ADD_ZEROS_BURST)
					{
						
#ifdef TX_DELAY_DEBUG
						PRINT("ENDING BLOCKING");
#endif							
						
						d_STATE = VIDEO_BLOCK;
						d_counter = 0;
						
						static const pmt::pmt_t eob_key = pmt::string_to_symbol("tx_eob");
						static const pmt::pmt_t value = pmt::PMT_T;
						static const pmt::pmt_t srcid = pmt::string_to_symbol(alias());
						add_item_tag(0, nitems_written(0)+produced, eob_key, value, srcid);	
					}

					break;
				}
				
				//_______________________
				case VIDEO_BLOCK:
				{					
					// put this thread to sleep
					boost::this_thread::sleep(boost::posix_time::milliseconds(TX_DELAY_THREAD_BLOCK_MS));
								
					std::chrono::time_point<std::chrono::system_clock> now_ms = std::chrono::high_resolution_clock::now();
					int64_t duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now_ms - d_start_time).count();
					
					// determine if enough time has passed to restart transmission
					if (duration_ms >= TX_DELAY_BLOCKING_MS)
					{
						d_start_time = now_ms;
						
						// the burst has to end, so append zeros at the end
						d_STATE = VIDEO_ZEROS_BURST_START;
						d_counter = 0;
					}
					
					break;
				}				

				//_______________________
				case SENSOR_PRE_PUSH:
				{
					int buffer = d_sensor_skip + d_sensor_pre_push*d_sensor_chunk_len - d_counter;
					buffer = std::min(noutput_items-produced, buffer);

					memset(&out0[produced], 0x00, buffer*sizeof(gr_complex));

					d_counter += buffer;
					produced  += buffer;

					if(d_counter == d_sensor_skip + d_sensor_pre_push*d_sensor_chunk_len)
					{
						d_counter = 0;
						d_STATE = SENSOR_CHOOSE;
						token = true;
					}

					break;
				}

				//_______________________
				case SENSOR_CHOOSE:
				{
					// debug mechanism 1
					if(nitems_written(0)-chunk::low_time > d_sensor_skip + d_sensor_pre_push*d_sensor_chunk_len)
					{
						PRINT("fbmc_sample_collector_impl warning: Time difference too big.");
						PRINT(chunk::low_time);
						PRINT(nitems_written(0));
						PRINT(nitems_written(0)-chunk::low_time);
					}

					// debug mechanism 2
					if(chunk::high_time - chunk::low_time > d_sensor_pre_push*d_sensor_chunk_len)
					{
						PRINT("fbmc_sample_collector_impl warning: Internal time difference too big.");
						PRINT(chunk::high_time);
						PRINT(chunk::low_time);
						PRINT(chunk::high_time - chunk::low_time);
					}

					// debug mechanism 3
					if(d_sensor_chunk_vec.size() > d_sensor_pre_push)
					{
						PRINT("fbmc_sample_collector_impl warning: Too much pre push.");
						PRINT(d_sensor_chunk_vec.size());
						PRINT(d_sensor_pre_push);
					}

					// if no chunks queued create one empty chunk
					if(d_sensor_chunk_vec.size() == 0)
					{
						// debug mechanism 4
						if(empty_p() == false)
						{
							PRINT("fbmc_sample_collector_impl warning: Appending empty chunk, but queue is not empty.");
						}

						d_sensor_chunk_vec.push_back(new chunk());
					}

					// prepare and seperate first chunk
					d_sensor_chunk_vec[0]->chunk_release();
					d_sensor_chunk = d_sensor_chunk_vec[0];
					d_sensor_chunk_vec.erase(d_sensor_chunk_vec.begin());

					d_counter = 0;
					d_STATE = SENSOR_COPY;

#ifdef SAMPLE_COLLECTOR_PRINT_RATE
					display();
#endif

					break;
				}

				//_______________________
				case SENSOR_COPY:
				{
					// copy chunk to output
					int buffer = d_sensor_chunk_len - d_counter;
					buffer = std::min(noutput_items-produced, buffer);

					memcpy(&out0[produced], &d_sensor_chunk->data[d_counter], buffer*sizeof(gr_complex));

					d_counter += buffer;
					produced  += buffer;

					if(d_counter == d_sensor_chunk_len)
					{
						delete d_sensor_chunk;
						d_sensor_chunk = NULL;
						d_STATE = SENSOR_CHOOSE;
					}

					break;
				}
	    	}
		}

        return produced;
    }

  } /* namespace fbmc1 */
} /* namespace gr */
