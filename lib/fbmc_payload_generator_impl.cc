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
#include "fbmc_payload_generator_impl.h"

#include "utils/debug.h"
#include "utils/encodings.h"
#include "sensor/scheduler.h"

namespace gr {
  namespace fbmc1 {

    fbmc_payload_generator::sptr
    fbmc_payload_generator::make(int mode, int symbols_per_frame, std::vector<int> logical_channels, int video_encoding, int sensor_chunk_len, int sensor_skip)
    {
      return gnuradio::get_initial_sptr
        (new fbmc_payload_generator_impl(mode, symbols_per_frame, logical_channels, video_encoding, sensor_chunk_len, sensor_skip));
    }

    /*
     * The private constructor
     */
    fbmc_payload_generator_impl::fbmc_payload_generator_impl(int mode, int symbols_per_frame, std::vector<int> logical_channels, int video_encoding, int sensor_chunk_len, int sensor_skip)
      : gr::sync_block("fbmc_payload_generator",
              gr::io_signature::make(1, 1, sizeof(gr_complex)),
              gr::io_signature::make(0,0,0))
    {
		// init variables
		if(mode == 0)
		{
			d_MODE = VIDEO;
		}
		else if(mode == 1)
		{
			d_MODE = SENSOR;
			FBMC_SENSOR_SCHEDU::init_sensor_module(symbols_per_frame, logical_channels);
		}
			
		d_spf 				= symbols_per_frame;
		d_logical_channels 	= logical_channels;
		d_video_encoding 	= video_encoding;
		d_channel2use		= 0;
		d_sensor_chunk_len 	= sensor_chunk_len;
		d_sensor_skip 		= sensor_skip;
		d_old_time			= 0;
		
		// input ports
		message_port_register_in(pmt::mp("control in"));
		set_msg_handler(pmt::mp("control in"), boost::bind(&fbmc_payload_generator_impl::control_in, this, _1));
		message_port_register_in(pmt::mp("app in"));
		set_msg_handler(pmt::mp("app in"), boost::bind(&fbmc_payload_generator_impl::app_in, this, _1));
		message_port_register_in(pmt::mp("ch state in"));
		set_msg_handler(pmt::mp("ch state in"), boost::bind(&fbmc_payload_generator_impl::ch_state_in, this, _1));		

		// output ports
		message_port_register_out(pmt::mp("control out"));
		message_port_register_out(pmt::mp("app out"));
	}

    /*
     * Our virtual destructor.
     */
    fbmc_payload_generator_impl::~fbmc_payload_generator_impl()
    {
		// clean up sensor module
		if(d_MODE == SENSOR);
			FBMC_SENSOR_SCHEDU::clean_sensor_module();
    }
    
    //______________________________________________________
    void fbmc_payload_generator_impl::control_in(pmt::pmt_t msg) 
    {
		const char *CTRL_blob;
		size_t CTRL_blob_len;

		// input blob extraction
		if(pmt::is_pair(msg))
		{
			CTRL_blob = reinterpret_cast<const char *>(pmt::blob_data(pmt::cdr(msg)));
			CTRL_blob_len = pmt::blob_length(pmt::cdr(msg));
		}
		else
		{
			throw std::invalid_argument("fbmc_payload_generator_impl error: Expect PDUs at control input port. Dropping data.");
			return;
		}
		
		// security mechanism
		if(CTRL_blob_len <= 0)
		{
			PRINT("fbmc_payload_generator_impl error: Zero input for CRTL blob.");
			return;
		}
		
		// feed sensor module with new control bytes
		FBMC_SENSOR_SCHEDU::feed_control_data(CTRL_blob, CTRL_blob_len);
	}

    //______________________________________________________
    void fbmc_payload_generator_impl::app_in(pmt::pmt_t msg) 
    {
		const char *APP_blob;
		size_t APP_blob_len;

		// input blob extraction
		if(pmt::is_pair(msg))
		{
			APP_blob = reinterpret_cast<const char *>(pmt::blob_data(pmt::cdr(msg)));
			APP_blob_len = pmt::blob_length(pmt::cdr(msg));
		}
		else
		{
			throw std::invalid_argument("fbmc_payload_generator_impl error: Expect PDUs. Dropping data.");
			return;
		}
		
		// security mechanism
		if(APP_blob_len <= 0)
		{
			PRINT("fbmc_payload_generator_impl error: Zero input for APP blob.");
			return;
		}
		
		// process input blob
		mode_video(APP_blob, APP_blob_len);
	}
	
    //______________________________________________________
    void fbmc_payload_generator_impl::ch_state_in(pmt::pmt_t msg) 
    {
		if(pmt::dict_has_key(pmt::car(msg), pmt::mp("channel2use")))
		{
			long channel2use_temp = pmt::to_long(pmt::dict_ref(pmt::car(msg), pmt::mp("channel2use"), pmt::pmt_t()));
			
			// change the channel that is currently used
			d_channel2use = (int) channel2use_temp;
		}
		else
		{
			throw std::invalid_argument("fbmc_payload_generator_impl error: Received a message without the channel2use-key-value pair.");
		}	
	}	
	
	//______________________________________________________
    void fbmc_payload_generator_impl::mode_video(const char *APP_blob, size_t APP_blob_len)
    {	
		// set payload information
		metadataPayload mdp;
		mdp.payload_len = APP_blob_len;
		mdp.log_chann 	= d_channel2use;
		mdp.spf			= d_spf;
		mdp.encoding 	= d_video_encoding;
		mdp.sender 		= 0;
		mdp.seq_no 		= 0;
		mdp.airtime 	= 0;
		mdp.scale 		= 1.0;
		
		// create new blob
		char MAC_blob[sizeof(metadataPayload) + mdp.payload_len];
		memcpy(MAC_blob, &mdp, sizeof(metadataPayload));
		memcpy(MAC_blob + sizeof(metadataPayload), APP_blob, mdp.payload_len);

		publish_blob(MAC_blob, sizeof(metadataPayload) + mdp.payload_len);
	}
	
	//______________________________________________________
    void fbmc_payload_generator_impl::mode_sensor(uint64_t latest_time)
    {	
		FBMC_SENSOR_SCHEDU::update_time(latest_time);
		
		// depending on state of sensor module, perform a certain action
		switch(FBMC_SENSOR_SCHEDU::get_sensor_module_state())
		{
			case FBMC_SENSOR_SCHEDU::SENSOR_IDLE:
				break;
				
			case FBMC_SENSOR_SCHEDU::SENSOR_SEND_CONTROL_MESSAGE:
			{
				char *out_control_stream = NULL;
				int out_control_stream_length = 0;
				
				FBMC_SENSOR_SCHEDU::read_control_data(out_control_stream, out_control_stream_length);
				
				if(out_control_stream_length > 0)
				{
					// publish message
					pmt::pmt_t CONTROL_MSG_blob = pmt::make_blob(out_control_stream, out_control_stream_length);
					message_port_pub(pmt::mp("control out"), pmt::cons(pmt::PMT_NIL, CONTROL_MSG_blob));
				}
				
				free(out_control_stream);
				
				break;
			}
				
			case FBMC_SENSOR_SCHEDU::SENSOR_GENERATE_PAYLOADS:
			{
				int n_payloads;
				int n_payload_bytes;
				
				FBMC_SENSOR_SCHEDU::get_max_new_payloads(n_payloads, n_payload_bytes);
				
				if(n_payloads == 0)
					break;
				
				// allocate memory for new payloads
				char MAC_blob[n_payloads*sizeof(metadataPayload) + n_payload_bytes];
				
				int MAC_blob_index = 0;
				
				// create payloads
				for(int i=0; i<n_payloads; i++)
				{
					metadataPayload mdp;
					mdp.spf = d_spf;
					
					// get payload information (stop if module time reached)
					if(FBMC_SENSOR_SCHEDU::get_payload_param(&mdp) == false)
						break;
					
					// copy meta data
					memcpy(MAC_blob + MAC_blob_index, &mdp, sizeof(metadataPayload));
					MAC_blob_index += sizeof(metadataPayload);
					
					// insert payload
					FBMC_SENSOR_SCHEDU::generate_payload(MAC_blob + MAC_blob_index);
					
					MAC_blob_index += mdp.payload_len;
				}
				
				publish_blob(MAC_blob, MAC_blob_index);
				
				break;
			}
		}
	}
		
	//______________________________________________________
    void fbmc_payload_generator_impl::publish_blob(const char *MAC_blob, size_t MAC_blob_len)
    {	
		// copy metadata for logical channels
		pmt::pmt_t PMT_log_chann = pmt::make_s32vector(d_logical_channels.size(), 0);
		size_t temp;
		int32_t *elements = pmt::s32vector_writable_elements(PMT_log_chann, temp);
		memcpy(elements, &d_logical_channels[0], d_logical_channels.size()*sizeof(int32_t));
		
		// publish blob
		pmt::pmt_t dict = pmt::make_dict();
		dict = pmt::dict_add(dict, pmt::mp("logical_channels"), PMT_log_chann);
		pmt::pmt_t PHY_MAC_blob = pmt::make_blob(MAC_blob, MAC_blob_len);
		message_port_pub(pmt::mp("app out"), pmt::cons(dict, PHY_MAC_blob));
	}

	//______________________________________________________
    int
    fbmc_payload_generator_impl::work(int noutput_items,
			  gr_vector_const_void_star &input_items,
			  gr_vector_void_star &output_items)
    {
		if(d_MODE == SENSOR)
		{
			uint64_t latest_time = nitems_read(0) + noutput_items;
			
			// check if enough samples skipped
			if(latest_time >= d_sensor_skip)
			{
				latest_time -= d_sensor_skip;
			
				int new_chunks = (latest_time - d_old_time)/d_sensor_chunk_len;
				
				// if new chunks arrived call sensor scheduler
				if(new_chunks > 0)
				{	
					d_old_time += new_chunks*d_sensor_chunk_len;
					mode_sensor(d_old_time);
				}
			}
		}

		consume_each(noutput_items);   
		
        return 0;
    }

  } /* namespace fbmc1 */
} /* namespace gr */

