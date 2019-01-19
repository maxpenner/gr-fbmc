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
#include "fbmc_sensor_impl.h"

#include <boost/crc.hpp>

#include "utils/debug.h"
#include "phy_coding/decoder.h"
#include "mac_coding/decoder.h"
#include "sensor/analyser.h"
#include "utils/receiver_tag.h"

namespace gr {
  namespace fbmc1 {

    fbmc_sensor::sptr
    fbmc_sensor::make(int veclength, int symbols_per_frame, std::vector<int> logical_channels)
    {
      return gnuradio::get_initial_sptr
        (new fbmc_sensor_impl(veclength, symbols_per_frame, logical_channels));
    }

    /*
     * The private constructor
     */
    fbmc_sensor_impl::fbmc_sensor_impl(int veclength, int symbols_per_frame, std::vector<int> logical_channels)
      : gr::sync_block("fbmc_sensor",
              gr::io_signature::make(1, 1, sizeof(gr_complex)*veclength),
              gr::io_signature::make(0,0,0))
    {
		// init variables
		d_STATE	    	 = DATA_ACQUISITION;
		d_veclength 	 = veclength; 
		d_spf	    	 = symbols_per_frame;
		d_receiver_tag   = 0;
		d_log_chann 	 = 0;
		d_samples_index	 = 0;
		d_counter   	 = 0;
		
		// init logical channels
		logical_channel_convert(&logical_channels[0], logical_channels.size(), d_log_channs);
		
		// find widest logical channel
		logical_channels_widths(&logical_channels[0], logical_channels.size(), d_widths);
		int widest_channel = *max_element(d_widths.begin(), d_widths.end());

		// init samples container
		d_samples = (gr_complex*) malloc(widest_channel*d_spf*sizeof(gr_complex));
		
		FBMC_SENSOR_ANALYSER::init_sensor_module();
		ref_frame_last_start = 0;

		// input ports
		message_port_register_in(pmt::mp("valve in"));
		set_msg_handler(pmt::mp("valve in"), boost::bind(&fbmc_sensor_impl::valve_in, this, _1));
		message_port_register_in(pmt::mp("control in"));
		set_msg_handler(pmt::mp("control in"), boost::bind(&fbmc_sensor_impl::control_in, this, _1));

		// output port
		message_port_register_out(pmt::mp("valve out"));
		message_port_register_out(pmt::mp("phy out"));
		message_port_register_out(pmt::mp("control out"));
	}

    /*
     * Our virtual destructor.
     */
    fbmc_sensor_impl::~fbmc_sensor_impl()
    {
		free(d_samples);
		
		FBMC_SENSOR_ANALYSER::clean_sensor_module();
    }
    
    //______________________________________________________
    void fbmc_sensor_impl::send_control_msg_2_transmitter()
    {
		// valve is put into it's mode or reset -> send msg2send back to transmitter
		char *out_control_stream = NULL;
		int out_control_stream_length = 0;
		
		FBMC_SENSOR_ANALYSER::read_control_data(out_control_stream, out_control_stream_length);
			
		if(out_control_stream_length > 0)
		{
			// publish message
			pmt::pmt_t CONTROL_MSG_blob = pmt::make_blob(out_control_stream, out_control_stream_length);
			message_port_pub(pmt::mp("control out"), pmt::cons(pmt::PMT_NIL, CONTROL_MSG_blob));
		}
		
		free(out_control_stream);
	}    
    
    //______________________________________________________
    void fbmc_sensor_impl::send_valve_msg()
    {
		pmt::pmt_t dict = pmt::make_dict();
		
		switch(mode_analyser)
		{			
			case MODE_SENSORS_FREERUN:
			{
				pmt::pmt_t PMT_NO_case_samples = pmt::from_long(0);
				dict = pmt::dict_add(dict, pmt::mp("NO_case_samples"), PMT_NO_case_samples);
				break;
			}
				
			case MODE_SENSORS_REFEREN:
			{
				pmt::pmt_t PMT_case_samples = pmt::from_long(mode_ref_data.case_samples);
				pmt::pmt_t PMT_known_offset = pmt::from_long(mode_ref_data.known_offset);
				pmt::pmt_t PMT_found_unsecu = pmt::from_long(mode_ref_data.found_unsecu);
				pmt::pmt_t PMT_refer_thresh = pmt::from_double((double) mode_ref_data.refer_thresh);
				dict = pmt::dict_add(dict, pmt::mp("case_samples"), PMT_case_samples);
				dict = pmt::dict_add(dict, pmt::mp("known_offset"), PMT_known_offset);
				dict = pmt::dict_add(dict, pmt::mp("found_unsecu"), PMT_found_unsecu);
				dict = pmt::dict_add(dict, pmt::mp("refer_thresh"), PMT_refer_thresh);
				break;
			}
		}	
			
		pmt::pmt_t PMT_dummy_blob = pmt::make_blob(nullptr, 0);
		message_port_pub(pmt::mp("valve out"), pmt::cons(dict, PMT_dummy_blob));
	}
   
    //______________________________________________________
    void fbmc_sensor_impl::valve_in(pmt::pmt_t msg) 
    {
		send_control_msg_2_transmitter();
		
		// valve is now reset -> next frame should be a reference frame
		d_REFERENCE_SUBMODE = EXPECT_REF_FRAME_INIT;
	} 
    
    //______________________________________________________
    void fbmc_sensor_impl::control_in(pmt::pmt_t msg) 
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
			throw std::invalid_argument("fbmc_sensor_impl error: Expect PDUs at control input port. Dropping data.");
			return;
		}
		
		// security mechanism
		if(CTRL_blob_len <= 0)
		{
			PRINT("fbmc_sensor_impl error: Zero input for CRTL blob.");
			return;
		}
		
		// feed sensor module with new control bytes
		FBMC_SENSOR_ANALYSER::feed_control_data(CTRL_blob, CTRL_blob_len);
		
		enum FBMC_SENSOR_ANALYSER::SENSOR_ANALYSER_ACTION action = FBMC_SENSOR_ANALYSER::get_sensor_module_state();
		
		// depending on state of sensor module, perform a certain action
		switch(action)
		{
			case FBMC_SENSOR_ANALYSER::ANALYSER_IDLE:
				break;
				
			case FBMC_SENSOR_ANALYSER::ANALYSER_SEND_CONTROL_MESSAGE:
			{
				send_control_msg_2_transmitter();
				break;
			}
			
			case FBMC_SENSOR_ANALYSER::ANALYSER_SEND_VALVE_MESSAGE:
			{
				send_valve_msg();
				break;
			}
		}	
	}
	
    //______________________________________________________
    bool fbmc_sensor_impl::decode_reference_frame() 
    {										
		rfs.recv_ref_frame++;

		// PHY-LAYER
		FBMC_PHY::load_decoder(d_samples_index, mode_ref_data.ref_enco, mode_ref_data.payload_len, d_widths[d_log_chann], FBMC_PHY_DECODER_DONT_CARE, FBMC_PHY_DECODER_DONT_CARE);

		int mpdu_len = 0;	

		if(FBMC_PHY::get_mpdu_len(mpdu_len, d_samples) != true)
			return false;
			
		char mpdu[mpdu_len];

		FBMC_PHY::extract_mpdu(mpdu, d_samples);

		// MAC-LAYER
		FBMC_MAC::load_decoder(mpdu_len, mode_ref_data.ref_enco);

		int payload_len = FBMC_MAC::get_payload_len(mpdu);

		if(payload_len == 0  || payload_len != mode_ref_data.payload_len)
			return false;

		// extract and set reference bytes
		FBMC_MAC::extract_payload(reference_bytes, mpdu);	
		
		return true;
	}	

	//______________________________________________________
    int
    fbmc_sensor_impl::work(int noutput_items,
			  gr_vector_const_void_star &input_items,
			  gr_vector_void_star &output_items)
    {
		const gr_complex *in = (const gr_complex *) input_items[0];

		int consumed = 0;

		while(consumed < noutput_items)
		{
			switch(d_STATE)
			{
				//______________________________________________________
				case DATA_ACQUISITION:
				{
					int buffer = d_spf - d_counter;
					buffer = std::min(noutput_items-consumed, buffer);

					// extract logical channel information
					if(d_counter == 0)
					{
						std::vector<gr::tag_t> tags;
						uint64_t offset = nitems_read(0) + consumed;
						get_tags_in_range(tags, 0, offset, offset+1, pmt::string_to_symbol("receiver_tag"));
						
						if(tags.size() == 0)
						{
							PRINT("fbmc_PHY_decoder_impl error: Tag for logical channel not found.");
						}
						else
						{
							d_receiver_tag = pmt::to_uint64(tags[0].value);
							d_log_chann = (int) (d_receiver_tag & RECEIVER_TAG_DOUBL_FRAM_OFFS);
						}
					}

					// collect frequency samples
					for(int i=0; i<buffer; i++)
					{
						int input_offset = (consumed+i)*d_veclength;
						
						for(int j=0; j<d_log_channs[d_log_chann].get_size(); j++)
							d_samples[d_samples_index++] = in[input_offset + d_log_channs[d_log_chann].subcarrier[j]];
					}
					
					d_counter += buffer;
					consumed += buffer;
					
					if(d_counter == d_spf)
					{	
						d_counter = 0;
						d_STATE = DECODE;
					}
					else
					{
						break;
					}
				}

				//______________________________________________________
				case DECODE:
				{	
					// Freerun vs. Reference
					switch(mode_analyser)
					{
						case MODE_SENSORS_FREERUN:
						{
							// N sensors: custom vs. non-custom coding
							switch(mode_fre_data.submode)
							{
								case FREERUN_N_SENSOR_CUSTOM:
								{
									// PHY-LAYER
									FBMC_PHY::load_decoder(d_samples_index, mode_fre_data.encoding, mode_fre_data.payload_len, d_widths[d_log_chann], mode_fre_data.hard_soft_decoding, mode_fre_data.interl_type);
						
									char mpdu[mode_fre_data.payload_len];

									FBMC_PHY::extract_mpdu(mpdu, d_samples);
									
									// MAC-LAYER
									boost::crc_32_type result;
									result.process_bytes(mpdu, mode_fre_data.payload_len);
									if(result.checksum() != 558161692)
										break;								

									// check if all bytes are the same
									FBMC_SENSOR_ANALYSER::add_sensor_data_freerun_mode(0, 0, 0, mpdu);
									
									break;
								}
									
								case FREERUN_N_SENSORS:
								{
									// PHY-LAYER
									FBMC_PHY::load_decoder(d_samples_index, mode_fre_data.encoding_family, -1, d_widths[d_log_chann], FBMC_PHY_DECODER_DONT_CARE, FBMC_PHY_DECODER_DONT_CARE);
									
									int mpdu_len = 0;
									
									if(FBMC_PHY::get_mpdu_len(mpdu_len, d_samples) != true)
										break;
									
									char mpdu[mpdu_len];
									
									FBMC_PHY::extract_mpdu(mpdu, d_samples);
									
									// MAC-LAYER
									FBMC_MAC::load_decoder(mpdu_len, mode_fre_data.encoding_family);
					
									int payload_len = FBMC_MAC::get_payload_len(mpdu);
									
									// check if frame correct
									if(payload_len == 0)
										break;
									
									char payload[payload_len];
									
									FBMC_MAC::extract_payload(payload, mpdu);
									
									// feed data to analyser
									uint16_t sender_id = FBMC_MAC::get_sender_id(mpdu);
									uint16_t seq_no = FBMC_MAC::get_seq_no(mpdu);
									
									FBMC_SENSOR_ANALYSER::add_sensor_data_freerun_mode(sender_id, seq_no, payload_len, NULL);
															
									break;
								}
							}
							
							break;	
						}
							
						case MODE_SENSORS_REFEREN:
						{
							uint64_t start_index = d_receiver_tag >> RECEIVER_TAG_SAMPL_NUMB_OFFS;
							int frame_type = (d_receiver_tag >> RECEIVER_TAG_FRAME_TYPE_OFFS) & 0x3;
							
							// expecting three different types of frames
							switch(d_REFERENCE_SUBMODE)
							{
								case EXPECT_REF_FRAME_INIT:
								{
									// first frame must (!) be a reference frame
									if(frame_type == VALVE_FRAME_TYPE_REFER)
									{
										// save time difference
										ref_frame_last_start = start_index;

										// try to decode frame
										if(decode_reference_frame() == true)
										{
											rfs.decoded_ref_frame++;											
										}
										else
										{																		
											rfs.reference_frame_undecodable = true;
											d_REFERENCE_SUBMODE = IDLE_DUE_TO_ERROR;
											break;
										}									
									}
									else
									{																			
										rfs.reference_frame_not_init = true;
										d_REFERENCE_SUBMODE = IDLE_DUE_TO_ERROR;
									}
									
									d_REFERENCE_SUBMODE = EXPECT_FOUND_FRAME;
									break;
								}
								
								case EXPECT_REF_FRAME:
								{
									// it must (!) be a reference frame
									if(frame_type == VALVE_FRAME_TYPE_REFER)
									{
										// check if time difference makes sense
										int diff = start_index - ref_frame_last_start;
										diff = abs(diff - mode_ref_data.ref2ref_samples);									
										if(diff > mode_ref_data.ref_delay_max)
										{
											rfs.reference_frame_delayed = true;
											d_REFERENCE_SUBMODE = IDLE_DUE_TO_ERROR;
											break;											
										}
										else
										{
											ref_frame_last_start = start_index;
										}

										// try to decode frame
										if(decode_reference_frame() == true)
										{
											rfs.decoded_ref_frame++;											
										}
										else
										{
											rfs.reference_frame_undecodable = true;
											d_REFERENCE_SUBMODE = IDLE_DUE_TO_ERROR;
											break;
										}									
									}
									else
									{
										rfs.reference_frame_missed = true;
										d_REFERENCE_SUBMODE = IDLE_DUE_TO_ERROR;
									}
									
									d_REFERENCE_SUBMODE = EXPECT_FOUND_FRAME;
									break;
								}
								
								case EXPECT_FOUND_FRAME:
								{
									// It can and will happen that no frame is found. 
									// But it cannot happen, that more than one frame is found if the 'found_security' is small against the skipped samples in valve (which it always should be).
									if(frame_type == VALVE_FRAME_TYPE_FOUND)
									{			
										rfs.recv_found_frames++;

										// PHY-LAYER
										FBMC_PHY::load_decoder(d_samples_index, mode_ref_data.encoding, mode_ref_data.payload_len, d_widths[d_log_chann], mode_ref_data.hard_soft_decoding, mode_ref_data.interl_type);
									
										char mpdu[mode_ref_data.payload_len];

										FBMC_PHY::extract_mpdu(mpdu, d_samples);
										
										// MAC-LAYER
										// -

										// check if all bytes are the same
										std::string ref(reference_bytes, mode_ref_data.payload_len);
										std::string mpd(mpdu, mode_ref_data.payload_len);
										if(ref == mpd)
											rfs.decoded_found_frames++;											
																			
										d_REFERENCE_SUBMODE = EXPECT_KNOWN_FRAME;
										break;
									}
									
									// If it's not a found frame it must (!) be a known frame.
									// Therefore no break.
								}
								
								case EXPECT_KNOWN_FRAME:
								{
									// it must (!) be a known frame
									if(frame_type == VALVE_FRAME_TYPE_KNOWN)
									{
										rfs.recv_known_frames++;

										// PHY-LAYER
										FBMC_PHY::load_decoder(d_samples_index, mode_ref_data.encoding, mode_ref_data.payload_len, d_widths[d_log_chann], mode_ref_data.hard_soft_decoding, mode_ref_data.interl_type);

										char mpdu[mode_ref_data.payload_len];

										FBMC_PHY::extract_mpdu(mpdu, d_samples);
										
										// MAC-LAYER
										// -										

										// check if all bytes are the same
										std::string ref(reference_bytes, mode_ref_data.payload_len);
										std::string mpd(mpdu, mode_ref_data.payload_len);
										if(ref == mpd)
											rfs.decoded_known_frames++;										
																		
										d_REFERENCE_SUBMODE = EXPECT_REF_FRAME;																			
									}
									else
									{
										rfs.known_frame_missed = true;
										d_REFERENCE_SUBMODE = IDLE_DUE_TO_ERROR;
									}									
									break;
								}
								
								case IDLE_DUE_TO_ERROR:
									break;
							}
					
							break;
						}
					}
					
					d_samples_index = 0;
					d_STATE = DATA_ACQUISITION;				
					
					break;
				}
			}
		}

        consume_each (noutput_items);
        
        return 0;
    }

  } /* namespace fbmc1 */
} /* namespace gr */

