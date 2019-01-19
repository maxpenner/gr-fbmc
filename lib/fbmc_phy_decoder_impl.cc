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
#include "fbmc_phy_decoder_impl.h"

#include "utils/debug.h"
#include "phy_coding/decoder.h"
#include "utils/receiver_tag.h"

namespace gr {
  namespace fbmc1 {

    fbmc_phy_decoder::sptr
    fbmc_phy_decoder::make(int veclength, int symbols_per_frame, int encoding_family, std::vector<int> logical_channels)
    {
      return gnuradio::get_initial_sptr
        (new fbmc_phy_decoder_impl(veclength, symbols_per_frame, encoding_family, logical_channels));
    }

    /*
     * The private constructor
     */
    fbmc_phy_decoder_impl::fbmc_phy_decoder_impl(int veclength, int symbols_per_frame, int encoding_family, std::vector<int> logical_channels)
      : gr::sync_block("fbmc_phy_decoder",
              gr::io_signature::make(1, 1, sizeof(gr_complex)*veclength),
              gr::io_signature::make(0,0,0))
    {
		// init variables
		d_STATE	    	 = DATA_ACQUISITION;
		d_veclength 	 = veclength; 
		d_spf	    	 = symbols_per_frame;
		d_encoding_fam	 = encoding_family;
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
		
#ifdef MAC_DECODER_PRINT_RATE
		d_phy_frames = 0;
		d_phy_errors = 0;

#ifdef VALVE_FD_DOUBLE_FRAMING
		d_phy_double = 0;
#endif
		
		d_thread = new boost::thread(boost::bind(&fbmc_phy_decoder_impl::send_stats, this));
#endif

		// output port
		message_port_register_out(pmt::mp("phy out"));
	}

    /*
     * Our virtual destructor.
     */
    fbmc_phy_decoder_impl::~fbmc_phy_decoder_impl()
    {
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
    
#ifdef MAC_DECODER_PRINT_RATE
    //______________________________________________________
    void fbmc_phy_decoder_impl::send_stats()
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
    fbmc_phy_decoder_impl::work(int noutput_items,
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
							d_log_chann = (int) (d_receiver_tag & RECEIVER_TAG_LOGIC_CHAN_MASK);
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
					
#ifdef MAC_DECODER_PRINT_RATE
					{
						gr::thread::scoped_lock(d_mutex);
						d_phy_frames++;

#ifdef VALVE_FD_DOUBLE_FRAMING		
						int doubleFrameBit = (int) ((d_receiver_tag >> RECEIVER_TAG_DOUBL_FRAM_OFFS) & 1ULL);
						if(doubleFrameBit == 1)
							d_phy_double++;
#endif

					}
#endif

					// load relevant data into decoder
					FBMC_PHY::load_decoder(d_samples_index, d_encoding_fam, -1, d_widths[d_log_chann], FBMC_PHY_DECODER_DONT_CARE, FBMC_PHY_DECODER_DONT_CARE);
					
					int mpdu_len = 0;
					
					// check if any data can be extracted
					if(FBMC_PHY::get_mpdu_len(mpdu_len, d_samples) != true)
					{
						d_samples_index = 0;
						d_STATE = DATA_ACQUISITION;
						
#ifdef MAC_DECODER_PRINT_RATE
						{
							gr::thread::scoped_lock(d_mutex);
							d_phy_errors++;
						}
#endif

						break;
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

