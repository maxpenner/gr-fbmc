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
#include "fbmc_mac_encoder_impl.h"

#include "utils/debug.h"
#include "utils/logical_channel.h"
#include "mac_coding/encoder.h"

namespace gr {
  namespace fbmc1 {

    fbmc_mac_encoder::sptr
    fbmc_mac_encoder::make()
    {
      return gnuradio::get_initial_sptr
        (new fbmc_mac_encoder_impl());
    }

    /*
     * The private constructor
     */
    fbmc_mac_encoder_impl::fbmc_mac_encoder_impl()
      : gr::sync_block("fbmc_mac_encoder",
              gr::io_signature::make(0,0,0),
              gr::io_signature::make(0,0,0))
    {
		// input port
		message_port_register_in(pmt::mp("mac in"));
		set_msg_handler(pmt::mp("mac in"), boost::bind(&fbmc_mac_encoder_impl::mac_in, this, _1));

		// output port
		message_port_register_out(pmt::mp("mac out"));
	}

    /*
     * Our virtual destructor.
     */
    fbmc_mac_encoder_impl::~fbmc_mac_encoder_impl()
    {
    }

    //______________________________________________________
    void fbmc_mac_encoder_impl::mac_in(pmt::pmt_t msg) 
    {
		const char *MAC_blob;
		size_t MAC_blob_len;
		int32_t *log_chann;
		size_t log_chann_len;

		// input blob extraction
		if(pmt::is_pair(msg))
		{
			MAC_blob = reinterpret_cast<const char *>(pmt::blob_data(pmt::cdr(msg)));
			MAC_blob_len = pmt::blob_length(pmt::cdr(msg));
			pmt::pmt_t meta_dict = pmt::dict_values(pmt::car(msg));
			log_chann = pmt::s32vector_writable_elements(pmt::nth(0, meta_dict), log_chann_len);
		}  
		else
		{
			throw std::invalid_argument("fbmc_MAC_encoder_impl error: Expect PDUs as input. Dropping data.");
			return;
		}
		
		// process input blob
		mac2phy_blob(MAC_blob, MAC_blob_len, log_chann, log_chann_len);
	}
	
	//______________________________________________________
	void fbmc_mac_encoder_impl::mac2phy_blob(const char *MAC_blob, size_t MAC_blob_len, int32_t *log_chann, size_t log_chann_len)
	{
		char *PHY_blob = NULL;
		int PHY_blob_len = 0;
		
		// logical channel widths
		std::vector<int> widths;
		logical_channels_widths(log_chann, log_chann_len, widths);
		
		unsigned int MAC_blob_index = 0;
		
		// process each payload
		while(MAC_blob_index != MAC_blob_len)
		{
			// payload information
			metadataPayload *mp = const_cast<metadataPayload*>(reinterpret_cast<const metadataPayload*> (MAC_blob + MAC_blob_index));
			MAC_blob_index += sizeof(metadataPayload);
			
			// load MAC-encoder module
			FBMC_MAC::load_encoder(mp, widths[mp->log_chann]);
			
			// fragmentate
			std::vector<int> fragment_lengths;
			FBMC_MAC::fragmentation(fragment_lengths);
								
			// security mechanism
			if(fragment_lengths.size() == 0)
			{
				PRINT("fbmc_mac_encoder_impl error: No fragments created. Dropping one payload from application layer.");
				MAC_blob_index += mp->payload_len;
				continue;
			}
			
			for(int i=0; i<fragment_lengths.size(); i++)
			{
				int mpdu_len = FBMC_MAC::get_mpdu_len(fragment_lengths[i]);
				
				// set mpdu information
				metadataMpdu mm;
				mm.mpdu_len 	= mpdu_len;
				mm.log_chann 	= mp->log_chann;
				mm.spf			= mp->spf;
				mm.encoding 	= mp->encoding;
				mm.airtime 		= mp->airtime;
				mm.scale 		= mp->scale;
				mm.interl_type 	= mp->interl_type;
				mm.altern_pream = mp->altern_pream;
				
				// fill new mpdu
				char mpdu[mpdu_len];
				FBMC_MAC::generate_mpdu(mpdu, MAC_blob + MAC_blob_index, fragment_lengths[i], i);
				
				// extend PHY blob
				int blob_chunk = sizeof(metadataMpdu) + mm.mpdu_len;
				char *temp = (char*) realloc(PHY_blob, PHY_blob_len + blob_chunk);
				PHY_blob = temp;
				memcpy(PHY_blob + PHY_blob_len, &mm, sizeof(metadataMpdu));
				memcpy(PHY_blob + PHY_blob_len + sizeof(metadataMpdu), mpdu, mm.mpdu_len);
				
				MAC_blob_index += fragment_lengths[i];
				PHY_blob_len += blob_chunk;
			}
		}
		
		if(PHY_blob_len > 0)
			publish_blob(PHY_blob, PHY_blob_len, log_chann, log_chann_len);
		else
			PRINT("fbmc_mac_encoder_impl error: Zero blob created.");
				
		// clean up
		free(PHY_blob);
	}
	
	//______________________________________________________
	void fbmc_mac_encoder_impl::publish_blob(const char *PHY_blob, size_t PHY_blob_len, int32_t *log_chann, size_t log_chann_len)
	{
		// copy metadata for logical channels
		pmt::pmt_t PMT_log_chann = pmt::make_s32vector(log_chann_len, 0);
		size_t temp;
		int32_t *elements = pmt::s32vector_writable_elements(PMT_log_chann, temp);
		memcpy(elements, log_chann, log_chann_len*sizeof(int32_t));
		
		// publish blob
		pmt::pmt_t dict = pmt::make_dict();
		dict = pmt::dict_add(dict, pmt::mp("logical_channels"), PMT_log_chann);
		pmt::pmt_t PMT_PHY_blob = pmt::make_blob(PHY_blob, PHY_blob_len);
		message_port_pub(pmt::mp("mac out"), pmt::cons(dict, PMT_PHY_blob));
	}

	//______________________________________________________
    int
    fbmc_mac_encoder_impl::work(int noutput_items,
			  gr_vector_const_void_star &input_items,
			  gr_vector_void_star &output_items)
    {
        //const <+ITYPE+> *in = (const <+ITYPE+> *) input_items[0];
        //<+OTYPE+> *out = (<+OTYPE+> *) output_items[0];

        // Do <+signal processing+>

        // Tell runtime system how many output items we produced.
        return noutput_items;
    }

  } /* namespace fbmc1 */
} /* namespace gr */

