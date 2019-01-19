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
#include "fbmc_phy_encoder_impl.h"

#include "utils/debug.h"
#include "phy_coding/encoder.h"
#include "utils/logical_channel.h"

namespace gr {
  namespace fbmc1 {

    fbmc_phy_encoder::sptr
    fbmc_phy_encoder::make()
    {
      return gnuradio::get_initial_sptr
        (new fbmc_phy_encoder_impl());
    }

    /*
     * The private constructor
     */
    fbmc_phy_encoder_impl::fbmc_phy_encoder_impl()
      : gr::sync_block("fbmc_phy_encoder",
              gr::io_signature::make(0,0,0),
              gr::io_signature::make(0,0,0))
    {
		// input port
		message_port_register_in(pmt::mp("phy in"));
		set_msg_handler(pmt::mp("phy in"), boost::bind(&fbmc_phy_encoder_impl::phy_in, this, _1));

		// output port
		message_port_register_out(pmt::mp("phy out"));
	}

    /*
     * Our virtual destructor.
     */
    fbmc_phy_encoder_impl::~fbmc_phy_encoder_impl()
    {
    }
    
    //______________________________________________________
    void fbmc_phy_encoder_impl::phy_in(pmt::pmt_t msg) 
    {
		const char *PHY_blob;
		size_t PHY_blob_len;
		int32_t *log_chann;
		size_t log_chann_len;

		// input blob extraction
		if(pmt::is_pair(msg))
		{
			PHY_blob = reinterpret_cast<const char *>(pmt::blob_data(pmt::cdr(msg)));
			PHY_blob_len = pmt::blob_length(pmt::cdr(msg));
			pmt::pmt_t meta_dict = pmt::dict_values(pmt::car(msg));
			log_chann = pmt::s32vector_writable_elements(pmt::nth(0, meta_dict), log_chann_len);
		}  
		else
		{
			throw std::invalid_argument("fbmc_PHY_encoder_impl error: Expect PDUs as input. Dropping data.");
			return;
		}
		
		// process input blob
		phy2caa_blob(PHY_blob, PHY_blob_len, log_chann, log_chann_len);
	}
	
	//______________________________________________________
	void fbmc_phy_encoder_impl::phy2caa_blob(const char *PHY_blob, size_t PHY_blob_len, int32_t *log_chann, size_t log_chann_len)
	{
		// new CAA blob container
		char *CAA_blob = NULL;
		int CAA_blob_len = 0;
		
		// logical channel widths
		std::vector<int> widths;
		logical_channels_widths(log_chann, log_chann_len, widths);		
		
		// byte index in PHY blob
		unsigned int PHY_blob_index = 0;
		
		while(PHY_blob_index != PHY_blob_len)
		{
			// mpdu information
			metadataMpdu *mm = const_cast<metadataMpdu*>(reinterpret_cast<const metadataMpdu*> (PHY_blob + PHY_blob_index));
			PHY_blob_index += sizeof(metadataMpdu);
		
			// load PHY-encoder module
			FBMC_PHY::load_encoder(mm, widths[mm->log_chann]);
			
			int ppdu_len = FBMC_PHY::get_ppdu_len();
			
			// security mechanism
			if(ppdu_len == 0)
			{
				PRINT("fbmc_phy_encoder_impl error: From mpdu a ppdu with length 0 was generated. Dropping mpdu from MAC layer");
				PHY_blob_index += mm->mpdu_len;
				continue;
			}			
				
			// set ppdu information
			metadataPpdu mp;
			mp.ppdu_len 	= ppdu_len;
			mp.log_chann 	= mm->log_chann;
			mp.encoding 	= mm->encoding;
			mp.airtime 		= mm->airtime;
			mp.scale 		= mm->scale;
			mp.altern_pream = mm->altern_pream;
			
			// fill new ppdu
			char ppdu[ppdu_len];
			FBMC_PHY::generate_ppdu(ppdu, PHY_blob + PHY_blob_index);
			
			// extend PHY blob
			int blob_chunk = sizeof(metadataPpdu) + mp.ppdu_len;
			char *temp = (char*) realloc(CAA_blob, CAA_blob_len + blob_chunk);
			CAA_blob = temp;
			memcpy(CAA_blob + CAA_blob_len, &mp, sizeof(metadataPpdu));
			memcpy(CAA_blob + CAA_blob_len + sizeof(metadataPpdu), ppdu, mp.ppdu_len);
			
			PHY_blob_index += mm->mpdu_len;
			CAA_blob_len += blob_chunk;
		}
		
		if(CAA_blob_len > 0)
			publish_blob(CAA_blob, CAA_blob_len, log_chann, log_chann_len);
		else
			PRINT("fbmc_phy_encoder_impl error: Zero blob created.");
		
		// clean up
		free(CAA_blob);
	}
	
	//______________________________________________________
	void fbmc_phy_encoder_impl::publish_blob(const char *CAA_blob, size_t CAA_blob_len, int32_t *log_chann, size_t log_chann_len)
	{
		// copy metadata for logical channels
		pmt::pmt_t PMT_log_chann = pmt::make_s32vector(log_chann_len, 0);
		size_t temp;
		int32_t *elements = pmt::s32vector_writable_elements(PMT_log_chann, temp);
		memcpy(elements, log_chann, log_chann_len*sizeof(int32_t));
		
		// publish blob
		pmt::pmt_t dict = pmt::make_dict();
		dict = pmt::dict_add(dict, pmt::mp("logical_channels"), PMT_log_chann);
		pmt::pmt_t PMT_CAA_blob = pmt::make_blob(CAA_blob, CAA_blob_len);
		message_port_pub(pmt::mp("phy out"), pmt::cons(dict, PMT_CAA_blob));
	}

	//______________________________________________________
    int
    fbmc_phy_encoder_impl::work(int noutput_items,
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

