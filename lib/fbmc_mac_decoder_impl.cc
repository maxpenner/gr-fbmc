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
#include "fbmc_mac_decoder_impl.h"

#include "utils/receiver_tag.h"
#include "mac_coding/decoder.h"

namespace gr {
  namespace fbmc1 {

    fbmc_mac_decoder::sptr
    fbmc_mac_decoder::make(int encoding_family)
    {
      return gnuradio::get_initial_sptr
        (new fbmc_mac_decoder_impl(encoding_family));
    }

    /*
     * The private constructor
     */
    fbmc_mac_decoder_impl::fbmc_mac_decoder_impl(int encoding_family)
      : gr::sync_block("fbmc_mac_decoder",
              gr::io_signature::make(0,0,0),
              gr::io_signature::make(0,0,0))
    {
		d_encoding_family = encoding_family;
		
#ifdef MAC_DECODER_PRINT_RATE
		d_mac_timer		= new fbmc_timer(MAC_DECODER_PRINT_RATE);
		d_phy_frames	= 0;
		d_phy_errors	= 0;
		d_mac_frames	= 0;
		d_mac_frames_ok	= 0;
		d_mac_bytes		= 0;
		d_mac_bytes_ok	= 0;

#ifdef VALVE_FD_DOUBLE_FRAMING
		d_phy_double = 0;
		d_mac_double = 0;
#endif
		
		d_thread = new boost::thread(boost::bind(&fbmc_mac_decoder_impl::display, this));
#endif
		
		// input ports
		message_port_register_in(pmt::mp("mac in"));
		set_msg_handler(pmt::mp("mac in"), boost::bind(&fbmc_mac_decoder_impl::mac_in, this, _1));

		// output ports
		message_port_register_out(pmt::mp("mac out"));
	}

    /*
     * Our virtual destructor.
     */
    fbmc_mac_decoder_impl::~fbmc_mac_decoder_impl()
    {
		
#ifdef MAC_DECODER_PRINT_RATE
		{
			gr::thread::scoped_lock(d_mutex);
			
			d_thread->interrupt();
			d_thread->join();
			delete d_thread;
		}
		
		delete d_mac_timer;
#endif

    }

    //______________________________________________________
    void fbmc_mac_decoder_impl::mac_in(pmt::pmt_t msg) 
    {
		const char *MAC_blob;
		size_t MAC_blob_len;
		uint64_t receiver_tag;

		// input blob extraction
		if(pmt::is_pair(msg))
		{

#ifdef MAC_DECODER_PRINT_RATE
			if(pmt::dict_has_key(pmt::car(msg), pmt::mp("phy_frames")))
			{
				d_phy_frames = pmt::to_uint64(pmt::dict_ref(pmt::car(msg), pmt::mp("phy_frames"), pmt::from_uint64(d_phy_frames)));
				d_phy_errors = pmt::to_uint64(pmt::dict_ref(pmt::car(msg), pmt::mp("phy_errors"), pmt::from_uint64(d_phy_errors)));
				
#ifdef VALVE_FD_DOUBLE_FRAMING
				d_phy_double = pmt::to_uint64(pmt::dict_ref(pmt::car(msg), pmt::mp("phy_double"), pmt::from_uint64(d_phy_double)));	
#endif	

				return;
			}
#endif
			
			// cdr = blob
			MAC_blob = reinterpret_cast<const char *>(pmt::blob_data(pmt::cdr(msg)));
			MAC_blob_len = pmt::blob_length(pmt::cdr(msg));
			
			// car == dictionary = keys + values
			receiver_tag = pmt::to_uint64(pmt::dict_ref(pmt::car(msg), pmt::mp("receiver_tag"), pmt::PMT_NIL));
		}  
		else
		{
			throw std::invalid_argument("fbmc_mac_decoder_impl error: Expect PDUs as input. Dropping data.");
			return;
		}
		
		// process input blob
		mac2app_blob(MAC_blob, MAC_blob_len, receiver_tag);
	}
	
	//______________________________________________________
	void fbmc_mac_decoder_impl::mac2app_blob(const char *MAC_blob, size_t MAC_blob_len, uint64_t receiver_tag)
	{
		
#ifdef MAC_DECODER_PRINT_RATE
		{
			gr::thread::scoped_lock(d_mutex);
			
			d_mac_frames++;
			d_mac_bytes += MAC_blob_len;

#ifdef VALVE_FD_DOUBLE_FRAMING		
			int doubleFrameBit = (int) ((receiver_tag >> RECEIVER_TAG_DOUBL_FRAM_OFFS) & 1ULL);
			if(doubleFrameBit == 1)
				d_mac_double++;
#endif

		}
#endif

		FBMC_MAC::load_decoder(MAC_blob_len, d_encoding_family);
			
		int payload_len = FBMC_MAC::get_payload_len(MAC_blob);
		
		if(payload_len == 0)
			return;
			
		char payload[payload_len];
		
		FBMC_MAC::extract_payload(payload, MAC_blob);
		
		publish_blob(payload, payload_len);
		
#ifdef MAC_DECODER_PRINT_RATE
		{
			gr::thread::scoped_lock(d_mutex);
			
			d_mac_frames_ok++;
			d_mac_bytes_ok += payload_len;
		}
#endif

	}
	
	//______________________________________________________
	void fbmc_mac_decoder_impl::publish_blob(const char *APP_blob, size_t APP_blob_len)
	{
		// publish PDU with empty car
		pmt::pmt_t PMT_APP_blob = pmt::make_blob(APP_blob, APP_blob_len);
		message_port_pub(pmt::mp("mac out"), pmt::cons(pmt::PMT_NIL, PMT_APP_blob));
	}
    
#ifdef MAC_DECODER_PRINT_RATE   
    //______________________________________________________
    void fbmc_mac_decoder_impl::display()
    {
		try 
		{
			while(1)
			{
				boost::this_thread::sleep(boost::posix_time::milliseconds(MAC_DECODER_PRINT_RATE*1000));
				
				{
					gr::thread::scoped_lock(d_mutex);
					
					d_mac_timer->new_update();	

#ifdef VALVE_FD_DOUBLE_FRAMING
					int phy_doubl_frames = d_phy_double/2;
					int phy_total_frames = d_phy_frames - phy_doubl_frames;
					
					if(d_mac_frames > 0)
					{
						std::cout << std::endl 	<< std::endl
						<< "------------------ MAC DECODER -------------------" << std::endl
						<< "passed time:         " 	<< floor(d_mac_timer->get_passed_time()/60/60) << " h  "
													<< floor((d_mac_timer->get_passed_time()/60)%60) << " min  "
													<< d_mac_timer->get_passed_time()%60 << " sec " << std::endl
						<< "phy frames total:    " 	<< phy_total_frames << std::endl
						<< "phy frames doubled:  " 	<< phy_doubl_frames << " "
													<< phy_doubl_frames/(double) phy_total_frames*100 << " %" << std::endl
						<< "phy frames error:    " 	<< d_phy_errors << " "
													<< d_phy_errors/(double) d_phy_frames*100 << " %" << std::endl
						<< "mac frames total:    " 	<< d_mac_frames << std::endl
						<< "mac frames doubled:  " 	<< d_mac_double << " "
													<< d_mac_double/(double) d_mac_frames*100 << " %" << std::endl
						<< "mac frames correct:  " 	<< d_mac_frames_ok << " "
													<< d_mac_frames_ok/(double) d_mac_frames*100 << " %" << std::endl
						<< "mac frames wrong:    " 	<< d_mac_frames-d_mac_frames_ok << "   " 
													<< (d_mac_frames-d_mac_frames_ok)/(double) d_mac_frames*100 << " %" << std::endl 
						<< "gross bytes:         " 	<< d_mac_bytes/(double) 1e6 << " MByte"<< std::endl
						<< "net bytes:           " 	<< d_mac_bytes_ok/(double) 1e6 << " MByte"<< std::endl
						<< "net bytes per frame: " 	<< d_mac_bytes_ok/(double) d_mac_frames_ok << std::endl
						<< "datarate:            " 	<< d_mac_bytes_ok*8/1e3/d_mac_timer->get_passed_time() << " kbps" 	<< std::endl 
						<< "framerate:           " 	<< d_mac_frames_ok/d_mac_timer->get_passed_time() << " frames per sec" << std::endl
						<< "--------------------------------------------------"	<< std::endl;
					}
#else	
					if(d_mac_frames > 0)
					{
						std::cout << std::endl 	<< std::endl
						<< "------------------ MAC DECODER -------------------" << std::endl
						<< "passed time:         " 	<< floor(d_mac_timer->get_passed_time()/60/60) << " h  "
													<< floor((d_mac_timer->get_passed_time()/60)%60) << " min  "
													<< d_mac_timer->get_passed_time()%60 << " sec " << std::endl
						<< "phy frames total:    " 	<< d_phy_frames << std::endl
						<< "phy frames error:    " 	<< d_phy_errors << " "
													<< d_phy_errors/(double) d_phy_frames*100 << " %" << std::endl
						<< "mac frames total:    " 	<< d_mac_frames << std::endl
						<< "mac frames correct:  " 	<< d_mac_frames_ok << " "
													<< d_mac_frames_ok/(double) d_mac_frames*100 << " %" << std::endl
						<< "mac frames wrong:    " 	<< d_mac_frames-d_mac_frames_ok << "   " 
													<< (d_mac_frames-d_mac_frames_ok)/(double) d_mac_frames*100 << " %" << std::endl 
						<< "gross bytes:         " 	<< d_mac_bytes/(double) 1e6 << " MByte"<< std::endl
						<< "net bytes:           " 	<< d_mac_bytes_ok/(double) 1e6 << " MByte"<< std::endl
						<< "net bytes per frame: " 	<< d_mac_bytes_ok/(double) d_mac_frames_ok << std::endl
						<< "datarate:            " 	<< d_mac_bytes_ok*8/1e3/d_mac_timer->get_passed_time() << " kbps" 	<< std::endl 
						<< "framerate:           " 	<< d_mac_frames_ok/d_mac_timer->get_passed_time() << " frames per sec" << std::endl
						<< "--------------------------------------------------"	<< std::endl;
					}
#endif
				}
			}
		}
		catch(boost::thread_interrupted) 
		{
			//PRINT("fbmc_mac_decoder_impl: Thread for displaying data properly interrupted.");
		}
	}
#endif

	//______________________________________________________
    int
    fbmc_mac_decoder_impl::work(int noutput_items,
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

