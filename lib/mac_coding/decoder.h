/* -*- c++ -*- */
/* 
 * Copyright 2014 <+YOU OR YOUR COMPANY+>.
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

#ifndef INCLUDED_FBMC_MAC_CODING_DECODER_H
#define INCLUDED_FBMC_MAC_CODING_DECODER_H

#include <complex>

namespace FBMC_MAC
{
	/*
	 * Load all information that is needed for MAC-decoding into module.
	 */	
	void load_decoder(int mpdu_len, int encoding_family);
	
	/*
	 * This functions returns the number of extractable payload bytes. Can be zero (e.g. if checksum is wrong).
	 */
	int get_payload_len(const char *mpdu);
	
	/*
	 * This functions should be called only if the data is extractable from the MAC-header.
	 */
	uint16_t get_sender_id(const char *mpdu);
	uint16_t get_seq_no(const char *mpdu);	
	 
	/*
	 * From the mpdu bytes extract the payload bytes.
	 * Memory has to be user-supplied, length is known from 'get_payload_len()'.
	 */
	void extract_payload(char *payload, const char* mpdu);
}

#endif /* INCLUDED_FBMC_MAC_CODING_DECODER_H */
