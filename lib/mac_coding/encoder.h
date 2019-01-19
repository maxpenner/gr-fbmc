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

#ifndef INCLUDED_FBMC_MAC_CODING_ENCODER_H
#define INCLUDED_FBMC_MAC_CODING_ENCODER_H

#include <vector>

#include "../utils/metadata.h"

namespace FBMC_MAC
{		
	/*
	 * Load all information that is needed for MAC-encoding into module.
	 */	
	void load_encoder(struct metadataPayload *metadata, int channel_width);
	
	/*
	 * Determine the fragmentation of the payload. Sum of fragments lengths will be the payload length.
	 * If payload can't be fragmentated (no data fits into frame), don't write to 'fragment_lengths' and leave it with size 0.
	 */	
	void fragmentation(std::vector<int> &fragment_lengths);	
	
	/*
	 * Return the length of the mpdu = fragment-bytes + MAC-encoding-bytes.
	 */	
	int get_mpdu_len(int fragment_len);
		 
	/*
	 * From a fragment, create a mpdu with the length returned from 'get_mpdu_len()'.
	 * Memory has to be user-supplied.
	 */	
	void generate_mpdu(char *mpdu, const char *fragment, int fragment_len, int frag_no);
}

#endif /* INCLUDED_FBMC_MAC_CODING_ENCODER_H */
