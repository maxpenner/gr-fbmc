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

#ifndef INCLUDED_FBMC_PHY_CODING_ENCODER_H
#define INCLUDED_FBMC_PHY_CODING_ENCODER_H

#include "../utils/metadata.h"

namespace FBMC_PHY
{
	/*
	 * Load all information that is needed for PHY-encoding into module.
	 */	
	void load_encoder(struct metadataMpdu *metadata, int channel_width);
	
	/*
	 * Return the length of the ppdu = encoded mpdu-bytes + PHY-encoding-bytes.
	 */
	int get_ppdu_len();
	 
	/*
	 * From a mpdu, create a ppdu with the length returned from 'get_ppdu_len()'.
	 * Memory for the ppdu has to be user-supplied.
	 */
	void generate_ppdu(char* ppdu, const char* mpdu);
}

#endif /* INCLUDED_FBMC_PHY_CODING_ENCODER_H */