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

#ifndef INCLUDED_FBMC_PHY_CODING_DECODER_H
#define INCLUDED_FBMC_PHY_CODING_DECODER_H

#include <complex>

#define FBMC_PHY_DECODER_DONT_CARE 0

namespace FBMC_PHY
{
	/*
	 * Load all information that is needed for PHY-decoding into module.
	 */	
	void load_decoder(int freq_samples_total, int encoding_family, int fixed_mpdu_len, int channel_width, int hard_soft_dec, int interleaving_type);
	
	/*
	 * Analyse the complex frequency samples and return the length of the mpdu within the samples.
	 * If extraction failed return false so frame can be dropped.
	 */
	bool get_mpdu_len(int &mpdu_len, const std::complex<float> *freq_samples);
	 
	/*
	 * From the complex frequency samples extract the mpdu.
	 * Memory has to be user-supplied, length is known from 'get_mpdu_len()'.
	 */
	void extract_mpdu(char *mpdu, const std::complex<float> *freq_samples);
}

#endif /* INCLUDED_FBMC_PHY_CODING_DECODER_H */
