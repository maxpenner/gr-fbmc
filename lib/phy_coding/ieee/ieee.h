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

#ifndef INCLUDED_FBMC_PHY_CODING_IEEE_IEEE_H
#define INCLUDED_FBMC_PHY_CODING_IEEE_IEEE_H

#include <complex>

#define SCRAMBLER_START_STATE 23

class IEEE_ofdm_param
{
	public:
		IEEE_ofdm_param(int encoding);
		int enc;
		char rate_field;
		int n_bpsc;
		int n_cbps;
		int n_dbps;
};

class IEEE_tx_param
{
	public:
		IEEE_tx_param(IEEE_ofdm_param ofdm, int mpdu_len_arg);
		int mpdu_len;
		int n_sym;
		int n_data;
		int n_pad;
		int n_encoded_bits;
};

// encoder (input, output, variables)
void IEEE_generate_bits(const char *mpdu, char *bits, IEEE_tx_param &tx);
void IEEE_scramble(const char *bits, char *scrambled_bits, IEEE_tx_param &tx, char initial_state);
void IEEE_reset_tail_bits(char *scrambled_bits, IEEE_tx_param &tx);
int IEEE_ones(int n); 
void IEEE_convolutional_encoding(const char *scrambled_bits, char *encoded_bits, IEEE_tx_param &tx);
void IEEE_puncturing(const char *encoded_bits, char *punctured_bits, IEEE_tx_param &tx, IEEE_ofdm_param &ofdm);
void IEEE_interleave(const char *punctured_bits, char *interleaved_bits, IEEE_tx_param &tx, IEEE_ofdm_param &ofdm, bool reverse = false);
void IEEE_split_symbols(const char *interleaved_bits, char *bit_symbols, IEEE_tx_param &tx, IEEE_ofdm_param &ofdm);
void IEEE_insert_plcp_header(char *output_position, char rate_field, int mpdu_len);

// decoder (input, output, variables)
void IEEE_plcp_deinterleave(float *freq_samples_real);
bool IEEE_plcp_decode(float *freq_samples_real_deinterleaved, int &rate_field, int &mpdu_len);
void IEEE_demod_hard(const std::complex<float> *received_symbols_comp, unsigned char *hard_bits, IEEE_tx_param &tx, IEEE_ofdm_param &ofdm);
void IEEE_deinterleave(unsigned char *hard_bits, unsigned char *deinterleaved_bits, IEEE_tx_param &tx, IEEE_ofdm_param &ofdm);
void IEEE_depuncture(unsigned char *deinterleaved_bits, unsigned char *depunctured_bits, IEEE_tx_param &tx, IEEE_ofdm_param &ofdm);
void IEEE_decode_conv(unsigned char *depunctured_bits, unsigned char *descrambled_bytes, IEEE_tx_param &tx, IEEE_ofdm_param &ofdm);
void IEEE_descramble(unsigned char *descrambled_bytes, IEEE_tx_param &tx, IEEE_ofdm_param &ofdm);

#endif /* INCLUDED_FBMC_PHY_CODING_IEEE_IEEE_H */
