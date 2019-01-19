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

#include <string.h>
#include <stdlib.h>
#include <vector>

#include "../utils/debug.h"
#include "../utils/encodings.h"

#include "decoder.h"
#include "ieee/ieee.h"
#include "ieee/spiral_viterbi_interface.h"

// general module variables
static enum {IEEE, SENSOR_MIN} d_MODULE_STATE;		
static int d_freq_samples_total;
static int d_encoding_family;
static int d_fixed_mpdu_len;
static int d_channel_width;
static int d_hard_soft_dec;
static int d_interleaving_type;

// general module functions
static void clipping(float &val);

// IEEE	PHY-decoding
static int IEEE_plcp_rate_field;
static int IEEE_plcp_mpdu_len;
static int IEEE_plcp_encoding;
static bool IEEE_get_mpdu_len(int &mpdu_len, const std::complex<float> *freq_samples);
static void IEEE_extract_mpdu(char *mpdu, const std::complex<float> *freq_samples);

// SENSOR_MIN PHY-decoding	//_________________________________________________
static const unsigned char SENSOR_MIN_BitReverseTable256[256] = 
{
	  0x00, 0x80, 0x40, 0xC0, 0x20, 0xA0, 0x60, 0xE0, 0x10, 0x90, 0x50, 0xD0, 0x30, 0xB0, 0x70, 0xF0, 
	  0x08, 0x88, 0x48, 0xC8, 0x28, 0xA8, 0x68, 0xE8, 0x18, 0x98, 0x58, 0xD8, 0x38, 0xB8, 0x78, 0xF8, 
	  0x04, 0x84, 0x44, 0xC4, 0x24, 0xA4, 0x64, 0xE4, 0x14, 0x94, 0x54, 0xD4, 0x34, 0xB4, 0x74, 0xF4, 
	  0x0C, 0x8C, 0x4C, 0xCC, 0x2C, 0xAC, 0x6C, 0xEC, 0x1C, 0x9C, 0x5C, 0xDC, 0x3C, 0xBC, 0x7C, 0xFC, 
	  0x02, 0x82, 0x42, 0xC2, 0x22, 0xA2, 0x62, 0xE2, 0x12, 0x92, 0x52, 0xD2, 0x32, 0xB2, 0x72, 0xF2, 
	  0x0A, 0x8A, 0x4A, 0xCA, 0x2A, 0xAA, 0x6A, 0xEA, 0x1A, 0x9A, 0x5A, 0xDA, 0x3A, 0xBA, 0x7A, 0xFA,
	  0x06, 0x86, 0x46, 0xC6, 0x26, 0xA6, 0x66, 0xE6, 0x16, 0x96, 0x56, 0xD6, 0x36, 0xB6, 0x76, 0xF6, 
	  0x0E, 0x8E, 0x4E, 0xCE, 0x2E, 0xAE, 0x6E, 0xEE, 0x1E, 0x9E, 0x5E, 0xDE, 0x3E, 0xBE, 0x7E, 0xFE,
	  0x01, 0x81, 0x41, 0xC1, 0x21, 0xA1, 0x61, 0xE1, 0x11, 0x91, 0x51, 0xD1, 0x31, 0xB1, 0x71, 0xF1,
	  0x09, 0x89, 0x49, 0xC9, 0x29, 0xA9, 0x69, 0xE9, 0x19, 0x99, 0x59, 0xD9, 0x39, 0xB9, 0x79, 0xF9, 
	  0x05, 0x85, 0x45, 0xC5, 0x25, 0xA5, 0x65, 0xE5, 0x15, 0x95, 0x55, 0xD5, 0x35, 0xB5, 0x75, 0xF5,
	  0x0D, 0x8D, 0x4D, 0xCD, 0x2D, 0xAD, 0x6D, 0xED, 0x1D, 0x9D, 0x5D, 0xDD, 0x3D, 0xBD, 0x7D, 0xFD,
	  0x03, 0x83, 0x43, 0xC3, 0x23, 0xA3, 0x63, 0xE3, 0x13, 0x93, 0x53, 0xD3, 0x33, 0xB3, 0x73, 0xF3, 
	  0x0B, 0x8B, 0x4B, 0xCB, 0x2B, 0xAB, 0x6B, 0xEB, 0x1B, 0x9B, 0x5B, 0xDB, 0x3B, 0xBB, 0x7B, 0xFB,
	  0x07, 0x87, 0x47, 0xC7, 0x27, 0xA7, 0x67, 0xE7, 0x17, 0x97, 0x57, 0xD7, 0x37, 0xB7, 0x77, 0xF7, 
	  0x0F, 0x8F, 0x4F, 0xCF, 0x2F, 0xAF, 0x6F, 0xEF, 0x1F, 0x9F, 0x5F, 0xDF, 0x3F, 0xBF, 0x7F, 0xFF
};
const int SENSOR_MIN_max_interleaver_3 = 14;
static const int SENSOR_MIN_interleaver_3_opt[SENSOR_MIN_max_interleaver_3][SENSOR_MIN_max_interleaver_3] =
{	
								0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
								0,  1,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,								
								0,  1,  2,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
								0,  2,  1,  3,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
								0,  2,  3,  1,  4,  0,  0,  0,  0,  0,  0,	0,  0,  0,				
								0,  3,  1,  4,  2,  5,  0,  0,  0,  0,  0,  0,  0,  0,
								0,  3,  5,  2,  6,  1,  4,  0,  0,  0,  0,  0,  0,  0,
								0,  4,  2,  6,  1,  5,  3,  7,  0,  0,  0,  0,  0,  0,
								0,  4,  6,  2,  7,  1,  5,  3,  8,  0,  0,	0,  0,  0,				 
								0,  5,  2,  7,  3,  8,  1,  6,  4,  9,  0,  0,  0,  0,
								0,  5,  8,  2,  6,  3,  9, 10,  1,  4,  7,  0,  0,  0,
								0,  6,  3,  9,  1,  7,  4, 10,  2,  8,  5, 11,  0,  0,
								0,  6,  9,  3, 11,  1,  7,  4,  2,  8,  5, 12, 10,  0,
								0,  7,  3, 10,  5, 12,  1,  8,  4, 11,  2,  9,  6, 13
};
static void SENSOR_MIN_extract_mpdu(char *mpdu, const std::complex<float> *freq_samples);

namespace FBMC_PHY
{
	//______________________________________________________
	void load_decoder(int freq_samples_total, int encoding_family, int fixed_mpdu_len, int channel_width, int hard_soft_dec, int interleaving_type)
	{
		d_freq_samples_total = freq_samples_total;
		d_encoding_family = encoding_family;
		d_fixed_mpdu_len = fixed_mpdu_len;
		d_channel_width = channel_width;
		d_hard_soft_dec = hard_soft_dec;
		d_interleaving_type = interleaving_type;
		
		switch(d_encoding_family)
		{
			case IEEE_BPSK_1_2:
			case IEEE_BPSK_3_4:
			case IEEE_QPSK_1_2:
			case IEEE_QPSK_3_4:
			case IEEE_QAM16_1_2:
			case IEEE_QAM16_3_4:
			case IEEE_QAM64_2_3:
			case IEEE_QAM64_3_4:
				d_MODULE_STATE = IEEE;
				break;
				
			case SENSOR_MINIMAL_BPSK_1_2:
			case SENSOR_MINIMAL_BPSK_2_3:
			case SENSOR_MINIMAL_BPSK_3_4:
			case SENSOR_MINIMAL_QPSK_1_2:
			case SENSOR_MINIMAL_QPSK_2_3:
			case SENSOR_MINIMAL_QPSK_3_4:
			case SENSOR_MINIMAL_QPSK_7_8:
			case SENSOR_MINIMAL_QAM16_1_2:
			case SENSOR_MINIMAL_QAM16_3_4:
			case SENSOR_MINIMAL_QAM64_2_3:
			case SENSOR_MINIMAL_QAM64_3_4:
				d_MODULE_STATE = SENSOR_MIN;
				break;	
				
			default:
				PRINT("load_encoder PHY stage decoder error: Unknown encoding.");				
		}
	}
	
	//______________________________________________________
	bool get_mpdu_len(int &mpdu_len, const std::complex<float> *freq_samples)
	{
		bool valid_data = false;
		
		switch(d_MODULE_STATE)
		{
			case IEEE:
				valid_data = IEEE_get_mpdu_len(mpdu_len, freq_samples);
				break;
				
			case SENSOR_MIN:
				PRINT("PHY decoder error: In mode sensor extracting the mpdu length is nonsense. It's known a priori. Returning known length.");
				valid_data = true;
				mpdu_len = d_fixed_mpdu_len;
				break;				
		}
		
		return valid_data;
	}
	
	//______________________________________________________
	void extract_mpdu(char *mpdu, const std::complex<float> *freq_samples)
	{
		switch(d_MODULE_STATE)
		{
			case IEEE:
				IEEE_extract_mpdu(mpdu, freq_samples);
				break;
				
			case SENSOR_MIN:
				SENSOR_MIN_extract_mpdu(mpdu, freq_samples);
				break;				
		}
	}
}

// general module functions
//______________________________________________________
void clipping(float &val)
{
	val = (val <= 255.0f) ? val : 255.0f;
	val = (val >= 0.0f) ? val : 0.0f;	
}

// IEEE	PHY-decoding
//______________________________________________________
bool IEEE_get_mpdu_len(int &mpdu_len, const std::complex<float> *freq_samples)
{
	float freq_symbols_real[48];
	
	for(int i=0; i<48; i++)
		freq_symbols_real[i] = freq_samples[i].real();

	// deinterleave samples
	IEEE_plcp_deinterleave(freq_symbols_real);
	
	// decode (and check parity bit in plcp header)
	if(IEEE_plcp_decode(freq_symbols_real, IEEE_plcp_rate_field, IEEE_plcp_mpdu_len) != true)
		return false;
	
	int n_dbps;
		
	// check rate field
	switch(IEEE_plcp_rate_field)
	{
		case 0x0D:
			IEEE_plcp_encoding = IEEE_BPSK_1_2;
			n_dbps = 24;
			break;
		case 0x0F:
			IEEE_plcp_encoding = IEEE_BPSK_3_4;
			n_dbps = 36;
			break;
		case 0x05:
			IEEE_plcp_encoding = IEEE_QPSK_1_2;
			n_dbps = 48;
			break;
		case 0x07:
			IEEE_plcp_encoding = IEEE_QPSK_3_4;
			n_dbps = 72;
			break;
		case 0x09:
			IEEE_plcp_encoding = IEEE_QAM16_1_2;
			n_dbps = 96;
			break;
		case 0x0B:
			IEEE_plcp_encoding = IEEE_QAM16_3_4;
			n_dbps = 144;
			break;
		case 0x01:
			IEEE_plcp_encoding = IEEE_QAM64_2_3;
			n_dbps = 192;
			break;
		case 0x03:
			IEEE_plcp_encoding = IEEE_QAM64_3_4;
			n_dbps = 216;
			break;
		default:
			return false;
			break;
	}
	
	// check mpdu length
	int usable_cp  	= d_freq_samples_total - 48;
	int vec48 		= floor(usable_cp/48);
	int max_bytes 	= floor((vec48*n_dbps-16-6)/8);
	
	if(IEEE_plcp_mpdu_len > max_bytes)
		return false;
	else
		mpdu_len = IEEE_plcp_mpdu_len;
		
	return true;
}

//______________________________________________________
void IEEE_extract_mpdu(char *mpdu, const std::complex<float> *freq_samples)
{	
	IEEE_ofdm_param ofdm(IEEE_plcp_encoding);
	IEEE_tx_param tx(ofdm, IEEE_plcp_mpdu_len);
	
	double rate_inv = ofdm.n_dbps/((double) (ofdm.n_cbps * 0.5));
	
	unsigned char *demod_bits 			= (unsigned char*) malloc(tx.n_encoded_bits);
	unsigned char *deinter_bits 		= (unsigned char*) malloc(tx.n_encoded_bits);
	unsigned char *depunc_bits 		= (unsigned char*) malloc(ceil(tx.n_encoded_bits*rate_inv));
	unsigned char *descrambled_bytes 	= (unsigned char*) malloc(tx.mpdu_len + 3);	// + 3 for 16 bits service field + mpdu + 6 tailbits

	// decode step by step
	IEEE_demod_hard(&freq_samples[48], demod_bits, tx, ofdm);
	IEEE_deinterleave(demod_bits, deinter_bits, tx, ofdm);
	IEEE_depuncture(deinter_bits, depunc_bits, tx, ofdm);
	IEEE_decode_conv(depunc_bits, descrambled_bytes, tx, ofdm);
	IEEE_descramble(descrambled_bytes, tx, ofdm);
	
	// skip two bytes of service field
	memmove(mpdu, descrambled_bytes + 2, tx.mpdu_len);

	// clean up
	free(demod_bits);
	free(deinter_bits);
	free(depunc_bits);
	free(descrambled_bytes);
}

// IEEE	PHY-decoding
//______________________________________________________
void SENSOR_MIN_extract_mpdu(char *mpdu, const std::complex<float> *freq_samples)
{		
	int M;
	int numerator;
	int denominator;
	
	switch(d_encoding_family)
	{
		case SENSOR_MINIMAL_BPSK_1_2:
			M = 1;
			numerator = 1;
			denominator = 2;
			break;
		case SENSOR_MINIMAL_BPSK_2_3:
			M = 1;
			numerator = 2;
			denominator = 3;
			break;			
		case SENSOR_MINIMAL_BPSK_3_4:
			M = 1;
			numerator = 3;
			denominator = 4;
			break;
		case SENSOR_MINIMAL_QPSK_1_2:
			M = 2;
			numerator = 1;
			denominator = 2;
			break;
		case SENSOR_MINIMAL_QPSK_2_3:
			M = 2;
			numerator = 2;
			denominator = 3;
			break;			
		case SENSOR_MINIMAL_QPSK_3_4:
			M = 2;
			numerator = 3;
			denominator = 4;
			break;
		case SENSOR_MINIMAL_QPSK_7_8:
			M = 2;
			numerator = 7;
			denominator = 8;
			break;			
		case SENSOR_MINIMAL_QAM16_1_2:
			M = 4;
			numerator = 1;
			denominator = 2;
			break;
		case SENSOR_MINIMAL_QAM16_3_4:
			M = 4;
			numerator = 3;
			denominator = 4;
			break;
		case SENSOR_MINIMAL_QAM64_2_3:
			M = 6;
			numerator = 2;
			denominator = 3;
			break;
		case SENSOR_MINIMAL_QAM64_3_4:
			M = 6;
			numerator = 3;
			denominator = 4;
			break;
		default:
			PRINT("SENSOR_MIN_extract_mpdu error: Unknown encoding. Returning nonsense-data frame.");	
			return;		
	}
	
	// AS AT TRANSMITTER: first calculate minimal number of symbols
	//int nBitsNetUser = d_fixed_mpdu_len*8;										// number of Bits to send from application layer
	int nApplicationBits = d_fixed_mpdu_len*8;										// number of Bits to send from application layer
	int nBitsNetUser = nApplicationBits + 6;										// 6 trailing zero bits for viterbi	
	
	int nBitsEncodedMin = (nBitsNetUser + numerator - 1)/numerator;					// minimum number of bits that have to be encoded to fit nBitsNetUser-many bits before encoder
	nBitsEncodedMin *= denominator;													// "
	int nBitsSymbGross = M*d_channel_width;											// number of bits that fit in one symbol
	int nSymbMin = (nBitsEncodedMin + nBitsSymbGross - 1)/nBitsSymbGross;			// number of symbols needed to fit nBitsNetUser-many bits
	
	// AS AT TRANSMITTER: now calculate how many bits we need in each stage
	int nBitsSendGross = M*nSymbMin*d_channel_width;								// total number of bits that will be on air
	int nBitsSendNetMax = nBitsSendGross/denominator;								// number of bits that have to be inputted to encoder
	nBitsSendNetMax *= numerator;													// "				
	int nPadBitsBefScra = nBitsSendNetMax - nBitsNetUser;							// number of padding bits before scrambler and encoder
	int nBitsSendCoded = nBitsSendNetMax/numerator;									// number of bits that encoder will output
	nBitsSendCoded *= denominator;													// "		
	int nPadBitsAftEncoder = nBitsSendGross - nBitsSendCoded;						// number of padding bits after the encoder (will be scrambled)	
	
	// added at receiver
	int nConstSymbSendGross = nSymbMin*d_channel_width;	

	// allocate memory
	unsigned char received_bits[nBitsSendGross];
	unsigned char deinterleaved_bits[nBitsSendGross];
	unsigned char depunc_bits[nBitsSendNetMax*2];
	unsigned char scrambled_bytes[d_fixed_mpdu_len];

	// demodulate
	unsigned char *received_bits_temp = received_bits;
	float t0, t1, t2, t3;
	
	// hard decoding
	if(d_hard_soft_dec == 0)
	{
		switch(d_encoding_family)
		{
			case SENSOR_MINIMAL_BPSK_1_2:
			case SENSOR_MINIMAL_BPSK_2_3:		
			case SENSOR_MINIMAL_BPSK_3_4:
				for(int i=0; i<nConstSymbSendGross;i++)
					*(received_bits_temp++) = (freq_samples[i].real() > 0) ? 255 : 0;		
				break;
				
			case SENSOR_MINIMAL_QPSK_1_2:
			case SENSOR_MINIMAL_QPSK_2_3:	
			case SENSOR_MINIMAL_QPSK_3_4:
			case SENSOR_MINIMAL_QPSK_7_8:
				for(int i=0; i<nConstSymbSendGross;i++)
				{
					//*(received_bits_temp++) = (freq_samples[i].real() > 0) ? 255 : 0;				// as in IEEE 802.11a	
					//*(received_bits_temp++) = (freq_samples[i].imag() > 0) ? 255 : 0;			
					*(received_bits_temp++) = (freq_samples[i].imag() > 0) ? 255 : 0;				// as in MATLAB
					*(received_bits_temp++) = (freq_samples[i].real() > 0) ? 255 : 0;
				}
				break;
				
			case SENSOR_MINIMAL_QAM16_1_2:
			case SENSOR_MINIMAL_QAM16_3_4:
				t0 = 2/sqrt(10);
				for(int i=0; i<nConstSymbSendGross;i++)
				{
					//*(received_bits_temp++) = (freq_samples[i].real() > 0) ? 255 : 0;				// as in IEEE 802.11a	
					//*(received_bits_temp++) = (std::abs(freq_samples[i].real()) < t0) ? 255 : 0;
					//*(received_bits_temp++) = (freq_samples[i].imag() > 0) ? 255 : 0;
					//*(received_bits_temp++) = (std::abs(freq_samples[i].imag()) < t0) ? 255 : 0;
					*(received_bits_temp++) = (std::abs(freq_samples[i].imag()) < t0) ? 255 : 0;	// as in MATLAB	
					*(received_bits_temp++) = (freq_samples[i].imag() > 0) ? 255 : 0;				
					*(received_bits_temp++) = (std::abs(freq_samples[i].real()) < t0) ? 255 : 0;
					*(received_bits_temp++) = (freq_samples[i].real() > 0) ? 255 : 0;									
				}
				break;
				
			case SENSOR_MINIMAL_QAM64_2_3:
			case SENSOR_MINIMAL_QAM64_3_4:
				t0 = 4/sqrt(42);
				t1 = 2/sqrt(42);
				t2 = 6/sqrt(42);
				for(int i=0; i<nConstSymbSendGross;i++)
				{
					float abso_r = std::abs(freq_samples[i].real());
					float abso_i = std::abs(freq_samples[i].imag());
					//*(received_bits_temp++) = (freq_samples[i].real() > 0) ? 255 : 0;				// as in IEEE 802.11a	
					//*(received_bits_temp++) = (abso_r < t0) ? 255 : 0;
					//*(received_bits_temp++) = ((abso_r > t1 && abso_r < t2) > 0) ? 255 : 0;
					//*(received_bits_temp++) = (freq_samples[i].imag() > 0) ? 255 : 0;
					//*(received_bits_temp++) = (abso_i < t0) ? 255 : 0;
					//*(received_bits_temp++) = ((abso_i > t1 && abso_i < t2) > 0) ? 255 : 0;
					*(received_bits_temp++) = ((abso_i > t1 && abso_i < t2) > 0) ? 255 : 0;			// as in MATLAB
					*(received_bits_temp++) = (abso_i < t0) ? 255 : 0;	
					*(received_bits_temp++) = (freq_samples[i].imag() > 0) ? 255 : 0;							
					*(received_bits_temp++) = ((abso_r > t1 && abso_r < t2) > 0) ? 255 : 0;	
					*(received_bits_temp++) = (abso_r < t0) ? 255 : 0;		
					*(received_bits_temp++) = (freq_samples[i].real() > 0) ? 255 : 0;		
				}
				break;
				
			default:
				PRINT("SENSOR_MIN_extract_mpdu error: Unknown encoding. Sending nonsense-data frame.");	
				return;			
		}
	}
	// soft decoding
	else if(d_hard_soft_dec == 1)
	{		
		float width, step, val;
		
		switch(d_encoding_family)
		{
			case SENSOR_MINIMAL_BPSK_1_2:
			case SENSOR_MINIMAL_BPSK_2_3:		
			case SENSOR_MINIMAL_BPSK_3_4:
				width = 1.0f;
				step = 2*width / 256.0f;
				for(int i=0; i<nConstSymbSendGross;i++)
				{
					val = (freq_samples[i].real() + width) / step;
					clipping(val);
					*(received_bits_temp++) = val;
				}
				break;
				
			case SENSOR_MINIMAL_QPSK_1_2:
			case SENSOR_MINIMAL_QPSK_2_3:	
			case SENSOR_MINIMAL_QPSK_3_4:
			case SENSOR_MINIMAL_QPSK_7_8:
				width = 1/sqrt(2);
				step = 2*width / 256.0f;
				for(int i=0; i<nConstSymbSendGross;i++)
				{
					val = (freq_samples[i].imag() + width) / step;
					clipping(val);
					*(received_bits_temp++) = val;
					
					val = (freq_samples[i].real() + width) / step;
					clipping(val);
					*(received_bits_temp++) = val;
				}
				break;
				
			case SENSOR_MINIMAL_QAM16_1_2:
			case SENSOR_MINIMAL_QAM16_3_4:
				t0 = 2/sqrt(10);
				step = t0 / 256.0f;
				for(int i=0; i<nConstSymbSendGross;i++)
				{
					val = (t0*1.5f - abs(freq_samples[i].imag())) / step;
					clipping(val);
					*(received_bits_temp++) = val;
					
					val = (t0*0.5f - freq_samples[i].imag()) / step;
					clipping(val);
					*(received_bits_temp++) = val;
					
					val = (t0*1.5f - abs(freq_samples[i].real())) / step;
					clipping(val);
					*(received_bits_temp++) = val;
					
					val = (t0*0.5f - freq_samples[i].real()) / step;
					clipping(val);
					*(received_bits_temp++) = val;							
				}
				break;
				
			case SENSOR_MINIMAL_QAM64_2_3:
			case SENSOR_MINIMAL_QAM64_3_4:
				t0 = 4/sqrt(42);
				t1 = 2/sqrt(42);
				t2 = 6/sqrt(42);
				step = t1 / 256.0f;				
				for(int i=0; i<nConstSymbSendGross;i++)
				{
					float abso_r = std::abs(freq_samples[i].real());
					float abso_i = std::abs(freq_samples[i].imag());
					float abso_r_tmp;
					float abso_i_tmp;
					
					// first bits
					if(abso_i <= t0)
					{
						val = (abso_i - t1*0.5f) / step;
						clipping(val);
						*(received_bits_temp++) = val;						
					}
					else
					{
						abso_i_tmp = abso_i - t0;
						val = (t1*1.5f - abso_i_tmp) / step;
						clipping(val);
						*(received_bits_temp++) = val;						
					}
					
					// second bit
					abso_i_tmp = abso_i - t1;
					val = (t1*1.5f - abso_i_tmp) / step;
					clipping(val);
					*(received_bits_temp++) = val;
					
					// third bit
					val = (t1*0.5f - freq_samples[i].imag()) / step;
					clipping(val);
					*(received_bits_temp++) = val;
					
					// fourth bits
					if(abso_r <= t0)
					{
						val = (abso_r - t1*0.5f) / step;
						clipping(val);
						*(received_bits_temp++) = val;						
					}
					else
					{
						abso_r_tmp = abso_r - t0;
						val = (t1*1.5f - abso_r_tmp) / step;
						clipping(val);
						*(received_bits_temp++) = val;						
					}
					
					// fifth bit
					abso_r_tmp = abso_r - t1;
					val = (t1*1.5f - abso_r_tmp) / step;
					clipping(val);
					*(received_bits_temp++) = val;
					
					// sixth bit
					val = (freq_samples[i].real() + t1*0.5f) / step;
					clipping(val);
					*(received_bits_temp++) = val;		
				}
				break;
				
			default:
				PRINT("SENSOR_MIN_extract_mpdu error: Unknown encoding. Sending nonsense-data frame.");	
				return;			
		}		
	}

	// deinterleave decoded bits
	if(d_interleaving_type == 0)
	{
		mempcpy(deinterleaved_bits, received_bits, nBitsSendGross);
	}
	else
	{
		int inter_scheme[nSymbMin];
		memset(inter_scheme, 0, sizeof(inter_scheme));
		
		switch(d_interleaving_type)
		{
			case 1:
				for(int i=0; i<nSymbMin; i++)
					inter_scheme[i] = i;
				break;
				
			case 2:
			{
				int dua_val = nSymbMin/2;
				
				for(int i=0; i<dua_val; i++)
				{
					inter_scheme[i*2] = i;
					inter_scheme[i*2+1] = i + dua_val;
				}
				
				if(nSymbMin % 2 == 1)
				{
					memmove(&inter_scheme[dua_val+1], &inter_scheme[dua_val], sizeof(int)*dua_val);
					inter_scheme[dua_val] = nSymbMin - 1;
				}
				
				break;
			}
			
			case 3:
				
				if(nSymbMin <= SENSOR_MIN_max_interleaver_3)
				{
					for(int i=0; i<nSymbMin; i++)
						inter_scheme[i] = SENSOR_MIN_interleaver_3_opt[nSymbMin-1][i];
				}
				else
				{
					PRINT("SENSOR_MIN_extract_mpdu decoder error: Packet length to big for interleaver type 3. Returning noninterleaved data.");
					for(int i=0; i<nSymbMin; i++)
						inter_scheme[i] = i;
				}
				
				break;			
				
			default:
				PRINT("SENSOR_MIN_extract_mpdu error: Undefined interleaving scheme. Returning un-deinterleaved data.");
				for(int i=0; i<nSymbMin; i++)
					inter_scheme[i] = i;
		}
		
		
		
		int deinter_scheme[nSymbMin];
		for(int i=0; i<nSymbMin; i++)
			deinter_scheme[inter_scheme[i]] = i;
		
		for(int i=0; i<d_channel_width*M; i++)
		{
			int offset = i*nSymbMin;
			for(int j=0; j<nSymbMin; j++)
				deinterleaved_bits[offset + j] = received_bits[i + deinter_scheme[j]*nBitsSymbGross];
		}	
	}	
	
	// depuncture
	int mod;
	unsigned char *depuntured_bits_temp = depunc_bits;
	switch(d_encoding_family) 
	{
		case SENSOR_MINIMAL_BPSK_1_2:
		case SENSOR_MINIMAL_QPSK_1_2:
		case SENSOR_MINIMAL_QAM16_1_2:
			memcpy(depuntured_bits_temp, deinterleaved_bits, nBitsSendGross);
			break;
			
		case SENSOR_MINIMAL_BPSK_2_3:  
		case SENSOR_MINIMAL_QPSK_2_3:  
		case SENSOR_MINIMAL_QAM64_2_3:
			for(int i = 0; i < nBitsSendGross; i++)
			{
				//if(i % 3 != 2)										// as in IEEE 802.11a
				//	*(depuntured_bits_temp++) = deinterleaved_bits[i];
				//else
				//{
				//	*(depuntured_bits_temp++) = deinterleaved_bits[i];
				//	*(depuntured_bits_temp++) = 127;
				//}
				if(i % 3 != 0)											// as in Matlab with 3/4 punc = [1; 0; 1; 1]
					*(depuntured_bits_temp++) = deinterleaved_bits[i];
				else
				{
					*(depuntured_bits_temp++) = deinterleaved_bits[i];
					*(depuntured_bits_temp++) = 127;
				}				
			}
			break;
			
		case SENSOR_MINIMAL_BPSK_3_4:	
		case SENSOR_MINIMAL_QPSK_3_4:	
		case SENSOR_MINIMAL_QAM16_3_4:
		case SENSOR_MINIMAL_QAM64_3_4:	
			for(int i = 0; i < nBitsSendGross; i++)
			{
				//if (!(i % 4 == 2))									// as in IEEE 802.11a
				//	*(depuntured_bits_temp++) = deinterleaved_bits[i];
				//else
				//{
				//	*(depuntured_bits_temp++) = deinterleaved_bits[i];
				//	*(depuntured_bits_temp++) = 127;
				//	*(depuntured_bits_temp++) = 127;
				//}
				mod = i % 4;
				if(!(mod == 1 || mod== 3))								// as in Matlab with 3/4 punc = [1,1,0,1,1,0]
					*(depuntured_bits_temp++) = deinterleaved_bits[i];
				else
				{
					*(depuntured_bits_temp++) = deinterleaved_bits[i];
					*(depuntured_bits_temp++) = 127;
				}			
			}
			break;
			
		case SENSOR_MINIMAL_QPSK_7_8:	
			for(int i = 0; i < nBitsSendGross; i++)
			{
				mod = i % 8;											// as in Matlab with 7/8 punc = [1; 1; 0; 1; 0; 1; 0; 1; 1; 0; 0; 1; 1; 0]
				if(mod == 1 || mod== 2 || mod== 3 || mod== 7)
				{
					*(depuntured_bits_temp++) = deinterleaved_bits[i];
					*(depuntured_bits_temp++) = 127;
				}
				else if(mod == 5)
				{
					*(depuntured_bits_temp++) = deinterleaved_bits[i];
					*(depuntured_bits_temp++) = 127;
					*(depuntured_bits_temp++) = 127;
				}
				else
					*(depuntured_bits_temp++) = deinterleaved_bits[i];
			}
			break;							
	}	
	
	// swap bits -> polynoms are swapped in matlab
	unsigned char swap_temp;
	for(int i=0; i<nBitsSendNetMax*2; i = i+2)
	{
		swap_temp = depunc_bits[i];
		depunc_bits[i] = depunc_bits[i+1];
		depunc_bits[i+1] = swap_temp;
	}

	// deconvolute
	unsigned char *data = NULL;
	unsigned char *freq_samples_demod = NULL;
	int framebits = nBitsSendNetMax - 6; 			// last six bits are tailbits, which is known a priori
	int tailbits = 6;
	
	if(posix_memalign((void**) &data, 16, (framebits+tailbits)/8+1))
	{
		printf("IEEE_decode_conv error: Allocation of data failed.\n");
		free(data);
		free(freq_samples_demod);
		return;
	}
	
	if(posix_memalign((void**) &freq_samples_demod, 16, 2*(framebits+tailbits)*sizeof(unsigned char)))
	{
		printf("IEEE_decode_conv error: Allocation of symbols failed.\n");
		free(data);
		free(freq_samples_demod);
		return;
	}

	memcpy(freq_samples_demod, depunc_bits, 2*(framebits+tailbits));
	
	spiral_viterbi_decoder(freq_samples_demod, data, framebits, tailbits);

	memcpy(scrambled_bytes, data, d_fixed_mpdu_len);
	
	free(data);
	free(freq_samples_demod);
	
	// descramble
	unsigned char scrambler_state = 127;
	int feedback;
	memcpy(mpdu, scrambled_bytes, d_fixed_mpdu_len);
	uint8_t *mpdu_temp = (uint8_t*) mpdu;
	for(int i=0; i<d_fixed_mpdu_len; i++)
	{
		for(int j = 0; j<8; j++) 
		{
			feedback = ((!!(scrambler_state & 64))) ^ (!!(scrambler_state & 8));
			scrambler_state = ((scrambler_state << 1)) | feedback;
		}
		mpdu_temp[i] = scrambler_state ^ mpdu_temp[i];
		mpdu[i] = SENSOR_MIN_BitReverseTable256[mpdu_temp[i]];
	}

	// FOR DEBUGGING
	/*	
	PRINT(" ");
	PRINT("CONSTELLATION SYMBOLS AT INPUT:");
	PRINT(d_freq_samples_total);
	PRINT(" ");
	for(int i=0; i<d_freq_samples_total; i++)
		std::cout << "i: " << i+1 << " " << freq_samples[i] << std::endl;
	*/	
	/*
	PRINT(" ");
	PRINT("ENCODING PROPERTIES:");
	PRINT(" ");	
	PRINT(M);
	PRINT(numerator);
	PRINT(denominator);
	PRINT(nSymbMin*d_channel_width); 
	PRINT(" ");
	PRINT(nBitsNetUser);
	PRINT(nBitsEncodedMin);
	PRINT(nBitsSymbGross);
	PRINT(nSymbMin);
	PRINT(" ");
	PRINT(nBitsSendGross);
	PRINT(nBitsSendNetMax);
	PRINT(nPadBitsBefScra);
	PRINT(nBitsSendCoded);
	PRINT(nPadBitsAftEncoder);
	*/
	/*
	PRINT(" ");
	PRINT("RECEIVED BITS:");
	PRINT(nBitsSendGross);
	PRINT(" ");
	for(int i=0; i<nBitsSendGross; i++)
		std::cout << "i: " << i+1 << " " << (int) received_bits[i] << std::endl;	
	*/
	/*
	PRINT(" ");
	PRINT("DEINTERLEAVED BITS:"); 
	PRINT(nBitsSendGross);
	PRINT(" ");
	for(int i=0; i<nBitsSendGross; i++)
		std::cout << "i: " << i+1 << " " << (int) deinterleaved_bits[i] << std::endl;		
	*/
	/*
	PRINT(" ");
	PRINT("DEPUNTURED BITS:");
	PRINT(nBitsSendNetMax*2);
	PRINT(" ");
	for(int i=0; i<nBitsSendNetMax*2; i++)
		std::cout << "i: " << i+1 << " " << (int) depunc_bits[i] << std::endl;
	*/	
	/*
	PRINT(" ");
	PRINT("DECODED BITS OR SCRAMBLED BITS:");
	PRINT(d_fixed_mpdu_len);
	PRINT("!!!!!!! WARNING: BITS ARE BYTE REVERSED -> But printed correctly !!!!!!!");
	PRINT(" ");
	int index = 1;
	for(int i=0; i<d_fixed_mpdu_len; i++)
	{
		for(int j=0; j<8; j++)
		{
			std::cout << "i: " << index++ << " " << (int) ((SENSOR_MIN_BitReverseTable256[scrambled_bytes[i]] >> j) & 1) << std::endl;				
		}
	}		
	*/
	/*
	PRINT(" ");
	PRINT("DESCRAMBLED BITS OR TRANSMITTED BITS:");
	PRINT(d_fixed_mpdu_len*8);
	PRINT(" ");
	int indexx = 1;
	for(int i=0; i<d_fixed_mpdu_len; i++)
	{
		for(int j=0; j<8; j++)
			std::cout << "i: " << indexx++ << " " << (int) ((mpdu[i] >> j) & 1) << std::endl;				
	}	
	*/
}
