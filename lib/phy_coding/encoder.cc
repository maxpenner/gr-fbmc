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
 
#include <vector>
#include <cassert>
#include <algorithm>
#include <complex>
#include <stdlib.h>
#include <boost/crc.hpp>
#include <string.h>

#include "../utils/debug.h"
#include "../utils/encodings.h"

#include "encoder.h"
#include "ieee/ieee.h"

// general module variables
static enum {IEEE, SENSOR_MIN} d_MODULE_STATE;
static struct metadataMpdu *d_metadata;
static int d_channel_width;	

// IEEE	PHY-encoding	
static int IEEE_get_ppdu_len();
static void IEEE_generate_ppdu(char* ppdu, const char* mpdu);

// SENSOR_MIN PHY-encoding
// SENSOR_MIN PHY-encoding
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
static int SENSOR_MIN_get_ppdu_len();
static void SENSOR_MIN_generate_ppdu(char* ppdu, const char* mpdu);

namespace FBMC_PHY
{
	//______________________________________________________
	void load_encoder(struct metadataMpdu *metadata, int channel_width)
	{
		d_metadata = metadata;
		d_channel_width = channel_width;
		
		switch(d_metadata->encoding)
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
				PRINT("load_encoder PHY stage encoder error: Unknown encoding.");								
		}
	}
	
	//______________________________________________________
	int get_ppdu_len()
	{
		int ppdu_len = 0;
		
		switch(d_MODULE_STATE)
		{
			case IEEE:
				ppdu_len = IEEE_get_ppdu_len();
				break;

			case SENSOR_MIN:
				ppdu_len = SENSOR_MIN_get_ppdu_len();
				break;				
		}
		
		return ppdu_len;
	}
	
	//______________________________________________________
	void generate_ppdu(char* ppdu, const char* mpdu)
	{
		switch(d_MODULE_STATE)
		{
			case IEEE:
				IEEE_generate_ppdu(ppdu, mpdu);
				break;
				
			case SENSOR_MIN:
				SENSOR_MIN_generate_ppdu(ppdu, mpdu);
				break;				
		}
	}
}

// IEEE PHY-encoding	
//______________________________________________________
static int IEEE_get_ppdu_len()
{
	IEEE_ofdm_param ofdm(d_metadata->encoding);
	IEEE_tx_param tx(ofdm, d_metadata->mpdu_len);

	return 48 + tx.n_encoded_bits / ofdm.n_bpsc;
}

//______________________________________________________
static void IEEE_generate_ppdu(char* ppdu, const char* mpdu)
{
	IEEE_ofdm_param ofdm(d_metadata->encoding);
	IEEE_tx_param tx(ofdm, d_metadata->mpdu_len);

	char *bits             = (char*) calloc(tx.n_data, sizeof(char));
	char *scrambled_bits   = (char*) calloc(tx.n_data, sizeof(char));
	char *encoded_bits     = (char*) calloc(tx.n_data*2, sizeof(char));
	char *punctured_bits   = (char*) calloc(tx.n_encoded_bits, sizeof(char));
	char *interleaved_bits = (char*) calloc(tx.n_encoded_bits, sizeof(char));

	IEEE_generate_bits(mpdu, bits, tx);
	
	// A: fixed value scrambler
	//IEEE_scramble(bits, scrambled_bits, tx, SCRAMBLER_START_STATE);
	
	// B: random value scrambler (should never be 0 -> high correlation in time domain)
	IEEE_scramble(bits, scrambled_bits, tx, rand() % 127 + 1);
	
	IEEE_reset_tail_bits(scrambled_bits, tx);
	IEEE_convolutional_encoding(scrambled_bits, encoded_bits, tx);
	IEEE_puncturing(encoded_bits, punctured_bits, tx, ofdm);
	IEEE_interleave(punctured_bits, interleaved_bits, tx, ofdm);
	IEEE_split_symbols(interleaved_bits, ppdu + 48, tx, ofdm);	

	free(bits);
	free(scrambled_bits);
	free(encoded_bits);
	free(punctured_bits);
	free(interleaved_bits);

	IEEE_insert_plcp_header(ppdu, ofdm.rate_field, d_metadata->mpdu_len);

	// shift symbols of plcp header so they are BPSK modulated at next block
	int offset = pow(2, d_metadata->encoding - d_metadata->encoding%2);
	if(offset != 1)
	{
		for(int i=0; i<48; i++)
			ppdu[i] += offset;
	}
}

// SENSOR_MIN PHY-encoding	
//______________________________________________________
static int SENSOR_MIN_get_ppdu_len()
{
	int M;
	int numerator;
	int denominator;
	
	switch(d_metadata->encoding)
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
	}
	
	//int nBitsNetUser = d_metadata->mpdu_len*8;									// number of Bits to send from application layer
	int nApplicationBits = d_metadata->mpdu_len*8;									// number of Bits to send from application layer
	int nBitsNetUser = nApplicationBits + 6;										// 6 trailing zero bits for viterbi	
	
	int nBitsEncodedMin = (nBitsNetUser + numerator - 1)/numerator;					// minimum number of bits that have to be encoded to fit nBitsNetUser-many bits before encoder
	nBitsEncodedMin *= denominator;													// "
	int nBitsSymbGross = M*d_channel_width;											// number of bits that fit in one symbol
	int nSymbMin = (nBitsEncodedMin + nBitsSymbGross - 1)/nBitsSymbGross;			// number of symbols needed to fit nBitsNetUser-many bits
	
	return nSymbMin*d_channel_width;
}

//______________________________________________________
static void SENSOR_MIN_generate_ppdu(char* ppdu, const char* mpdu)
{
	int M;
	int numerator;
	int denominator;
	
	switch(d_metadata->encoding)
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
			PRINT("SENSOR_MIN_generate_ppdu error: Unknown encoding. Sending nonsense-data frame.");	
			return;		
	}
	
	// first calculate minimal number of symbols
	//int nBitsNetUser = d_metadata->mpdu_len*8;									// number of Bits to send from application layer
	int nApplicationBits = d_metadata->mpdu_len*8;									// number of Bits to send from application layer
	int nBitsNetUser = nApplicationBits + 6;										// 6 trailing zero bits for viterbi	
	
	int nBitsEncodedMin = (nBitsNetUser + numerator - 1)/numerator;					// minimum number of bits that have to be encoded to fit nBitsNetUser-many bits before encoder
	nBitsEncodedMin *= denominator;													// "
	int nBitsSymbGross = M*d_channel_width;											// number of bits that fit in one symbol
	int nSymbMin = (nBitsEncodedMin + nBitsSymbGross - 1)/nBitsSymbGross;			// number of symbols needed to fit nBitsNetUser-many bits
	
	// now calculate how many bits we need in each stage in a minimal frame
	int nBitsSendGross = M*nSymbMin*d_channel_width;								// total number of bits that will be on air
	int nBitsSendNetMax = nBitsSendGross/denominator;								// number of bits that have to be inputted to encoder
	nBitsSendNetMax *= numerator;													// "				
	int nPadBitsBefScra = nBitsSendNetMax - nBitsNetUser;							// number of padding bits before scrambler and encoder
	int nBitsSendCoded = nBitsSendNetMax/numerator;									// number of bits that encoder will output
	nBitsSendCoded *= denominator;													// "		
	int nPadBitsAftEncoder = nBitsSendGross - nBitsSendCoded;						// number of padding bits after the encoder (will be scrambled)
	
	// security mechanism
	if(nBitsSendGross/M != nSymbMin*d_channel_width)
	{
		PRINT("SENSOR_MIN_generate_ppdu error: Security mechanism failed. Sending nonsense-data frame.");
		return;
	}
	
	// allocate memory
	char bits[nBitsSendNetMax];
	char scrambled_bits[nBitsSendNetMax];
	char encoded_bits[nBitsSendNetMax*2];
	char punctured_bits[nBitsSendCoded];
	char padded_bits[nBitsSendGross];
	char interleaved_bits[nBitsSendGross];
	
	// set bits to zero
	memset(bits, 0x00, nBitsSendNetMax);

	// generate bits
	char *bits_temp = bits;
	for(int i=0; i<d_metadata->mpdu_len; i++)
	{
		for(int b = 0; b < 8; b++)
			*(bits_temp++) = (mpdu[i] >> b) & 1;
	}			
	
	// scramble
	int scrambler_state = 127;
	int feedback;
	char *scrambled_bits_temp = scrambled_bits;
	for(int i = 0; i < nBitsSendNetMax; i++)
	{
		feedback = (!!(scrambler_state & 64)) ^ (!!(scrambler_state & 8));
		*(scrambled_bits_temp++) = feedback ^ bits[i];
		scrambler_state = ((scrambler_state << 1) & 0x7e) | feedback;
	}
	
	// WITH 6 TRAILING ZERO BITS: reset zero bits
	memset(&scrambled_bits[nBitsSendNetMax-6], 0x00, 6);	
	
	// convolute bits
	int state = 0;
	char *encoded_bits_temp = encoded_bits;
	for(int i = 0; i < nBitsSendNetMax; i++)
	{
		assert(bits[i] == 0 || scrambled_bits[i] == 1);
		state = ((state << 1) & 0x7e) | scrambled_bits[i];
		//*(encoded_bits_temp++) = IEEE_ones(state & 0155) % 2;	// 0155 octal, bits swapped makes 0133	// as in IEEE 802.11a
		//*(encoded_bits_temp++) = IEEE_ones(state & 0117) % 2;	// 0117 octal, bits swapped makes 0171
		*(encoded_bits_temp++) = IEEE_ones(state & 0117) % 2;	// 0117 octal, bits swapped makes 0171	// as in MATLAB
		*(encoded_bits_temp++) = IEEE_ones(state & 0155) % 2;	// 0155 octal, bits swapped makes 0133	
	}				
	
	// puncture bits
	int mod;
	char *punctured_bits_temp = punctured_bits;
	switch(d_metadata->encoding)
	{
		case SENSOR_MINIMAL_BPSK_1_2:
		case SENSOR_MINIMAL_QPSK_1_2:
		case SENSOR_MINIMAL_QAM16_1_2:
			for(int i = 0; i < nBitsSendNetMax*2; i++)
				*(punctured_bits_temp++) = encoded_bits[i];				// IEEE 802.11a and Matlab are the same
			break;
			
		case SENSOR_MINIMAL_BPSK_2_3:
		case SENSOR_MINIMAL_QPSK_2_3:
		case SENSOR_MINIMAL_QAM64_2_3:
			for(int i = 0; i < nBitsSendNetMax*2; i++)
			{
				//if (i % 4 != 3)										// as in IEEE 802.11a
				//	*(punctured_bits_temp++) = encoded_bits[i];			
				if (i % 4 != 1)											// as in Matlab with 3/4 punc = [1; 0; 1; 1]
					*(punctured_bits_temp++) = encoded_bits[i];						
			}
			break;
			
		case SENSOR_MINIMAL_BPSK_3_4:
		case SENSOR_MINIMAL_QPSK_3_4:
		case SENSOR_MINIMAL_QAM16_3_4:
		case SENSOR_MINIMAL_QAM64_3_4:
			for(int i = 0; i < nBitsSendNetMax*2; i++)
			{
				mod = i % 6;
				//if (!(mod == 3 || mod == 4))							// as in IEEE 802.11a
				//	*(punctured_bits_temp++) = encoded_bits[i];	
				if (!(mod == 2 || mod == 5))							// as in Matlab with 3/4 punc = [1,1,0,1,1,0]
					*(punctured_bits_temp++) = encoded_bits[i];			
			}	
			break;
			
		case SENSOR_MINIMAL_QPSK_7_8:
			for(int i = 0; i < nBitsSendNetMax*2; i++)
			{
				mod = i % 14;											// as in Matlab with 7/8 punc = [1; 1; 0; 1; 0; 1; 0; 1; 1; 0; 0; 1; 1; 0]
				if (!(mod == 2 || mod == 4 || mod == 6 || mod == 9 || mod == 10 || mod == 13))	
					*(punctured_bits_temp++) = encoded_bits[i];			
			}	
			break;			
			
		default:
			PRINT("SENSOR_MIN_generate_ppdu error: Unknown encoding. Sending nonsense-data frame.");	
			return;
	}				
				
	// append scramble pad bits after encoder to fill entire frame
	memcpy(padded_bits, punctured_bits, nBitsSendCoded);
	scrambler_state = 127;
	for(int i = nBitsSendCoded; i < nBitsSendGross; i++)
	{
		feedback = (!!(scrambler_state & 64)) ^ (!!(scrambler_state & 8));
		padded_bits[i] = feedback ^ 0;
		scrambler_state = ((scrambler_state << 1) & 0x7e) | feedback;
	}				
	
	// interleave
	if(d_metadata->interl_type == 0)
	{
		memcpy(interleaved_bits, padded_bits, nBitsSendGross);
	}
	else
	{
		int inter_scheme[nSymbMin];
		switch(d_metadata->interl_type)
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
					PRINT("SENSOR_MIN_extract_mpdu encoder error: Packet length to big for interleaver type 3. Returning noninterleaved data.");
					for(int i=0; i<nSymbMin; i++)
						inter_scheme[i] = i;
				}
				
				break;
				
			default:
				PRINT("SENSOR_MIN_extract_mpdu error: Undefined interleaving scheme. Returning un-deinterleaved data.");
				for(int i=0; i<nSymbMin; i++)
					inter_scheme[i] = i;
		}
		
		for(int i=0; i<d_channel_width*M; i++)
		{
			int offset = i*nSymbMin;
			for(int j=0; j<nSymbMin; j++)
				interleaved_bits[i + j*nBitsSymbGross] = padded_bits[offset + inter_scheme[j]];
		}
	}
	
	// map on symbols
	char *interleaved_bits_temp = interleaved_bits;
	for (int i=0; i < nSymbMin*d_channel_width; i++)
	{
		ppdu[i] = 0;
		for(int k=0; k < M; k++)
		{
			assert(*interleaved_bits_temp == 1 || *interleaved_bits_temp == 0);
			//ppdu[i] |= (*interleaved_bits_temp << (M - k - 1));		// as in IEEE 802.11a
			ppdu[i] |= (*interleaved_bits_temp << k);					// as in Matlab
			interleaved_bits_temp++;
		}
	}		

	// FOR DEBUGGING
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
	PRINT("GENERATED BITS WITH PADDING BITS:");
	PRINT(d_metadata->mpdu_len*8);
	PRINT(nBitsSendNetMax);
	PRINT(" ");
	for(int i=0; i<nBitsSendNetMax; i++)
		std::cout << "i: " << i+1 << " " << (int) bits[i] << std::endl;
	*/
	/*
	PRINT(" ");
	PRINT("SCRAMBLED BITS:");
	PRINT(nBitsSendNetMax);
	PRINT(" ");
	for(int i=0; i<nBitsSendNetMax; i++)
		std::cout << "i: " << i+1 << " " << (int) scrambled_bits[i] << std::endl;	
	*/
	/*
	PRINT(" ");
	PRINT("ENCODED BITS BEFORE PUNCTURING:");
	PRINT(nBitsSendNetMax*2);
	PRINT(" ");
	for(int i=0; i<nBitsSendNetMax*2; i++)
		std::cout << "i: " << i+1 << " " << (int) encoded_bits[i] << std::endl;		
	*/
	/*
	PRINT(" ");
	PRINT("PUNCTURED BITS:");
	PRINT(nBitsSendCoded);
	PRINT(" ");
	for(int i=0; i<nBitsSendCoded; i++)
		std::cout << "i: " << i+1 << " " << (int) punctured_bits[i] << std::endl;
	*/
	/*
	PRINT(" ");
	PRINT("BITS AFTER SECOND PADDING STAGE AND BEFORE INTERLEAVER:");
	PRINT(nBitsSendGross);
	PRINT(" ");
	for(int i=0; i<nBitsSendGross; i++)
		std::cout << "i: " << i+1 << " " << (int) padded_bits[i] << std::endl;			
	*/
	/*
	PRINT(" ");
	PRINT("BITS AFTER INTERLEAVER:");
	PRINT(nBitsSendGross);
	PRINT(" ");
	for(int i=0; i<nBitsSendGross; i++)
		std::cout << "i: " << i+1 << " " << (int) interleaved_bits[i] << std::endl;			
	*/
	/*
	PRINT(" ");
	PRINT("BITS MAPPING ON INDEX VALUE:");
	PRINT(nSymbMin*d_channel_width);
	PRINT(" ");
	for(int i=0; i<nSymbMin*d_channel_width; i++)
		std::cout << "i: " << i+1 << " " << (int) ppdu[i] << std::endl;			
	*/

}
