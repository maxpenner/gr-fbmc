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
 
#include <assert.h>
#include <string.h>
#include <math.h>
#include <algorithm>
 
#include "../../utils/debug.h"
#include "../../utils/encodings.h"

#include "ieee.h"
#include "spiral_viterbi_interface.h"

//_________________________________________________
static const unsigned char BitReverseTable256[256] = 
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

// #################################################################
// #################################################################
// #################################################################
 
//_________________________________________________
IEEE_ofdm_param::IEEE_ofdm_param(int encoding)
{
	enc = encoding;

	switch(enc)
	{
		case IEEE_BPSK_1_2:
			n_bpsc = 1;
			n_cbps = 48;
			n_dbps = 24;
			rate_field = 0x0D; // 0b00001101
			break;
		case IEEE_BPSK_3_4:
			n_bpsc = 1;
			n_cbps = 48;
			n_dbps = 36;
			rate_field = 0x0F; // 0b00001111
			break;
		case IEEE_QPSK_1_2:
			n_bpsc = 2;
			n_cbps = 96;
			n_dbps = 48;
			rate_field = 0x05; // 0b00000101
			break;
		case IEEE_QPSK_3_4:
			n_bpsc = 2;
			n_cbps = 96;
			n_dbps = 72;
			rate_field = 0x07; // 0b00000111
			break;
		case IEEE_QAM16_1_2:
			n_bpsc = 4;
			n_cbps = 192;
			n_dbps = 96;
			rate_field = 0x09; // 0b00001001
			break;
		case IEEE_QAM16_3_4:
			n_bpsc = 4;
			n_cbps = 192;
			n_dbps = 144;
			rate_field = 0x0B; // 0b00001011
			break;
		case IEEE_QAM64_2_3:
			n_bpsc = 6;
			n_cbps = 288;
			n_dbps = 192;
			rate_field = 0x01; // 0b00000001
			break;
		case IEEE_QAM64_3_4:
			n_bpsc = 6;
			n_cbps = 288;
			n_dbps = 216;
			rate_field = 0x03; // 0b00000011
			break;
		default:
			assert(false);
			break;
	}
}

//_________________________________________________
IEEE_tx_param::IEEE_tx_param(IEEE_ofdm_param ofdm, int mpdu_len_arg)
{
	// inverse to fragmentation: 
	// mpdu_len is definitly smaller than the maximum number of bytes in a frame.
	mpdu_len = mpdu_len_arg;
	n_sym = (int) ceil((16 + 8 * mpdu_len + 6) / (double) ofdm.n_dbps);
	n_data = n_sym * ofdm.n_dbps;
	n_pad = n_data - (16 + 8 * mpdu_len + 6);
	n_encoded_bits = n_sym * ofdm.n_cbps;
} 

// #################################################################
// #################################################################
// #################################################################

//_________________________________________________
void IEEE_generate_bits(const char *mpdu, char *bits, IEEE_tx_param &tx)
{
	// first 16 bits are zero (SERVICE field)
	memset(bits, 0x00, 16);
	bits += 16;

	for(int i=0; i<tx.mpdu_len; i++)
	{
		for(int b = 0; b < 8; b++)
			*(bits++) = (mpdu[i] >> b) & 1;
	}
}

//_________________________________________________
void IEEE_scramble(const char *bits, char *scrambled_bits, IEEE_tx_param &tx, char initial_state)
{
	int state = initial_state;
	int feedback;

	for(int i = 0; i < tx.n_data; i++)
	{
		feedback = (!!(state & 64)) ^ (!!(state & 8));
		*(scrambled_bits++) = feedback ^ bits[i];
		state = ((state << 1) & 0x7e) | feedback;
	}
}

//_________________________________________________
void IEEE_reset_tail_bits(char *scrambled_bits, IEEE_tx_param &tx)
{
	// reset padding bits -> trailing zeros for viterbi decoder
	memset(scrambled_bits + tx.n_data - 6, 0x00, 6 * sizeof(char));
}

//_________________________________________________
int IEEE_ones(int n) 
{
	int sum = 0;
	for(int i = 0; i < 8; i++)
	{
		if(n & (1 << i))
			sum++;
	}
	return sum;
}

//_________________________________________________
void IEEE_convolutional_encoding(const char *scrambled_bits, char *encoded_bits, IEEE_tx_param &tx)
{
	int state = 0;

	for(int i = 0; i < tx.n_data; i++)
	{
		assert(scrambled_bits[i] == 0 || scrambled_bits[i] == 1);
		state = ((state << 1) & 0x7e) | scrambled_bits[i];
		*(encoded_bits++) = IEEE_ones(state & 0155) % 2;	// 0155 octal, bits swapped makes 0133
		*(encoded_bits++) = IEEE_ones(state & 0117) % 2;	// 0117 octal, bits swapped makes 0171
	}
}

//_________________________________________________
void IEEE_puncturing(const char *encoded_bits, char *punctured_bits, IEEE_tx_param &tx, IEEE_ofdm_param &ofdm)
{
	int mod;
	
	switch(ofdm.enc)
	{
		case IEEE_BPSK_1_2:
		case IEEE_QPSK_1_2:
		case IEEE_QAM16_1_2:
			for(int i = 0; i < tx.n_data * 2; i++)
				*(punctured_bits++) = encoded_bits[i];
			break;
			
		case IEEE_QAM64_2_3:
			for(int i = 0; i < tx.n_data * 2; i++)
			{
				if (i % 4 != 3)
					*(punctured_bits++) = encoded_bits[i];
			}
			break;
			
		case IEEE_BPSK_3_4:
		case IEEE_QPSK_3_4:
		case IEEE_QAM16_3_4:
		case IEEE_QAM64_3_4:
			for(int i = 0; i < tx.n_data * 2; i++)
			{
				mod = i % 6;
				if (!(mod == 3 || mod == 4))
					*(punctured_bits++) = encoded_bits[i];
			}	
			break;
			
		defaut:
		
			assert(false);
			break;
	}
}

//_________________________________________________
void IEEE_interleave(const char *punctered_bits, char *interleaved_bits, IEEE_tx_param &tx, IEEE_ofdm_param &ofdm, bool reverse)
{
	int n_cbps = ofdm.n_cbps;
	int first[n_cbps];
	int second[n_cbps];
	int s = std::max(ofdm.n_bpsc / 2, 1);

	for(int j = 0; j < n_cbps; j++)
		first[j] = s * (j / s) + ((j + int(floor(16.0 * j / n_cbps))) % s);

	for(int i = 0; i < n_cbps; i++)
		second[i] = 16 * i - (n_cbps - 1) * int(floor(16.0 * i / n_cbps));

	for(int i = 0; i < tx.n_sym; i++)
	{
		for(int k = 0; k < n_cbps; k++)
		{
			if(reverse)
				interleaved_bits[i * n_cbps + second[first[k]]] = punctered_bits[i * n_cbps + k];
			else
				interleaved_bits[i * n_cbps + k] = punctered_bits[i * n_cbps + second[first[k]]];
		}
	}
}

//_________________________________________________
void IEEE_split_symbols(const char *interleaved_bits, char *bit_symbols, IEEE_tx_param &tx, IEEE_ofdm_param &ofdm)
{
	for (int i=0; i < tx.n_sym * 48; i++)
	{
		bit_symbols[i] = 0;
		for(int k=0; k < ofdm.n_bpsc; k++)
		{
			assert(*interleaved_bits == 1 || *interleaved_bits == 0);
			bit_symbols[i] |= (*interleaved_bits << (ofdm.n_bpsc - k - 1));
			interleaved_bits++;
		}
	}
}

//_________________________________________________
void IEEE_insert_plcp_header(char *output_position, char rate_field, int mpdu_len)
{
	// data_bits
	char data_bits[24];
	memset(data_bits, 0x00, 24);

	// set rate field
	for(int i=0; i<4; i++)
		data_bits[i] = (rate_field >> i) & 1;

	// set length field
	for(int i=0; i<12; i++)
		data_bits[5+i] = (mpdu_len >> i) & 1;

	// set parity bit
	int sum = 0;	
	for(int i=0; i<17; i++)
		sum += (int) data_bits[i];
	data_bits[17] = ((sum%2) & 1);
	
	// encoding parameter
	IEEE_ofdm_param ofdm(IEEE_BPSK_1_2);
	IEEE_tx_param tx(ofdm, 24/8);
	tx.n_sym  = 1;
	tx.n_data = 24;
	tx.n_pad  = 0;
	tx.n_encoded_bits = 48;

	char encoded_data[48];
	char interleaved_data[48];

	// encode and write to output
	IEEE_convolutional_encoding(data_bits, encoded_data, tx);
	IEEE_interleave(encoded_data, interleaved_data, tx, ofdm);
	IEEE_split_symbols(interleaved_data, output_position, tx, ofdm);
}

// #################################################################
// #################################################################
// #################################################################

//_________________________________________________
void IEEE_plcp_deinterleave(float *freq_samples_real)
{
	float temp[48];
	
	int inter[48] = 
	{ 
		0, 3, 6, 9,12,15,18,21,
		24,27,30,33,36,39,42,45,
		1, 4, 7,10,13,16,19,22,
		25,28,31,34,37,40,43,46,
		2, 5, 8,11,14,17,20,23,
		26,29,32,35,38,41,44,47
	};
	
	for(int i = 0; i < 48; i++)
		temp[i] = freq_samples_real[inter[i]];
		
	memcpy(freq_samples_real, temp, 48*sizeof(float));
};

//_________________________________________________
bool IEEE_plcp_decode(float *freq_samples_real_deinterleaved, int &rate_field, int &mpdu_len)
{
	int plcp_bits = 18;
	int plcp_tail = 6;
	
	unsigned char *data_bytes = NULL;
	unsigned char *freq_samples_demod = NULL;
	
	if(posix_memalign((void**) &data_bytes, 16, (plcp_bits + plcp_tail)/8 + 1))
	{
		free(data_bytes);
		free(freq_samples_demod);
		PRINT("IEEE_spiral_plcp_decode error: Allocation of data failed.\n");
		return false;
	}
	
	if(posix_memalign((void**) &freq_samples_demod, 16, 2*(plcp_bits + plcp_tail)*sizeof(unsigned char)))
	{
		free(data_bytes);
		free(freq_samples_demod);
		PRINT("IEEE_spiral_plcp_decode error: Allocation of symbols failed.\n");
		return false;
	}

	// hard decision for BPSK symbols
	for(int i=0; i<48; i++)
		freq_samples_demod[i] = (freq_samples_real_deinterleaved[i] > 0) ? 255 : 0;

	// decode
	spiral_viterbi_decoder(freq_samples_demod, data_bytes, plcp_bits, plcp_tail);

	rate_field 	= 0;
	mpdu_len  	= 0;
	bool parity	;

	// extract rate
	for(int i=0; i<4; i++)
		rate_field |= ((data_bytes[0] >> (7-i)) & 1) << i;

	// extract mpdu length
	for(int i=0; i<3; i++)
		mpdu_len |= ((data_bytes[0] >> (2-i)) & 1)  << i;
	for(int i=3; i<11; i++)
		mpdu_len |= ((data_bytes[1] >> (10-i)) & 1) << i;
	for(int i=11; i<12; i++)
		mpdu_len |= ((data_bytes[2] >> (18-i)) & 1) << i;

	// extract parity bit
	int sum = 0;	
	for(int i=0; i<4; i++)
		sum += (rate_field >> i) & 1;
	for(int i=0; i<12; i++)
		sum += (mpdu_len >> i) & 1;
	parity = (bool) (sum % 2);
	
	// check parity
	if(parity != (bool) ((data_bytes[2] >> 6) & 1))
		return false;
	
	// clean up
	free(data_bytes);
	free(freq_samples_demod);
	
	return true;
};

//_________________________________________________
void IEEE_demod_hard(const std::complex<float> *received_symbols_comp, unsigned char *hard_bits, IEEE_tx_param &tx, IEEE_ofdm_param &ofdm)
{
	float t0, t1, t2, t3;

	switch(ofdm.enc)
	{
		case IEEE_BPSK_1_2:
		case IEEE_BPSK_3_4:
			for(int i=0; i<tx.n_sym*48;i++)
				*(hard_bits++) = (received_symbols_comp[i].real() > 0) ? 255 : 0;		
			break;
			
		case IEEE_QPSK_1_2:
		case IEEE_QPSK_3_4:
			for(int i=0; i<tx.n_sym*48;i++)
			{
				*(hard_bits++) = (received_symbols_comp[i].real() > 0) ? 255 : 0;
				*(hard_bits++) = (received_symbols_comp[i].imag() > 0) ? 255 : 0;		
			}
			break;
			
		case IEEE_QAM16_1_2:
		case IEEE_QAM16_3_4:
			t0 = 2/sqrt(10);
			for(int i=0; i<tx.n_sym*48;i++)
			{
				*(hard_bits++) = (received_symbols_comp[i].real() > 0) ? 255 : 0;
				*(hard_bits++) = (std::abs(received_symbols_comp[i].real()) < t0) ? 255 : 0;
				*(hard_bits++) = (received_symbols_comp[i].imag() > 0) ? 255 : 0;
				*(hard_bits++) = (std::abs(received_symbols_comp[i].imag()) < t0) ? 255 : 0;		
			}
			break;
			
		case IEEE_QAM64_3_4:
		case IEEE_QAM64_2_3:
			t0 = 4/sqrt(42);
			t1 = 2/sqrt(42);
			t2 = 6/sqrt(42);
			for(int i=0; i<tx.n_sym*48;i++)
			{
				float abso_r = std::abs(received_symbols_comp[i].real());
				float abso_i = std::abs(received_symbols_comp[i].imag());
				*(hard_bits++) = (received_symbols_comp[i].real() > 0) ? 255 : 0;
				*(hard_bits++) = (abso_r < t0) ? 255 : 0;
				*(hard_bits++) = ((abso_r > t1 && abso_r < t2) > 0) ? 255 : 0;
				*(hard_bits++) = (received_symbols_comp[i].imag() > 0) ? 255 : 0;
				*(hard_bits++) = (abso_i < t0) ? 255 : 0;
				*(hard_bits++) = ((abso_i > t1 && abso_i < t2) > 0) ? 255 : 0;		
			}
			break;
	}
};

//_________________________________________________
void IEEE_deinterleave(unsigned char *hard_bits, unsigned char *deinterleaved_bits, IEEE_tx_param &tx, IEEE_ofdm_param &ofdm)
{
	int n_cbps = ofdm.n_cbps;
	int first[n_cbps];
	int second[n_cbps];
	int s = std::max(ofdm.n_bpsc / 2, 1);

	for(int j = 0; j < n_cbps; j++)
		first[j] = s * (j / s) + ((j + int(floor(16.0 * j / n_cbps))) % s);
		
	for(int i = 0; i < n_cbps; i++)
		second[i] = 16 * i - (n_cbps - 1) * int(floor(16.0 * i / n_cbps));
		
	for(int i = 0; i < tx.n_sym; i++)
	{
		for(int k = 0; k < n_cbps; k++)
			deinterleaved_bits[i * n_cbps + second[first[k]]] = hard_bits[i * n_cbps + k];
	}
};

//_________________________________________________
void IEEE_depuncture(unsigned char *deinterleaved_bits, unsigned char *depunctured_bits, IEEE_tx_param &tx, IEEE_ofdm_param &ofdm)
{
	switch(ofdm.enc) 
	{
		case IEEE_BPSK_1_2:
		case IEEE_QPSK_1_2:
		case IEEE_QAM16_1_2:
			memcpy(depunctured_bits, deinterleaved_bits, tx.n_encoded_bits);
			break;
			
		case IEEE_QAM64_2_3:  
			for(int i = 0; i < tx.n_encoded_bits; i++)
			{
				if(i % 3 != 2)
					*(depunctured_bits++) = deinterleaved_bits[i];
				else
				{
					*(depunctured_bits++) = deinterleaved_bits[i];
					*(depunctured_bits++) = 127;
				}
			}
			break;
			
		case IEEE_BPSK_3_4:	
		case IEEE_QPSK_3_4:	
		case IEEE_QAM16_3_4:
		case IEEE_QAM64_3_4:	
			for(int i = 0; i < tx.n_encoded_bits; i++)
			{
				if (!(i % 4 == 2))
					*(depunctured_bits++) = deinterleaved_bits[i];
				else
				{
					*(depunctured_bits++) = deinterleaved_bits[i];
					*(depunctured_bits++) = 127;
					*(depunctured_bits++) = 127;
				}
			}
			break;
	}
};

//_________________________________________________
void IEEE_decode_conv(unsigned char *depunctured_bits, unsigned char *descrambled_bytes, IEEE_tx_param &tx, IEEE_ofdm_param &ofdm)
{
	unsigned char *data = NULL;
	unsigned char *freq_samples_demod = NULL;

	// Service field + mpdu + pad_bits + 6 tail-bits
	int framebits = 16 + tx.mpdu_len*8 + tx.n_pad; 
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

	// copy depunctured bits into aligned symbols buffer
	memcpy(freq_samples_demod, depunctured_bits, 2*(framebits+tailbits));
	
	// decode
	spiral_viterbi_decoder(freq_samples_demod, data, framebits, tailbits);

	// make decoded data available in scrambled_bytes
	memcpy(descrambled_bytes, data, tx.mpdu_len + 3);	 	// + 3 for 16 bits service field + mpdu + 6 tailbits
	
	// clean up
	free(data);
	free(freq_samples_demod);
};

//_________________________________________________
void IEEE_descramble(unsigned char *descrambled_bytes, IEEE_tx_param &tx, IEEE_ofdm_param &ofdm)
{
	// extract 'Scrambler Initialization' from Service Field
	unsigned char state = 0;
	for(int i = 0; i < 7; i++)
	{
		if((descrambled_bytes[0] >> (7-i)) & 1)
			state |= 1 << (6 - i);
	}

	unsigned char feedback;
	unsigned char temp = 0;

	// set the msb of the first byte of the service field
	feedback = ((!!(state & 64))) ^ (!!(state & 8));
	state = (state << 1) | feedback;
	
	// iterator over all relevant bytes
	for(int i=1; i<tx.mpdu_len+2; i++)		// + 2 for service field at the beginning
	{
		for(int j = 0; j<8; j++) 
		{
			feedback = ((!!(state & 64))) ^ (!!(state & 8));
			state = (state << 1) | feedback;
		}
		descrambled_bytes[i] = state ^ descrambled_bytes[i];
		descrambled_bytes[i] = BitReverseTable256[descrambled_bytes[i]];
	}
};
