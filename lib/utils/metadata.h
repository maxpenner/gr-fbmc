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

#ifndef INCLUDED_FBMC_UTILS_METADATA_H
#define INCLUDED_FBMC_UTILS_METADATA_H

struct metadataPayload
{
	uint16_t payload_len;
	uint16_t log_chann;
	uint16_t spf;
	uint8_t encoding;
	uint16_t sender;
	uint16_t seq_no;
	uint64_t airtime;
	float scale;
	uint16_t interl_type;
	bool altern_pream;
};

struct metadataMpdu
{
	uint16_t mpdu_len;
	uint16_t log_chann;
	uint16_t spf;
	uint8_t encoding;
	uint64_t airtime;
	float scale;
	uint16_t interl_type;
	bool altern_pream;
};

struct metadataPpdu
{
	uint16_t ppdu_len;
	uint16_t log_chann;
	uint8_t encoding;
	uint64_t airtime;
	float scale;
	bool altern_pream;
};

#endif /* INCLUDED_FBMC_UTILS_METADATA_H */
