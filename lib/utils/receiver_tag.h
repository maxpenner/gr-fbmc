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

#ifndef INCLUDED_FBMC_UTILS_RECEIVER_TAG_H
#define INCLUDED_FBMC_UTILS_RECEIVER_TAG_H

#include <stdint.h>

/* Tag structure
 * 
 *  0. bit -> LSB
 * 63. bit -> MSB
 * 
 *  0. to  4. -> logical channel index (maximum 2^5 = 32 logical channels)
 *  5. to  5. -> double framing (0 not doubles, 1 doubled)
 *  6. to  7. -> valve frame art
 *  8. to 63. -> sample number
 * 
 */
 
#define RECEIVER_TAG_SAMPL_NUMB_OFFS 8
#define RECEIVER_TAG_FRAME_TYPE_OFFS 6
#define RECEIVER_TAG_DOUBL_FRAM_OFFS 5
#define RECEIVER_TAG_LOGIC_CHAN_MASK 0x1F

#define VALVE_FRAME_TYPE_FREER 0
#define VALVE_FRAME_TYPE_REFER 1
#define VALVE_FRAME_TYPE_FOUND 2
#define VALVE_FRAME_TYPE_KNOWN 3

int receiver_tag_logical_channel(uint64_t tag);

#endif /* INCLUDED_FBMC_UTILS_RECEIVER_TAG_H */
