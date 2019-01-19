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

#ifndef INCLUDED_FBMC_UTILS_TX_DELAY_RX_MEASURE_H
#define INCLUDED_FBMC_UTILS_TX_DELAY_RX_MEASURE_H

// TRANSMITTER:

//#define TX_DELAY_DEBUG
#define TX_DELAY_TRANSMITTING_MS	1000	// period after that the tx path is interrupted -> 0 signalizes that period if infintive
#define TX_DELAY_BLOCKING_MS		100	// how long the tx path is interrupted
#define TX_DELAY_THREAD_BLOCK_MS	50	// how long the thread should be blocked when in blocking state
#define TX_DELAY_ADD_ZEROS_BURST	100000	// append or prepending zeros to burst


// RECEIVER MEASURING CHANNEL:

#define RX_DELAY_DEBUG

#endif /* INCLUDED_FBMC_UTILS_TX_DELAY_RX_MEASURE_H */
