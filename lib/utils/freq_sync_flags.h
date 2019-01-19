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

#ifndef INCLUDED_FBMC_UTILS_FREQ_SYNC_FLAGS_H
#define INCLUDED_FBMC_UTILS_FREQ_SYNC_FLAGS_H

// Flag for entire vector
#define FSYNC_FLAG_NO_FRAME_AT_ALL_FOUND				0
#define FSYNC_FLAG_AT_LEAST_ONE_FRAME_FOUND				1

// flag for a particular logical channel
#define FSYNC_FLAG_NO_FRAME_IN_THIS_LOG_CHANN_FOUND		0
#define FSYNC_FLAG_FRAME_IN_THIS_LOG_CHANN_FOUND		1

#define FSYNC_FLAG_CFO_PRECISION						10000000.0f			// We calculate the cfo as an float, if will be below 1, but we need to pass it as an integer. Therefore increase precision.
#define FSYNC_FLAG_INFO_OFFSET							5

#endif /* INCLUDED_FBMC_UTILS_FREQ_SYNC_FLAGS_H */
