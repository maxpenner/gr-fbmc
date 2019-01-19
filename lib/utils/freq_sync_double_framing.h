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

#ifndef INCLUDED_FBMC_UTILS_FREQ_SYNC_DOUBLE_FRAMING_H
#define INCLUDED_FBMC_UTILS_FREQ_SYNC_DOUBLE_FRAMING_H

#define VALVE_FD_DOUBLE_FRAMING

#ifdef VALVE_FD_DOUBLE_FRAMING

#define BOUNDARY_64_LOW 	13
#define BOUNDARY_64_HIGH 	19
#define BOUNDARY_128_LOW 	28
#define BOUNDARY_128_HIGH	36
#define BOUNDARY_256_LOW 	56
#define BOUNDARY_256_HIGH 	72

#endif

#endif /* INCLUDED_FBMC_UTILS_FREQ_SYNC_DOUBLE_FRAMING_H */
