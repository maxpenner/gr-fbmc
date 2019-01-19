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

#ifndef INCLUDED_FBMC_UTILS_PREAMBLE_SET_TIME_DOMAIN_H
#define INCLUDED_FBMC_UTILS_PREAMBLE_SET_TIME_DOMAIN_H

#include <complex>

void get_preamble_set_conj_time_domain(int set, int veclength, int filterlength, int offset, int size, std::complex<float> *d_preamble);

#endif /* INCLUDED_FBMC_UTILS_PREAMBLE_SET_TIME_DOMAIN_H */
