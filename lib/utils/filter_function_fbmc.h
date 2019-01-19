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

#ifndef INCLUDED_FBMC_UTILS_FILTER_FUNCTION_FBMC_H
#define INCLUDED_FBMC_UTILS_FILTER_FUNCTION_FBMC_H

#include <complex>

/*
 * mode = 0 -> transmitter (reverse order)
 * mode = 1 -> receiver
 * 
 */

void calculate_filter_fbmc(std::complex<float>* vec_begin, int n_subc, int k, float amp, int mode);

#endif /* INCLUDED_FBMC_UTILS_FILTER_FUNCTION_FBMC_H */
