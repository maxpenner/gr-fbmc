/* -*- c++ -*- */
/* 
 * Copyright 2015 <+YOU OR YOUR COMPANY+>.
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

#ifndef INCLUDED_FBMC1_FBMC_PPN_IMPL_H
#define INCLUDED_FBMC1_FBMC_PPN_IMPL_H

#include <fbmc1/fbmc_ppn.h>

#include "utils/filter_function_fbmc.h"

namespace gr {
  namespace fbmc1 {

    class fbmc_ppn_impl : public fbmc_ppn
    {
     private:

		// variables
		int d_veclength;
		int d_filterlength;
		
		gr_complex *d_filter;

     public:
      fbmc_ppn_impl(int veclength, int filterlength, float normalize);
      ~fbmc_ppn_impl();

      // Where all the action really happens
      int work(int noutput_items,
	       gr_vector_const_void_star &input_items,
	       gr_vector_void_star &output_items);
    };

  } // namespace fbmc1
} // namespace gr

#endif /* INCLUDED_FBMC1_FBMC_PPN_IMPL_H */

