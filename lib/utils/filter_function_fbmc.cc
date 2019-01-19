/* -*- c++ -*- */
/* 
 * Copyright 2014 <+YOU OR YOUR COMPANY+>.
 * 
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

#include <cstring>
#include <vector>
#include <iostream>

#include "filter_function_fbmc.h"
#include "frame_param.h"
#include "debug.h"

void g_alpha(std::vector<float> &g, float alpha, std::vector<float> &n);
float d_c_alpha_nu(float c, float alpha, float nu, float big_K);

void calculate_filter_fbmc(std::complex<float>* vec_begin, int n_subc, int k, float amp, int mode)
{	
//
//
//	PHYDYAS PHYDYAS PHYDYAS PHYDYAS PHYDYAS PHYDYAS PHYDYAS PHYDYAS PHYDYAS PHYDYAS PHYDYAS PHYDYAS
//
//
#if PROTOTYPE_FILTER_PHYDYAS_0_IOTA_1 == 0

	int flt_len = k*n_subc;
	float A;	
	float pi = 3.14159265f;
	std::vector<float> h_coef;
	std::vector<float> filter_func_f;
	std::vector<std::complex<float>> filter_func;

	switch(k)
	{
		case 3:
		
			A = 0.971960;
			h_coef.push_back(1.0);
			h_coef.push_back(A);
			h_coef.push_back(sqrt(1.0-A*A));
			filter_func_f = h_coef;
			
			for(int i=0; i < flt_len-(2*k-1); i++)
				filter_func_f.push_back(0.0);
			
			filter_func_f.push_back(h_coef[2]);
			filter_func_f.push_back(h_coef[1]);
			
			break;
		
		case 4:
		
			A = 0.971960;
			h_coef.push_back(1.0);
			h_coef.push_back(A);
			h_coef.push_back(sqrt(2.0)/2.0);
			h_coef.push_back(sqrt(1.0-A*A));
			filter_func_f = h_coef;
			
			for(int i=0; i < flt_len-(2*k-1); i++)
				filter_func_f.push_back(0.0);
			
			filter_func_f.push_back(h_coef[3]);
			filter_func_f.push_back(h_coef[2]);
			filter_func_f.push_back(h_coef[1]);	
			
			break;
	}

	// idft
	int a1 = filter_func_f.size();

	for(int i=0; i < a1; i++)
	{
		filter_func.push_back(std::complex<float>(0,0));
		
		for(int j=0; j< a1; j++)
			filter_func[i] += std::complex<float>(filter_func_f[j] * cos(2*pi/a1*i*j), filter_func_f[j] * sin(2*pi/a1*i*j));
		
		// normalize with idft length
		filter_func[i] = std::complex<float>(filter_func[i].real()/a1, filter_func[i].imag()/a1);
	}

	// normalize and multiply with amp + zero imag part as in phydas.m
	float norm = 0;

	for(int i=0; i<a1; i++)
		norm += filter_func[i].real()*filter_func[i].real() + filter_func[i].imag()*filter_func[i].imag();

	norm = sqrt(norm);

	for(int i=0; i<a1; i++)
		filter_func[i] = std::complex<float>(filter_func[i].real()/norm*amp, 0*filter_func[i].imag()/norm*amp);

	// ifft-shift
	std::vector<std::complex<float>> filter_func_buffer;
	filter_func_buffer = filter_func;

	for(int i = 0; i<a1/2; i++)
	{
		filter_func[i] = filter_func_buffer[i+a1/2];
		filter_func[i+a1/2] = filter_func_buffer[i];
	}
	
//
//
//	IOTA IOTA IOTA IOTA IOTA IOTA IOTA IOTA IOTA IOTA IOTA IOTA IOTA IOTA IOTA IOTA IOTA IOTA IOTA IOTA IOTA IOTA
//
//	
#elif PROTOTYPE_FILTER_PHYDYAS_0_IOTA_1 == 1

	int flt_len = k*n_subc;
	float alpha = 1.0f;
	float nu_0 = 1.0f/sqrt(2.0f);
	float tau_0 = 1.0f/sqrt(2.0f);
	float big_K = 14.0f;
	float t_sampl = 1.0f/(n_subc*nu_0);
	float limit = 2.0f*k/(4*nu_0);
	
	// create time vector n, sum1m sum2 and the filter function
	std::vector<float> n;
	std::vector<float> sum1;
	std::vector<float> sum2;
	for(int i=0; i<=flt_len; i++)
	{
		float tmp_val = -limit + ((float) i)*t_sampl;
		n.push_back(tmp_val);
		sum1.push_back(0.0f);
		sum2.push_back(0.0f);
	}
	
	for(int c=0; c<=big_K; c++)
	{		
		// create temporary container for g_alpha
		std::vector<float> g_alpha_VAL0;
		std::vector<float> g_alpha_VAL1;
		std::vector<float> n_tmp0;
		std::vector<float> n_tmp1;
		for(int i=0; i<=n_subc*k; i++)
		{
			g_alpha_VAL0.push_back(0.0f);
			g_alpha_VAL1.push_back(0.0f);
			n_tmp0.push_back(n[i] + ((float) c)/nu_0);
			n_tmp1.push_back(n[i] - ((float) c)/nu_0);
		}
		
		// calculate vectors
		g_alpha(g_alpha_VAL0, alpha, n_tmp0);
		g_alpha(g_alpha_VAL1, alpha, n_tmp1);
		
		// fills sums
		for(int i=0; i<sum1.size(); i++)
		{
			sum1[i] = sum1[i] + d_c_alpha_nu(c, alpha, nu_0, big_K)*(g_alpha_VAL0[i] + g_alpha_VAL1[i]);
			sum2[i] = sum2[i] + d_c_alpha_nu(c, 1.0f/alpha, tau_0, big_K)*cos(6.283185307f*c*n[i]/tau_0);
		}		
				
	}
	
	// time domain prototype	
	std::vector<float> filter_func_f;
	for(int i=0; i<flt_len; i++)
		filter_func_f.push_back(0.0f);
	
	// LINE 1
	for(int i=0; i<filter_func_f.size();i++)
	{
		filter_func_f[i] = 0.5f*sum1[i]*sum2[i];
	}
	
	// LINE 2
	float sum_filter_func_f = 0.0f;
	for(int i=0; i<flt_len;i++)
	{
		sum_filter_func_f += filter_func_f[i];
	}
	for(int i=0; i<flt_len;i++)
	{
		filter_func_f[i] = filter_func_f[i] / sum_filter_func_f;
	}	
	
	// LINE 3
	float norm_filter_func_f = 0.0f;
	for(int i=0; i<flt_len;i++)
	{
		norm_filter_func_f += filter_func_f[i]*filter_func_f[i];
	}
	norm_filter_func_f = sqrt(norm_filter_func_f);
	for(int i=0; i<flt_len;i++)
	{
		filter_func_f[i] = filter_func_f[i] / norm_filter_func_f;
	}
	
	// normalize with amp
	for(int i=0; i<flt_len;i++)
	{
		filter_func_f[i] = filter_func_f[i] * amp;
	}
	
	// create complex vector to write to output
	std::vector<std::complex<float>> filter_func;
	for(int i=0; i<flt_len; i++)
		filter_func.push_back(std::complex<float>(filter_func_f[i], 0.0f));
	
		/*
		// LINE 4
		std::vector<std::complex<float>> filter_func_f_final;
		
		// LINE 4: dft
		for(int i=0; i < flt_len; i++)
		{
			filter_func_f_final.push_back(std::complex<float>(0.0f,0.0f));
			
			for(int j=0; j< flt_len; j++)
				filter_func_f_final[i] += std::complex<float>(filter_func_f[j] * cos(6.283185f/flt_len*i*j), filter_func_f[j] * (-1.0f) * sin(6.283185f/flt_len*i*j));
			
			// normalize with idft length
			filter_func_f_final[i] = std::complex<float>(filter_func_f_final[i].real()/flt_len, filter_func_f_final[i].imag()/flt_len);
		}

		// LINE 4: fft-shift
		std::vector<std::complex<float>> filter_func_buffer;
		filter_func_buffer = filter_func_f_final;

		for(int i = 0; i<flt_len/2; i++)
		{
			filter_func_f_final[i] = filter_func_buffer[i+flt_len/2];
			filter_func_f_final[i+flt_len/2] = filter_func_buffer[i];
		}
		*/
	
#endif

	// oder depends on mode
	if(mode == 0)
	{
		// write result to d_filter in transmitter mode (reverse order)
		for(int i = 0; i<k; i++)
			memcpy(&vec_begin[(k-1-i)*n_subc], &filter_func[i*n_subc], n_subc*sizeof(std::complex<float>));
	}
	else if(mode == 1)
	{
		// write result to d_filter in receiver mode
		for(int i = 0; i<k; i++)
			memcpy(&vec_begin[i*n_subc], &filter_func[i*n_subc], n_subc*sizeof(std::complex<float>));		
	}
}

void g_alpha(std::vector<float> &g, float alpha, std::vector<float> &n)
{
	for(int i=0; i<g.size(); i++)
	{
		g[i] = pow((2.0f*alpha),0.25f)*exp(-3.141592654f*alpha*n[i]*n[i]);
	}
}

float d_c_alpha_nu(float c, float alpha, float nu, float big_K)
{
	const float b[120] = {	1.0f, 3.0f/4.0f, 105.0f/64.0f, 675.0f/256.0f, 76233.0f/16384.0f, 457107.0f/65536.0f, 12097169.0f/1048576.0f, 70545315.0f/4194304.0f,
							-1.0f, -15.0f/8.0f, -219.0f/64.0f, -6055.0f/1024.0f, -161925.0f/16384.0f, -2067909.0f/131072.0f, -26060847.0f/1048576.0f, 0.0f,
							3.0f/4.0f, 19.0f/16.0f, 1545.0f/512.0f, 9765.0f/2048.0f, 596277.0f/65536.0f, 3679941.0f/262144.0f, 394159701.0f/16777216.0f,  0.0f,
							-5.0f/8.0f, -123.0f/128.0f, -2289.0f/1024.0f, -34871.0f/8192.0f, -969375.0f/131072.0f, -51182445.0f/4194304.0f, 0.0f, 0.0f,
							35.0f/64.0f, 213.0f/256.0f, 7797.0f/4096.0f, 56163.0f/16384.0f, 13861065.0f/2097152.0f, 87185895.0f/8388608.0f, 0.0f, 0.0f,
							-63.0f/128.0f, -763.0f/1024.0f, -13875.0f/8192.0f, -790815.0f/262144.0f, -23600537.0f/4194304.0f, 0.0f, 0.0f, 0.0f,
							231.0f/512.0f, 1395.0f/2048.0f, 202281.0f/131072.0f, 1434705.0f/524288.0f,  85037895.0f/16777216.0f, 0.0f, 0.0f, 0.0f,
							-429.0f/1024.0f, -20691.0f/32768.0f, -374325.0f/262144.0f, -5297445.0f/2097152.0f, 0.0f, 0.0f, 0.0f, 0.0f,
							6435.0f/16384.0f, 38753.0f/65536.0f, 1400487.0f/1048576.0f, 9895893.0f/4194304.0f, 0.0f, 0.0f, 0.0f, 0.0f,
							-12155.0f/32768.0f, -146289.0f/262144.0f, -2641197.0f/2097152.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
							46189.0f/131072.0f,  277797.0f/524288.0f, 20050485.0f/16777216.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
							-88179.0f/262144.0f, -2120495.0f/4194304.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
							676039.0f/2097152.0f, 4063017.0f/8388608.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
							-1300075.0f/4194304.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f,
							5014575.0f/16777216.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
											
	float d = 0.0f;
	for(int j=0; j<=floor((big_K-c)/2.0f); j++)
	{		
		d = d + b[((int)c)*8 + j]*exp(-(3.141592654f*alpha/2.0f/(nu*nu)*(2.0f*j+c)));
	}
	
	return d;
}
