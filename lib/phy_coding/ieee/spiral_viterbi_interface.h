/*
 * MAXIM PENNER, 6th April 2015
 * 
 * Interface function to enter viterbi decoder.
 * 
 * 		rate = 1/2
 * 		K = 7
 * 		polynomials = 109 79 
 * 
 * frambits is the number of bits that has to be extracted. 
 * framebits MUST BE A MULTIPLE OF 2 (which it always is if communication is byte oriented).
 * 
 * tailbits are the bits appended to framebits. Since the constraint length K is 7, tailbits must be K-1 = 6.
 * tailbits IS FIXED TO 6. 
 * It was added to make it more obvious that framebits does not include the tailbits.
 * 
 * samples: Each byte of samples represents one bit. A 0 is a strong 0, 255 is a strong 1. So it's a soft decoder.
 * 			127 is unknown and can be used for depunctered bits.
 * 			The code rate is 1/2. Therefore samples must have a length of 2 * (framebits + tailbits - 1).
 * 
 * data: 	Memory has to be user-supplied. The bytes extracted from samples are written into data.
 * 			data must have a length of (framebits+(K-1))/8+1 = (framebits+6)/8+1.
 * 
 * The files also include a non-SIMD generaic C-decoder. But it is not used here since it is significantly slower.
 * But the code works.
 */
void spiral_viterbi_decoder(unsigned char *samples, unsigned char *data, int framebits, int tailbits);
