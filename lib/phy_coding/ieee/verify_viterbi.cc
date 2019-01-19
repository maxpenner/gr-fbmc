/* Generic Viterbi decoder,
 * Copyright Phil Karn, KA9Q, 
 * Code has been slightly modified for use with Spiral (www.spiral.net)
 * Karn's original code can be found here: http://www.ka9q.net/code/fec/
 * May be used under the terms of the GNU Lesser General Public License (LGPL)
 * see http://www.gnu.org/copyleft/lgpl.html
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <memory.h>
#include <sys/resource.h>
#include "verify_viterbi.h"
#include <pmmintrin.h>
#include <emmintrin.h>
#include <xmmintrin.h>
#include <mmintrin.h>

// ################################
// ################################
#include "spiral_def.h"
#include "spiral_viterbi_interface.h"
// ################################
// ################################

#define OFFSET (127.5)
#define CLIP 255

extern int posix_memalign(void **memptr, size_t alignment, size_t size);
//decision_t is a BIT vector
typedef union {
  DECISIONTYPE t[NUMSTATES/DECISIONTYPE_BITSIZE];
  unsigned int w[NUMSTATES/32];
  unsigned short s[NUMSTATES/16];
  unsigned char c[NUMSTATES/8];
} decision_t __attribute__ ((aligned (16)));

typedef union {
  COMPUTETYPE t[NUMSTATES];
} metric_t __attribute__ ((aligned (16)));

inline void renormalize(COMPUTETYPE* X, COMPUTETYPE threshold){
      if (X[0]>threshold){
    COMPUTETYPE min=X[0];
    for(int i=0;i<NUMSTATES;i++)
    if (min>X[i])
      min=X[i];
    for(int i=0;i<NUMSTATES;i++)
      X[i]-=min;
      }
}

COMPUTETYPE Branchtab[NUMSTATES/2*RATE] __attribute__ ((aligned (16)));

/* State info for instance of Viterbi decoder
 */
struct v {
  __attribute__ ((aligned (16))) metric_t metrics1; /* path metric buffer 1 */
  __attribute__ ((aligned (16))) metric_t metrics2; /* path metric buffer 2 */
  metric_t *old_metrics,*new_metrics; /* Pointers to path metrics, swapped on every bit */
  decision_t *decisions;   /* decisions */
};

/* Initialize Viterbi decoder for start of new frame */
int init_viterbi(void *p,int starting_state){
	
  //struct v *vp = p;
  struct v *vp = (struct v*) p;
  
  int i;

  if(p == NULL)
    return -1;
  for(i=0;i<NUMSTATES;i++)
      vp->metrics1.t[i] = 63;

  vp->old_metrics = &vp->metrics1;
  vp->new_metrics = &vp->metrics2;
  vp->old_metrics->t[starting_state & (NUMSTATES-1)] = 0; /* Bias known start state */
  return 0;
}

/* Create a new instance of a Viterbi decoder */
void *create_viterbi(int len){
  void *p;
  struct v *vp;
  static int Init = 0;

  if(!Init){
    int state, i;
    int polys[RATE] = POLYS;
    for(state=0;state < NUMSTATES/2;state++){
      for (i=0; i<RATE; i++){
        Branchtab[i*NUMSTATES/2+state] = (polys[i] < 0) ^ parity((2*state) & abs(polys[i])) ? 255 : 0;
      }
    }
    Init++;
  }

  if(posix_memalign((void**)&p, 16,sizeof(struct v)))
    return NULL;

  vp = (struct v *)p;

  if(posix_memalign((void**)&vp->decisions, 16,(len+(K-1))*sizeof(decision_t))){
    free(vp);
    return NULL;
  }
  init_viterbi(vp,0);

  return vp;
}

/* Viterbi chainback */
int chainback_viterbi(
      void *p,
      unsigned char *data, /* Decoded output data */
      unsigned int nbits, /* Number of data bits */
      unsigned int endstate){ /* Terminal encoder state */
      
  //struct v *vp = p;
  struct v *vp = (struct v*) p;
  
  decision_t *d;

  /* ADDSHIFT and SUBSHIFT make sure that the thing returned is a byte. */
#if (K-1<8)
#define ADDSHIFT (8-(K-1))
#define SUBSHIFT 0
#elif (K-1>8)
#define ADDSHIFT 0
#define SUBSHIFT ((K-1)-8)
#else
#define ADDSHIFT 0
#define SUBSHIFT 0
#endif

  if(p == NULL)
    return -1;
  d = vp->decisions;
  /* Make room beyond the end of the encoder register so we can
   * accumulate a full byte of decoded data
   */

  endstate = (endstate%NUMSTATES) << ADDSHIFT;

  /* The store into data[] only needs to be done every 8 bits.
   * But this avoids a conditional branch, and the writes will
   * combine in the cache anyway
   */
  d += (K-1); /* Look past tail */
  while(nbits-- != 0){
    int k;
    k = (d[nbits].w[(endstate>>ADDSHIFT)/32] >> ((endstate>>ADDSHIFT)%32)) & 1;
    endstate = (endstate >> 1) | (k << (K-2+ADDSHIFT));
    data[nbits>>3] = endstate>>SUBSHIFT;
  }
  return 0;
}

/* Delete instance of a Viterbi decoder */
void delete_viterbi(void *p){
	
  //struct v *vp = p;
  struct v *vp = (struct v*) p;

  if(vp != NULL){
    free(vp->decisions);
    free(vp);
  }
}

/* C-language butterfly */
void BFLY(int i, int s, COMPUTETYPE * syms, struct v * vp, decision_t * d) {
  int j, decision0, decision1;
  COMPUTETYPE metric,m0,m1,m2,m3;

  metric =0;
  for (j=0;j<RATE;j++) metric += (Branchtab[i+j*NUMSTATES/2] ^ syms[s*RATE+j])>>METRICSHIFT ;
  metric=metric>>PRECISIONSHIFT;
  
  const COMPUTETYPE max = ((RATE*((256 -1)>>METRICSHIFT))>>PRECISIONSHIFT);
  
  m0 = vp->old_metrics->t[i] + metric;
  m1 = vp->old_metrics->t[i+NUMSTATES/2] + (max - metric);
  m2 = vp->old_metrics->t[i] + (max - metric);
  m3 = vp->old_metrics->t[i+NUMSTATES/2] + metric;
  
  decision0 = (signed int)(m0-m1) > 0;
  decision1 = (signed int)(m2-m3) > 0;
  
  vp->new_metrics->t[2*i] = decision0 ? m1 : m0;
  vp->new_metrics->t[2*i+1] =  decision1 ? m3 : m2;
  
  d->w[i/(sizeof(unsigned int)*8/2)+s*(sizeof(decision_t)/sizeof(unsigned int))] |= 
    (decision0|decision1<<1) << ((2*i)&(sizeof(unsigned int)*8-1));
}


/* Update decoder with a block of demodulated symbols
 * Note that nbits is the number of decoded data bits, not the number
 * of symbols!
 */
    COMPUTETYPE max_spread = 0;

int update_viterbi_blk_GENERIC(void *p, COMPUTETYPE *syms,int nbits){
	
  //struct v *vp = p;
  struct v *vp = (struct v*) p;

  decision_t *d;
  int s,i;

  if(p == NULL)
    return -1;
  d = (decision_t *)vp->decisions;

  for (s=0;s<nbits;s++)
    memset(d+s,0,sizeof(decision_t));

  for (s=0;s<nbits;s++){
    void *tmp;
    for(i=0;i<NUMSTATES/2;i++){
      BFLY(i, s, syms, vp, vp->decisions);
    }

#ifdef GENERICONLY
    COMPUTETYPE min=vp->new_metrics->t[0];
    COMPUTETYPE max=vp->new_metrics->t[0];
    
    /* Compute Spread */
    for(int i=0;i<NUMSTATES;i++)
      if (min>vp->new_metrics->t[i]) 
        min=vp->new_metrics->t[i];
      else if (max<vp->new_metrics->t[i])
        max=vp->new_metrics->t[i];
    if (max_spread<max-min)
      max_spread=max-min;
#endif

    renormalize(vp->new_metrics->t, RENORMALIZE_THRESHOLD);
    
    ///     Swap pointers to old and new metrics
    tmp = vp->old_metrics;
    vp->old_metrics = vp->new_metrics;
    
    //vp->new_metrics = tmp;
    vp->new_metrics = (metric_t*) tmp;
    
  }

  return 0;
}

#ifndef GENERICONLY
extern void FULL_SPIRAL(COMPUTETYPE *Y, COMPUTETYPE *X, COMPUTETYPE *syms, DECISIONTYPE *dec, COMPUTETYPE *Branchtab, int nbits);

int update_viterbi_blk_SPIRAL(void *p, COMPUTETYPE *syms,int nbits){
	
  //struct v *vp = p;
  struct v *vp = (struct v*) p;

  decision_t *d;
  int s,i;

  if(p == NULL)
    return -1;
  d = (decision_t *)vp->decisions;

  for (s=0;s<nbits;s++)
    memset(d+s,0,sizeof(decision_t));

  //FULL_SPIRAL( vp->new_metrics->t, vp->old_metrics->t, syms, d->t, Branchtab);
  FULL_SPIRAL( vp->new_metrics->t, vp->old_metrics->t, syms, d->t, Branchtab, nbits);

  return 0;
}
#endif

// #################################################
// #################################################
void spiral_viterbi_decoder(unsigned char *samples, unsigned char *data, int framebits, int tailbits)
{
	struct v *vp;
	
	if((vp = (struct v*) create_viterbi(framebits)) == NULL)
	{
		printf("create_viterbi failed\n");
		exit(1);
	}
	
    init_viterbi(vp,0);
    
    update_viterbi_blk_SPIRAL(vp,samples,framebits + tailbits);
    
    chainback_viterbi(vp,data,framebits,0);
    
    delete_viterbi((void*) vp);
}
// #################################################
// #################################################

// ################################
// ################################
#ifdef THIS_IS_NOT_DEFINED_TO_DEACTIVATE_CODE
// ################################
// ################################

int main(int argc,char *argv[]){
  unsigned char *bits;
  unsigned char *data;
  unsigned char *xordata;
  COMPUTETYPE *symbols;
  struct v *vp;
  int i, j, tr;
  int d; 
  int sr;
  int errcnt;
  int badframes;
  long long int tot_errs;
  struct rusage start,finish;
  double extime;
  double gain,esn0, SPIRAL_BER = 0, GENERIC_BER =0;
  time_t t;
  double Gain = 32.0;
  int polys[RATE] = POLYS;

  if (posix_memalign((void**)&bits, 16, (FRAMEBITS+(K-1))/8+1)){
    printf("Allocation of bits array failed\n");
    exit(1);
  }

  if (posix_memalign((void**)&data, 16, (FRAMEBITS+(K-1))/8+1)){
    printf("Allocation of data array failed\n");
    exit(1);
  }

  if (posix_memalign((void**)&xordata, 16, (FRAMEBITS+(K-1))/8+1)){
    printf("Allocation of xordata array failed\n");
    exit(1);
  }

  if (posix_memalign((void**)&symbols, 16, RATE*(FRAMEBITS+(K-1))*sizeof(COMPUTETYPE))){
    printf("Allocation of symbols array failed\n");
    exit(1);
  }
  
  if((vp = create_viterbi(FRAMEBITS)) == NULL){
    printf("create_viterbi failed\n");
    exit(1);
  }

  fprintf(stderr, "Code specifications:\n");
  fprintf(stderr, "\t rate = 1/%d \n", RATE);
  fprintf(stderr, "\t K = %d (thus %d states) \n", K, NUMSTATES);
  fprintf(stderr, "\t frame size = %d (thus padded frame size = %d) \n", FRAMEBITS, FRAMEBITS+(K-1));
  fprintf(stderr, "\t polynomials = { ");
  for(int i=0; i<RATE-1; i++)
    fprintf(stderr, "%d, ", polys[i]);
  fprintf(stderr,"%d }\n", polys[RATE-1]);
  fprintf(stderr,"\n");






#if ((!defined(DONOTVERIFY))||(!DONOTVERIFY))
  fprintf(stderr, "Computing error rates over %d frames, ebn0 = %.2f dB, gain = %g\n", TRIALS, (double)EBN0, Gain);

  /* Es/No in dB */
  esn0 = EBN0 + 10*log10(1./((double)RATE)); 
    
  /* Compute noise voltage. The 0.5 factor accounts for BPSK seeing
   * only half the noise power, and the sqrt() converts power to
   * voltage.
   */
  gain = 1./sqrt(0.5/pow(10.,esn0/10.));

#ifndef GENERICONLY
  tot_errs = 0;
  badframes = 0;
  sr = 0;
  srandom(0);
  
  for(tr=0;tr<TRIALS;tr++){
      /* Encode a frame of random data */
    for(i=0;i<FRAMEBITS+(K-1);i++){
      int bit = (i < FRAMEBITS) ? (random() & 1) : 0;
      
      sr = (sr << 1) | bit;
      bits[i/8] = sr & 0xff;
      for(j=0; j<RATE; j++)
        symbols[RATE*i+j] = addnoise(parity(sr & polys[j]), gain, Gain, OFFSET, CLIP);
    }

    /* Decode it and make sure we get the right answer */
    /* Initialize Viterbi decoder */
    init_viterbi(vp,0);
    
    /* Decode block */
    update_viterbi_blk_SPIRAL(vp,symbols,FRAMEBITS+(K-1));
    
    /* Do Viterbi chainback */
    chainback_viterbi(vp,data,FRAMEBITS,0);

    /* Count errors */
    errcnt = 0;
    for(i=0;i<FRAMEBITS/8;i++){
      int e = Bitcnt[xordata[i] = data[i] ^ bits[i]];
      errcnt += e;
      tot_errs += e;
    }
    if(errcnt != 0)
      badframes++;

    fprintf(stderr, "BER %lld/%lld (%10.3g) FER %d/%d (%10.3g)\r",
           tot_errs,(long long)FRAMEBITS*(tr+1),tot_errs/((double)FRAMEBITS*(tr+1)),
           badframes,tr+1,(double)badframes/(tr+1));
    fflush(stderr);

  }
  fprintf(stderr, "\n\n");
  SPIRAL_BER=tot_errs/((double)FRAMEBITS*(tr+1));
#endif

  
  fprintf(stderr, "Verification: Error rates of a generic decoder\n");

  tot_errs = 0;
  badframes = 0;
  sr = 0;
  srandom(0);
 
  for(tr=0;tr<TRIALS;tr++){
      /* Encode a frame of random data */
    for(i=0;i<FRAMEBITS+(K-1);i++){
      int bit = (i < FRAMEBITS) ? (random() & 1) : 0;
      
      sr = (sr << 1) | bit;
      bits[i/8] = sr & 0xff;
      for(j=0; j<RATE; j++)
        symbols[RATE*i+j] = addnoise(parity(sr & polys[j]), gain, Gain, OFFSET, CLIP);
    }

    /* Decode it and make sure we get the right answer */
    /* Initialize Viterbi decoder */
    init_viterbi(vp,0);
    
    /* Decode block */
    update_viterbi_blk_GENERIC(vp,symbols,FRAMEBITS+(K-1));

    /* Do Viterbi chainback */
    chainback_viterbi(vp,data,FRAMEBITS,0);

    /* Count errors */
    errcnt = 0;
    for(i=0;i<FRAMEBITS/8;i++){
      int e = Bitcnt[xordata[i] = data[i] ^ bits[i]];
      errcnt += e;
      tot_errs += e;
    }
    if(errcnt != 0)
      badframes++;

    fprintf(stderr, "BER %lld/%lld (%10.3g) FER %d/%d (%10.3g)\r",
           tot_errs,(long long)FRAMEBITS*(tr+1),tot_errs/((double)FRAMEBITS*(tr+1)),
           badframes,tr+1,(double)badframes/(tr+1));
    fflush(stderr);

  }
  fprintf(stderr, "\n\n");
  GENERIC_BER=tot_errs/((double)FRAMEBITS*(tr+1));

#ifdef GENERICONLY
    printf("%d;\n",max_spread);
    exit(0);
#endif

#ifdef INSIDE_SPIRAL
  printf("[%g, %g];\n", SPIRAL_BER, GENERIC_BER);
#endif
#endif
  

#ifndef DONOTMEASURE
  /* Do time trials */
  memset(symbols,127,sizeof(symbols));
  fprintf(stderr, "Execution time for %d %d-bit frames: ", TRIALS, FRAMEBITS+(K-1));
  fflush(stderr);

  getrusage(RUSAGE_SELF,&start);
  for(tr=0;tr < TRIALS;tr++){
    /* Initialize Viterbi decoder */
    init_viterbi(vp, 0);
    
    /* Decode block */
    update_viterbi_blk_SPIRAL(vp, symbols, FRAMEBITS + (K-1));
    
    /* Do Viterbi chainback */
    chainback_viterbi(vp,data,FRAMEBITS,0);
  }
  getrusage(RUSAGE_SELF,&finish);
  extime = finish.ru_utime.tv_sec - start.ru_utime.tv_sec + 1e-6*(finish.ru_utime.tv_usec - start.ru_utime.tv_usec);
  fprintf(stderr, "%.2f sec\n", extime);
  fprintf(stderr, "decoder speed: %g kbits/s\n", TRIALS*FRAMEBITS/extime*1e-3);
#ifdef INSIDE_SPIRAL
  printf("%g;\n", TRIALS*FRAMEBITS/extime*1e-3);
#endif
#endif

  free(bits);
  free(data);
  free(xordata);
  free(symbols);

  exit(0);
}

// ################################
// ################################
#endif
// ################################
// ################################
