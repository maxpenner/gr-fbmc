/* Generic Viterbi decoder,
 * Copyright Phil Karn, KA9Q,
 * Code has been slightly modified for use with Spiral (www.spiral.net)
 * Karn's original code can be found here: http://www.ka9q.net/code/fec/
 * May be used under the terms of the GNU Lesser General Public License (LGPL)
 * see http://www.gnu.org/copyleft/lgpl.html
 */

#define	MAX_RANDOM	0x7fffffff

/* Determine parity of argument: 1 = odd, 0 = even */
#ifdef __i386__
static inline int parityb(unsigned char x){
  __asm__ __volatile__ ("test %1,%1;setpo %0" : "=g"(x) : "r" (x));
  return x;
}
#else
void partab_init();

static inline int parityb(unsigned char x){
  extern unsigned char Partab[256];
  extern int P_init;
  if(!P_init){
    partab_init();
  }
  return Partab[x];
}
#endif

static inline int parity(int x){
  /* Fold down to one byte */
  x ^= (x >> 16);
  x ^= (x >> 8);
  return parityb(x);
}

unsigned char Partab[256];
int P_init;

/* Create 256-entry odd-parity lookup table
 * Needed only on non-ia32 machines
 */
void partab_init(void){
  int i,cnt,ti;

  /* Initialize parity lookup table */
  for(i=0;i<256;i++){
    cnt = 0;
    ti = i;
    while(ti){
      if(ti & 1)
	cnt++;
      ti >>= 1;
    }
    Partab[i] = cnt & 1;
  }
  P_init=1;
}

// ################################
// ################################
#ifdef THIS_IS_NOT_DEFINED_TO_DEACTIVATE_CODE
// ################################
// ################################

/* Generate gaussian random double with specified mean and std_dev */
double normal_rand(double mean, double std_dev)
{
  double fac,rsq,v1,v2;
  static double gset;
  static int iset;

  if(iset){
    /* Already got one */
    iset = 0;
    return mean + std_dev*gset;
  }
  /* Generate two evenly distributed numbers between -1 and +1
   * that are inside the unit circle
   */
  do {
    v1 = 2.0 * (double)random() / MAX_RANDOM - 1;
    v2 = 2.0 * (double)random() / MAX_RANDOM - 1;
    rsq = v1*v1 + v2*v2;
  } while(rsq >= 1.0 || rsq == 0.0);
  fac = sqrt(-2.0*log(rsq)/rsq);
  gset = v1*fac;
  iset++;
  return mean + std_dev*v2*fac;
}

unsigned char addnoise(int sym,double amp,double gain,double offset,int clip){
  int sample;
    
  sample = offset + gain*normal_rand(sym?amp:-amp,1.0);
  /* Clip to 8-bit offset range */
  if(sample < 0)
    sample = 0;
  else if(sample > clip)
    sample = clip;
  return sample;
}

/* Lookup table giving count of 1 bits for integers 0-255 */
int Bitcnt[] = {
 0, 1, 1, 2, 1, 2, 2, 3,
 1, 2, 2, 3, 2, 3, 3, 4,
 1, 2, 2, 3, 2, 3, 3, 4,
 2, 3, 3, 4, 3, 4, 4, 5,
 1, 2, 2, 3, 2, 3, 3, 4,
 2, 3, 3, 4, 3, 4, 4, 5,
 2, 3, 3, 4, 3, 4, 4, 5,
 3, 4, 4, 5, 4, 5, 5, 6,
 1, 2, 2, 3, 2, 3, 3, 4,
 2, 3, 3, 4, 3, 4, 4, 5,
 2, 3, 3, 4, 3, 4, 4, 5,
 3, 4, 4, 5, 4, 5, 5, 6,
 2, 3, 3, 4, 3, 4, 4, 5,
 3, 4, 4, 5, 4, 5, 5, 6,
 3, 4, 4, 5, 4, 5, 5, 6,
 4, 5, 5, 6, 5, 6, 6, 7,
 1, 2, 2, 3, 2, 3, 3, 4,
 2, 3, 3, 4, 3, 4, 4, 5,
 2, 3, 3, 4, 3, 4, 4, 5,
 3, 4, 4, 5, 4, 5, 5, 6,
 2, 3, 3, 4, 3, 4, 4, 5,
 3, 4, 4, 5, 4, 5, 5, 6,
 3, 4, 4, 5, 4, 5, 5, 6,
 4, 5, 5, 6, 5, 6, 6, 7,
 2, 3, 3, 4, 3, 4, 4, 5,
 3, 4, 4, 5, 4, 5, 5, 6,
 3, 4, 4, 5, 4, 5, 5, 6,
 4, 5, 5, 6, 5, 6, 6, 7,
 3, 4, 4, 5, 4, 5, 5, 6,
 4, 5, 5, 6, 5, 6, 6, 7,
 4, 5, 5, 6, 5, 6, 6, 7,
 5, 6, 6, 7, 6, 7, 7, 8,
};

// ################################
// ################################
#endif
// ################################
// ################################
