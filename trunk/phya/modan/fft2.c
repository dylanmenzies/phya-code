

/*      fft2(a, b, m, dx, inverse)
 *      is an "in place" Fast Fourier Transform for input length a power
 *      of 2.  It will forward transform a time series of real values to
 *      a spectrum of complex values or inverse transform a complex
 *      spectrum to a time series of complex values.
 *	Written by Dale Carstensen, Antares project, Los Alamos National
 *	Laboratory, March 16, 1981 for Unix version 6.
 *	=op operators changed to op= operators for newer than version 6
 *	C compilers by Dale Carstensen, May 14, 1984.
 */


#define RBNEXT  0
double  pi;


int
revbin(int initialize);

stepfft(float a[], float b[], int n, int span, int inverse);


fft2(float* a, float* b, int m, float dx, int inverse)

#if 0
float   a[];            /* real values of input and returned output     */
float   b[];            /* imaginary values of input (fft2 will zero for
                         * forward transform) and returned output.
                         */
int	m;		/* 1 subtracted from the log (base 2) of the length
			 * e. g., 7 for length 256, 9 for length 1024.
			 */
float   dx;             /* sampling period              */
int     inverse;        /* zero for forward transform   */
#endif

{
int     i;              /* index for permutation and, if inverse, scaling */
int     j;              /* reverse binary index for permutation */
int     n;              /* length of operand    */
int     span;           /* butterfly span distance      */
float   factor;         /* for scaling transforms       */
float   temp;           /* for permutation interchange  */
extern  double  atan2();

pi = atan2(0., -1.);
n = 2;                  /* get n = 2**(m+1)     */
    while (m--)
        n <<= 1;

/*      permute input to reverse binary order so trig functions will
 *      be required in the normal order.
 */

revbin(n);              /* initialize reverse binary counter    */
    for (i = 0;  i < n;  i++)
        {
            if ((j = revbin(RBNEXT)) > i)
                {
                temp = a[i];
                a[i] = a[j];
                a[j] = temp;
                };
            if (!inverse)
                b[i] = 0;
            else if (j > i)
                {
                temp = b[i];
                b[i] = b[j];
                b[j] = temp;
                };
        };

/*      Perform butterfly calculations using a power of 2 decimation in
 *      time algorithm.
 *      Reference:      G-AE Subcommittee:  The Fast Fourier Transform
 *                      IEEE Transactions on Acoustics, etc. (then Audio
 *                        and Electroacoustics), vol. AU-15, No. 2, June
 *                        1967, pp. 49-50.
 */

    for (span = 1;  span < n;  span <<= 1)
        stepfft(a, b, n, span, inverse);
    if (inverse)                /* scale by 1/n */
        factor = (float)1. / (float)(n * dx); /* scale by 1/n */
    else
        factor = (float)dx;
    for (i = 0;  i < n;  i++)
        {
        a[i] *= factor;
        b[i] *= factor;         /* should be zero, anyway ??    */
        };
}       /* end of fft2(a, b, m, dx, inverse)    */

/*      revbin(initialize)
 *      provides a counter in reverse binary order for 0<=count<initialize.
 *      One call with initialize identifies the high order bit.  Each
 *      subsequent call with initialize = 0 (RBNEXT is defined to be 0
 *      for this purpose) will increment the count by one and return it
 *      as an integer function value.
 */

int
revbin(int initialize)
{
	static  int     counter, top_order;
	register        int     bit, c;

    if (initialize)
        {
        top_order = initialize >> 1;
        counter = initialize - 1;
        }
    else
        {
        bit = top_order;
        c = counter ^ bit;
            while (!(c & bit) && bit)
                {
                bit >>= 1;
                c ^= bit;
                };
        return(counter = c);
        };

	return 0;
	
}       /* end of revbin(initialize)    */



/*      stepfft(a, b, n, span, inverse)
 *      performs one step of the decimation in time butterfly algorithm
 *      for length a power of 2.  The input is assumed to be permuted
 *      to reverse binary order, so the cosine and sine factors can
 *      be generated and used in normal order.
 */

stepfft(float* a, float* b, int n, int span, int inverse)
{
	float   angle, cosine, dcossin, inccos, incsin, sine;   /* for trig functions */
	float   tempi, tempr, termi, termr;             /* for swapping in place        */
	int     i, j;                           /* butterfly indices    */
	int     twospan;                        /* for loop termination */
	extern  double  pi;
	extern  double  sin();

	/*      Cosine and sine functions are approximated by a technique using
	 *      second difference relations.  The formulas are:
	 *          func((k+1)a) = func(ka) + inc(k+1, func)
	 *              where   func = cos | sin,
	 *                      inc((k+1)a, func) = dcossin*func(ka) + inc(ka, func),
	 *                      dcossin = -4sin(a/2)sin(a/2),
	 *                      inc(0, cos) = 2sin(a/2)sin(a/2),
	 *                      inc(0, sin) = sin(a),
	 *                      cos(0) = 1,
	 *                      sin(0) = 0,
	 *                      a = pi/span, and
	 *                      k = 0, 1, . . . n/2 - 1
	 *
	 *      Reference:
	 *              Richard C. Singleton:  On Computing the Fast Fourier Transform
	 *              Communications of the ACM, vol. 10, No. 10, Oct. 1967
	 *              pp. 650, 651.
	 */

	angle = (float)(pi / span);              /* for last step, this is 2pi/n */
	incsin = (float)sin(angle);
	inccos = 2.f * (float)sin(0.5 * angle) * (float)sin(0.5 * angle);
	dcossin = -2.f * inccos;
	cosine = 1.;
	sine = 0.;
	twospan = span << 1;

    for (i = 0;  i < span;  i++)
        {
            for (j = i;  j < n;  j += twospan)
                {
                    if (inverse)
                        {
                        termr = cosine * a[j+span] + sine * b[j+span];
                        termi = -sine * a[j+span] + cosine * b[j+span];
                        }
                    else
                        {
                        termr = cosine * a[j+span] - sine * b[j+span];
                        termi = sine * a[j+span] + cosine * b[j+span];
                        };
                tempr = a[j] - termr;
                tempi = b[j] - termi;
                a[j] += termr;
                b[j] += termi;
                a[j+span] = tempr;
                b[j+span] = tempi;
                };
        inccos += dcossin * cosine;
        cosine += inccos;
        incsin += dcossin * sine;
        sine += incsin;
        };
}       /* end of stepfft(a, b, n, span, inverse)       */
