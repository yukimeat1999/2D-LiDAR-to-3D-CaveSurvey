#include "rnd.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>

double rnd() /* uniform random number  */
{
	return((double)(rand() % 30001) / 30000.0);
}


double rndn()            /*   normal random number */
{
	return (rnd() + rnd() + rnd() + rnd() + rnd() + rnd() +
		rnd() + rnd() + rnd() + rnd() + rnd() + rnd() - 6.0);
}