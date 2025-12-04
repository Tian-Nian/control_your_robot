#include "timer.h"


/*! For Windows machines */
#if (defined WIN32 || _WIN64)

void tic(qp_timer* t)
{
	QueryPerformanceFrequency(&t->freq);
	QueryPerformanceCounter(&t->tic);
}

qp_real toc(qp_timer* t)
{
	QueryPerformanceCounter(&t->toc);
	return ((t->toc.QuadPart - t->tic.QuadPart) / (qp_real)t->freq.QuadPart);
}

/*! For macOS */
#elif (defined __APPLE__)

void tic(qp_timer* t)
{
	t->tic = mach_absolute_time();
}

qp_real toc(qp_timer* t)
{

	uint64_t duration; 
	t->toc = mach_absolute_time();
	duration = t->toc - t->tic;

	mach_timebase_info(&(t->tinfo));
	duration *= t->tinfo.numer;
	duration /= t->tinfo.denom;

	return (qp_real)duration / 1000000000;
}



#else
/*! For Posix machines */

void tic(qp_timer* t)
{
	clock_gettime(CLOCK_MONOTONIC, &t->tic);
}



double toc(qp_timer* t)
{
	struct timespec temp;

	clock_gettime(CLOCK_MONOTONIC, &t->toc);

	if ((t->toc.tv_nsec - t->tic.tv_nsec)<0) {
		temp.tv_sec = t->toc.tv_sec - t->tic.tv_sec - 1;
		temp.tv_nsec = 1000000000 + t->toc.tv_nsec - t->tic.tv_nsec;
	}
	else {
		temp.tv_sec = t->toc.tv_sec - t->tic.tv_sec;
		temp.tv_nsec = t->toc.tv_nsec - t->tic.tv_nsec;
	}
	return (qp_real)temp.tv_sec + (qp_real)temp.tv_nsec / 1000000000;
}

#endif
/*! @file */