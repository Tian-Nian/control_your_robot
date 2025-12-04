
#ifndef __TIMER_H__
#define __TIMER_H__
#include "GlobalOptions.h"

/*! qp timers and their defintions */


/*! For Windows */
#if (defined _WIN32 || defined _WIN64 || defined _WINDLL )

/* Use Windows QueryPerformanceCounter for timing */
#include <windows.h>
/*! \struct */
/*! Timer structure to store time information */
typedef struct qp_timer{
	LARGE_INTEGER tic; /*!< tic time */
	LARGE_INTEGER toc; /*!< tic time */
	LARGE_INTEGER freq; /*!< cpu frequency */
} qp_timer;

/*! For macOS */
#elif (defined __APPLE__)
/*! \struct */
/*! Timer structure to store time information */
#include <mach/mach_time.h>
typedef struct qp_timer{
	uint64_t tic; /*!< tic time */
	uint64_t toc; /*!< toc time */
	mach_timebase_info_data_t tinfo; /*!< time base info */
} qp_timer;



#else

/*! For POSIX machines */
/*! \struct */
/*! Timer structure to store time information */
#include <time.h>
#include <sys/time.h>

typedef struct qp_timer{
	struct timespec tic; /*!< tic time */
	struct timespec toc; /*!< toc time */
} qp_timer;

#endif

/*!
 * @brief  timer tic functions, similar to matlab tic, starts recording time from the instant the function is invoked
 *
 * 
 * @param[in]  t     	    qp_timer structure
 *
 */
void tic(qp_timer* t);

/*!
 * @brief  timer toc functions, similar to matlab toc, returns the recorded time
 *
 * 
 * @param[in]  t     	    qp_timer structure
 * @param[out] diff         recorded time
 * 
 */
qp_real toc(qp_timer* t);
#endif
/* END IFDEF __TIMER_H__ */

/*! @file */