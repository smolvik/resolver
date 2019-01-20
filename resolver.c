#include <scilab/scicos_block4.h>
#include <time.h>
#include <math.h>
#include <stdint.h>
#include <stdlib.h>

#define MY_PI 512

#define r_IN(n, i) ((GetRealInPortPtrs(blk, n+1))[(i)]) 
#define r_OUT(n, i) ((GetRealOutPortPtrs(blk, n+1))[(i)])

extern const int32_t cos_tb[1024];

static inline int32_t mycos(int32_t a)
{
	return cos_tb[1023&a];
}

static inline int32_t mysin(int32_t a)
{
	return cos_tb[1023&(a+3*MY_PI/2)];
}

// calc magnitude and angle of vector using CORDIC algorithm
static inline void cord_atan(int32_t *v, int32_t *ang, int32_t *mag)
{
	const int32_t AngTable[] = {128, 76, 40, 20, 10, 5, 3, 1};
	const int32_t kc[] = {724,  648, 628,  623,  623,  622,  622,  622};
	int32_t SumAngle = 0; 
	int i = 0;
	int x, y, x1, y1;
	int ns = 0;

	x = abs(v[0]);
	y = v[1];

	for(i = 0; i < 8; i++)
	{		
		ns++;
		
		x1 = x;
		y1 = y;
			
		if(y > 0){
			x = x1 + (y1 >> i); 
			y = y1 - (x1 >> i); 
			SumAngle = SumAngle + AngTable[i]; 
		}else{
			x = x1 - (y1 >> i); 
			y = y1 + (x1 >> i); 
			SumAngle = SumAngle - AngTable[i]; 
		}
		if(y == 0) break;
	}
	
	if(v[0] < 0) SumAngle = MY_PI-SumAngle;		
	if(SumAngle < 0) SumAngle += 2*MY_PI;	
	
	*ang = SumAngle;
	*mag = (kc[ns-1]*x) >> 10;
}

void angle_estimator(scicos_block *blk, int flag)
{
	int32_t v[2];
	static int32_t mag = 0;
	static int32_t ang = 0;
	
    switch(flag)
    {
		case Initialization:
		// Initialization
		break;

		case Ending:
		// Simulation ending
		break;		
		
		case StateUpdate:
		//  Update block internal state
			v[0] = (int32_t)(1024*r_IN(0, 0));
			v[1] = (int32_t)(1024*r_IN(0, 1));

			cord_atan(v, &ang, &mag);
		break;

		case OutputUpdate:
		//  Put the value on the output port
			r_OUT(0, 0) = 180*((double)ang)/MY_PI;
		break;
		
    }
}

void envelope_extractor(scicos_block *blk, int flag)
{
	static int32_t xc = 0;
	static int32_t xs = 0;
	
    switch(flag)
    {
		case Initialization:
		// Initialization
			xc = 0;
			xs = 0;
		break;

		case Ending:
		// Simulation ending
		break;		
		
		case StateUpdate:
		//  Update block internal state
		{
			xc = (int32_t)(1024*r_IN(0, 0));
			xs = (int32_t)(1024*r_IN(0, 1));
		}
		break;
		
		case OutputUpdate:
		//  Put the value on the output port
		{	
			r_OUT(0, 0) = ((double)xc)/1024.0;
			r_OUT(0, 1) = ((double)xs)/1024.0;
		}
		break;
		
    }
}

void exciter(scicos_block *blk, int flag)
{
	static uint32_t phase = 0;
	
    switch(flag)
    {
		case Initialization:
		// Initialization
		phase = 0;
		break;

		case Ending:
		// Simulation ending
		break;		
		
		case StateUpdate:
		//  Update block internal state
		{
			uint32_t freq = (int32_t)(1024*r_IN(0, 0)/100e3);
			phase = 1023&(phase + freq);
		}
		break;
		
		case OutputUpdate:
		//  Put the value on the output port
		r_OUT(0, 0) = ((double)mycos(phase))/1024.0;
		break;
		
    }
}
