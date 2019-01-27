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
/*
struct pi_reg_state{
	double ki;
	double kp;
	double a;
	double y;	
};
static inline void update(scicos_block *blk, struct pi_reg_state *s, double e, int32_t fs)
{
	double a = s->a;
	double d = s->ki*e;
	
	// will accumulator grow up?
	if(fs) if( ((a>0)&&(d>0))||((a<0)&&(d<0)) ) d = 0;

	a += d;
	s->y = e*s->kp + a;
	s->a = a;
}
void angle_tracker(scicos_block *blk, int flag)
{
	static struct pi_reg_state areg = {0.0, 0.0, 0.0, 0.0};
	static double speed = 0.0;
	static double phase = 0.0;
	double err = 0.0;
	const double fs = 3e3;
	
    switch(flag)
    {
		case Initialization:
		// Initialization
		phase = 0.0;
		speed = 0.0;
		
		areg.ki = 100/fs; //(int32_t)(r_IN(0, 0)*1024);
		areg.kp = 20;  //(int32_t)(r_IN(0, 1)*1024);
		areg.a = 0;
		areg.y = 0;

		break;

		case Ending:
		// Simulation ending
		break;		
		
		case StateUpdate:
		//  Update block internal state
		
		err = r_IN(0, 0) - r_IN(1, 0);
		update(blk, &areg, err , 0);
		speed = areg.y;
		
		phase += speed/fs;

		break;
		
		case OutputUpdate:
		//  Put the value on the output port
		r_OUT(0, 0) = phase;
		r_OUT(1, 0) = speed;
		break;
		
    }
}
*/

struct pi_reg_state{
	int32_t ki;
	int32_t kp;
	int32_t a;
	int32_t y;	
};
static inline void update(scicos_block *blk, struct pi_reg_state *s, int32_t e, int32_t fs)
{
	int32_t a = s->a;
	int32_t d = s->ki*e;
	
	// will accumulator grow up?
	if(fs) if( ((a>0)&&(d>0))||((a<0)&&(d<0)) ) d = 0;

	a += d;
	s->y = e*s->kp + a;
	s->a = a;
}
void angle_tracker(scicos_block *blk, int flag)
{
	static struct pi_reg_state areg = {0, 0, 0, 0};
	static int32_t speed = 0;
	static int32_t phase = 0;
	int32_t err = 0;
	
    switch(flag)
    {
		case Initialization:
		// Initialization
		phase = 0;
		speed = 0;
		
		areg.ki = (100*256)/3000; //(int32_t)(r_IN(0, 0)*1024);
		areg.kp = 20*256;  //(int32_t)(r_IN(0, 1)*1024);
		areg.a = 0;
		areg.y = 0;

		break;

		case Ending:
		// Simulation ending
		break;		
		
		case StateUpdate:
		//  Update block internal state
		
		err = (int32_t)(1024*r_IN(0, 0)) - (int32_t)(1024*r_IN(1, 0));
		update(blk, &areg, err , 0);
		speed = areg.y;
		
		phase += speed;

		break;
		
		case OutputUpdate:
		//  Put the value on the output port
		r_OUT(0, 0) = ((double)phase)/1024.0/256/3000.0;
		r_OUT(1, 0) = ((double)speed)/1024.0/256;
		break;
		
    }
}
