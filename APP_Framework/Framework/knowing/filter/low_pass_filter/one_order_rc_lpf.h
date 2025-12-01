#ifndef _ONE_ORDER_RC_LPF_H 
#define _ONE_ORDER_RC_LPF_H


typedef struct 
{
	float  vi;
	float  vo_prev;
	float  vo;
	float  fcutoff;
	float  fs;
} OneOrderRcLpfHander;

void OneOrderRcLpfInit(OneOrderRcLpfHander* hander,float cutoff_fre,float sample);
float OneOrderRcLpfFun(OneOrderRcLpfHander *hander);
#endif