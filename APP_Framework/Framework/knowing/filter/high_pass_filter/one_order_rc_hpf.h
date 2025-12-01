#ifndef _ONE_ORDER_RC_HPF_H 
#define _ONE_ORDER_RC_HPF_H

typedef struct 
{
	float  vi;
	float  vi_prev;
    float  vo_prev;
	float  vo;
	float  fcutoff;
	float  fs;
} OneOrderRcHpfHander;

void OneOrderRcHpfInit(OneOrderRcHpfHander* hander,float cutoff_fre,float sampling);
float OneOrderRcHpfFun(OneOrderRcHpfHander *hander);

#endif