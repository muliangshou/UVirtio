#include<one_order_rc_lpf.h>
void OneOrderRcLpfInit(OneOrderRcLpfHander* hander,float cutoff_fre,float sampling)
{
    hander->vi = 0;
    hander->fcutoff = cutoff_fre;   // low pass filter cutoff frequency
    hander->vo = 0;
    hander->vo_prev = 0;
    hander->fs = sampling;          //sampling rate
}

float OneOrderRcLpfFun(OneOrderRcLpfHander *hander)
{
    float rc;
    float alpha1;
    float alpha2;
    rc = (float) 1.0/2.0/3.1415926/hander->fcutoff;
    alpha1 = 1/(1+rc*hander->fs);
    alpha2 = rc*hander->fs/(1+rc*hander->fs);
    hander->vo = alpha1*hander->vi + alpha2 *hander->vo_prev;
    hander->vo_prev = hander ->vo;
    return hander->vo;
}
