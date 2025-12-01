#include<one_order_rc_hpf.h>
void OneOrderRcHpfInit(OneOrderRcHpfHander* hander,float cutoff_fre,float sampling)
{
    hander->vi = 0;
    hander->fcutoff = cutoff_fre;   // low pass filter cutoff frequency
    hander->vo = 0;
    hander->vo_prev = 0;
    hander->fs = sampling;          //sampling rate
    hander->vi_prev = 0;
}

float OneOrderRcHpfFun(OneOrderRcHpfHander *hander)
{
    float rc;
    float alpha;
    rc = (float) 1.0/2.0/3.1415926/hander->fcutoff;
    alpha = rc/(rc+1/hander->fs);
    hander->vo = (hander->vi - hander->vi_prev + hander->vo_prev)*alpha;
    hander->vi_prev = hander ->vi;
    hander->vo_prev = hander ->vo;
    return hander->vo;
}
