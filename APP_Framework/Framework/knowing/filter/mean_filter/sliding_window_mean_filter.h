#ifndef _SLIDING_WINDOW_MEAN_FILTER_H
#define _SLIDING_WINDOW_MEAN_FILTER_H
#include <queue.h>

typedef struct 
{
    SqQueue window_queue;
    unsigned int window_len; //the window_len value must less than MAXSIZE 1024 
    long long int sumlast;
}SlidingWindowsHander;      


void SlidingWindowsMeanFliterInit(SlidingWindowsHander * hander,unsigned int window_len);
int SlidingWindowsMeanFliterFun(SlidingWindowsHander * hander,int newvalue);
#endif