#include <sliding_window_mean_filter.h>
void SlidingWindowsMeanFliterInit(SlidingWindowsHander * hander,unsigned int window_len)
{
    hander->window_len = window_len;
    hander->sumlast = 0;
    InitQueue(&(hander->window_queue));
}

int SlidingWindowsMeanFliterFun(SlidingWindowsHander * hander,int newvalue)
{
    int headtemf = 0;
    if(QueueLength(&(hander->window_queue)) < hander->window_len)
    {
        EnQueue(&(hander->window_queue),newvalue);
        hander->sumlast = hander->sumlast + newvalue;
        return (int)((hander->sumlast)/(QueueLength(&(hander->window_queue))));
    }
    else
    {
        DeQueue(&(hander->window_queue),&headtemf);
        hander->sumlast = hander->sumlast - headtemf +  newvalue;
        EnQueue(&(hander->window_queue),newvalue);
        return (int)((hander->sumlast)/(hander->window_len));
    }

}