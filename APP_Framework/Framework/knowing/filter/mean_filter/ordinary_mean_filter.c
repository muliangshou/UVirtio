#include <ordinary_mean_filter.h>
/**
 * @description: 
 * @param {int *} hander array address
 * @param {int} len array length
 * @return {*}
 */
int OrdinaryMeanFilter(int * hander,int len)
{
    int sum = 0;
    for(int i = 0;i < len ;i++)
    {
        sum = sum + *(hander +i);
    }
    return (int)(sum/len);
}