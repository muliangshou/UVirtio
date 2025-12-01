#include <one_order_kalman.h>
/**
 *      equation of state       x = A*x + w; w~N(0,Q)
 *      Measurement equation    y = C*x + v; v~N(0,R)
 * 
 */

void OneOrderKalmamInit(OneOrderKalmanHander *hander,float A,float C,float Q ,float R)
{
    hander->A = A;
	hander->A2 = A * A;
	hander->C = C;
	hander->C2 = C * C;
	hander->Q = Q;
	hander->R = R;
	hander->x = 0;
	hander->P = Q;
	hander->K = 1;
}

float OneOrderKalmanFun(OneOrderKalmanHander *hander,float measure)
{
    hander->x = hander->A * hander->x;                                               //state prediction 
    hander->P = hander->A2 * hander->P + hander->Q;                                  //covariance prediction
    hander->K = hander->P * hander->C / (hander->C2 * hander->P + hander->R);        //calculate kalman gain
    hander->x = hander->x + hander->K * (measure- hander->C * hander->x);            //state correct
    hander->P = (1 - hander->K * hander->C) * hander->P;                             //covariance correction
    return hander->C * hander->x; 	                                                 //return resault   
}