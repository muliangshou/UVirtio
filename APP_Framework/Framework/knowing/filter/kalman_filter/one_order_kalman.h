#ifndef _ONE_ORDER_KALMAN_H_
#define _ONE_ORDER_KALMAN_H_
/**
 *      equation of state       x = A*x + w; w~N(0,Q)
 *      Measurement equation    y = C*x + v; v~N(0,R)
 * 
 */
typedef struct 
{
    float A;
    float C;
	float A2;  //A*A
    float C2;  //C*C
	float Q;
    float R;
    float K;
	float P;
	float x;
}OneOrderKalmanHander;

void OneOrderKalmamInit(OneOrderKalmanHander *hander,float A,float C,float Q ,float R);
float OneOrderKalmanFun(OneOrderKalmanHander *hander,float measure);
#endif