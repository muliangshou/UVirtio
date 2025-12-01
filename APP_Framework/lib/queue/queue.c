/*
 * @Author: chunyexixiaoyu
 * @Date: 2021-08-16 15:16:51
 * @LastEditTime: 2021-08-18 14:45:47
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \xiuos\APP_Framework\lib\queue\queue.c
 */
#include <queue.h>


/**
 * @description: Initialize an empty queue
 * @param {SqQueue} *Q queue struct
 * @return {*}
 */
Status InitQueue(SqQueue *Q)
{
	Q->front=0;
	Q->rear=0;
	return  OK;
}

/**
 * @description: Clear Q to an empty queue
 * @param {SqQueue} *Q queue struct
 * @return {*}
 */
Status ClearQueue(SqQueue *Q)
{
	Q->front=Q->rear=0;
	return OK;
}

/**
 * @description: Return TRUE if Q is an empty queue, FALSE otherwise
 * @param {SqQueue} *Q queue struct
 * @return TRUE
 * 		   FALSE
 */
Status QueueEmpty(SqQueue *Q)
{ 
	if(Q->front==Q->rear) 
		return TRUE;
	else
		return FALSE;
}

/**
 * @description: Returns the number of elements of Q, which is the current length of the queue
 * @param {SqQueue} *Q queue struct
 * @return length of the queue
 */
int QueueLength(SqQueue *Q)
{
	return  (Q->rear-Q->front+MAXSIZE)%MAXSIZE;
}

/**
 * @description: If the queue is not empty, return the header element of Q with e and OK, otherwise return ERROR 
 * @param {SqQueue} *Q queue struct
 * @param {QElemType} *e  header element	
 * @return TRUE
 * 		   FALSE
 */
Status GetHead(SqQueue *Q,QElemType *e)
{
	if(Q->front==Q->rear) 
		return ERROR;
	*e=Q->data[Q->front];
	return OK;
}

/**
 * @description: If the queue is not full, insert element E as Q's new tail element
 * @param {SqQueue} *Q queue struct
 * @param {QElemType} e new element
 * @return TRUE insert successfully
 * 		   FALSE
 */
Status EnQueue(SqQueue *Q,QElemType e)
{
	if ((Q->rear+1)%MAXSIZE == Q->front)	
		return ERROR;
	Q->data[Q->rear]=e;			
	Q->rear=(Q->rear+1)%MAXSIZE;								
	return  OK;
}

/**
 * @description: If the queue is not empty, the header element in Q is removed and its value is returned with e
 * @param {SqQueue} *Q
 * @param {QElemType} *e the header element
 * @return {*}
 */
Status DeQueue(SqQueue *Q,QElemType *e)
{
	if (Q->front == Q->rear)		
		return ERROR;
	*e=Q->data[Q->front];			
	Q->front=(Q->front+1)%MAXSIZE;
	return  OK;
}

