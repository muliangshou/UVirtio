/*
 * @Author: chunyexixiaoyu
 * @Date: 2021-08-16 15:16:51
 * @LastEditTime: 2021-08-18 14:48:11
 * @LastEditors: Please set LastEditors
 * @Description: In User Settings Edit
 * @FilePath: \xiuos\APP_Framework\lib\queue\queue.h
 */
#ifndef __QUEUE_H__
#define __QUEUE_H__
#include <stdio.h>
#define OK 1
#define ERROR 0
#define TRUE 1
#define FALSE 0
#define MAXSIZE 1024 

typedef int Status; 
typedef int QElemType;
typedef struct
{
	QElemType data[MAXSIZE];
	int front;    	
	int rear;		
}SqQueue;

Status InitQueue(SqQueue *Q);
Status ClearQueue(SqQueue *Q);
Status QueueEmpty(SqQueue *Q);
Status GetHead(SqQueue *Q,QElemType *e);
Status EnQueue(SqQueue *Q,QElemType e);
Status DeQueue(SqQueue *Q,QElemType *e);
int QueueLength(SqQueue *Q);
#endif