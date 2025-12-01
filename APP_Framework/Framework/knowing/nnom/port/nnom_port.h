/*
 * Copyright (c) 2018-2020
 * Jianjia Ma
 * majianjia@live.com
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-02-05     Jianjia Ma   The first version
 */

#ifndef __NNOM_PORT_H__
#define __NNOM_PORT_H__

#include <stdlib.h>
#include <stdio.h>
#include <transform.h>

/* use static memory */
// must set buf using "nnom_set_static_buf()" before creating a model. 

/* dynamic memory interfaces */
/* when libc is not available, you shall implement the below memory interfaces (libc equivalents). */
#ifndef NNOM_USING_STATIC_MEMORY    
    #define nnom_malloc(n)      malloc(n)       
    #define nnom_free(p)        free(p)
#endif

/* memory interface */
/* when libc is not available, you shall implement your equivalent functions here */
#define nnom_memset(p,v,s)        memset(p,v,s)        
#define nnom_memcpy(dst,src,len)  memcpy(dst,src,len)  

/* runtime & debug */
#define nnom_us_get()       0       // return a microsecond timestamp
#define nnom_ms_get()       0       // return a millisecond timestamp
#define NNOM_LOG(...)       printf(__VA_ARGS__)

/* NNoM configuration */
#define NNOM_BLOCK_NUM  	(8)		// maximum number of memory blocks, increase it when log request.   
#define DENSE_WEIGHT_OPT 	(1)		// if used fully connected layer optimized weights. 

#endif



