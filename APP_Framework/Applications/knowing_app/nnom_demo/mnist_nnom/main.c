/*
 * Copyright (c) 2018-2020, Jianjia Ma
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2019-03-29     Jianjia Ma   first implementation
 */

#include <stdio.h>
#include <transform.h>

#include "nnom.h"
#include "image.h"
#include "weights.h"

nnom_model_t *model;

const char codeLib[] = "@B%8&WM#*oahkbdpqwmZO0QLCJUYXzcvunxrjft/\\|()1{}[]?-_+~<>i!lI;:,\"^`'.   ";
void print_img(int8_t * buf)
{
    for(int y = 0; y < 28; y++) 
	{
        for (int x = 0; x < 28; x++) 
		{
            int index =  69 / 127.0 * (127 - buf[y*28+x]); 
			if(index > 69) index =69;
			if(index < 0) index = 0;
            printf("%c",codeLib[index]);
			printf("%c",codeLib[index]);
        }
        printf("\n");
    }
}

// Do simple test using image in "image.h" with model created previously.
void mnist_nnom(int argc, char **argv)
{
	model = nnom_model_create();

	uint32_t tick, time;
	uint32_t predic_label;
	float prob;
	int32_t index = atoi(argv[1]);

	if (index >= TOTAL_IMAGE || argc != 2)
	{
		printf("Please input image number within %d\n", TOTAL_IMAGE - 1);
		return;
	}

	printf("\nprediction start.. \n");
	#ifdef __RT_THREAD_H__
	tick = rt_tick_get();
	#endif

	memcpy(nnom_input_data, (int8_t *)&img[index][0], 784);
	nnom_predict(model, &predic_label, &prob);

	#ifdef __RT_THREAD_H__
	time = rt_tick_get() - tick;
	#endif
	// print original image to console
	print_img((int8_t *)&img[index][0]);

	#ifdef __RT_THREAD_H__
	printf("Time: %d tick\n", time);
	#endif
	printf("Truth label: %d\n", label[index]);
	printf("Predicted label: %d\n", predic_label);
	printf("Probability: %d%%\n", (int)(prob * 100));
}

#ifdef __RT_THREAD_H__
MSH_CMD_EXPORT(mnist_nnom, nnom mnist demo and image number should be followed);
#endif
