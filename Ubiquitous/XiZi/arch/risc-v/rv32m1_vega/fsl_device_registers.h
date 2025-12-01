/*
 * Copyright 2014-2016 Freescale Semiconductor, Inc.
 * Copyright 2016-2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 *
 */

/**
* @file fsl_device_registers.h
* @brief support fsl device
* @version 1.0 
* @author AIIT XUOS Lab
* @date 2022-02-16
*/

/*************************************************
File name: fsl_device_registers.h
Description: support fsl device
Others: take for references
                https://github.com/open-isa-org/open-isa.org
History: 
1. Date: 2022-02-16
Author: AIIT XUOS Lab
Modification:  
*************************************************/

#ifndef __FSL_DEVICE_REGISTERS_H__
#define __FSL_DEVICE_REGISTERS_H__

#define CPU_RV32M1_ri5cy
/*
 * Include the cpu specific register header files.
 *
 * The CPU macro should be declared in the project or makefile.
 */
#if defined(CPU_RV32M1_cm0plus)

#define RV32M1_cm0plus_SERIES

/* CMSIS-style register definitions */
#include "RV32M1_cm0plus.h"
/* CPU specific feature definitions */
#include "RV32M1_cm0plus_features.h"

#elif defined(CPU_RV32M1_cm4)

#define RV32M1_cm4_SERIES

/* CMSIS-style register definitions */
#include "RV32M1_cm4.h"
/* CPU specific feature definitions */
#include "RV32M1_cm4_features.h"

#elif defined(CPU_RV32M1_zero_riscy)

#define RV32M1_zero_riscy_SERIES

/* CMSIS-style register definitions */
#include "RV32M1_zero_riscy.h"
/* CPU specific feature definitions */
#include "RV32M1_zero_riscy_features.h"

#elif defined(CPU_RV32M1_ri5cy)

#define RV32M1_ri5cy_SERIES

/* CMSIS-style register definitions */
#include "RV32M1_ri5cy.h"
/* CPU specific feature definitions */
#include "RV32M1_ri5cy_features.h"

#else
    #error "No valid CPU defined!"
#endif

#endif /* __FSL_DEVICE_REGISTERS_H__ */

/*******************************************************************************
 * EOF
 ******************************************************************************/
