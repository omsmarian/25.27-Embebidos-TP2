/*
 * Copyright 2016-2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    Template.c
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "SysTick.h"


/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

void test_1 (void);
void test_2 (void);
void test_3 (void);

//Create the structure for the function pointers
SysTick_Callback_t callback[3] = { {test_1, 1000}, {test_2, 2000}, {test_3, 3000}  };


int main(void) {

    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();

    /* Init FSL debug console. */
    BOARD_InitDebugConsole();

    PRINTF("Running\r\n");
    SysTick_Init(3, callback[0], callback[1], callback[2]);
    PRINTF("SysTick Initialized\r\n");

    while(1)
    {

    }

    return 0 ;
}


void test_1 (void)
{
	PRINTF("1\r\n");
}

void test_2(void) {
	PRINTF("2\r\n");
}

void test_3(void) {
	PRINTF("3\r\n");
}
