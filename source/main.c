/*
 * Copyright 2016-2024 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file    tp1_interrupciones.c
 * @brief   Application entry point.
 */


#include <stdio.h>
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"





/* TODO: insert other include files here. */

/* TODO: insert other definitions and declarations here. */

/*
 * @brief   Application entry point.
 */

int main(void) {

    /* Init board hardware. */
	BOARD_InitDebugConsole();
	hw_Init();
    hw_DisableInterrupts();

	MY_PRINTF("Starting the program\n");



    hw_EnableInterrupts();






	while(1)
	{
		PRINTF("TEST\n");
	}





    return 0 ;
}



















