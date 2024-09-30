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

#include "fsl.h"
#include "display.h"
#include "magcard.h"
#include "LEDs.h"




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
	init_fsl();



    hw_EnableInterrupts();






	while(1)
	{


//	    int key = read_key();
//	    if(key != 0)
//	    	MY_PRINTF("%d\n", key);
		update_fsl();

//		bool flag = MagCardGetStatus();
//		if (flag) {
//			uint64_t data = MagCardGetCardNumber();
//			MagCardClearData();
//			MY_PRINTF("Data: %llu\n", data);
//		}
	}





    return 0 ;
}



















