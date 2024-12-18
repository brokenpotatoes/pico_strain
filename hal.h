/**
 * \copyright Copyright (C) 2019 Texas Instruments Incorporated - http://www.ti.com/
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *    Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *    Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the
 *    distribution.
 *
 *    Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef INTERFACE_H_
#define INTERFACE_H_

#include "ads131m0x.h"


//****************************************************************************
//
// Standard libraries
//
//****************************************************************************

#include <stdbool.h>
#include <stdint.h>



//****************************************************************************
//
// Insert processor specific header file(s) here
//
//****************************************************************************

/*  --- INSERT YOUR CODE HERE --- */
#include "pico/stdlib.h"
#include "hardware/spi.h"
#include "hardware/clocks.h"

//*****************************************************************************
//
// Pin definitions (RP2350)
//
//*****************************************************************************

#define nDRDY_PIN           7
#define nCS_PIN             9
#define nSYNC_nRESET_PIN    12
//#define CLK_PIN             13
#define CLK_PIN             21 //Only for RP2040
#define MISO_PIN            8
#define MOSI_PIN            11
#define SCK_PIN             10 

//*****************************************************************************
//
// Function Prototypes
//
//*****************************************************************************
void    initGPIO(void);
void    initSPI(void);
void    delay_ms(const uint32_t delay_time_ms);
void    delay_us(const uint32_t delay_time_us);
void    setCS(const bool state);
void    setSYNC_RESET(const bool state);
void    toggleSYNC(void);
void    toggleRESET(void);
void    spiSendReceiveArrays(const uint8_t DataTx[], uint8_t DataRx[], const uint8_t byteLength);
uint8_t spiSendReceiveByte(const uint8_t dataTx);
bool    waitForDRDYinterrupt(const uint32_t timeout_ms);

// Functions used for testing only
bool    getCS(void);
bool    getSYNC_RESET(void);
void    setSYNC_RESET(const bool state);



//*****************************************************************************
//
// Macros
//
//*****************************************************************************
/** Alias used for setting GPIOs pins to the logic "high" state */
#define HIGH                ((bool) true)

/** Alias used for setting GPIOs pins to the logic "low" state */
#define LOW                 ((bool) false)


// Flag to indicate if a /DRDY interrupt has occurred
extern volatile bool flag_nDRDY_INTERRUPT;


#endif /* INTERFACE_H_ */
