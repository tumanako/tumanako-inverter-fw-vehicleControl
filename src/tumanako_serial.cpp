//------------------------------------------------------------------------------
//   TumanakoVC - Electric Vehicle and Motor control software
//   Copyright (C) 2011 Philip Court <philip@greenstage.co.nz>
//
//   This file is part of TumanakoVC.
//
//   TumanakoVC is free software: you can redistribute it and/or modify
//   it under the terms of the GNU Lesser General Public License as published
//   by the Free Software Foundation, either version 3 of the License, or
//   (at your option) any later version.
//
//   TumanakoVC is distributed in the hope that it will be useful,
//   but WITHOUT ANY WARRANTY; without even the implied warranty of
//   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//   GNU Lesser General Public License for more details.
//
//   You should have received a copy of the GNU Lesser General Public License
//   along with TumanakoVC.  If not, see <http://www.gnu.org/licenses/>.
//
// DESCRIPTION:
//   This module setups up the serial port for diognostic output and it 
//   provides a few helper functions.
//
//------------------------------------------------------------------------------

#define STM32F1  //applicable to the STM32F1 series of devices

#include "tumanako_serial.hpp"
#include "tumanako_global.hpp"

#include <iostream>
#include <string.h>  //c string
#include <string> //c++ string

using namespace std;

extern "C"  {
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/f1/gpio.h>
#include <libopencm3/stm32/f1/rcc.h>
#include <libopencm3/cm3/common.h>
#include <stdio.h>
#include <stdarg.h>  //variable args
}

// Private defines
#define TK_USART_PORT USART1
#define TK_TxBufferSize   (countof(TxBuffer) - 1)

// Private macros
#define countof(a)   (sizeof(a) / sizeof(*(a)))

// Private variables
char TxBuffer[] = \
                  " \f\x1b\x5b\x48\x1b\x5b\x32\x4a" \
                  " TumanakoVC - Electric Vehicle and Motor control software\n\r" \
                  " Copyright (C) 2011 Philip Court <philip@greenstage.co.nz>\n\r" \
                  " This program comes with ABSOLUTELY NO WARRANTY.\n\r" \
                  " This is free software, and you are welcome to redistribute it\n\r" \
                  " under certain conditions.\n\r" \
                  " See the GNU Lesser General Public License for more details.\n\r";

void usartInit() {
  gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,
                GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX);

  /* Setup UART parameters. */
  usart_set_baudrate(USART1, 115200);
  //usart_set_baudrate(USART1, 38400);
  usart_set_databits(USART1, 8);
  usart_set_stopbits(USART1, USART_STOPBITS_1);
  usart_set_mode(USART1, USART_MODE_TX_RX);
  usart_set_parity(USART1, USART_PARITY_NONE);
  usart_set_flow_control(TK_USART_PORT, USART_FLOWCONTROL_NONE);

  /* Finally enable the USART. */
  usart_enable(TK_USART_PORT);

  usartWriteDisclaimer();
}

//Crusty hack left over from before we had printf function
void printFormat( const char* Format, ... ) {
  char tempChars[60];

  va_list Arguments;
  va_start(Arguments, Format);
  double FArg;
  int IArg;
  char * SArg;

  for (int i = 0; Format[i] != '\0'; ++i ) {
    if (Format[i] == 'f') {
      FArg=va_arg(Arguments, double);
      sprintf(tempChars,"%.1lf",FArg);

    } else if (Format[i] == 'F') {
      FArg=va_arg(Arguments, double);
      sprintf(tempChars,"%.4lf",FArg);

    } else if (Format[i] == 'i') {
      IArg=va_arg(Arguments, int);
      sprintf(tempChars, "%3d",IArg);
    } else if (Format[i] == 'b') {
      IArg=va_arg(Arguments, int);
      if (IArg==0)
        sprintf(tempChars, " off");
      else
        sprintf(tempChars, "  ON");
    } else if (Format[i] == 'I') {
      IArg=va_arg(Arguments, int);
      sprintf(tempChars, "%5d",IArg);
    } else if (Format[i] == 's') {
      SArg=va_arg(Arguments, char *);
      sprintf(tempChars, "%s",SArg);
    } else if (Format[i] == 'c') {
      SArg=va_arg(Arguments, char *);
      sprintf(tempChars, "%s",SArg);
    }
    usartWriteChars(tempChars);
  }
  va_end(Arguments);
}

void usartWriteDisclaimer() {
  // Figure out what features this software has been built with
  // and inform the user
  string features("");

#ifdef TUMANAKO_SKAI
  features += "SKAI";
#else
  features += "SKiiP";
#endif

#ifdef TUMANAKO_KIWIAC
  features += " KiwiAC";
#else
  features += " STMDevKit";
#endif

#ifdef TUMANAKO_WS28
  features += " WS28";
#elif defined (TUMANAKO_WS20)
  features += " WS20";
#else
  features += " TestMotor";
#endif

#ifdef TUMANAKO_SHIRE
  features += " - ShireVan";
#endif

  if (features.size()!=0) features = " (" + features + ")";

  usartWriteChars(TxBuffer);
  printFormat( "ssss", "Version: ",TUMANKAO_VERSION, features.c_str(),"\n\r");
}

//write chars to USART
void usartWriteChars(char const * chars) {
  printf("%s", chars);
}

void usartWriteChars(char * chars) {
  printf("%s", chars);
}

//write numbers to USART
void usartWrite(long a, long b, long c, long d, long e) {
  char tempChars[30];

  sprintf(tempChars, "% 5ld, ", a);
  usartWriteChars(tempChars);

  sprintf(tempChars, "% 5ld, ", b);
  usartWriteChars(tempChars);

  sprintf(tempChars, "% 5ld, ", c);
  usartWriteChars(tempChars);

  sprintf(tempChars, "% 5ld, ", d);
  usartWriteChars(tempChars);

  sprintf(tempChars, "% 5ld      \r", e);
  usartWriteChars(tempChars);
}
