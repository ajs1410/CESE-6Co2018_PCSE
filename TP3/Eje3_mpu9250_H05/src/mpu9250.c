/* Copyright 2016, Alejandro Permingeat.
 * Copyright 2016, Eric Pernia.
 * Copyright 2018, Sergio Renato De Jesus Melean <sergiordj@gmail.com>.
 *
 * All rights reserved.
 *
 * This file is part sAPI library for microcontrollers.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/* Date: 2018-07-06 */

/*==================[inclusions]=============================================*/

#include "sapi.h"               // <= sAPI header
#include <string.h>
/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

// MPU9250 Address
MPU9250_address_t addr = MPU9250_ADDRESS_0; // If MPU9250 AD0 pin is connected to GND

#define UART_PC        UART_USB
#define UART_BLUETOOTH UART_232

/*==================[definiciones de datos internos]=========================*/

DEBUG_PRINT_ENABLE
CONSOLE_PRINT_ENABLE

/*==================[definiciones de datos externos]=========================*/

/*==================[declaraciones de funciones internas]====================*/

/*==================[declaraciones de funciones externas]====================*/

bool_t hm10bleTest( int32_t uart );
void hm10blePrintATCommands( int32_t uart );


/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

/* FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE RESET. */
int main(void){
   /* ------------- INICIALIZACIONES ------------- */
char strGiroscopo[400], strMagnetometro[400],strAcelerometro[400],strTemperatura[400];
boardConfig();

   // Inicializar UART_232 para conectar al modulo bluetooth
     consolePrintConfigUart( UART_BLUETOOTH, 9600 );
     debugPrintlnString( "UART_BLUETOOTH para modulo Bluetooth configurada." );

     // Inicializar UART_USB para conectar a la PC
     debugPrintConfigUart( UART_PC, 115200 );
     debugPrintlnString( "UART_PC configurada." );


     uint8_t data = 0;

     uartWriteString( UART_PC, "Testeto si el modulo esta conectado enviando: AT\r\n" );
      if( hm10bleTest( UART_BLUETOOTH ) ){
    	  uartWriteString( UART_PC,"Modulo conectado correctamente." );
      }



   // Inicializar la IMU
      uartWriteString( UART_PC,"Inicializando IMU MPU9250...\r\n" );
   int8_t status;
   status = mpu9250Init( addr );

   if( status < 0 ){
	   uartWriteString( UART_PC, "IMU MPU9250 no inicializado, chequee las conexiones:\r\n\r\n" );
	   uartWriteString( UART_PC, "MPU9250 ---- EDU-CIAA-NXP\r\n\r\n" );
	   uartWriteString( UART_PC, "    VCC ---- 3.3V\r\n" );
	   uartWriteString( UART_PC, "    GND ---- GND\r\n" );
	   uartWriteString( UART_PC, "    SCL ---- SCL\r\n" );
	   uartWriteString( UART_PC, "    SDA ---- SDA\r\n" );
	   uartWriteString( UART_PC, "    AD0 ---- GND\r\n\r\n" );
	   uartWriteString( UART_PC, "Se detiene el programa.\r\n" );
      while(1);
   }
   uartWriteString( UART_PC,"IMU MPU9250 inicializado correctamente.\r\n\r\n" );

   /* ------------- REPETIR POR SIEMPRE ------------- */
   while(TRUE){

      //Leer el sensor y guardar en estructura de control
      mpu9250Read();

      // Imprimir resultados
       sprintf( strGiroscopo,"Giroscopo:      (%f, %f, %f)   [rad/s]\r\n",
              mpu9250GetGyroX_rads(),
              mpu9250GetGyroY_rads(),
              mpu9250GetGyroZ_rads()
            );
      uartWriteString( UART_PC,strGiroscopo);
      uartWriteString( UART_BLUETOOTH,strGiroscopo);


		sprintf(strAcelerometro, "Acelerometro:   (%f, %f, %f)   [m/s2]\r\n",
              mpu9250GetAccelX_mss(),
              mpu9250GetAccelY_mss(),
              mpu9250GetAccelZ_mss()
            );
	      uartWriteString( UART_PC,strAcelerometro);
	      uartWriteString( UART_BLUETOOTH,strAcelerometro);


		sprintf(strMagnetometro, "Magnetometro:   (%f, %f, %f)   [uT]\r\n",
              mpu9250GetMagX_uT(),
              mpu9250GetMagY_uT(),
              mpu9250GetMagZ_uT()
            );
		 uartWriteString( UART_PC,strMagnetometro);
		 uartWriteString( UART_BLUETOOTH,strMagnetometro);


		sprintf(strTemperatura, "Temperatura:    %f   [C]\r\n\r\n",
              mpu9250GetTemperature_C()
            );
		 uartWriteString( UART_PC,strTemperatura);
		 uartWriteString( UART_BLUETOOTH,strTemperatura);


      delay(3000);
   }

   /* NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa no es llamado
      por ningun S.O. */
   return 0 ;
}

/*==================[end of file]============================================*/
bool_t hm10bleTest( int32_t uart )
{
   uartWriteString( uart, "AT\r\n" );
   return waitForReceiveStringOrTimeoutBlocking( uart,
                                                 "OK\r\n", strlen("OK\r\n"),
                                                 50 );
}

void hm10blePrintATCommands( int32_t uart )
{
   delay(500);
   uartWriteString( uart, "AT+HELP\r\n" );
}
