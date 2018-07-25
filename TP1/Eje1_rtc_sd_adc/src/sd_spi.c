/* Copyright 2014, ChaN
 * Copyright 2016, Matias Marando
 * Copyright 2016, Eric Pernia
 * All rights reserved.
 *
 * This file is part of Workspace.
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
 *
 */

/*==================[inlcusiones]============================================*/

#include "sd_spi.h"   // <= own header (optional)
#include "sapi.h"     // <= sAPI header

#include "ff.h"       // <= Biblioteca FAT FS


#include <string.h>   // <= Biblioteca de manejo de Strings, ver:
// https://es.wikipedia.org/wiki/String.h
// http://www.alciro.org/alciro/Programacion-cpp-Builder_12/funciones-cadenas-caracteres-string.h_448.htm

/*==================[definiciones y macros]==================================*/

#define FILENAME  "log.txt"

#define SEPARATOR ","
/*==================[definiciones de datos internos]=========================*/

static FATFS fs;           // <-- FatFs work area needed for each volume
static FIL fp;             // <-- File object needed for each open file
static char rtcString[22];
static char adc1String[6],adc2String[6],adc3String[6];
static char textToWrite[62];


DEBUG_PRINT_ENABLE;

/*==================[definiciones de datos externos]=========================*/

/*==================[declaraciones de funciones internas]====================*/

/*==================[declaraciones de funciones externas]====================*/

// FUNCION que se ejecuta cada vezque ocurre un Tick
void diskTickHook( void *ptr );

/*==================[declaraciones de funciones internas]====================*/

// Se termina el programa dejandolo en un loop infinito
static void stopProgram( void );

// Se monta el sistema de archivos y Tarjeta SD
static void mountSDCard( void );

// Guarda en str la fecha y hora convertido a string en formato "DD/MM/YYYY HH:MM:SS"
static void rtcToString( char* str, rtc_t* rtc );





// Se termina el programa dejandolo en un loop infinito
static void stopProgram( void ){
   debugPrintlnString( "Fin del programa." );
   while(TRUE){
      sleepUntilNextInterrupt();
   }
}


// Se monta el sistema de archivos y Tarjeta SD
static void mountSDCard( void ){
   // Give a work area to the default drive
   if( f_mount( &fs, "", 0 ) != FR_OK ){
      // If this fails, it means that the function could not register a file
      // system object. Check whether the SD card is correctly connected.
      debugPrintlnString( "Error intentando montar la tarjeta SD." );
      stopProgram();
   }
}

// Guarda en str la fecha y hora convertido a string en formato "DD/MM/YYYY HH:MM:SS"
static void rtcToString( char* str, rtc_t* rtc ){

   char rtcMemberString[10];
   rtcString[0] = 0;

   // "DD/MM/YYYY "

   // Conversion de entero a string con base 10 (decimal)
   int64ToString( (rtc->mday), rtcMemberString, 10 );
   if( (rtc->mday)<10 ){
      // Concateno rtcString+"0"
      strncat( rtcString, "0", strlen("0") );
   }
   // Concateno rtcString+mday+"/"
   strncat( rtcString, rtcMemberString, strlen(rtcMemberString) );
   strncat( rtcString, "/", strlen("/") );

   // Conversion de entero a string con base 10 (decimal)
   int64ToString( (rtc->month), rtcMemberString, 10 );
   if( (rtc->month)<10 ){
      // Concateno rtcString+"0"
      strncat( rtcString, "0", strlen("0") );
   }
   // Concateno rtcString+month+"/"
   strncat( rtcString, rtcMemberString, strlen(rtcMemberString) );
   strncat( rtcString, "/", strlen("/") );

   // Conversion de entero a string con base 10 (decimal)
   int64ToString( (rtc->year), rtcMemberString, 10 );
   // Concateno rtcString+year+" "
   strncat( rtcString, rtcMemberString, strlen(rtcMemberString) );
   strncat( rtcString, " ", strlen(" ") );

   // "HH:MM:SS"

   // Conversion de entero a string con base 10 (decimal)
   int64ToString( (rtc->hour), rtcMemberString, 10 );
   if( (rtc->hour)<10 ){
      // Concateno rtcString+"0"
      strncat( rtcString, "0", strlen("0") );
   }
   // Concateno rtcString+hour+":"
   strncat( rtcString, rtcMemberString, strlen(rtcMemberString) );
   strncat( rtcString, ":", strlen(":") );

   // Conversion de entero a string con base 10 (decimal)
   int64ToString( (rtc->min), rtcMemberString, 10 );
   if( (rtc->min)<10 ){
      // Concateno rtcString+"0"
      strncat( rtcString, "0", strlen("0") );
   }
   // Concateno rtcString+min+":"
   strncat( rtcString, rtcMemberString, strlen(rtcMemberString) );
   strncat( rtcString, ":", strlen(":") );

   // Conversion de entero a string con base 10 (decimal)
   int64ToString( (rtc->sec), rtcMemberString, 10 );
   if( (rtc->sec)<10 ){
      // Concateno rtcString+"0"
      strncat( rtcString, "0", strlen("0") );
   }
   // Concateno rtcString+sec
   strncat( rtcString, rtcMemberString, strlen(rtcMemberString) );
}


/*==================[funcion principal]======================================*/

// FUNCION PRINCIPAL, PUNTO DE ENTRADA AL PROGRAMA LUEGO DE ENCENDIDO O RESET.
int main( void ){

   // ---------- CONFIGURACIONES ------------------------------
   // Inicializar y configurar la plataforma
   boardConfig();

   // UART for debug messages
   debugPrintConfigUart( UART_USB, 115200 );
   debugPrintlnString( "Este programa loguea el 3 canales ADC en un archivo en la tarjeta SD" );
   debugPrintEnter();



   // Inicializar el conteo de Ticks con resolucion de 10ms,
   // con tickHook diskTickHook
   tickConfig( 10 );
   tickCallbackSet( diskTickHook, NULL );

   // ------ PROGRAMA QUE ESCRIBE EN LA SD -------
   // Inicializar ADC
   adcConfig( ADC_ENABLE );

   // SPI configuration
   spiConfig( SPI0 );

   // Estructura RTC
    rtc_t rtc;
    rtc.year = 2017;
    rtc.month = 6;
    rtc.mday = 10;
    rtc.wday = 1;
    rtc.hour = 10;
    rtc.min = 35;
    rtc.sec= 0;
    // Inicializar RTC
    rtcConfig( &rtc );

    uint8_t counter = 0;
    uint16_t adcValue = 0;


    // bloquear durante 1s para darle tiempo a cambiar el valor del RTC
    delay_t delay1s;
    delayConfig( &delay1s, 1000 );

    delay(8000); // El RTC tarda en setear la hora, por eso el delay

    // Montar sistema de archivo y tarjeta SD
    mountSDCard();

   UINT nbytes;



     /* ------------- REPETIR POR SIEMPRE ------------- */
   while(1) {

      if( delayRead( &delay1s ) ){
    	    // Leer fecha y hora
    	      rtcRead( &rtc );
    	      // Convertir lectura de fecha y hora a string en formato "DD/MM/YYYY HH:MM:SS"
    	      rtcToString( rtcString, &rtc );
    	      // Agrego SEPARATOR al final
    	      strncat( rtcString, SEPARATOR, 1 );

    	      // Leer adc
    	      // Se debe esperar minimo 67ms entre lecturas su la tasa es de 15Hz
    	      adcValue = adcRead( CH1 );
    	      // Convertir lectura del ADC a string (va a dar un valor de "0" a "1023")
    	      int64ToString( (int64_t)adcValue, adc1String, 10 );

    	      delay(70);
    	      adcValue = adcRead( CH2 );
    	      // Convertir lectura del ADC a string (va a dar un valor de "0" a "1023")
    	      int64ToString( (int64_t)adcValue, adc2String, 10 );

    	      delay(70);
      	      adcValue = adcRead( CH3);
       	      // Convertir lectura del ADC a string (va a dar un valor de "0" a "1023")
       	      int64ToString( (int64_t)adcValue, adc3String, 10 );


    	      // Concatento textToWrite con rtcString quedando en textToWrite: rtcString
    	      strncat( textToWrite, rtcString, strlen(rtcString) );
    	      // Concatento textToWrite con magString quedando textToWrite: rtcString+adcString
    	      strncat( textToWrite, "CH1 = ", 6 );
    	      strncat( textToWrite, adc1String, strlen(adc1String) );
    	      strncat( textToWrite, "; CH2 = ", 8 );
    	      strncat( textToWrite, adc2String, strlen(adc2String) );
    	      strncat( textToWrite, "; CH3 = ", 8 );
    	      strncat( textToWrite, adc3String, strlen(adc3String) );


    	      // Concatento textToWrite con "\r\n" quedando textToWrite: rtcString+adcString+"\r\n"
    	      strncat( textToWrite, "\r\n", 2 );
    	      // Muestro por UART
       	      debugPrintString( textToWrite );
        	  debugPrintEnter();


    	      // Grabar muestra en la Tarjeta SD
    	      // Create/open a file, then write a string and close it
    	      if( f_open( &fp, FILENAME, FA_WRITE | FA_OPEN_APPEND ) == FR_OK ){

    	         f_write( &fp, textToWrite, strlen(textToWrite), &nbytes );
    	         f_close(&fp);
    	      }

    	      // Incremento el contador de muestras guardadas
    	      counter++;

    	      // Reeteo variables para la proxima muestra
    	      adcValue = 0;
    	      textToWrite[0] = 0;

      }

   }
   //---------------------------------------------------------------



   // NO DEBE LLEGAR NUNCA AQUI, debido a que a este programa se ejecuta
   // directamenteno sobre un microcontroladore y no es llamado/ por ningun
   // Sistema Operativo, como en el caso de un programa para PC.
   return 0;
}

/*==================[definiciones de funciones internas]=====================*/

/*==================[definiciones de funciones externas]=====================*/

// FUNCION que se ejecuta cada vezque ocurre un Tick
void diskTickHook( void *ptr ){
   disk_timerproc();   // Disk timer process
}


/*==================[fin del archivo]========================================*/
