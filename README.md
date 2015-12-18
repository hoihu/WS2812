WS2812 led strip support code for MSP430 launchpad
==================================================

This code runs on the TI MSP430 5529 launchpad:
http://www.ti.com/tool/msp-exp430f5529lp?DCMP=msp-f5529&HQS=msp-f5529-b

it supports the led strips with the 800kHz WS2812 controller chips, e.g.  
http://www.adafruit.com/products/1138

The code uses the DMA controller to efficiently transfer the LED data to the strip 
so it releases the CPU from time-critical work.

The LED strip must be connected to P3.3 on the launchpad header

An LED strip of 60 LEDs takes about 2msec to update

The code serves demonstration purposes only, may need cleanup. Have fun!
