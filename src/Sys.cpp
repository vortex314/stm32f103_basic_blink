/*
 * Sys.cpp
 *
 *  Created on: May 15, 2016
 *      Author: lieven
 */

#include <Sys.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
extern "C" void sys_tick_handler(void)
{
	Sys::tick();
}

uint32_t _upTime=0;

void Sys::tick(){
	_upTime++;
}

uint64_t Sys::millis(){ 
	return _upTime;
}

void Sys::delay(uint32_t delta){
	uint64_t endTime = millis()+delta;
	while ( endTime > millis());
}

