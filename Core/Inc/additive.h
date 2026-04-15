/*
 * additive.h
 *
 *  Created on: Dec 27, 2023
 *      Author: jeffsnyder
 */

#ifndef INC_ADDITIVE_H_
#define INC_ADDITIVE_H_

#include "parameters.h"
#include "leaf.h"
#include "main.h"
#include "audiostream.h"

void audioFrameAdditive(uint16_t buffer_offset);
float audioTickAdditive(void);
void audioInitAdditive();
void  audioFreeAdditive();
void audioSwitchToAdditive();



#endif /* INC_ADDITIVE_H_ */
