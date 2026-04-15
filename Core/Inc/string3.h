/*
 * string3.h
 *
 *  Created on: Jan 11, 2024
 *      Author: jeffsnyder
 */

#ifndef INC_STRING3_H_
#define INC_STRING3_H_


#include "parameters.h"
#include "leaf.h"
#include "main.h"
#include "audiostream.h"

void audioFrameString3(uint16_t buffer_offset);
float audioTickString3(void);
void audioInitString3();
void  audioFreeString3();
void audioSwitchToString3();


#endif /* INC_STRING3_H_ */
