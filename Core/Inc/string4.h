/*
 * string4.h
 *
 *  Created on: Mar 9, 2026
 *      Author: josnyder
 */

#ifndef INC_STRING4_H_
#define INC_STRING4_H_

#include "parameters.h"
#include "leaf.h"
#include "main.h"
#include "audiostream.h"

void audioFrameString4(uint16_t buffer_offset);
float audioTickString4(void);
void audioInitString4();
void  audioFreeString4();
void  audioSwitchToString4();

#endif /* INC_STRING4_H_ */
