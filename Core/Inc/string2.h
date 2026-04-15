/*
 * string2.h
 *
 *  Created on: Dec 27, 2023
 *      Author: jeffsnyder
 */

#ifndef INC_STRING2_H_
#define INC_STRING2_H_

#include "parameters.h"
#include "leaf.h"
#include "main.h"
#include "audiostream.h"

void audioFrameString2(uint16_t buffer_offset);
float audioTickString2(void);
void audioInitString2();
void  audioFreeString2();
void  audioSwitchToString2();


#endif /* INC_STRING2_H_ */
