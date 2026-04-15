/*
 * string1.h
 *
 *  Created on: Dec 27, 2023
 *      Author: jeffsnyder
 */

#ifndef SRC_STRING1_H_
#define SRC_STRING1_H_

#include "parameters.h"
#include "leaf.h"
#include "main.h"
#include "audiostream.h"

void audioFrameString1(uint16_t buffer_offset);
float audioTickString1(void);
void audioInitString1();
void  audioFreeString1();
void  audioSwitchToString1();


#endif /* SRC_STRING1_H_ */
