/*
 * vocal.h
 *
 *  Created on: Dec 27, 2023
 *      Author: jeffsnyder
 */

#ifndef INC_VOCAL_H_
#define INC_VOCAL_H_

#include "parameters.h"
#include "leaf.h"
#include "main.h"
#include "audiostream.h"

void audioFrameVocal(uint16_t buffer_offset);
float audioTickVocal(void);
void audioInitVocal();
void  audioFreeVocal();
void  audioSwitchToVocal();




#endif /* INC_VOCAL_H_ */
