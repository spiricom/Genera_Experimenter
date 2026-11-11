/*
 * synth.h
 *
 *  Created on: Jun 11, 2026
 *      Author: josnyder
 */

#ifndef SYNTH_H_
#define SYNTH_H_

#include "leaf.h"
#include "main.h"
#include "audiostream.h"
#include "mapping.h"

extern tFullPresetStateUnion preset;

extern float voiceAmp;
extern float voiceMIDIPitch;
extern float prevVoiceMIDIPitch;

void parseFullPreset();
void initSynth();
void audioFrameSynth(uint32_t bufferOffset);
void processNoteOn(float velocity);



#endif /* SYNTH_H_ */
