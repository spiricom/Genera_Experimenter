/*
 * synth.c
 *
 *  Created on: Jun 11, 2026
 *      Author: josnyder
 */


#include "synth.h"
#include "audiostream.h"
#include "SimpleOscModule.h"
#include "FilterModule.h"
#include "EnvModule.h"
#include "LFOModule.h"
#include "NoiseModule.h"
#include "ADEnvModule.h"
#include "StringModule.h"
#include "SampleAndHoldModule.h"
#include "PerlinNoiseModule.h"
#include "SimpleEnvModule.h"
#include "SimpleNoiseModule.h"
#include "SineModule.h"
#include "SoftClipModule.h"
#include "VCAModule.h"
#include "DelayModule.h"
#include "event_emitter.h"
#include "stdbool.h"
#include "defs.h"


#define INIT_ARRAY(arrayVar, type, sizeVal, _mempool)         \
        (arrayVar).size = (sizeVal);                         \
        (arrayVar).array = (type**) mpool_alloc(sizeof(type*) * (sizeVal), (_mempool)); \
        (arrayVar).mempool = (_mempool);


float voiceAmp = 0.0f;
float voiceMIDIPitch = 60.0f;
float prevVoiceMIDIPitch = 60.0f;

tFullPresetStateUnion preset;
tEnvModule masterEnv;

typedef struct genericModule
{
	ModuleHeader header;
}genericModule;

typedef struct oscModuleArray
{
	tOscModule* array;
	tMempool* mempool;
	uint32_t size;
} oscModuleArray;

typedef struct filtModuleArray
{
	tFiltModule* array;
	tMempool* mempool;
	uint32_t size;
} filtModuleArray;


typedef struct envModuleArray
{
	tEnvModule* array;
	tMempool* mempool;
	uint32_t size;
} envModuleArray;


typedef struct moduleHandle
{
	ModuleType type;
	uint8_t isChainExit;
	uint8_t isTerminalModule;
	void* module;
} moduleHandle;

typedef struct terminalOutputs
{
	uint8_t isChainExit;
	float* output;
} terminalOutputs;

oscModuleArray myOscModuleArray;
filtModuleArray myFiltModuleArray;
envModuleArray myEnvModuleArray;




void initModuleArraySize(ModuleType type, int size)
{
    tMempool* m = leaf.mempool;
	switch (type)
	{
		case ModuleTypeOscModule:
			INIT_ARRAY(myOscModuleArray, _tOscModule, size, m);
			break;
		case ModuleTypeFilterModule:
			INIT_ARRAY(myFiltModuleArray, _tFiltModule, size, m);
			break;
		case ModuleTypeEnvModule:
			INIT_ARRAY(myEnvModuleArray, _tEnvModule, size, m);
			break;
		default:
			break;
	}
}

ModuleHeader* eventListenerList[MAX_NUM_MODULES];
moduleHandle moduleArray[MAX_NUM_MODULES];
tMapping* mappingArray[MAX_NUM_MAPPINGS];
tAudioConnectionHardware audioConnectionArray[MAX_NUM_AUDIO_CONNECTIONS];
terminalOutputs terminalOutputsArray[MAX_NUM_MODULES];
volatile uint8_t numModulesToTick = 0;
volatile uint8_t numModulationMappings = 0;
volatile uint8_t numAudioConnections = 0;
volatile uint8_t numTerminalOutputs = 0;
volatile uint8_t numEventListeners = 0;
uint8_t masterEnvInitalized = 0;

void parseFullPreset()
{
	//clear previous preset
	presetReady = 0;
	if (masterEnvInitalized)
	{
		tEnvModule_free((void **)&masterEnv);
	}
	for (int i = 0; i < numModulesToTick; i++)
	{
		switch (moduleArray[i].type)
		{
			case ModuleTypeOscModule:
				{
					tOscModule_free(&moduleArray[i].module);
				}
				break;
			case ModuleTypeFilterModule:
				{
					tFiltModule_free(&moduleArray[i].module);
				}
				break;
			case ModuleTypeEnvModule:
				{
					tEnvModule_free(&moduleArray[i].module);
				}
				break;

			case ModuleTypeLFOModule:
				{
					tLFOModule_free(&moduleArray[i].module);
				}
				break;
			case ModuleTypeNoiseModule:
				{
					tNoiseModule_free(&moduleArray[i].module);
				}
				break;
			case ModuleTypeADEnvModule:
				{
					tADEnvModule_free(&moduleArray[i].module);
				}
				break;
			case ModuleTypeDelayModule:
				{
					tDelayModule_free(&moduleArray[i].module);
				}
				break;
			case ModuleTypePerlNoiseModule:
				{
					tPerlNoiseModule_free(&moduleArray[i].module);
				}
				break;
			case ModuleTypeSimpNoiseModule:
				{
					tSimpNoiseModule_free(&moduleArray[i].module);
				}
				break;
			case ModuleTypeSimpleEnvModule:
				{
					tSimpleEnvModule_free(&moduleArray[i].module);
				}
				break;
			case ModuleTypeSoftClipModule:
				{
					tSoftClipModule_free(&moduleArray[i].module);
				}
				break;
			case ModuleTypeStringModule:
				{
					tStringModule_free(&moduleArray[i].module);
				}
				break;
			case ModuleTypeVCAModule:
				{
					tVCAModule_free(&moduleArray[i].module);
				}
				break;
			case ModuleTypeSampleAndHoldModule:
				{
					tSampleAndHoldModule_free(&moduleArray[i].module);
				}
				break;
			default:
				break;
		}
	}
	for (int i = 0; i < numModulationMappings; i++)
	{
		tMapping_free(&mappingArray[i]);
	}
	numTerminalOutputs = 0;
	numEventListeners = 0;
	//init the master envelope
	tEnvModule_init((void **)&masterEnv, presetBuffer.data.fullstate.master.params, presetBuffer.data.fullstate.master.uuid, &leaf);
	masterEnvInitalized = 1;
	uint8_t currentModuleNumber = 0;
	//go through the modules, create each one in memory, and store pointers for them all in an array of module pointers
	for (int i = 0; i < MAX_NUM_MODULES; i++)
	{
		tModuleSmall currentModule = presetBuffer.data.fullstate.modules[i];
		if (currentModule.active)
		{
			moduleArray[currentModuleNumber].isChainExit = currentModule.chainEnd;
			moduleArray[currentModuleNumber].isTerminalModule = currentModule.terminal;

			switch (currentModule.type)
			{
				case ModuleTypeOscModule:
					{
						tOscModule_init((void **)&moduleArray[currentModuleNumber].module, presetBuffer.data.fullstate.modules[i].params, presetBuffer.data.fullstate.modules[i].uuid, &leaf);
						moduleArray[currentModuleNumber++].type = ModuleTypeOscModule;
					}
					break;
				case ModuleTypeFilterModule:
					{
						tFiltModule_init((void **)&moduleArray[currentModuleNumber].module, presetBuffer.data.fullstate.modules[i].params, presetBuffer.data.fullstate.modules[i].uuid, &leaf);
						moduleArray[currentModuleNumber++].type = ModuleTypeFilterModule;
					}
					break;
				case ModuleTypeEnvModule:
					{
						tEnvModule_init((void **)&moduleArray[currentModuleNumber].module, presetBuffer.data.fullstate.modules[i].params, presetBuffer.data.fullstate.modules[i].uuid, &leaf);
						moduleArray[currentModuleNumber++].type = ModuleTypeEnvModule;
					}
					break;
				case ModuleTypeLFOModule:
					{
						tLFOModule_init((void **)&moduleArray[currentModuleNumber].module, presetBuffer.data.fullstate.modules[i].params, presetBuffer.data.fullstate.modules[i].uuid, &leaf);
						moduleArray[currentModuleNumber++].type = ModuleTypeLFOModule;
					}
					break;
				case ModuleTypeNoiseModule:
					{
						tNoiseModule_init((void **)&moduleArray[currentModuleNumber].module, presetBuffer.data.fullstate.modules[i].params, presetBuffer.data.fullstate.modules[i].uuid, &leaf);
						moduleArray[currentModuleNumber++].type = ModuleTypeNoiseModule;
					}
					break;
				case ModuleTypeADEnvModule:
					{
						tADEnvModule_init((void **)&moduleArray[currentModuleNumber].module, presetBuffer.data.fullstate.modules[i].params, presetBuffer.data.fullstate.modules[i].uuid, &leaf);
						moduleArray[currentModuleNumber++].type = ModuleTypeADEnvModule;
					}
					break;
				case ModuleTypeDelayModule:
					{
						tDelayModule_initToPool((void **)&moduleArray[currentModuleNumber].module, presetBuffer.data.fullstate.modules[i].params, presetBuffer.data.fullstate.modules[i].uuid, &largePool);
						moduleArray[currentModuleNumber++].type = ModuleTypeDelayModule;
					}
					break;
				case ModuleTypePerlNoiseModule:
					{
						tPerlNoiseModule_init((void **)&moduleArray[currentModuleNumber].module, presetBuffer.data.fullstate.modules[i].params, presetBuffer.data.fullstate.modules[i].uuid, &leaf);
						moduleArray[currentModuleNumber++].type = ModuleTypePerlNoiseModule;
					}
					break;
				case ModuleTypeSimpNoiseModule:
					{
						tSimpNoiseModule_init((void **)&moduleArray[currentModuleNumber].module, presetBuffer.data.fullstate.modules[i].params, presetBuffer.data.fullstate.modules[i].uuid, &leaf);
						moduleArray[currentModuleNumber++].type = ModuleTypeSimpNoiseModule;
					}
					break;
				case ModuleTypeSimpleEnvModule:
					{
						tSimpleEnvModule_init((void **)&moduleArray[currentModuleNumber].module, presetBuffer.data.fullstate.modules[i].params, presetBuffer.data.fullstate.modules[i].uuid, &leaf);
						moduleArray[currentModuleNumber++].type = ModuleTypeSimpleEnvModule;
					}
					break;
				case ModuleTypeSoftClipModule:
					{
						tSoftClipModule_init((void **)&moduleArray[currentModuleNumber].module, presetBuffer.data.fullstate.modules[i].params, presetBuffer.data.fullstate.modules[i].uuid, &leaf);
						moduleArray[currentModuleNumber++].type = ModuleTypeSoftClipModule;
					}
					break;
				case ModuleTypeStringModule:
					{
						tStringModule_initToPool((void **)&moduleArray[currentModuleNumber].module, presetBuffer.data.fullstate.modules[i].params, presetBuffer.data.fullstate.modules[i].uuid, &largePool);
						moduleArray[currentModuleNumber++].type = ModuleTypeStringModule;
					}
					break;
				case ModuleTypeVCAModule:
					{
						tVCAModule_init((void **)&moduleArray[currentModuleNumber].module, presetBuffer.data.fullstate.modules[i].params, presetBuffer.data.fullstate.modules[i].uuid, &leaf);
						moduleArray[currentModuleNumber++].type = ModuleTypeVCAModule;
					}
					break;
				case ModuleTypeSampleAndHoldModule:
					{
						tSampleAndHoldModule_init((void **)&moduleArray[currentModuleNumber].module, presetBuffer.data.fullstate.modules[i].params, presetBuffer.data.fullstate.modules[i].uuid, &leaf);
						moduleArray[currentModuleNumber++].type = ModuleTypeSampleAndHoldModule;
					}
					break;
				default:
					break;
			}
			if (currentModule.terminal) //add it to the terminal module array if it has a terminal output
			{
				genericModule* tempMod = (genericModule*) moduleArray[currentModuleNumber-1].module; //we just incremented so look back 1
				terminalOutputsArray[numTerminalOutputs].output = &tempMod->header.outputs[0];
				terminalOutputsArray[numTerminalOutputs].isChainExit = currentModule.chainEnd;
				numTerminalOutputs++;
			}
			if (currentModule.params[0]) //assuming Event Watch Flag is always element 0 (it is)
			{
				genericModule* tempMod = (genericModule*) moduleArray[currentModuleNumber-1].module; //we just incremented so look back 1
				eventListenerList[numEventListeners] = &tempMod->header;
				numEventListeners++;
			}
		}
	}
	numModulesToTick = currentModuleNumber;

	uint8_t currentMappingNumber = 0;
	//go through the modules, create each one in memory, and store pointers for them all in an array of module pointers
	for (int i = 0; i < MAX_NUM_MAPPINGS; i++)
	{
		tMappingSmall currentMapping = presetBuffer.data.fullstate.mappings[i];
		if (currentMapping.active)
		{
			uint16_t destID = currentMapping.dUuid;
			uint8_t dParamID = currentMapping.dParamId;
			uint8_t alreadyGotOne = 0;
			uint8_t oneWeGot = 0;
			//check the mapping array and if there is already a mapping to that destination, mark that, and we'll later add this to the existing one.
			for (int j = 0; j < currentMappingNumber; j++)
			{
				if ((mappingArray[j]->destinationProcessorUniqueID == destID) && (mappingArray[j]->paramID == dParamID))
				{
					alreadyGotOne = 1;
					oneWeGot = j;
				}
			}

			if (alreadyGotOne)
			{

				//find insource
				float* insource = NULL;

				//check if master envelope is source
				if (masterEnv->header.uniqueID == currentMapping.sUuid)
				{
					//found it!
					//set the insource pointer to look for the correct data. Will this work? it doesn't know the type, but hopefully it can work.
					insource = &masterEnv->header.outputs[0];
				}
				//check if some other module is source
				for (int j = 0; j < numModulesToTick; j++)
				{

					genericModule* tempMod = (genericModule*) moduleArray[j].module;

					if (tempMod->header.uniqueID == currentMapping.sUuid)
					{
						//found it!
						//set the insource pointer to look for the correct data. Will this work? it doesn't know the type, but hopefully it can work.
						insource = &tempMod->header.outputs[0];
					}
				}
				float* destinationPointer = NULL;
				void* destinationObj = NULL;
				//find destinationPointer

				//check if master envelope is destination
				if (masterEnv->header.uniqueID == currentMapping.dUuid)
				{
					//found it!
					//set the destination pointer to look for the correct data. Will this work? it doesn't know the type, but hopefully it can work.
					destinationPointer = &masterEnv->header.params[currentMapping.dParamId];
					destinationObj = &masterEnv;
				}

				//check if some other module is destination
				for (int j = 0; j < numModulesToTick; j++)
				{

					genericModule* tempMod = (genericModule*) moduleArray[j].module;

					if (tempMod->header.uniqueID == currentMapping.dUuid)
					{
						//found it!
						//set the destination pointer to look for the correct data. Will this work? it doesn't know the type, but hopefully it can work.
						destinationPointer = &tempMod->header.params[currentMapping.dParamId];
						destinationObj = moduleArray[j].module;
					}
				}

				//should have an error case here where it checks for void pointers



				tMappingAdd_(mappingArray[oneWeGot], insource, currentMapping.sUuid,
						destinationPointer, currentMapping.dUuid, currentMapping.dParamId,
				        destinationObj, &leaf, currentMapping.scalar);


			}
			//otherwise, create a new one.
			else
			{
				tMapping_init(&mappingArray[currentMappingNumber], &leaf);

				//set the mapping uuid
				mappingArray[currentMappingNumber]->uuid = currentMapping.mapUuid;

				//find insource
				float* insource = NULL;

				//check if master envelope is source
				if (masterEnv->header.uniqueID == currentMapping.sUuid)
				{
					//found it!
					//set the insource pointer to look for the correct data. Will this work? it doesn't know the type, but hopefully it can work.
					insource = &masterEnv->header.outputs[0];
				}
				//then check the other modules
				else
				{
					for (int j = 0; j < numModulesToTick; j++)
					{

						genericModule* tempMod = (genericModule*) moduleArray[j].module;

						if (tempMod->header.uniqueID == currentMapping.sUuid)
						{
							//found it!
							//set the insource pointer to look for the correct data. Will this work? it doesn't know the type, but hopefully it can work.
							insource = &tempMod->header.outputs[0];
						}
					}
				}
				float* destinationPointer = NULL;
				void* destinationObj = NULL;
				//find destinationPointer
				//check if master envelope is destination
				if (masterEnv->header.uniqueID == currentMapping.dUuid)
				{
					//found it!
					//set the destination pointer to look for the correct data. Will this work? it doesn't know the type, but hopefully it can work.
					destinationPointer = &masterEnv->header.params[currentMapping.dParamId];
					destinationObj = &masterEnv;
				}
				else
				{
					for (int j = 0; j < numModulesToTick; j++)
					{

						genericModule* tempMod = (genericModule*) moduleArray[j].module;

						if (tempMod->header.uniqueID == currentMapping.dUuid)
						{
							//found it!
							//set the destination pointer to look for the correct data. Will this work? it doesn't know the type, but hopefully it can work.
							destinationPointer = &tempMod->header.params[currentMapping.dParamId];
							destinationObj = moduleArray[j].module;
						}
					}
				}
				tMappingAdd_(mappingArray[currentMappingNumber], insource, currentMapping.sUuid,
				destinationPointer, currentMapping.dUuid, currentMapping.dParamId,
				destinationObj, &leaf, currentMapping.scalar);
				currentMappingNumber++;

			}


		}

	}
	numModulationMappings = currentMappingNumber;


	//now parse the audio connections
	uint8_t currentAudioConnectionNumber = 0;
	for (int i = 0; i < MAX_NUM_AUDIO_CONNECTIONS; i++)
	{
		tAudioConnectionSmall currentConnection = presetBuffer.data.fullstate.routing[i];
		if (currentConnection.active)
		{
			audioConnectionArray[currentAudioConnectionNumber].amount = currentConnection.amount;
			//find source pointer
			float* sourcePointer = NULL;
			float* destPointer = NULL;

			for (int j = 0; j < numModulesToTick; j++)
			{
				genericModule* tempMod = (genericModule*) moduleArray[j].module;

				if (tempMod->header.uniqueID == currentConnection.sUuid)
				{
					//this is the one. Get the pointer to the output.
					sourcePointer = &tempMod->header.outputs[0];
					audioConnectionArray[currentAudioConnectionNumber].chainExit = moduleArray[j].isChainExit;
				}
				if (tempMod->header.uniqueID == currentConnection.dUuid)
				{
					destPointer = &tempMod->header.inputs[0];
				}
			}
			audioConnectionArray[currentAudioConnectionNumber].sourcePointer = sourcePointer;
			audioConnectionArray[currentAudioConnectionNumber].destinationPointer = destPointer;
			currentAudioConnectionNumber++;
		}
	}
	numAudioConnections = currentAudioConnectionNumber;
	presetWaitingToParse = 0;
	presetReady = 1;
}



void initSynth()
{



	if(leaf.resTable == NULL)
	{
		tLookupTable_create(&mediumPool,&leaf.resTable);
		tLookupTable_init(&leaf, leaf.resTable,  0.0f, 10.0f, 0.5f, 2048);
	}

	if (leaf.glideTimeTable == NULL)
    {
        tLookupTable_create(&mediumPool, &leaf.glideTimeTable);
        tLookupTable_init (&leaf, leaf.glideTimeTable, 0.f, 8000.f, 500.f, 2048);
    }
    if (leaf.mtofTable == NULL)
    {
    	tLookupTable_create(&mediumPool, &leaf.mtofTable);
    	tLookupTable_init (&leaf, leaf.mtofTable, 0.f, 0.f, 0.f, 16384);
    	LEAF_generate_mtof (leaf.mtofTable->table, 0, LEAF_MODULE_MAX_MIDI_NOTE, 16384);
    }

    if (leaf.ftomTable == NULL)
    {
    	tLookupTable_create(&mediumPool, &leaf.ftomTable);
    	tLookupTable_init (&leaf, leaf.ftomTable, 0.f, 0.f, 0.f, 16384);
    	LEAF_generate_ftom (leaf.ftomTable->table, 0.f, 20000.0f, 16384);
    }

    if (leaf.gainAmpTable == NULL)
    {
    	tLookupTable_create(&mediumPool, &leaf.gainAmpTable);
    	tLookupTable_init (&leaf, leaf.gainAmpTable, 0.f, TWELVE_DB_AMPLITUDE, 1.f, 2048);
    }
    if (leaf.decayExpBuffer == NULL)
    {
        tLookupTable_create (&mediumPool, &leaf.decayExpBuffer);
        tLookupTable_init (&leaf, leaf.decayExpBuffer, 0.0001f, 20000.f, 4000.f, DECAY_EXP_BUFFER_SIZE);
    	LEAF_generate_exp (leaf.decayExpBuffer->table, 0.001f, 0.0f, 1.0f, -0.0008f, DECAY_EXP_BUFFER_SIZE);
    }
    if(leaf.lfoRateTable == NULL)
    {
        tLookupTable_create(&mediumPool, &leaf.lfoRateTable);
        tLookupTable_init(&leaf,leaf.lfoRateTable, 0.f, 30.f, 2.f,2048);
       // LEAF_generate_table_skew_non_sym(skewTableLFORate, 0.0f, 30.0f, 2.0f, 2048);
    }
    if (leaf.envTimeTable == NULL)
    {
	   tLookupTable_create (&mediumPool, &leaf.envTimeTable);
	   tLookupTable_init (&leaf, leaf.envTimeTable, 0.0001f, 20000.f, 4000.f, 2048);
    }
    if (leaf.filtFreqTable == NULL)
    {
    	tLookupTable_create(&mediumPool, &leaf.filtFreqTable);
    	tLookupTable_init (&leaf, leaf.filtFreqTable, 0.0f, 20000.f, 1000.f, 16384);
    }
/*
    if(leaf.delayTimeTable == NULL)
    {
        tLookupTable_create(&mediumPool, &leaf.delayTimeTable);
        tLookupTable_init(&leaf, leaf.delayTimeTable,  0.0f, 2250.0f, 200.0f, 2048);
    }
    */
}

void processNoteOn(float velocity)
{
	if (velocity >= 0.0f)
	{
		velocity = ((0.007685533519034f * velocity * 127.f) + 0.0239372430f);
		velocity = velocity * velocity;
		//note -= midiKeyMin;
		//note -= midiKeyMin;


		for (int i = 0; i < numEventListeners; i++)
		{
			callNoteOn (eventListenerList[i],
				velocity);
			//printf("On On 1.  ");
		}


		callNoteOn (&masterEnv->header, velocity);

		//printf("On On 2.  ");
		// float norm = key / float(mkkkidiKeyMax - midiKeyMin);
	}

}


float audioTickSynth()
{


	float output = 0.0f;

	//get the amplitude of the master envelope
	tEnvModule_tick(masterEnv);

	float masterEnvValue = masterEnv->header.outputs[0];
	//processMappings(snapshot); // process modulations !! :)
	for (int i = 0; i < numModulationMappings; i++)
	{
		processMapping(mappingArray[i]);
	}

	//processAudioConnections(*masterEnvelope, snapshot); // process all audio connections (e.g. route their audio to and fro) - robert frost
	for (int i = 0; i < numAudioConnections; i++)
	{
		float sourceAudio = *audioConnectionArray[i].sourcePointer * audioConnectionArray[i].amount;
		if (audioConnectionArray[i].chainExit)
		{
			sourceAudio *= masterEnvValue;
		}
		//get the destination value
		float destinationAudio = *audioConnectionArray[i].destinationPointer;
		destinationAudio += sourceAudio;
		destinationAudio = LEAF_clip(-1.0f, destinationAudio, 1.0f);
		*audioConnectionArray[i].destinationPointer = destinationAudio; // is this the best way to do this?
	}
	//tick the modules
	for (int i = 0; i < numModulesToTick; i++)
	{
		//tickAllModules(snapshot.modulesToTick); // tick all the modules, one by one, indiscriminately (cold)
		switch (moduleArray[i].type)
		{
			case ModuleTypeOscModule:
				tOscModule_setInputNote ((tOscModule)moduleArray[i].module, voiceMIDIPitch);
				tOscModule_tick((tOscModule)moduleArray[i].module);
				break;
			case ModuleTypeFilterModule:
				tFiltModule_setParameter((tFiltModule)moduleArray[i].module, FiltMidiPitch, voiceMIDIPitch);
				tFiltModule_tick((tFiltModule)moduleArray[i].module);
				break;
			case ModuleTypeEnvModule:
				tEnvModule_tick((tEnvModule)moduleArray[i].module);
				break;
			case ModuleTypeLFOModule:
				tLFOModule_tick((tLFOModule)moduleArray[i].module);
				break;
			case ModuleTypeNoiseModule:
				tNoiseModule_tick((tNoiseModule)moduleArray[i].module);
				break;
			case ModuleTypePerlNoiseModule:
				tPerlNoiseModule_tick((tPerlNoiseModule)moduleArray[i].module);
				break;
			case ModuleTypeSimpNoiseModule:
				tSimpNoiseModule_tick((tSimpNoiseModule)moduleArray[i].module);
				break;
			case ModuleTypeSimpleEnvModule:
				tSimpleEnvModule_tick((tSimpleEnvModule)moduleArray[i].module);
				break;
			case ModuleTypeSineModule:
				tSineModule_tick((tSineModule)moduleArray[i].module);
				break;
			case ModuleTypeSoftClipModule:
				tSoftClipModule_tick((tSoftClipModule)moduleArray[i].module);
				break;
			case ModuleTypeDelayModule:
				tDelayModule_tick((tDelayModule)moduleArray[i].module);
				break;
			case ModuleTypeStringModule:
				tStringModule_tick((tStringModule)moduleArray[i].module);
				break;
			case ModuleTypeVCAModule:
				tVCAModule_tick((tVCAModule)moduleArray[i].module);
				break;
			case ModuleTypeADEnvModule:
				tADEnvModule_tick((tADEnvModule)moduleArray[i].module);
				break;
			default:
				break;
		}
	}
	//mix the terminal outputs
	for (int i = 0; i < numTerminalOutputs; i++)
	{
		float sourceAudio = *terminalOutputsArray[i].output;
		if (terminalOutputsArray[i].isChainExit)
		{
			sourceAudio *= masterEnvValue;
		}
		//add it in
		output += sourceAudio;
	}
	output *= 0.5f;
	//mixTerminalModulesToOutput(audio_buffer, *masterEnvelope, snapshot, i); //
	output = LEAF_clip(-.9995f, output, .9995f); //can't go all the way to -1 and 1 or the later integer conversion can go bad and invert on clipping
	return output;
}


void audioFrameSynth(uint32_t bufferOffset)
{
	uint32_t tempCountFrame = DWT->CYCCNT;
	if (presetWaitingToParse)
	{
		parseFullPreset();
	}
	for (int i = 0; i < HALF_BUFFER_SIZE; i+=2)
	{
		int iplusbuffer = bufferOffset + i;
		float outVal = audioTickSynth();
		audioOutBuffer[iplusbuffer] = (int32_t)(outVal * TWO_TO_23);
		audioOutBuffer[iplusbuffer + 1] = (int32_t)(outVal * TWO_TO_23);
	}
	uint32_t timeFrame = DWT->CYCCNT - tempCountFrame;
	frameLoadPercentage = (float)timeFrame * frameMult;
	if (frameLoadPercentage > .99f)
	{
		frameLoadOverCount++;
	}
}


