/*
 * wave.h
 *
 *  Created on: Jul 4, 2020
 *      Author: josnyder
 */

#ifndef WAVE_H_
#define WAVE_H_
#include "main.h"
#include "fatfs.h"
struct tWaveHeader {

    unsigned char riff[4] __attribute__ ((aligned (32)));                      // RIFF string

    unsigned int overall_size  __attribute__ ((aligned (32))) ;               // overall size of file in bytes

    unsigned char wave[4] __attribute__ ((aligned (32)));                      // WAVE string

    unsigned char fmt_chunk_marker[4] __attribute__ ((aligned (32)));          // fmt string with trailing null char

    unsigned int length_of_fmt __attribute__ ((aligned (32)));                 // length of the format data

    unsigned int format_type __attribute__ ((aligned (32)));                   // format type. 1-PCM, 3- IEEE float, 6 - 8bit A law, 7 - 8bit mu law

    unsigned int channels __attribute__ ((aligned (32)));                      // no.of channels

    unsigned int sample_rate __attribute__ ((aligned (32)));                   // sampling rate (blocks per second)

    unsigned int byterate __attribute__ ((aligned (32)));                      // SampleRate * NumChannels * BitsPerSample/8

    unsigned int block_align __attribute__ ((aligned (32)));                   // NumChannels * BitsPerSample/8

    unsigned int bits_per_sample __attribute__ ((aligned (32)));               // bits per sample, 8- 8bits, 16- 16 bits etc

    unsigned char data_chunk_header [4] __attribute__ ((aligned (32)));        // DATA string or FLLR string

    unsigned int data_size __attribute__ ((aligned (32)));                     // NumSamples * NumChannels * BitsPerSample/8 - size of the next chunk that will be read

};

extern struct tWaveHeader header;
int readWave(FIL *ptr);

#endif /* WAVE_H_ */
