/*
 * wave.c
 *
 *  Created on: Jul 4, 2020
 *      Author: josnyder
 */

#include "wave.h"
#include "string.h"
#include "audiostream.h"
//#include "stdio.h"
// WAVE header structure

unsigned char buffer4[4];
unsigned char buffer2[2];

char* seconds_to_time(float seconds);

struct tWaveHeader header;

unsigned char garbageBuffer[2048];
uint32_t waveTimeout = 2048;
uint32_t numberOfGarbage = 0;

int readWave(FIL *ptr) {

 int read = 0;
 uint32_t numBytesRead = 0;

 // read header parts

 read = f_read(ptr, header.riff, sizeof(header.riff), &numBytesRead);
 //printf("(1-4): %s \n", header.riff);

 if ((header.riff[0] != 'R') && (header.riff[1] != 'I') && (header.riff[2] != 'F') && (header.riff[3] != 'F'))
 {
	 return 0;
 }
 read = f_read(ptr, buffer4, sizeof(buffer4), &numBytesRead);
 //printf("%u %u %u %u\n", buffer4[0], buffer4[1], buffer4[2], buffer4[3]);

 // convert little endian to big endian 4 byte int
 header.overall_size  = buffer4[0] |
						(buffer4[1]<<8) |
						(buffer4[2]<<16) |
						(buffer4[3]<<24);

 //printf("(5-8) Overall size: bytes:%u, Kb:%u \n", header.overall_size, header.overall_size/1024);

 read = f_read(ptr, header.wave, sizeof(header.wave), &numBytesRead);
 //printf("(9-12) Wave marker: %s\n", header.wave);

read = f_read(ptr, header.fmt_chunk_marker, sizeof(header.fmt_chunk_marker), &numBytesRead);
int numIterations = 0;

while ((header.fmt_chunk_marker[0] != 102) && (header.fmt_chunk_marker[1] != 109) && (header.fmt_chunk_marker[2] != 116) && (header.fmt_chunk_marker[3] != 32))
{
	 //printf("(13-16) Fmt marker: %s\n", header.fmt_chunk_marker);

	 read = f_read(ptr, buffer4, sizeof(buffer4), &numBytesRead);
	// printf("%u %u %u %u\n", buffer4[0], buffer4[1], buffer4[2], buffer4[3]);

	 // convert little endian to big endian 4 byte integer
	 numberOfGarbage = buffer4[0] |
								(buffer4[1] << 8) |
								(buffer4[2] << 16) |
								(buffer4[3] << 24);
	 //printf("(17-20) Length of Fmt header: %u \n", header.length_of_fmt);


	 read = f_read(ptr, garbageBuffer, numberOfGarbage, &numBytesRead);

	 read = f_read(ptr, header.fmt_chunk_marker, sizeof(header.fmt_chunk_marker), &numBytesRead);
	 numIterations++;
	 if (numIterations > waveTimeout)
	 {
		 return 0;
	 }

}

 //printf("(13-16) Fmt marker: %s\n", header.fmt_chunk_marker);

 read = f_read(ptr, buffer4, sizeof(buffer4), &numBytesRead);
// printf("%u %u %u %u\n", buffer4[0], buffer4[1], buffer4[2], buffer4[3]);

 // convert little endian to big endian 4 byte integer
 header.length_of_fmt = buffer4[0] |
							(buffer4[1] << 8) |
							(buffer4[2] << 16) |
							(buffer4[3] << 24);
 //printf("(17-20) Length of Fmt header: %u \n", header.length_of_fmt);

 read = f_read(ptr, buffer2, sizeof(buffer2), &numBytesRead);
 //printf("%u %u \n", buffer2[0], buffer2[1]);

 header.format_type = buffer2[0] | (buffer2[1] << 8);
 char format_name[10] = "";
 if (header.format_type == 1)
   strcpy(format_name,"PCM");
 else if (header.format_type == 6)
  strcpy(format_name, "A-law");
 else if (header.format_type == 7)
  strcpy(format_name, "Mu-law");

 //printf("(21-22) Format type: %u %s \n", header.format_type, format_name);

 read = f_read(ptr, buffer2, sizeof(buffer2), &numBytesRead);
 //printf("%u %u \n", buffer2[0], buffer2[1]);

 header.channels = buffer2[0] | (buffer2[1] << 8);
 //printf("(23-24) Channels: %u \n", header.channels);

 read = f_read(ptr, buffer4, sizeof(buffer4), &numBytesRead);
 //printf("%u %u %u %u\n", buffer4[0], buffer4[1], buffer4[2], buffer4[3]);

 header.sample_rate = buffer4[0] |
						(buffer4[1] << 8) |
						(buffer4[2] << 16) |
						(buffer4[3] << 24);

 //printf("(25-28) Sample rate: %u\n", header.sample_rate);

 read = f_read(ptr, buffer4, sizeof(buffer4), &numBytesRead);
 //printf("%u %u %u %u\n", buffer4[0], buffer4[1], buffer4[2], buffer4[3]);

 header.byterate  = buffer4[0] |
						(buffer4[1] << 8) |
						(buffer4[2] << 16) |
						(buffer4[3] << 24);
 //printf("(29-32) Byte Rate: %u , Bit Rate:%u\n", header.byterate, header.byterate*8);

 read = f_read(ptr, buffer2, sizeof(buffer2), &numBytesRead);
 //printf("%u %u \n", buffer2[0], buffer2[1]);

 header.block_align = buffer2[0] |
					(buffer2[1] << 8);
 //printf("(33-34) Block Alignment: %u \n", header.block_align);

 read = f_read(ptr, buffer2, sizeof(buffer2), &numBytesRead);
 //printf("%u %u \n", buffer2[0], buffer2[1]);

 header.bits_per_sample = buffer2[0] |
					(buffer2[1] << 8);
 //printf("(35-36) Bits per sample: %u \n", header.bits_per_sample);

 read = f_read(ptr, header.data_chunk_header, sizeof(header.data_chunk_header), &numBytesRead);
 //printf("(37-40) Data Marker: %s \n", header.data_chunk_header);

 numIterations = 0;

 while ((header.data_chunk_header[0] != 'd') && (header.data_chunk_header[1] != 'a') && (header.data_chunk_header[2] != 't') && (header.data_chunk_header[3] != 'a'))
 {
 	 //printf("(13-16) Fmt marker: %s\n", header.fmt_chunk_marker);

 	 read = f_read(ptr, buffer4, sizeof(buffer4), &numBytesRead);
 	// printf("%u %u %u %u\n", buffer4[0], buffer4[1], buffer4[2], buffer4[3]);

 	 // convert little endian to big endian 4 byte integer
 	 numberOfGarbage = buffer4[0] |
 								(buffer4[1] << 8) |
 								(buffer4[2] << 16) |
 								(buffer4[3] << 24);
 	 //printf("(17-20) Length of Fmt header: %u \n", header.length_of_fmt);


 	 read = f_read(ptr, garbageBuffer, numberOfGarbage, &numBytesRead);

 	 read = f_read(ptr, header.data_chunk_header, sizeof(header.data_chunk_header), &numBytesRead);
 	 numIterations++;
	 if (numIterations > waveTimeout)
	 {
		 return 0;
	 }
 }


 read = f_read(ptr, buffer4, sizeof(buffer4), &numBytesRead);
// printf("%u %u %u %u\n", buffer4[0], buffer4[1], buffer4[2], buffer4[3]);

 header.data_size = buffer4[0] |
				(buffer4[1] << 8) |
				(buffer4[2] << 16) |
				(buffer4[3] << 24 );
 //printf("(41-44) Size of data chunk: %u \n", header.data_size);


 // calculate no.of samples
 long num_samples = (8 * header.data_size) / (header.channels * header.bits_per_sample);
 //printf("Number of samples:%lu \n", num_samples);

 long size_of_each_sample = (header.channels * header.bits_per_sample) / 8;
 //printf("Size of each sample:%ld bytes\n", size_of_each_sample);

 // calculate duration of file
 float duration_in_seconds = (float) header.overall_size / header.byterate;
 //printf("Approx.Duration in seconds=%f\n", duration_in_seconds);
 //printf("Approx.Duration in h:m:s=%s\n", seconds_to_time(duration_in_seconds));



 // read each sample from data chunk if PCM
 if (header.format_type == 1)
 { // PCM

	long i =0;
	char data_buffer[size_of_each_sample];
	int  size_is_correct = 1;

	// make sure that the bytes-per-sample is completely divisible by num.of channels
	long bytes_in_each_channel = (size_of_each_sample / header.channels);
	if ((bytes_in_each_channel  * header.channels) != size_of_each_sample)
	{
		//printf("Error: %ld x %ud <> %ld\n", bytes_in_each_channel, header.channels, size_of_each_sample);
		size_is_correct = 0;
	}

	if (size_is_correct)
	{
				// the valid amplitude range for values based on the bits per sample
		long low_limit = 0l;
		long high_limit = 0l;
		float inv_high_limit = 1.0f;

		switch (header.bits_per_sample) {
			case 8:
				low_limit = -128;
				high_limit = 127;

				break;
			case 16:
				low_limit = -32768;
				high_limit = 32767;
				break;
			case 24: //packed left like a 32 bit
				low_limit = -2147483648;
				high_limit = 2147483647;
				break;
			case 32:
				low_limit = -2147483648;
				high_limit = 2147483647;
				break;
		}
		inv_high_limit = 1.0f / high_limit;

		if (header.data_size < remainingScratchBytes)
		{
			read = f_read(ptr, largeMemoryScratch[scratchPosition], sizeof(header.data_size), (void *)&numBytesRead);
			//printf("\n\n.Valid range for data values : %ld to %ld \n", low_limit, high_limit);
			for (i =1; i <= num_samples; i++)
			{
				// dump the data read
				unsigned int  xchannels = 0;

				int32_t data_in_channel_32 = 0;
				int16_t data_in_channel_16 = 0;
				int8_t data_in_channel_8 = 0;
				float float_data = 0.0f;

				for (xchannels = 0; xchannels < header.channels; xchannels ++ ) {
					//printf("Channel#%d : ", (xchannels+1));
					// convert data from little endian to big endian based on bytes in each channel sample
					unsigned int byteOffset =  xchannels * bytes_in_each_channel;
					if (bytes_in_each_channel == 4) {
						data_in_channel_32 =	largeMemoryScratch[scratchPosition + byteOffset] |
											(largeMemoryScratch[scratchPosition + 1 + byteOffset]<<8) |
											(largeMemoryScratch[scratchPosition + 2 + byteOffset]<<16) |
											(largeMemoryScratch[scratchPosition + 3 + byteOffset]<<24);
						float_data = ((float)data_in_channel_32) * inv_high_limit;
						scratchPosition = scratchPosition + 4;
					}
					if (bytes_in_each_channel == 3) {
						data_in_channel_32 =	largeMemoryScratch[scratchPosition +  byteOffset]<<8 |
											(largeMemoryScratch[scratchPosition + 1 + byteOffset]<<16) |
											(largeMemoryScratch[scratchPosition + 2 + byteOffset]<<24);
						float_data = ((float)data_in_channel_32) * inv_high_limit;
						scratchPosition = scratchPosition + 3;
					}
					else if (bytes_in_each_channel == 2) {
						data_in_channel_16 = (int16_t)(largeMemoryScratch[scratchPosition + byteOffset] |
											(largeMemoryScratch[scratchPosition + 1 + byteOffset] << 8));
						float_data = ((float)data_in_channel_16) * inv_high_limit;
						scratchPosition = scratchPosition + 2;

					}
					else if (bytes_in_each_channel == 1) {
						data_in_channel_8 = (int8_t)largeMemoryScratch[scratchPosition + byteOffset];
						float_data = ((float)data_in_channel_8) * inv_high_limit;
						scratchPosition = scratchPosition + 1;
					}

					largeMemory[memoryPointer] = float_data;
					memoryPointer++;
					if (memoryPointer >= LARGE_MEM_SIZE_IN_FLOAT)
					{
						//ran out of space in SDRAM
						OutOfSpace = 1;
						return 0;
					}

				}
			}

		}

		remainingScratchBytes -= header.data_size;
		return 1;
	 }
 }

 //printf("Closing file..\n");
 //fclose(ptr);

  // cleanup before quitting
 //free(filename);
 return 0;

}

/**
 * Convert seconds into hh:mm:ss format
 * Params:
 *	seconds - seconds value
 * Returns: hms - formatted string
 **/

