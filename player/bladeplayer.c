#define _CRT_SECURE_NO_WARNINGS

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <libbladeRF.h>
#ifdef _WIN32
#include "getopt.h"
#else
#include <unistd.h>
#endif

#define TX_FREQUENCY    1575420000
#define TX_SAMPLERATE   2600000
#define TX_BANDWIDTH    2500000
#define TX_VGA1         -25
#define TX_VGA2         0

#define NUM_BUFFERS         32
#define SAMPLES_PER_BUFFER  (32 * 1024)
#define NUM_TRANSFERS       16
#define TIMEOUT_MS          1000

#define AMPLITUDE (1000) // Default amplitude for 12-bit I/Q

void usage(void)
{
	fprintf(stderr, "Usage: bladeplayer [options]\n"
		"  -f <tx_file>  I/Q sampling data file (required)\n"
		"  -b <iq_bits>  I/Q data format [1/16] (default: 16)\n"
		"  -g <tx_vga1>  TX VGA1 gain (default: %d)\n",
		TX_VGA1);

	return;
}
int main(int argc, char *argv[])
{
	int status;
	char *devstr = NULL;
	struct bladerf *dev = NULL;

	FILE *fp;
	int16_t *tx_buffer;
	enum state {INIT, READ_FILE, PAD_TRAILING, DONE};
	enum state state = INIT;

	int compressed = 0;
	uint8_t *read_buffer;
	size_t samples_read;
	int16_t lut[256][8];
	int16_t amp = AMPLITUDE;
	int i,k;

	int gain = TX_VGA1;
	int result;
	int data_format;
	char txfile[128];

	// Empty TX file name
	txfile[0] = 0;

	if (argc<3) {
		usage();
		exit(1);
	}

	while ((result=getopt(argc,argv,"g:b:f:"))!=-1)
	{
		switch (result)
		{
		case 'g':
			gain = atoi(optarg);
			if (gain>-4 || gain<-35)
			{
				printf("ERROR: Invalid TX VGA1 gain.\n");
				exit(1);
			}
			break;
		case 'b':
			data_format = atoi(optarg);
			if (data_format!=1 && data_format!=16)
			{
				printf("ERROR: Invalid I/Q data format.\n");
				exit(1);
			}
			else if (data_format==1)
				compressed = 1;
			break;
		case 'f':
			strcpy(txfile, optarg);
			break;
		case ':':
		case '?':
			usage();
			exit(1);
		default:
			break;
		}
	}

	// Open TX file.
	if (txfile[0]==0)
	{
		printf("ERROR: I/Q sampling data file is not specified.\n");
		exit(1);
	}

	fp = fopen(txfile, "rb");

	if (fp==NULL) {
		fprintf(stderr, "ERROR: Failed to open TX file: %s\n", argv[1]);
		exit(1);
	}

	// Initializing device.
	printf("Opening and initializing device...\n");

	status = bladerf_open(&dev, devstr);
	if (status != 0) {
		fprintf(stderr, "Failed to open device: %s\n", bladerf_strerror(status));
		goto out;
	}

	status = bladerf_set_frequency(dev, BLADERF_MODULE_TX, TX_FREQUENCY);
	if (status != 0) {
		fprintf(stderr, "Faield to set TX frequency: %s\n", bladerf_strerror(status));
		goto out;
	} 
	else {
		printf("TX frequency: %u Hz\n", TX_FREQUENCY);
	}

	status = bladerf_set_sample_rate(dev, BLADERF_MODULE_TX, TX_SAMPLERATE, NULL);
	if (status != 0) {
		fprintf(stderr, "Failed to set TX sample rate: %s\n", bladerf_strerror(status));
		goto out;
	}
	else {
		printf("TX sample rate: %u sps\n", TX_SAMPLERATE);
	}

	status = bladerf_set_bandwidth(dev, BLADERF_MODULE_TX, TX_BANDWIDTH, NULL);
	if (status != 0) {
		fprintf(stderr, "Failed to set TX bandwidth: %s\n", bladerf_strerror(status));
		goto out;
	}
	else {
		printf("TX bandwidth: %u Hz\n", TX_BANDWIDTH);
	}

	status = bladerf_set_txvga1(dev, gain);
	if (status != 0) {
		fprintf(stderr, "Failed to set TX VGA1 gain: %s\n", bladerf_strerror(status));
		goto out;
	}
	else {
		printf("TX VGA1 gain: %d dB\n", gain);
	}

	status = bladerf_set_txvga2(dev, TX_VGA2);
	if (status != 0) {
		fprintf(stderr, "Failed to set TX VGA2 gain: %s\n", bladerf_strerror(status));
		goto out;
	}
	else {
		printf("TX VGA2 gain: %d dB\n", TX_VGA2);
	}

	// Application code goes here.
	printf("Running...\n");

	// Allocate a buffer to hold each block of samples to transmit.
	tx_buffer = (int16_t*)malloc(SAMPLES_PER_BUFFER * 2 * sizeof(int16_t));
	
	if (tx_buffer == NULL) {
		fprintf(stderr, "Failed to allocate TX buffer.\n");
		goto out;
	}

	// if compressed
	read_buffer = (uint8_t*)malloc(SAMPLES_PER_BUFFER / 4);

	if (read_buffer == NULL) {
		fprintf(stderr, "Failed to allocate read buffer.\n");
		goto out;
	}

	for (i=0; i<256; i++)
	{
		for (k=0; k<8; k++)
			lut[i][k] = ((i>>(7-k))&0x1)?amp:-amp;
	}

	// Configure the TX module for use with the synchronous interface.
	status = bladerf_sync_config(dev,
			BLADERF_MODULE_TX,
			BLADERF_FORMAT_SC16_Q11,
			NUM_BUFFERS,
			SAMPLES_PER_BUFFER,
			NUM_TRANSFERS,
			TIMEOUT_MS);

	if (status != 0) {
		fprintf(stderr, "Failed to configure TX sync interface: %s\n", bladerf_strerror(status));
		goto out;
	}

	// We must always enable the modules *after* calling bladerf_sync_config().
	status = bladerf_enable_module(dev, BLADERF_MODULE_TX, true);
	if (status != 0) {
		fprintf(stderr, "Failed to enable TX module: %s\n", bladerf_strerror(status));
		goto out;
	}

	// Keep writing samples while there is more data to send and no failures have occurred.
	while (state != DONE && status == 0) {

		int16_t *tx_buffer_current = tx_buffer;
		unsigned int buffer_samples_remaining = SAMPLES_PER_BUFFER;

		// if compressed
		unsigned int read_samples_remaining = SAMPLES_PER_BUFFER / 4;

		// Keep adding to the buffer until it is full or a failure occurs
		while (buffer_samples_remaining > 0 && status == 0 && state != DONE) {
			size_t samples_populated = 0;

			switch(state) {
				case INIT:
				case READ_FILE:
					// Read from the input file
					if (compressed)
					{
						int16_t *write_buffer_current = tx_buffer;

						samples_read = fread(read_buffer,
								sizeof(uint8_t),
								read_samples_remaining,
								fp);

						samples_populated = samples_read * 4;
						buffer_samples_remaining = read_samples_remaining * 4;

						// Expand compressed data into TX buffer
						for (i=0; i<samples_read; i++)
						{
							memcpy(write_buffer_current, lut[read_buffer[i]], 8);
						
							// Advance the write buffer pointer
							write_buffer_current += 8;
						}
					}
					else
					{
						samples_populated = fread(tx_buffer_current,
								2 * sizeof(int16_t),
								buffer_samples_remaining,
								fp);
					}

					// If the end of the file was reached, pad the rest of the buffer and finish.
					if (feof(fp)) {
						state = PAD_TRAILING;
					}
					// Check for errors
					else if (ferror(fp)) {
						status = errno;
					}

					break;

				case PAD_TRAILING:
					// Populate the remainder of the buffer with zeros.
					memset(tx_buffer_current, 0, buffer_samples_remaining * 2 * sizeof(uint16_t));

					state = DONE;
					break;

				case DONE:
				default:
					break;
			}

			// Advance the buffer pointer.
			buffer_samples_remaining -= (unsigned int)samples_populated;
			tx_buffer_current += (2 * samples_populated);
		}

		// If there were no errors, transmit the data buffer.
		if (status == 0) {
			bladerf_sync_tx(dev, tx_buffer, SAMPLES_PER_BUFFER, NULL, TIMEOUT_MS);
		}
	}

	// Disable TX module, shutting down our underlying TX stream.
	status = bladerf_enable_module(dev, BLADERF_MODULE_TX, false);
	if (status != 0) {
		fprintf(stderr, "Failed to disable TX module: %s\n", bladerf_strerror(status));
	}

	// Free up our resources
	free(tx_buffer);

	// if compressed
	free(read_buffer);

	// Close TX file
	fclose(fp);

out:
	printf("Closing device...\n");
	bladerf_close(dev);

	return(0);
}
