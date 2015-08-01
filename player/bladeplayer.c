#define _CRT_SECURE_NO_WARNINGS

#include <stdlib.h>
#include <stdio.h>
#include <libbladeRF.h>

#define TX_FREQUENCY    1575420000
#define TX_SAMPLERATE   2600000
#define TX_BANDWIDTH    2500000
#define TX_VGA1         -25
#define TX_VGA2         0

#define NUM_BUFFERS         32
#define SAMPLES_PER_BUFFER  (32 * 1024)
#define NUM_TRANSFERS       16
#define TIMEOUT_MS          1000

int main(int argc, char *argv[])
{
	int status;
	char *devstr = NULL;
	struct bladerf *dev = NULL;

	FILE *fp;
	int16_t *tx_buffer;
	enum state {INIT, READ_FILE, PAD_TRAILING, DONE};
	enum state state = INIT;

	if (argc!=2) {
		fprintf(stderr, "Usage: bladeplayer <tx_file>\n");
		exit(1);
	}

	// Open TX file.
	fp = fopen(argv[1], "rb");

	if (fp==NULL) {
		fprintf(stderr, "Failed to open TX file: %s\n", argv[1]);
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

	status = bladerf_set_txvga1(dev, TX_VGA1);
	if (status != 0) {
		fprintf(stderr, "Failed to set TX VGA1 gain: %s\n", bladerf_strerror(status));
		goto out;
	}
	else {
		printf("TX VGA1 gain: %d dB\n", TX_VGA1);
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
	tx_buffer = (int16_t*)malloc(SAMPLES_PER_BUFFER* 2 * sizeof(int16_t));
	
	if (tx_buffer == NULL) {
		fprintf(stderr, "Failed to allocate TX buffer.\n");
		goto out;
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

		// Keep adding to the buffer until it is full or a failure occurs
		while (buffer_samples_remaining > 0 && status == 0 && state != DONE) {
			size_t samples_populated = 0;

			switch(state) {
				case INIT:
				case READ_FILE:
					// Read from the input file
					samples_populated = fread(tx_buffer_current,
							2 * sizeof(int16_t),
							buffer_samples_remaining,
							fp);
			
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

	// Close TX file
	fclose(fp);

out:
	printf("Closing device...\n");
	bladerf_close(dev);

	return(0);
}
