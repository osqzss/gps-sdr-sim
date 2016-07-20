#define _CRT_SECURE_NO_WARNINGS

#include <hackrf.h>

#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>

#include <windows.h>

#ifdef _WIN64
typedef int64_t ssize_t;
#else
typedef int32_t ssize_t;
#endif

typedef int bool;
#define true 1
#define false 0

static hackrf_device* device = NULL;

FILE* fd = NULL;
volatile uint32_t byte_count = 0;

volatile bool do_exit = false;

static transceiver_mode_t transceiver_mode = TRANSCEIVER_MODE_TX;

#define FD_BUFFER_SIZE (8*1024)
#define FREQ_ONE_MHZ (1000000ull)

BOOL WINAPI sighandler(int signum)
{
	if (CTRL_C_EVENT == signum) {
		fprintf(stdout, "Caught signal %d\n", signum);
		do_exit = true;
		return TRUE;
	}
	return FALSE;
}

int tx_callback(hackrf_transfer* transfer) {
	size_t bytes_to_read;

	if( fd != NULL )
	{
		ssize_t bytes_read;
		byte_count += transfer->valid_length;
		bytes_to_read = transfer->valid_length;
		
		bytes_read = fread(transfer->buffer, 1, bytes_to_read, fd);
		
		if (bytes_read != bytes_to_read) {
			return -1; // EOF
		} else {
			return 0;
		}
	} else {
		return -1;
	}
}

static void usage() {
	fprintf(stderr, "Usage: hackplayer [options]\n"
		"  -t <filename>  Transmit data from file (required)\n");

	return;
}

int main(int argc, char** argv) {
	int opt;
	int result;
	const char* path = NULL;
	uint32_t sample_rate_hz = 2600000;
	uint32_t baseband_filter_bw_hz = 0;
	unsigned int txvga_gain=0;
	uint64_t freq_hz = 1575420000;
	uint32_t amp_enable = 1;

	while( (opt = getopt(argc, argv, "t:")) != EOF )
	{
		result = HACKRF_SUCCESS;
		switch( opt ) 
		{
		case 't':
			path = optarg;
			break;
		default:
			printf("unknown argument '-%c %s'\n", opt, optarg);
			usage();
			return EXIT_FAILURE;
		}

		if( result != HACKRF_SUCCESS ) {
			printf("argument error: '-%c %s' %s (%d)\n", opt, optarg, hackrf_error_name(result), result);
			usage();
			return EXIT_FAILURE;
		}	
	}

	if( path == NULL ) {
		printf("specify a path to a file to transmit\n");
		usage();
		return EXIT_FAILURE;
	}

	// Compute default value depending on sample rate
	baseband_filter_bw_hz = hackrf_compute_baseband_filter_bw_round_down_lt(sample_rate_hz);

	result = hackrf_init();
	if( result != HACKRF_SUCCESS ) {
		printf("hackrf_init() failed: %s (%d)\n", hackrf_error_name(result), result);
		usage();
		return EXIT_FAILURE;
	}

	result = hackrf_open_by_serial(NULL, &device);
	if( result != HACKRF_SUCCESS ) {
		printf("hackrf_open() failed: %s (%d)\n", hackrf_error_name(result), result);
		usage();
		return EXIT_FAILURE;
	}

	fd = fopen(path, "rb");
	if( fd == NULL ) {
		printf("Failed to open file: %s\n", path);
		return EXIT_FAILURE;
	}

	// Change fd buffer to have bigger one to store or read data on/to HDD
	result = setvbuf(fd , NULL , _IOFBF , FD_BUFFER_SIZE);
	if( result != 0 ) {
		printf("setvbuf() failed: %d\n", result);
		usage();
		return EXIT_FAILURE;
	}

	SetConsoleCtrlHandler( (PHANDLER_ROUTINE) sighandler, TRUE );

	printf("call hackrf_sample_rate_set(%.03f MHz)\n", ((float)sample_rate_hz/(float)FREQ_ONE_MHZ));
	result = hackrf_set_sample_rate_manual(device, sample_rate_hz, 1);
	if( result != HACKRF_SUCCESS ) {
		printf("hackrf_sample_rate_set() failed: %s (%d)\n", hackrf_error_name(result), result);
		usage();
		return EXIT_FAILURE;
	}

	printf("call hackrf_baseband_filter_bandwidth_set(%.03f MHz)\n",
		((float)baseband_filter_bw_hz/(float)FREQ_ONE_MHZ));
	result = hackrf_set_baseband_filter_bandwidth(device, baseband_filter_bw_hz);
	if( result != HACKRF_SUCCESS ) {
		printf("hackrf_baseband_filter_bandwidth_set() failed: %s (%d)\n", hackrf_error_name(result), result);
		usage();
		return EXIT_FAILURE;
	}

	result = hackrf_set_txvga_gain(device, txvga_gain);
	result |= hackrf_start_tx(device, tx_callback, NULL);

	if( result != HACKRF_SUCCESS ) {
		printf("hackrf_start_?x() failed: %s (%d)\n", hackrf_error_name(result), result);
		usage();
		return EXIT_FAILURE;
	}

	printf("call hackrf_set_freq(%.03f MHz)\n", ((double)freq_hz/(double)FREQ_ONE_MHZ));
	result = hackrf_set_freq(device, freq_hz);
	if( result != HACKRF_SUCCESS ) {
		printf("hackrf_set_freq() failed: %s (%d)\n", hackrf_error_name(result), result);
		usage();
		return EXIT_FAILURE;
	}

	printf("call hackrf_set_amp_enable(%u)\n", amp_enable);
	result = hackrf_set_amp_enable(device, (uint8_t)amp_enable);
	if( result != HACKRF_SUCCESS ) {
		printf("hackrf_set_amp_enable() failed: %s (%d)\n", hackrf_error_name(result), result);
		usage();
		return EXIT_FAILURE;
	}

	printf("Stop with Ctrl-C\n");
	while( (hackrf_is_streaming(device) == HACKRF_TRUE) && (do_exit == false) ) {
		// Show something?
	}

	result = hackrf_is_streaming(device);
	if (do_exit) {
		printf("\nUser cancel, exiting...\n");
	} else {
		printf("\nExiting... hackrf_is_streaming() result: %s (%d)\n", hackrf_error_name(result), result);
	}

	if(device != NULL) {
		result = hackrf_stop_tx(device);
		if( result != HACKRF_SUCCESS ) {
			printf("hackrf_stop_tx() failed: %s (%d)\n", hackrf_error_name(result), result);
		} else {
			printf("hackrf_stop_tx() done\n");
		}

		result = hackrf_close(device);
		if( result != HACKRF_SUCCESS ) 
		{
			printf("hackrf_close() failed: %s (%d)\n", hackrf_error_name(result), result);
		} else {
			printf("hackrf_close() done\n");
		}
		
		hackrf_exit();
		printf("hackrf_exit() done\n");
	}

	if(fd != NULL) {
		fclose(fd);
		fd = NULL;
		printf("fclose(fd) done\n");
	}

	printf("exit\n");
	return EXIT_SUCCESS;
}
