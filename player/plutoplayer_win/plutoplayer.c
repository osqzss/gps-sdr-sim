#define _CRT_SECURE_NO_WARNINGS

#include <stdio.h>
#include <stdlib.h>
#ifdef _WIN32
#include "getopt.h"
#include "ad9361.h"
#include "iio.h"
#else
#include <getopt.h>
#include <ad9361.h>
#include <iio.h>
#endif
#include <errno.h>
#include <signal.h>
#include <string.h>

#define NOTUSED(V) ((void) V)
#define MHZ(x) ((long long)(x*1000000.0 + .5))
#define GHZ(x) ((long long)(x*1000000000.0 + .5))
#define NUM_SAMPLES 2600000
#define BUFFER_SIZE (NUM_SAMPLES * 2 * sizeof(int16_t))


struct stream_cfg {
    long long bw_hz; // Analog banwidth in Hz
    long long fs_hz; // Baseband sample rate in Hz
    long long lo_hz; // Local oscillator frequency in Hz
    const char* rfport; // Port name
    double gain_db; // Hardware gain
};

static void usage() {
    fprintf(stderr, "Usage: plutoplayer [options]\n"
        "  -t <filename>      Transmit data from file (required)\n"
        "  -a <attenuation>   Set TX attenuation [dB] (default -20.0)\n"
        "  -b <bw>            Set RF bandwidth [MHz] (default 5.0)\n"
        "  -u <uri>           ADALM-Pluto URI\n"
        "  -n <network>       ADALM-Pluto network IP or hostname (default pluto.local)\n");
    return;
}

static bool stop = false;

static void handle_sig(int sig)
{
    NOTUSED(sig);
    stop = true;
}

static char* readable_fs(double size, char* buf, size_t buf_size) {
    int i = 0;
    const char* units[] = { "B", "kB", "MB", "GB", "TB", "PB", "EB", "ZB", "YB" };
    while (size > 1024) {
        size /= 1024;
        i++;
    }
    snprintf(buf, buf_size, "%.*f %s", i, size, units[i]);
    return buf;
}

/*
 *
 */
int main(int argc, char** argv) {
    char buf[1024];
    int opt;
    const char* path = NULL;
    struct stream_cfg txcfg;
    FILE* fp = NULL;
    const char* uri = NULL;
    const char* ip = NULL;

    // TX stream default config
    txcfg.bw_hz = MHZ(3.0); // 3.0 MHz RF bandwidth
    txcfg.fs_hz = MHZ(2.6); // 2.6 MS/s TX sample rate
    txcfg.lo_hz = GHZ(1.575420); // 1.57542 GHz RF frequency
    txcfg.rfport = "A";
    txcfg.gain_db = -20.0;

    struct iio_context* ctx = NULL;
    struct iio_device* tx = NULL;
    struct iio_device* phydev = NULL;
    struct iio_channel* tx0_i = NULL;
    struct iio_channel* tx0_q = NULL;
    struct iio_buffer* tx_buffer = NULL;

    while ((opt = getopt(argc, argv, "t:a:b:n:u:")) != EOF) {
        switch (opt) {
        case 't':
            path = optarg;
            break;
        case 'a':
            txcfg.gain_db = atof(optarg);
            if (txcfg.gain_db > 0.0) txcfg.gain_db = 0.0;
            if (txcfg.gain_db < -80.0) txcfg.gain_db = -80.0;
            break;
        case 'b':
            txcfg.bw_hz = MHZ(atof(optarg));
            if (txcfg.bw_hz > MHZ(5.0)) txcfg.bw_hz = MHZ(5.0);
            if (txcfg.bw_hz < MHZ(1.0)) txcfg.bw_hz = MHZ(1.0);
            break;
        case 'u':
            uri = optarg;
            break;
        case 'n':
            ip = optarg;
            break;
        default:
            printf("Unknown argument '-%c %s'\n", opt, optarg);
            usage();
            return EXIT_FAILURE;
        }
    }

    //signal(SIGINT, handle_sig);

    if (path == NULL) {
        printf("Specify a path to a file to transmit\n");
        usage();
        return EXIT_FAILURE;
    }

    fp = fopen(path, "rb");
    if (fp == NULL) {
        fprintf(stderr, "ERROR: Failed to open TX file: %s\n", path);
        return EXIT_FAILURE;
    }
    fseek(fp, 0L, SEEK_END);
    size_t sz = ftell(fp);
    fseek(fp, 0L, SEEK_SET);
    readable_fs((double)sz, buf, sizeof(buf));
    printf("* Transmit file size: %s\n", buf);

    printf("* Acquiring IIO context\n");
    ctx = iio_create_default_context();
    if (ctx == NULL) {
        if (ip != NULL) {
            ctx = iio_create_network_context(ip);
        }
        else if (uri != NULL) {
            ctx = iio_create_context_from_uri(uri);
        }
        else {
            ctx = iio_create_network_context("pluto.local");
        }
    }

    if (ctx == NULL) {
        iio_strerror(errno, buf, sizeof(buf));
        fprintf(stderr, "Failed creating IIO context: %s\n", buf);
        return false;
    }

    struct iio_scan_context* scan_ctx;
    struct iio_context_info** info;
    scan_ctx = iio_create_scan_context(NULL, 0);
    if (scan_ctx) {
        int info_count = iio_scan_context_get_info_list(scan_ctx, &info);
        if (info_count > 0) {
            printf("* Found %s\n", iio_context_info_get_description(info[0]));
            iio_context_info_list_free(info);
        }
        iio_scan_context_destroy(scan_ctx);
    }

    printf("* Acquiring devices\n");
    int device_count = iio_context_get_devices_count(ctx);
    if (!device_count) {
        fprintf(stderr, "No supported PLUTOSDR devices found.\n");
        goto error_exit;
    }
    fprintf(stderr, "* Context has %d device(s).\n", device_count);

    printf("* Acquiring TX device\n");
    tx = iio_context_find_device(ctx, "cf-ad9361-dds-core-lpc");
    if (tx == NULL) {
        iio_strerror(errno, buf, sizeof(buf));
        fprintf(stderr, "Error opening PLUTOSDR TX device: %s\n", buf);
        goto error_exit;
    }

    iio_device_set_kernel_buffers_count(tx, 8);

    phydev = iio_context_find_device(ctx, "ad9361-phy");
    struct iio_channel* phy_chn = iio_device_find_channel(phydev, "voltage0", true);
    iio_channel_attr_write(phy_chn, "rf_port_select", txcfg.rfport);
    iio_channel_attr_write_longlong(phy_chn, "rf_bandwidth", txcfg.bw_hz);
    iio_channel_attr_write_longlong(phy_chn, "sampling_frequency", txcfg.fs_hz);
    iio_channel_attr_write_double(phy_chn, "hardwaregain", txcfg.gain_db);

    iio_channel_attr_write_bool(
        iio_device_find_channel(phydev, "altvoltage0", true)
        , "powerdown", true); // Turn OFF RX LO

    iio_channel_attr_write_longlong(
        iio_device_find_channel(phydev, "altvoltage1", true)
        , "frequency", txcfg.lo_hz); // Set TX LO frequency

    printf("* Initializing streaming channels\n");
    tx0_i = iio_device_find_channel(tx, "voltage0", true);
    if (!tx0_i)
        tx0_i = iio_device_find_channel(tx, "altvoltage0", true);

    tx0_q = iio_device_find_channel(tx, "voltage1", true);
    if (!tx0_q)
        tx0_q = iio_device_find_channel(tx, "altvoltage1", true);

    printf("* Enabling IIO streaming channels\n");
    iio_channel_enable(tx0_i);
    iio_channel_enable(tx0_q);

    ad9361_set_bb_rate(iio_context_find_device(ctx, "ad9361-phy"), txcfg.fs_hz);

    printf("* Creating TX buffer\n");

    tx_buffer = iio_device_create_buffer(tx, NUM_SAMPLES, false);
    if (!tx_buffer) {
        fprintf(stderr, "Could not create TX buffer.\n");
        goto error_exit;
    }

    iio_channel_attr_write_bool(
        iio_device_find_channel(iio_context_find_device(ctx, "ad9361-phy"), "altvoltage1", true)
        , "powerdown", false); // Turn ON TX LO

    int32_t ntx = 0;
    short* ptx_buffer = (short*)iio_buffer_start(tx_buffer);

    printf("* Transmit starts...\n");
    // Keep writing samples while there is more data to send and no failures have occurred.
    while (!feof(fp) && !stop) {
        fread(ptx_buffer, sizeof(short), BUFFER_SIZE / sizeof(short), fp);
        // Schedule TX buffer
        ntx = iio_buffer_push(tx_buffer);
        if (ntx < 0) {
            printf("Error pushing buf %d\n", (int)ntx);
            break;
        }
    }
    printf("Done.\n");

error_exit:
    fclose(fp);
    iio_channel_attr_write_bool(
        iio_device_find_channel(iio_context_find_device(ctx, "ad9361-phy"), "altvoltage1", true)
        , "powerdown", true); // Turn OFF TX LO                

    if (tx_buffer) { iio_buffer_destroy(tx_buffer); }
    if (tx0_i) { iio_channel_disable(tx0_i); }
    if (tx0_q) { iio_channel_disable(tx0_q); }
    if (ctx) { iio_context_destroy(ctx); }
    return EXIT_SUCCESS;
}
