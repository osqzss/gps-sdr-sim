#define _CRT_SECURE_NO_DEPRECATE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <unistd.h>
#include <getopt.h>
#include <signal.h>

#include <lime/LimeSuite.h>

#define EXIT_CODE_CONTROL_C (-3)
#define EXIT_CODE_NO_DEVICE (-2)
#define EXIT_CODE_LMS_OPEN  (-1)

#define TX_FREQUENCY    1575420000.0
#define TX_SAMPLERATE   2500000.0
#define TX_BANDWIDTH    5000000.0
#define DEFAULT_ANTENNA 1 // antenna with BW [30MHz .. 2000MHz]

#define STRINGIFY2(X) #X
#define STRINGIFY(X) STRINGIFY2(X)

static int control_c_received = 0;

static void control_c_handler (int sig, siginfo_t *siginfo, void *context){
    control_c_received = 1;
}
 
static void print_usage(const char *progname){
    printf("Usage: %s [option] < file" "\n"
            "\t" "-g <gain> or --gain <gain> with gain in [0.0 .. 1.0] set the so-called normalized RF gain in LimeSDR (default: 1.0 max RF power)" "\n"
            "\t" "-c <channel> or --channel <channel> with channel either 0 or 1 (default: 0)" "\n"
            "\t" "-a <antenna> or --antenna <antenna> with antenna in { 0, 1, 2 } (default:" STRINGIFY(DEFAULT_ANTENNA) ")" "\n"
            "\t" "-i <index> or --index <index> select LimeSDR if multiple devices connected (default: 0)" "\n"
            "\t" "-b <bits> or --bits <bits> select bit count in IQ sample in { 1, 8, 12, 16 }, (default: 16)" "\n"
            "\t" "-s <samplerate> or --samplerate <samplerate> configure BB sample rate (default: " STRINGIFY(TX_SAMPLERATE) ")" "\n"
            "\t" "-d <dynamic> --dynamic <dynamic> configure dynamic for the 1-bit mode (default: 2047, max 12-bit signed value supported by LimeSDR)" "\n"
        "Example:" "\n"
        "\t" "./limeplayer -s 1000000 -b 1 -d 1023 -g 0.1 < ../circle.1b.1M.bin" "\n", progname);
    exit(0);
}

int main(int argc, char *const argv[]){
    struct sigaction control_c;

    memset(&control_c, 0, sizeof(control_c));
    control_c.sa_sigaction = &control_c_handler;
 
    /* The SA_SIGINFO flag tells sigaction() to use the sa_sigaction field, not sa_handler. */
    control_c.sa_flags = SA_SIGINFO;
 
    if (sigaction(SIGTERM, &control_c, NULL) < 0) {
        perror ("sigaction");
        return(EXIT_CODE_CONTROL_C);
    }
    if (sigaction(SIGQUIT, &control_c, NULL) < 0) {
        perror ("sigaction");
        return(EXIT_CODE_CONTROL_C);
    }
    if (sigaction(SIGINT, &control_c, NULL) < 0) {
        perror ("sigaction");
        return(EXIT_CODE_CONTROL_C);
    }

    int device_count = LMS_GetDeviceList(NULL);
    if(device_count < 1){
        return(EXIT_CODE_NO_DEVICE);
    }
    lms_info_str_t *device_list = malloc(sizeof(lms_info_str_t) * device_count);
    device_count = LMS_GetDeviceList(device_list);

    int i = 0;
    while(i < device_count){
        // printf("device[%d/%d]=%s" "\n", i + 1, device_count, device_list[i]);
        i++;
    }

    double gain = 1.0;
    int32_t antenna = DEFAULT_ANTENNA;
    int32_t channel = 0;
    int32_t index = 0;
    int32_t bits = 16;
    double sampleRate = TX_SAMPLERATE;
    int32_t dynamic = 2047;

    while (1) {
        int option_index = 0;
        static struct option long_options[] = {
            {"gain",   required_argument, 0,  'g' },
            {"channel",   required_argument, 0,  'c' },
            {"antenna",   required_argument, 0,  'a' },
            {"index", required_argument, 0, 'i'},
            {"bits", required_argument, 0, 'b'},
            {"samplerate", required_argument, 0, 's'},
            {"dynamic", required_argument, 0, 'd'},
            {0,         0,                 0,  0 }
        };

        int c = getopt_long(argc, argv, "g:c:a:i:s:b:d:", long_options, &option_index);
        if (c == -1)
        break;

        switch (c) {
            case 0:
            #if 1
            fprintf(stderr, "option %s", long_options[option_index].name);
            if (optarg)
                fprintf(stderr, " with arg %s", optarg);
            fprintf(stderr, "\n");
            #endif

            break;

            case 'a':
                antenna = strtol(optarg, NULL, 0);
            break;
            case 'b':
                bits = strtol(optarg, NULL, 0);
            break;
            case 'c':
                channel = strtol(optarg, NULL, 0);
            break;
            case 'g':
                gain = strtod(optarg, NULL);
            break;
            case 'i':
                index = strtol(optarg, NULL, 0);
            break;
        case 's':
            sampleRate = strtod(optarg, NULL);
        break;
        case 'd':
            dynamic = strtol(optarg, NULL, 0);
        if(dynamic > 2047){
            dynamic = 2047;
        }
        break;
        default:
            print_usage(argv[0]);
        break;
        }
    }
    // Use correct values
    // Use existing device
    if(index < 0){
        index = 0;
    }
    if(index >= device_count){
        index = 0;
    }
    printf("Using device index %d [%s]" "\n", index, device_list[index]);

    // Normalized gain shall be in [0.0 .. 1.0]
    if(gain < 0.0){
        gain = 0.0;
    }
    if(gain > 1.0){
        gain = 1.0;
    }
    printf("Using normalized gain %lf" "\n", gain);


    lms_device_t *device = NULL;

    if(LMS_Open(&device, device_list[index], NULL)){
        return(EXIT_CODE_LMS_OPEN);
    }

    int lmsReset = LMS_Reset(device);
    if(lmsReset){
        printf("lmsReset %d(%s)" "\n", lmsReset, LMS_GetLastErrorMessage());
    }
    int lmsInit = LMS_Init(device);
    if(lmsInit){
        printf("lmsInit %d(%s)" "\n", lmsInit, LMS_GetLastErrorMessage());
    }

    int channel_count = LMS_GetNumChannels(device, LMS_CH_TX);
    // printf("Tx channel count %d" "\n", channel_count);
    if(channel < 0){
        channel = 0;
    }
    if(channel >= channel_count){
        channel = 0;
    }
    printf("Using channel %d" "\n", channel);

    int antenna_count = LMS_GetAntennaList(device, LMS_CH_TX, channel, NULL);
    // printf("TX%d Channel has %d antenna(ae)" "\n", channel, antenna_count);
    lms_name_t antenna_name[antenna_count];
    if(antenna_count > 0){
        int i = 0;
        lms_range_t antenna_bw[antenna_count];
        LMS_GetAntennaList(device, LMS_CH_TX, channel, antenna_name);
        for(i = 0 ; i < antenna_count ; i++){
            LMS_GetAntennaBW(device, LMS_CH_TX, channel, i, antenna_bw + i);
            // printf("Channel %d, antenna [%s] has BW [%lf .. %lf] (step %lf)" "\n", channel, antenna_name[i], antenna_bw[i].min, antenna_bw[i].max, antenna_bw[i].step);
        }
    }
    if(antenna < 0){
        antenna = DEFAULT_ANTENNA;
    }
    if(antenna >= antenna_count){
        antenna = DEFAULT_ANTENNA;
    }
    // LMS_SetAntenna(device, LMS_CH_TX, channel, antenna); // SetLOFrequency should take care of selecting the proper antenna
    
    LMS_SetNormalizedGain(device, LMS_CH_TX, channel, gain);
    // Disable all other channels
    LMS_EnableChannel(device, LMS_CH_TX, 1 - channel, false);
    LMS_EnableChannel(device, LMS_CH_RX, 0, true); /* LimeSuite bug workaround (needed since LimeSuite git rev 52d6129 - or v18.06.0) */
    LMS_EnableChannel(device, LMS_CH_RX, 1, false);
    // Enable our Tx channel
    LMS_EnableChannel(device, LMS_CH_TX, channel, true);

    int setLOFrequency = LMS_SetLOFrequency(device, LMS_CH_TX, channel, TX_FREQUENCY);
    if(setLOFrequency){
        printf("setLOFrequency(%lf)=%d(%s)" "\n", TX_FREQUENCY, setLOFrequency, LMS_GetLastErrorMessage());
    }

#ifdef __USE_LPF__
    lms_range_t LPFBWRange;
    LMS_GetLPFBWRange(device, LMS_CH_TX, &LPFBWRange);
    // printf("TX%d LPFBW [%lf .. %lf] (step %lf)" "\n", channel, LPFBWRange.min, LPFBWRange.max, LPFBWRange.step);
    double LPFBW = TX_BANDWIDTH;
    if(LPFBW < LPFBWRange.min){
        LPFBW = LPFBWRange.min;
    }
    if(LPFBW > LPFBWRange.max){
        LPFBW = LPFBWRange.min;
    }
    int setLPFBW = LMS_SetLPFBW(device, LMS_CH_TX, channel, LPFBW);
    if(setLPFBW){
        printf("setLPFBW(%lf)=%d(%s)" "\n", LPFBW, setLPFBW, LMS_GetLastErrorMessage());
    }
    int enableLPF = LMS_SetLPF(device, LMS_CH_TX, channel, true);
    if(enableLPF){
        printf("enableLPF=%d(%s)" "\n", enableLPF, LMS_GetLastErrorMessage());
    }
#endif

    lms_range_t sampleRateRange;
    int getSampleRateRange = LMS_GetSampleRateRange(device, LMS_CH_TX, &sampleRateRange);
    if(getSampleRateRange){
        printf("getSampleRateRange=%d(%s)" "\n", getSampleRateRange, LMS_GetLastErrorMessage());
    }else{
        // printf("sampleRateRange [%lf MHz.. %lf MHz] (step=%lf Hz)" "\n", sampleRateRange.min / 1e6, sampleRateRange.max / 1e6, sampleRateRange.step);
    }

    printf("Set sample rate to %lf ..." "\n", sampleRate);
    int setSampleRate = LMS_SetSampleRate(device, sampleRate, 0);
    if(setSampleRate){
        printf("setSampleRate=%d(%s)" "\n", setSampleRate, LMS_GetLastErrorMessage());
    }
    double actualHostSampleRate = 0.0;
    double actualRFSampleRate = 0.0;
    int getSampleRate = LMS_GetSampleRate(device, LMS_CH_TX, channel, &actualHostSampleRate, &actualRFSampleRate);
    if(getSampleRate){
        printf("getSampleRate=%d(%s)" "\n", getSampleRate, LMS_GetLastErrorMessage());
    }else{
        printf("actualRate %lf (Host) / %lf (RF)" "\n", actualHostSampleRate, actualRFSampleRate);
    }

    printf("Calibrating ..." "\n");
    int calibrate = LMS_Calibrate(device, LMS_CH_TX, channel, TX_BANDWIDTH, 0);
    if(calibrate){
        printf("calibrate=%d(%s)" "\n", calibrate, LMS_GetLastErrorMessage());
    }

    printf("Setup TX stream ..." "\n");
    lms_stream_t tx_stream = {.channel = channel, .fifoSize = 1024*1024, .throughputVsLatency = 0.5, .isTx = true, .dataFmt = LMS_FMT_I12};
    int setupStream = LMS_SetupStream(device, &tx_stream);
    if(setupStream){
        printf("setupStream=%d(%s)" "\n", setupStream, LMS_GetLastErrorMessage());
    }

    struct s16iq_sample_s {
        signed short int i;
        signed short int q;
    };

    int nSamples = (int)sampleRate / 100;
    struct s16iq_sample_s *sampleBuffer = (struct s16iq_sample_s*)malloc(sizeof(struct s16iq_sample_s) * nSamples);

    LMS_StartStream(&tx_stream);

    int loop = 0;
    if((12 == bits) || (16 == bits)){
        // File contains interleaved 16-bit IQ values, either with only 12-bit data, or with 16-bit data
        while((0 == control_c_received) && fread(sampleBuffer, sizeof(struct s16iq_sample_s), nSamples, stdin)){
            loop++;
            if(0 == (loop % 100)){
                struct timeval tv;
                gettimeofday(&tv, NULL);
                printf("gettimeofday()=> %ld:%06ld ; ", tv.tv_sec, tv.tv_usec);
                lms_stream_status_t status;
                LMS_GetStreamStatus(&tx_stream, &status); //Obtain TX stream stats
                printf("TX rate:%lf MB/s" "\n", status.linkRate / 1e6);
            }
            if(16 == bits){
                // Scale down to 12-bit
                // Quick and dirty, so -1 (0xFFFF) to -15 (0xFFF1) scale down to -1 instead of 0
                int i = 0;
                while(i < nSamples){
                    sampleBuffer[i].i >>= 4;
                    sampleBuffer[i].q >>= 4;
                    i++;
                }
            }
            int sendStream = LMS_SendStream(&tx_stream, sampleBuffer, nSamples, NULL, 1000);
            if(sendStream < 0){
                printf("sendStream %d(%s)" "\n", sendStream, LMS_GetLastErrorMessage());
            }
        }
    }else if(8 == bits){
        // File contains interleaved signed 8-bit IQ values
        struct s8iq_sample_s {
            signed char i;
            signed char q;
        };
        struct s8iq_sample_s *fileSamples = (struct s8iq_sample_s*)malloc(sizeof(struct s8iq_sample_s) * nSamples);
        while((0 == control_c_received) && fread(fileSamples, sizeof(struct s8iq_sample_s), nSamples, stdin)){
            loop++;
            if(0 == (loop % 100)){
                struct timeval tv;
                gettimeofday(&tv, NULL);
                printf("gettimeofday()=> %ld:%06ld ; ", tv.tv_sec, tv.tv_usec);
                lms_stream_status_t status;
                LMS_GetStreamStatus(&tx_stream, &status); //Obtain TX stream stats
                printf("TX rate:%lf MB/s" "\n", status.linkRate / 1e6);
            }
            // Up-Scale to 12-bit
            int i = 0;
            while(i < nSamples){
                sampleBuffer[i].i = (fileSamples[i].i << 4);
                sampleBuffer[i].q = (fileSamples[i].q << 4);
                i++;
            }
            int sendStream = LMS_SendStream(&tx_stream, sampleBuffer, nSamples, NULL, 1000);
            if(sendStream < 0){
                printf("sendStream %d(%s)" "\n", sendStream, LMS_GetLastErrorMessage());
            }
        }
        free(fileSamples);
    }else if(1 == bits){
        // File contains interleaved signed 1-bit IQ values
        // Each byte is IQIQIQIQ
        int16_t expand_lut[256][8];
        int i, j;
        for (i=0; i<256; i++){
            for (j=0; j<8; j++){
                expand_lut[i][j] = ((i>>(7-j))&0x1)?dynamic:-dynamic;
            }
        }
        printf("1-bit mode: using dynamic=%d" "\n", dynamic);
        // printf("sizeof(expand_lut[][])=%d, sizeof(expand_lut[0])=%d" "\n", sizeof(expand_lut), sizeof(expand_lut[0]));
        int8_t *fileBuffer = (int8_t*)malloc(sizeof(int8_t) * nSamples);
        while((0 == control_c_received) && fread(fileBuffer, sizeof(int8_t), nSamples / 4, stdin)){
            loop++;
            if(0 == (loop % 100)){
                struct timeval tv;
                gettimeofday(&tv, NULL);
                printf("gettimeofday()=> %ld:%06ld ; ", tv.tv_sec, tv.tv_usec);
                lms_stream_status_t status;
                LMS_GetStreamStatus(&tx_stream, &status); //Obtain TX stream stats
                printf("TX rate:%lf MB/s" "\n", status.linkRate / 1e6);
            }
            // Expand
            int src = 0;
            int dst = 0;
            while(src < (nSamples / 4)){
                memcpy(sampleBuffer + dst, expand_lut + fileBuffer[src], sizeof(expand_lut[0]));
                dst += 4;
                src++;
            }
            int sendStream = LMS_SendStream(&tx_stream, sampleBuffer, nSamples, NULL, 1000);
            if(sendStream < 0){
                printf("sendStream %d(%s)" "\n", sendStream, LMS_GetLastErrorMessage());
            }
        }
        free(fileBuffer);
    }

    LMS_StopStream(&tx_stream);
    LMS_DestroyStream(device, &tx_stream);

    free(sampleBuffer);

    LMS_EnableChannel(device, LMS_CH_TX, channel, false);
    LMS_Close(device);

    if(control_c_received){
        return(EXIT_CODE_CONTROL_C);
    }
    return(0);
}

