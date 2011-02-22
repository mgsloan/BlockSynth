#ifndef GLOBAL_AUDIO_H
#define GLOBAL_AUDIO_H

#include <ao/ao.h>

using namespace cv;

pthread_t ao_thread;

ao_device *ao_dev;
ao_sample_format ao_format;
int ao_default_driver;
char *ao_buffer;
int ao_buf_size;
int ao_sample;
int ao_cnt;
bool die = false;

Mat sound;

bool consumed = false;

// Libao thread which mixes two sine waves.
void *libao_threadfunc(void* arg) {
    while (!die) {
        for (int i = 0; i < ao_cnt ; i++) {
            int ix = i;// + j;
            ao_buffer[4*ix] = ao_buffer[4*ix+2] =
                sound.at<short>(0, i) & 0xff;
            ao_buffer[4*ix+1] = ao_buffer[4*ix+3] = 
                (sound.at<short>(0, i) >> 8) & 0xff;
        }
        ao_play(ao_dev, ao_buffer, ao_buf_size);
    }
    return NULL;
}

// libao initialization stuff.
int libao_start() {
    ao_initialize();

    ao_default_driver = ao_default_driver_id();

    memset(&ao_format, 0, sizeof(ao_sample_format));
    ao_format.bits = 16;
    ao_format.channels = 2;
    ao_format.rate = 44100;
    ao_format.byte_format = AO_FMT_LITTLE;

    /* -- Open driver -- */
    ao_dev = ao_open_live(ao_default_driver, &ao_format, NULL );
    if (ao_dev == NULL) {
        fprintf(stderr, "Error opening audio device.\n");
        return 1;
    }

    ao_cnt = ao_format.rate / 16;
    ao_buf_size = ao_format.bits/8 * ao_format.channels * ao_cnt;
    ao_buffer = (char*)calloc(ao_buf_size, sizeof(char));

    sound = Mat(1, ao_cnt, CV_16UC1);

    // Spin off the thread to periodically supply the audio.
    int res = pthread_create(&ao_thread, NULL, libao_threadfunc, NULL);
    if (res) {
        printf("pthread_create failed\n");
        return 1;
    }

    return 0;
}

// libao cleanup stuff
void libao_finish() {
    die = true;
    pthread_join(ao_thread, NULL);
    ao_close(ao_dev);
    ao_shutdown();
}

#endif
