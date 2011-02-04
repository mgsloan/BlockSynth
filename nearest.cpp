#include <cv.h>
#include <highgui.h>
#include <cvaux.h>

#include <cmath>
#include <iostream>
#include <stdio.h>
#include <limits>
#include <string.h>

#include <libusb.h>
#include "libfreenect.h"

#include <ao/ao.h>

#include <pthread.h>

using namespace cv;
using namespace std;

Mat depthMat(Size(640,480),CV_16UC1),
    rgbMat(Size(640,480),CV_8UC3,Scalar(0));
pthread_t fnkt_thread;
pthread_t ao_thread;
freenect_device *f_dev;
pthread_mutex_t buf_mutex = PTHREAD_MUTEX_INITIALIZER;
freenect_context *f_ctx;
pthread_cond_t frame_cond = PTHREAD_COND_INITIALIZER;
 
int got_frames = 0;
bool die = false;

// Callback to handle 11-bit depth buffer, marshalling them as openCV Mat
void depth_cb(freenect_device *dev, void*depth, uint32_t timestamp)
{
    pthread_mutex_lock(&buf_mutex);
 
    //copy to ocv buf...
    memcpy(depthMat.data, depth, FREENECT_DEPTH_11BIT_SIZE);
 
    got_frames++;
    pthread_cond_signal(&frame_cond);
    pthread_mutex_unlock(&buf_mutex);
}
 
// Callback to handle rgb frames, marshalling them as openCV Mat.
void rgb_cb(freenect_device *dev, void*rgb, uint32_t timestamp)
{
    pthread_mutex_lock(&buf_mutex);
    got_frames++;
    //copy to ocv_buf..
    memcpy(rgbMat.data, rgb, FREENECT_VIDEO_RGB_SIZE);
 
    pthread_cond_signal(&frame_cond);
    pthread_mutex_unlock(&buf_mutex);
}


// Thread to loop and query the kinect data as fast as possible.
void *freenect_threadfunc(void* arg) {
    cout << "freenect thread"<<endl;
    while(!die && freenect_process_events(f_ctx) >= 0 ) {}
    cout << "freenect die"<<endl;
    return NULL;
}


// Globar parameters for libao thread.

double ao_freq;
double ao_freq2;
ao_device *ao_dev;
ao_sample_format ao_format;
int ao_default_driver;
char *ao_buffer;
int ao_buf_size;
int ao_sample;
int ao_cnt;

// Libao thread which mixes two sine waves.
void *libao_threadfunc(void* arg) {
    while (!die) {
        for (int i = 0; i < ao_cnt; i++) {
            ao_sample = (int)(0.75 * 32768.0 *
            sin(2 * M_PI * ao_freq * ((double) i/ao_format.rate)) / 2) + 
                        (int)(0.75 * 32768.0 *
            sin(2 * M_PI * ao_freq2 * ((double) i/ao_format.rate)) / 2);

            /* Put the same stuff in left and right channel */
            ao_buffer[4*i] = ao_buffer[4*i+2] = ao_sample & 0xff;
            ao_buffer[4*i+1] = ao_buffer[4*i+3] = (ao_sample >> 8) & 0xff;
        }
        ao_play(ao_dev, ao_buffer, ao_buf_size);
    }
    return NULL;
}

int main(int argc, char **argv)
{
    int res;
 
    // Freenect initialization stuff follows

    if (freenect_init(&f_ctx, NULL) < 0) {
        printf("freenect_init() failed\n");
        return 1;
    }
 
    freenect_set_log_level(f_ctx, FREENECT_LOG_INFO);
 
    int nr_devices = freenect_num_devices (f_ctx);
    printf ("Number of devices found: %d\n", nr_devices);
 
    int user_device_number = 0;
    if (argc > 1)
        user_device_number = atoi(argv[1]);
 
    if (nr_devices < 1)
        return 1;
 
    if (freenect_open_device(f_ctx, &f_dev, user_device_number) < 0) {
        printf("Could not open device\n");
        return 1;
    }
 
    freenect_set_tilt_degs(f_dev,0);
    freenect_set_led(f_dev,LED_RED);
    freenect_set_video_format(f_dev, FREENECT_VIDEO_RGB);
    freenect_set_depth_format(f_dev, FREENECT_DEPTH_11BIT);
    freenect_set_depth_callback(f_dev, depth_cb);
    freenect_set_video_callback(f_dev, rgb_cb);
 
    freenect_start_depth(f_dev);
    freenect_start_video(f_dev);

    // libao initialization stuff follows.

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

    // Spin off the thread to periodically poll the Kinect.
    res = pthread_create(&fnkt_thread, NULL, freenect_threadfunc, NULL);
    if (res) {
        printf("pthread_create failed\n");
        return 1;
    }
 
    // Spin off the thread to periodically supply the audio.
    res = pthread_create(&ao_thread, NULL, libao_threadfunc, NULL);
    if (res) {
        printf("pthread_create failed\n");
        return 1;
    }


    int val = 3;

    while (!die) {
        cvCreateTrackbar("val", "dx", &val, 10, NULL);

        Mat depthf; //= depthMat.clone();
        depthMat.convertTo(depthf, CV_8UC1, 255.0/2048.0);

        if (val == 1 || val == 3 || val == 5 || val == 7) {
            Mat dx, dy, gt;
            Sobel(depthf, dx, CV_8UC1, 1, 0, val);
            compare(dx, 20, gt, CMP_GT);
            dx.setTo(0, gt);
            Sobel(depthf, dy, CV_8UC1, 0, 1, val);
            compare(dy, 20, gt, CMP_GT);
            dy.setTo(0, gt);
            dx *= 10;
            dy *= 10;

            imshow("dx", dx);
            imshow("dy", dy);
        }

        char k = cvWaitKey(5);
        if( k == 27 ) break;
    }
 
    printf("-- done!\n");
 
//    destroyWindow("rgb");
//    destroyWindow("depth");
 
    pthread_join(fnkt_thread, NULL);
    pthread_join(ao_thread, NULL);
    pthread_exit(NULL);

    ao_close(ao_dev);

    ao_shutdown();
}
