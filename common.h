#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cmath>
#include <iostream>
#include <stdio.h>
#include <limits>
#include <string.h>

#include <libusb.h>
#include "libfreenect.h"

#include <pthread.h>

using namespace cv;
using namespace std;

Mat depthMat(Size(640,480),CV_16UC1),
    rgbMat(Size(640,480),CV_8UC3,Scalar(0));
pthread_t fnkt_thread;
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
    cvtColor(rgbMat, rgb, CV_RGB2BGR);
 
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

int freenect_start() {
    // Freenect initialization stuff follows

    if (freenect_init(&f_ctx, NULL) < 0) {
        printf("freenect_init() failed\n");
        return 1;
    }
 
    freenect_set_log_level(f_ctx, FREENECT_LOG_INFO);
 
    int nr_devices = freenect_num_devices (f_ctx);
    printf ("Number of devices found: %d\n", nr_devices);
 
    int user_device_number = 0;
 
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

    // Spin off the thread to periodically poll the Kinect.
    int res = pthread_create(&fnkt_thread, NULL, freenect_threadfunc, NULL);
    if (res) {
        printf("pthread_create failed\n");
        return 1;
    }   
    return 0;
}

// freenect cleanup
void freenect_finish() {
    pthread_join(fnkt_thread, NULL);
}
