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

ao_device *ao_dev;
ao_sample_format ao_format;
int ao_default_driver;
char *ao_buffer;
int ao_buf_size;
int ao_sample;
int ao_cnt;
Mat sound(1, 3000, CV_16UC1);

// Libao thread which mixes two sine waves.
void *libao_threadfunc(void* arg) {

    while (!die) {
      for (int i = 0; i < ao_cnt ; i++) {


                int ix = i;// + j;
                ao_buffer[4*ix] = ao_buffer[4*ix+2] =
                sound.at<short>(0, i) & 0xff;
                ao_buffer[4*ix+1] = ao_buffer[4*ix+3] = 
                  (sound.at<short>(0, i) >> 8) & 0xff;
            //}
        }
        ao_play(ao_dev, ao_buffer, ao_buf_size);
    }

    return NULL;
}

void extractSpectrum(Mat img, Mat spect,// bool drawIt,
  int x, int fy, int ty,    // line in camera space, maps to
  int ff, int tf,           // interval in frequency
  unsigned fd, unsigned td, // interval in distance, maps to
  float fa, float ta) {     // interval in amplitude

    float fratio = (float)(tf - ff) / (ty - fy),
          aratio = (ta - fa) / (td - fd);
    for (int y = fy; y <= ty; y++) {
        unsigned val = img.at<unsigned>(y, x);
        int ix = (int)((y - fy) * fratio) + ff;
        if (val > td || val < fd) continue;
        printf("val = %u\n", val);
        val = max(fd, val);
        float inc = (val - fd) * aratio + fa;
        spect.at<float>(0, ix) += inc;
    }
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

    Mat prev;
    int prevSet = false;


    Mat spectrum(1, 300, CV_32F);
    while (!die) {
 
        Mat depthf = depthMat.clone();
//        Mat depthf;
//        depthMat.convertTo(depthf, CV_8UC1, 255.0/2048.0);
//        display the sample reticle
        int xp = 200;
        line(rgbMat, Point(xp, 30), Point(xp, 330), Scalar(0,0,255));

        if (prevSet) {
//            prev -= 10;
//            Mat diff = depthf - prev, gt;
//            compare(diff,  20, gt, CMP_GT);
//            diff.setTo(0, gt);
//            diff *= 12;
//            imshow("diff", diff);
/*
            for (int i = 30; i < 330; i++) {
                char val1 = depthf.at<char>(i - 30, xp);
                char prev = 0;
                spectrum.at<float>(0,i-30)  = 0;
                if (val1 < 120 & val1 > 0) {
                    printf("%i\n", val1);
                    line(rgbMat, Point(xp, i), Point(xp + val1, i), Scalar(255,0,0));
//                    if (prev == 0)
                    spectrum.at<float>(0,i-30) = (val1 - 40) * 5;
                    prev = val1;
                } else prev = 0;
            }
 */

 //           Mat gt;
//            compare(depthf, 24379763, 
            printf("%i %i \n", depthf.size().width, depthf.size().height);
            Mat spect(1, 3000, CV_32F), result;
            for (int i = 0; i < 3000; i++) spect.at<float>(0, i) = 0;
            extractSpectrum(depthf, spect,
                xp, 30, 330,        // line
                100, 1000,            // frequency
                24379763, 60000000, // distance
                0, 50);            // amplitude

            extractSpectrum(depthf, spect,
                xp + 5, 30, 330,        // line
                200, 2000,            // frequency
                24379763, 60000000, // distance
                0, 50);            // amplitude

            extractSpectrum(depthf, spect,
                xp + 10, 30, 330,        // line
                400, 4200,            // frequency
                24379763, 60000000, // distance
                0, 50);            // amplitude

            //resize(spectrum, spect, Size(3000,1));
  //          int fromSz = spectrum.size().width, toSz = spect.size().width;
  //          float ratio = (float)toSz / fromSz;
  //          printf("%i %i %f\n", fromSz, toSz, ratio);
  //          for(int i = 0; i < fromSz; i++) {
  //              spect.at<float>(0, (int)(i * ratio)) = spectrum.at<float>(0, i);
  //          }
            dft(spect, result, DFT_INVERSE);
            float prev = 0;
            for (int i = 0; i < result.size().width; i++) {
                float i2 = (float)i / 1732.0;
                float val = result.at<float>(0,i) * i2 * i2;
            //    printf("%i %f\n", i, result.at<float>(0,i));
                if (i % 10 == 0) { 
                    line(rgbMat, Point((i/10)-1, prev + 100), Point(i/10, val + 100), Scalar(0,255,0));
                }
                sound.at<short>(0,i) = (short)(val + 16000);
                prev = val;
            }
        }

        prev = depthf.clone();
        prevSet = true;

        imshow("rgb", rgbMat);
        imshow("depth", depthf);


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
