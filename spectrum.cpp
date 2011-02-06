#include "audio.h"

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
    int res = freenect_start();
    if (res != 0) return res;

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

    while (!die) {
 
        Mat rgb = rgbMat.clone(), depthfx, depthf, depthfy;
        depthMat.convertTo(depthf, CV_8UC1, 255.0/2048.0);
        depthMat.convertTo(depthfy, CV_8UC1, 255.0/2048.0);
        /*Sobel(depthfx, dx, CV_8UC1, 1, 0, 3);
        Sobel(depthfy, dy, CV_8UC1, 0, 1, 3);*/
        int histSize[] = {256};
        float hranges[] = { 0, 256.0 };
        const float* ranges[] = { hranges };
        MatND hist;
        int channels[] = {0};

        // Calculate depth histogram in order to make assumptions regarding
        // background clip depth.
        calcHist( &depthfx, 1, channels, Mat(), // do not use mask
                 hist, 1, histSize, ranges,
                 true, // the histogram is uniform
                 false );
        double histMax=0;
        int histIx;
        minMaxLoc(hist, 0, &histMax, 0, &histIx);

        // draw histogram
        rectangle(rgb, Point(0, 0), Point(256, 100), Scalar::all(0),
        CV_FILLED);
        if(histMax > 0) {
          for( int h = 0; h < 256; h++ ) {
              double val = hist.at<float>(h);
              rectangle(rgb, Point(h, 0), Point((h+1), val * 100 /
                  histMax), Scalar::all(255), CV_FILLED);
          }
        }

        int xp = 200;
        line(rgb, Point(xp, 30), Point(xp, 330), Scalar(0,0,255));

        Mat spect(1, 3000, CV_32F), result;
        for (int i = 0; i < 3000; i++) spect.at<float>(0, i) = 0;

        extractSpectrum(depthf, spect,
            xp, 30, 330,        // line
            0, 1000,            // frequency
            24379763, 60000000, // distance
            0, 50);            // amplitude

        dft(spect, result, DFT_INVERSE);
        float prev = 0;
        for (int i = 0; i < result.size().width; i++) {
            float i2 = (float)i / 1732.0;
            float val = result.at<float>(0,i) * i2 * i2;
            if (i % 10 == 0) { 
                line(rgb, Point((i/10)-1, prev + 100), Point(i/10, val + 100), Scalar(0,255,0));
            }
            sound.at<short>(0,i) = (short)(val + 16000);
            prev = val;
        }

        imshow("rgb", rgb);
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
