#include "common.h"
// enable if you use AUDIO
// #include "audio.h"

int main(int argc, char **argv)
{
    int res = freenect_start();
    if (res != 0) return res;
// AUDIO
//    res = libao_start();
//    if (res != 0) return res;

    while (!die) {
        // clone off some safe copies.
        Mat rgb = rgbMat.clone(), depth = depthMat.clone();

        // 8-bit version of depth map
        Math depthf;
        depth.convertTo(depthf, CV_8U, 256.0 / 2048.0);

        // Do things.

        imshow("rgb", rgb);
        imshow("depth", depthf);

        char k = cvWaitKey(5);
        if( k == 27 ) break;
    }
 
    freenect_finish();
// AUDIO
//    libao_finish();

    pthread_exit(NULL);
}
