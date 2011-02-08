#include "common.h"

int main(int argc, char **argv)
{
    int res = freenect_start();
    if (res != 0) return res;

    int val = 3;

    while (!die) {
        cvCreateTrackbar("val", "dx", &val, 10, NULL);

        // clone off some safe copies.
        Mat rgb = rgbMat.clone(), depth = depthMat.clone();
        Mat depthf;
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
 
    freenect_finish();

    pthread_exit(NULL);
}
