#include "common.h"

int main(int argc, char **argv)
{
    int res = freenect_start();
    if (res != 0) return res;

    int hf = 250, ht = 10, lf = 128;

    while (!die) {
        cvCreateTrackbar("Hue_From", "depth", &hf, 255, NULL);
        cvCreateTrackbar("Hue_To", "depth", &ht, 255, NULL);
        cvCreateTrackbar("Lum_From", "depth", &lf, 255, NULL);

        Mat depthf = depthMat.clone();
        cvtColor(rgb, hsv, CV_RGB2HSV);

        int cwidth = hsv.size().width, cheight = hsv.size().height;

        Mat result(cheight, cwidth, CV_8UC1);

        imshow("rgb", rgb);
        vector<Mat> planes;
        split(rgb, planes);

/*      imshow("r", planes[0]);
        imshow("g", planes[1]);
        imshow("b", planes[2]);
*/
//        split(hsv, planes);
//        imshow("hue", planes[0]);

        for (int x = 0; x < cwidth; x++) {
            for (int y = 0; y < cheight; y++) {
                uchar hue = planes[0].at<uchar>(y, x);
                uchar sat = planes[1].at<uchar>(y, x);
                uchar val = planes[2].at<uchar>(y, x);
                result.at<char>(y, x) = 0;
                if (val < lf) continue;
                if (hf > ht ? hue > hf || hue < ht
                            : hue < hf && hue > ht) {
                    result.at<char>(y, x) = 255;
                }
            }
        }
        imshow("depth", result);

//        int dwidth = depthf.size().width, dheight = depthf.size().height;

//        minMaxLoc(depthf, NULL, NULL, &headLoc, NULL);

        char k = cvWaitKey(5);
        if( k == 27 ) break;
    }
 
    freenect_finish();

    pthread_exit(NULL);
}
