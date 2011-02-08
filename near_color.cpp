#include "common.h"

int main(int argc, char **argv)
{
    int res = freenect_start();
    if (res != 0) return res;

    int hf = 250, ht = 10, lf = 128, sf = 128;

    while (!die) {
        cvCreateTrackbar("Hue_From", "h", &hf, 255, NULL);
        cvCreateTrackbar("Hue_To", "h", &ht, 255, NULL);
        cvCreateTrackbar("Lum_From", "v", &lf, 255, NULL);
        cvCreateTrackbar("Sat_From", "s", &sf, 255, NULL);

        Mat rgb = rgbMat.clone(), depthf = depthMat.clone();
        Mat hsv;
        cvtColor(rgb, hsv, CV_RGB2HSV);

        int cwidth = hsv.size().width, cheight = hsv.size().height;

        Mat resultR(cheight, cwidth, CV_8UC1);
        Mat resultG(cheight, cwidth, CV_8UC1);
        Mat resultB;

        vector<Mat> planes, result;
        split(hsv, planes);

        imshow("s", planes[0]);
        imshow("h", planes[1]);
        imshow("v", planes[2]);

        for (int i = 0; i < 3; i++) result.push_back(Mat(cheight, cwidth, CV_8UC1));

        for (int x = 0; x < cwidth; x++) {
            for (int y = 0; y < cheight; y++) {
                uchar hue = planes[0].at<uchar>(y, x);
                uchar sat = planes[1].at<uchar>(y, x);
                uchar val = planes[2].at<uchar>(y, x);
                result[0].at<char>(y, x) = result[1].at<char>(y, x) = 
                result[2].at<char>(y, x) = 0;
                if (val > lf && sat > sf) {
                    if (hue < 10 || hue > 250) result[0].at<char>(y, x) = 255;
                    if(hf > ht ? hue > hf || hue < ht
                               : hue > hf && hue < ht)
                        result[1].at<char>(y, x) = 255;
                }
            }
        }
        Mat out;
        merge(result, out);
        display("rgb", out);

        char k = cvWaitKey(5);
        if( k == 27 ) break;
    }
 
    freenect_finish();

    pthread_exit(NULL);
}
