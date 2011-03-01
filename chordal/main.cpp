#include <ntk/camera/kinect_grabber.h>
#include <ntk/camera/rgbd_processor.h>
#include <ntk/utils/opencv_utils.h>

#define OSC_COUNT 5
#include "audio.h"

using namespace ntk;
using namespace cv;

float logeto2 = 1 / log (2);

static const double pi = 3.14159265358979323846;

inline static double square(int a) { return a * a; }

void extractSpectrum(Mat img, Mat spect, // depth input, spectrum output
  int x, int fy, int ty,    // line in camera space, maps to
  int ff, int tf,           // interval in frequency.
  float fd, float td,       // interval in distance, maps to
  float fa, float ta,       // interval in amplitude.
  bool logScale) {

    float deltaF = logScale ? log ((float) tf / ff) * logeto2
                            : tf - ff;
    float fratio = deltaF / (ty - fy),
          aratio = (ta - fa) / (td - fd);
    float logfrom = log(ff) * logeto2;
    bool prev = false;
    for (int y = fy; y <= ty; y++) {
        // calculate amplitude of value.
        float val = img.at<float>(y, x);
        if (val > td || val < fd) {
            prev = false;
            continue;
        }
        if (prev) continue; else prev = true;
        //val = max(fd, val);
        float amp = (td - val) * aratio + fa;

        float freq = (y - fy) * fratio;
        freq *= logScale ? pow(2, freq + logfrom)
                         : freq + ff;

        // calculate frequency / increment appropriate index in spectrum.
        spect.at<float>(0, (int)freq) += amp;
    }
}

//int ssize = 3000, dsize = 3000;
//int decay = 90, fscale = 50;

void drawSpectrum(Mat img, Mat spect, int x, int fy, int ty) {
    int size = spect.size().width;
    float ratio = (float)(ty - fy) / size;
    for (int i = 0; i < size; i++) {
       float a = spect.at<float>(0,i);
       int y = fy + (int)((float)i * ratio);
       line(img, Point(x, y), Point(x + a, y), Scalar(255,0,0));
    }
}

Mat getIntervals(Mat slice, float ratio) {
    vector<float> results;
    int height = slice.size().height;
    int prev_y = -1;
    float start_val;
    for (int y = 0; y < height; y++) {
        float value = slice.at<float>(y, 0);
        if (prev_y != -1) {
            if (value < 0.4 || fabs(value - start_val) > ratio * start_val) {
            //    printf("%f %i %i\n", start_val, prev_y, y - 1);
                results.push_back(start_val);
                results.push_back(prev_y);
                results.push_back(y - 1);
                prev_y = -1;
            }
        } else if (value > 0.4) {
            prev_y = y;
            start_val = value;
        }
    }
    Mat result(results, true);
    result.reshape(0, results.size() / 3);
    return result;
}

int scaleValueLog(float y, float fy, float ty, float ff, float octaves) {
    float deltaF = log (octaves * 2) * logeto2;
    float freq = (y - fy) * (deltaF / (ty - fy));
    return pow(2, freq + log(ff) * logeto2);
}

int scaleValueLin(float y, float fy, float ty, float ff, float tf) {
    return (y - fy) * ((tf - ff) / (ty - fy)) + ff;
}

int main() {
  int res = libao_start();
  if (res) return res;

  KinectGrabber grabber;
  grabber.initialize();

  // Set camera tilt.
  grabber.setTiltAngle(15);
  grabber.start();

  // Postprocess raw kinect data.
  RGBDProcessor processor;
  processor.setFilterFlag(RGBDProcessor::ComputeKinectDepthBaseline, true);

  // OpenCV windows.
  namedWindow("dac");

  int scale = 10, fromIx = 10, toIx = 1000;
  int sound_width = sound.size().width;
  int ssize = sound_width * 2; 

  Mat spectrum(1, ssize, CV_32F);
  Mat prev_result(1, ssize, CV_32F);
  Mat prev_slice(480, 1, CV_32F);
  for (int i = 0; i < 480; i++) prev_slice.at<float>(i, 0) = 0;
  for (int i = 0; i < ssize; i++) spectrum.at<float>(0, i) = 0;

  Mat old_frame, flow;

  // Current image. An RGBDImage stores rgb and depth data.
  RGBDImage current_frame;
  while (true)
  {
    cvCreateTrackbar("scale", "dac", &scale, 100, NULL);
    cvCreateTrackbar("from", "dac", &fromIx, ssize, NULL);
//    cvCreateTrackbar("to", "dac", &toIx, ssize, NULL);

    grabber.waitForNextFrame();
    grabber.copyImageTo(current_frame);
    processor.processImage(current_frame);

    // Show the frames per second of the grabber
    int fps = grabber.frameRate();
    cv::putText(current_frame.rgbRef(),
        cv::format("%d fps", fps),
        Point(10,20), 0, 0.5, Scalar(255,0,0,255));

    // Compute color encoded depth.
    cv::Mat3b dac;
    compute_color_encoded_depth(current_frame.depth(), dac);

    Mat depth = current_frame.depth();

    Mat slice = depth.col(120), slmask, dy;
    compare(slice, 0.4, slmask, CMP_GT);

    Mat ivls = getIntervals(slice, 0.5);
    int ivlCount = ivls.size().height;
    vector<float> pos;
    for (int i = 0; i < ivlCount; i++) {
        float value = ivls.at<float>(i, 0),
              from  = ivls.at<float>(i, 1),
              to    = ivls.at<float>(i, 2);
        if (value < 1 && to - from > 2) {
            float mean = (from + to) / 2;
//            spectrum.at<float>(0, (int)mean) = 100;
            pos.push_back(mean); 
            //printf("%i ", (int)mean);
        }
    }

    float height = depth.size().height, octaves = 2;
    float spacing = height / (octaves * 12.0);
    for (int i = 0; i < octaves * 12; i++) {
        line(dac, Point(150, i * spacing), Point(155, i * spacing), Scalar(0,0,0));
    }
    for (int i = 0; i < pos.size(); i++) {
        float val = floor(pos[i] / spacing);
        float ffreq = scaleValueLog(val, 0, 24, fromIx, octaves);
        spectrum.at<float>(0, ffreq) = 100;
    }

/*
    vector<Mat> rgbplanes;
    split(current_frame.rgb(), rgbplanes);
    Mat frame = rgbplanes[1];
    if (!old_frame.empty()) {
        int flags = 0; //flow.empty() ? 0 : OPTFLOW_USE_INITIAL_FLOW;
        calcOpticalFlowFarneback(old_frame, frame, flow, 0.5, 3, 3, 3, 5, 1.1, flags);
        vector<Mat> planes;
        split(flow, planes);
        planes[0] += 20;
        planes[1] += 20;
        planes[0] /= 40;
        planes[1] /= 40;
        imshow("flowx", planes[0]);
        imshow("flowy", planes[1]);
    }
    frame.copyTo(old_frame);
    */
		

/*
    if (pos.size() >= 1) {
        float height = depth.size().height, octaves = 2;
        float val = floor(pos[0] * (octaves * 12.0) / height);
        float ffreq = scaleValueLog(val, 0, 24, fromIx, octaves);
        printf("%f \n", ffreq);
        for (int j = 1; j < 2; j++) {
            int fund = ffreq * j;
            if (fund > ssize) break;
            //if (pos.size() == 1)
            spectrum.at<float>(0, fund) = 100 / j;
            if (pos.size() >= 2) {
                int fifth = ffreq * log (4.0 / 3.0) * logeto2 * j;
                if (fifth < ssize)
                    spectrum.at<float>(0, fifth) = 100 / j;
            }
            if (pos.size() > 2) {
                if (pos[1] - pos[0] > pos[2] - pos[1]) {
                    int third = ffreq * 5.0 / 4.0 * j;
                    if (third < ssize)
                        spectrum.at<float>(0, third) = 100 / j;
    //                printf("major\n");
                } else {
                    int third = ffreq * 6.0 / 5.0 * j;
                    if (third < ssize)
                        spectrum.at<float>(0, third)  = 100 / j;
    //                printf("minor\n");
                }
            } else {
            }
        }
    }
    */

    //spectrum.at<float>(0, (int) mean) = (0.7 - value) * 300;

//    Sobel(depth, dy, CV_32F, 0, 1);
//    imshow("dy", dy);

    line(dac, Point(150, 0), Point(150, 480), Scalar(0,0,0));

//    for (int i = -4; i < 5; i++) {
//        int ypos = minLoc.y + i * 30;
//        line(dac, Point(150, ypos), Point(160, ypos), Scalar(64,64,64));
//    }
    
    drawSpectrum(dac, spectrum, 150, 1, 479);

    Mat result;
    dft(spectrum, result, DFT_INVERSE);
    for (int i = ssize / 2; i < ssize; i++) spectrum.at<float>(0, i) = 0;

    float prev = 0;
    for (int i = 0; i < sound_width; i++) {
        float coef = (float)i / (sound_width - 1) / 2;
        float prev_val = prev_result.at<float>(0, i + sound_width);
        float cur_val  = result.at<float>(0, i);
        prev_val *= 0.52 - 0.46 * cos (3.14159 * (coef - 0.5));
        cur_val  *= 0.52 - 0.46 * cos (3.14159 * coef);
        float val = cur_val + prev_val;
        int prev_xpos = (float)(scale / 100.0) * (i - 1);
        int xpos = (float)(scale / 100.0) * i;
        line(dac, Point(prev_xpos, prev / 2 + 100), 
                  Point(xpos,   val / 2 + 100), Scalar(0,0,0));
        line(dac, Point(prev_xpos, prev / 2 + 101), 
                  Point(xpos,   val / 2 + 101), Scalar(0,0,0));
        sound.at<short>(0,i) = (short)(val * 10 + 16000);
        prev = val;
    }
    spectrum *= 0;

    result.copyTo(prev_result);

    imshow("dac", dac);

    // Enable switching to InfraRead mode.
    unsigned char c = cv::waitKey(10) & 0xff;
    if (c == 'q')
      exit(0);
  }

  libao_finish();

  return 0;
}
