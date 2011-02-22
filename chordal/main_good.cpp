#include <ntk/camera/kinect_grabber.h>
#include <ntk/camera/rgbd_processor.h>
#include <ntk/utils/opencv_utils.h>

#define OSC_COUNT 5
#include "audio.h"

using namespace ntk;
using namespace cv;

void extractSpectrum(Mat img, Mat spect, // depth input, spectrum output
  int x, int fy, int ty,    // line in camera space, maps to
  int ff, int tf,           // interval in frequency.
  float fd, float td,       // interval in distance, maps to
  float fa, float ta,       // interval in amplitude.
  bool logScale) {

    float deltaF = logScale ? log ((float) tf / ff)
                            : tf - ff;
    float fratio = deltaF / (ty - fy),
          aratio = (ta - fa) / (td - fd);
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

        // calculate frequency / increment appropriate index in spectrum.
        float freq = (y - fy) * fratio;
        freq = logScale ? exp(freq + log(ff))
                        : freq + ff;      
        //printf("%f\n", freq);
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
//  namedWindow("color");
//  namedWindow("depth");
  namedWindow("dac");

  int scale = 10;
  int sound_width = sound.size().width;
  int ssize = sound_width * 2; 

  Mat spectrum(1, ssize, CV_32F);
  Mat prev_result(1, ssize, CV_32F);
  Mat prev_slice(480, 1, CV_32F);
  for (int i = 0; i < 480; i++) prev_slice.at<float>(i, 0) = 0;

  // Current image. An RGBDImage stores rgb and depth data.
  RGBDImage current_frame;
  while (true)
  {
    cvCreateTrackbar("scale", "dac", &scale, 100, NULL);

  /*
    int wid = sound.size().width;
    for (int i = 0; i < sound.size().width; i++) {
      float frac = (float)i / wid;
      float coef = frac < 0.05 ? frac : (frac > 0.95 ? (1 - frac) : 0.05);
      sound.at<short>(0, i) = (short)(sin((float)i / 10.0) * 0x2010 * coef * 20 + 0x3010);
    }
  */

    grabber.waitForNextFrame();
    grabber.copyImageTo(current_frame);
    processor.processImage(current_frame);

    // Show the frames per second of the grabber
    int fps = grabber.frameRate();
    cv::putText(current_frame.rgbRef(),
        cv::format("%d fps", fps),
        Point(10,20), 0, 0.5, Scalar(255,0,0,255));

    Mat depth = current_frame.depth();

/*
    Mat slice = depth.col(150);
    Mat x, acc;
    compare(slice, 0.4, x, CMP_LT);
    compare(prev_slice, 0.4, acc, CMP_LT);
    bitwise_or(x, acc, acc);
    compare(prev_slice, 2, x, CMP_GT);
    bitwise_or(x, acc, acc);
    compare(slice, 2, x, CMP_GT);
    bitwise_or(x, acc, acc);

    Mat diff = slice - prev_slice;
//    diff.setTo(0, acc);
//    prev_slice = slice;
    slice.copyTo(prev_slice);

    extractSpectrum(diff, spectrum,
            0, 1, 479,            // line
            10, ssize,            // frequency
            -0.2, 0.2,             // distance
            -50, 50,              // amplitude
            true);

    compare(spectrum, 0, x, CMP_LT);
    spectrum.setTo(0, x);
 */
   
    extractSpectrum(depth, spectrum,
            150, 1, 479,            // line
            10, ssize,              // frequency
            0.4, 2,                 // distance
            0, 50,                  // amplitude
            true);
 
    // Compute color encoded depth.
    cv::Mat3b dac;
    compute_color_encoded_depth(current_frame.depth(), dac);

    drawSpectrum(dac, spectrum, 150, 1, 479);

    Mat result;
    dft(spectrum, result, DFT_INVERSE);

    float prev = 0;
    for (int i = 0; i < sound_width; i++) {
        float coef = (float)i / (sound_width - 1) / 2;
        float prev_val = prev_result.at<float>(0, i + sound_width);
        float cur_val  = result.at<float>(0, i);
        prev_val *= 0.46 - 0.46 * cos (3.14159 * (coef - 0.5));
        cur_val  *= 0.46 - 0.46 * cos (3.14159 * coef);
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
    spectrum *= 0.9;

/*
    for (int i = 0; i < sound_width; i++) {
        psound.at<short>(0,i) = sin()
    }
 */

    result.copyTo(prev_result);

    // Display the color image
//    imshow("color", current_frame.rgb());

    // Show the depth image as normalized gray scale
//    imshow_normalized("depth", current_frame.depth());


    imshow("dac", dac);

    // Enable switching to InfraRead mode.
    unsigned char c = cv::waitKey(10) & 0xff;
    if (c == 'q')
      exit(0);
  }

  libao_finish();

  return 0;
}
