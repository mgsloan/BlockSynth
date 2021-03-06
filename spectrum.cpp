#include "audio.h"

void extractSpectrum(Mat img, Mat spect, // depth input, spectrum output
  int x, int fy, int ty,    // line in camera space, maps to
  int ff, int tf,           // interval in frequency.
  unsigned fd, unsigned td, // interval in distance, maps to
  float fa, float ta,       // interval in amplitude.
  bool logScale) {

    float deltaF = logScale ? log ((float) tf / ff)
                            : tf - ff;
    float fratio = deltaF / (ty - fy),
          aratio = (ta - fa) / (td - fd);
    for (int y = fy; y <= ty; y++) {
        // calculate amplitude of value.
        unsigned val = img.at<unsigned>(y, x);
        if (val > td || val < fd) continue;
        val = max(fd, val);
        float amp = (td - val) * aratio + fa;

        // calculate frequency / increment appropriate index in spectrum.
        float freq = (y - fy) * fratio;
        freq = logScale ? exp(freq + log(ff))
                        : freq + ff;      
        //printf("%f\n", freq);
        spect.at<float>(0, (int)freq) += amp;
    }
}

int ssize = 3000, dsize = 3000;
int decay = 90, fscale = 50;

void drawSpectrum(Mat img, Mat spect, int x, int fy, int ty) {
    int size = spect.size().width;
    float ratio = (float)(ty - fy) / size;
    for (int i = 0; i < size; i++) {
       float a = spect.at<float>(0,i); //* fscale / 100;
       int y = fy + (int)((float)i * ratio);
       line(img, Point(x, y), Point(x + a, y), Scalar(255,0,0));
    }
}

int main(int argc, char **argv)
{
    int res = freenect_start();
    if (res != 0) return res;
    res = libao_start();
    if (res != 0) return res;
    
    vector<float> stringPos(3000);
    vector<float> stringVel(3000);
    vector<float> tempPos(3000);
    vector<float> tempVel(3000);
    for (int i = 0; i < 1501; i++)
    {
        stringPos[i] = i/250;
        stringPos[3000-i] = i/250;
        stringVel[i] = 0;
        stringVel[3000-i] = 0;
    }
    Mat spectrum(1, ssize, CV_32F);
    for (int i = 0; i < ssize; i++) spectrum.at<float>(i) = 0;
    Mat spectrum2(1, ssize, CV_32F);
    for (int i = 0; i < ssize; i++) spectrum2.at<float>(i) = 0;
    while (!die) {
        cvCreateTrackbar("dsize", "rgb", &dsize, 7000, NULL);
        cvCreateTrackbar("decay", "rgb", &decay, 100, NULL);
        cvCreateTrackbar("fscale", "rgb", &fscale, 100, NULL);

        Mat rgb = rgbMat.clone(), depth = depthMat.clone();
        Mat depthf;
        depth.convertTo(depthf, CV_8U, 256.0 / 2048.0);

        int histSize[] = {256};
        float hranges[] = { 0, 256.0 };
        const float* ranges[] = { hranges };
        MatND hist;
        int channels[] = {0};

        // Calculate depth histogram in order to make assumptions regarding
        // background clip depth.
        calcHist(&depthf, 1, channels, Mat(), // do not use mask
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
        //tempstring[1200][1] = 100;
        tempPos[0] = 0;
        tempVel[0] = 0;
        tempVel[2999] = 0;
        tempPos[2999] = 0;
        for (int i = 1; i < 2999; i++)
        {
            tempPos[i] = stringPos[i] + stringVel[i];
            tempVel[i] = stringVel[i] + (stringPos[i-1] + stringPos[i + 1] - 2*stringPos[i]) / 8;
        }
        
        std::swap(tempPos, stringPos);
        std::swap(tempVel, stringVel);     
//        printf("%f\n", tempstring[40][0]);
        
        for (int i = 0; i < 299; i++)
        {
            line(rgb, Point(i, 130 + stringPos[i*10]), Point(i+1, 130 + stringPos[(i+1)*10]), Scalar(0, 255, 255));
 	    } 
        printf("%i\n", stringPos[1200]);
        int xpos = 200;

        for(int y = 30; y < 40; y++) 
            depthf.at<char>(y, xpos) = 0;

        extractSpectrum(depth, spectrum,
                150, 1, 479,        // line
                1, ssize,            // frequency
                24379763, 60000000,  // distance
                0, 50,              // amplitude
                true);

        drawSpectrum(rgb, spectrum, xpos + 100, 1, 479);

        extractSpectrum(depth, spectrum2,
                150, 1, 479,        // line
                1, ssize,            // frequency
                24379763, 60000000,  // distance
                0, 50,              // amplitude
                true);

        drawSpectrum(rgb, spectrum2, xpos + 100, 1, 479);


        Mat result;
        dft(spectrum, result, DFT_INVERSE);

        int width = sound.size().width, clip = (dsize - width) / 2;
        float prev = 0;
        for (int i = 0; i < width; i++) {
            float val = result.at<float>(0, i + clip);
            line(rgb, Point(i-1, prev + 100), Point(i, val * .01 + 100), Scalar(0,255,0));
            sound.at<short>(0,i) = (short)(val + 16000) * fscale / 100;
            prev = val * 0.01;
        }

        display("rgb", rgb);
        //display("depth", depth);

        spectrum *= (float)(decay / 100.0);
        spectrum2 *= (float)(decay / 100.0);

        char k = cvWaitKey(5);
        if( k == 27 ) break;
    }
     
    freenect_finish();
    libao_finish();

    pthread_exit(NULL);
}
