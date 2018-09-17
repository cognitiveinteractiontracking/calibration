/**
Copyright 2017 Rafael Muñoz Salinas. All rights reserved.

Redistribution and use in source and binary forms, with or without modification, are
permitted provided that the following conditions are met:

   1. Redistributions of source code must retain the above copyright notice, this list of
      conditions and the following disclaimer.

   2. Redistributions in binary form must reproduce the above copyright notice, this list
      of conditions and the following disclaimer in the documentation and/or other materials
      provided with the distribution.

THIS SOFTWARE IS PROVIDED BY Rafael Muñoz Salinas ''AS IS'' AND ANY EXPRESS OR IMPLIED
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL Rafael Muñoz Salinas OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those of the
authors and should not be interpreted as representing official policies, either expressed
or implied, of Rafael Muñoz Salinas.
*/

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include "aruco/aruco.h"
#include "aruco/cvdrawingutils.h"
#include <fstream>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <sstream>
#include <string>
#include <stdexcept>

using namespace std;
using namespace cv;
using namespace aruco;

MarkerDetector MDetector;
VideoCapture TheVideoCapturer;
vector<Marker> TheMarkers;
Mat TheInputImage, TheInputImageGrey, TheInputImageCopy;
CameraParameters TheCameraParameters;
void cvTackBarEvents(int pos, void *);
string dictionaryString;
int iDetectMode = 0, iMinMarkerSize = 0, iCorrectionRate = 0, iShowAllCandidates = 0, iEnclosed = 0, iThreshold, iCornerMode, iDictionaryIndex = 0;

int waitTime = 0;
bool showMennu = false, bPrintHelp = false, isVideo = false;

//parameters
string camera_parameter;
string config_file_save;
float TheMarkerSize;
float resizeFactor;
string topicInImage;
string dictionary_string;


class CmdLineParser {
    int argc;
    char **argv;
public:
    CmdLineParser(int _argc, char **_argv) : argc(_argc), argv(_argv) {}

    bool operator[](string param) {
      int idx = -1;
      for (int i = 0; i < argc && idx == -1; i++)if (string(argv[i]) == param)idx = i;
      return (idx != -1);
    }

    string operator()(string param, string defvalue = "-1") {
      int idx = -1;
      for (int i = 0; i < argc && idx == -1; i++)if (string(argv[i]) == param)idx = i;
      if (idx == -1)return defvalue; else return (argv[idx + 1]);
    }
};

struct TimerAvrg {
    std::vector<double> times;
    size_t curr = 0, n;
    std::chrono::high_resolution_clock::time_point begin, end;

    TimerAvrg(int _n = 30) {
      n = _n;
      times.reserve(n);
    }

    inline void start() { begin = std::chrono::high_resolution_clock::now(); }

    inline void stop() {
      end = std::chrono::high_resolution_clock::now();
      double duration = double(std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count()) * 1e-6;
      if (times.size() < n) times.push_back(duration);
      else {
        times[curr] = duration;
        curr++;
        if (curr >= times.size()) curr = 0;
      }
    }

    double getAvrg() {
      double sum = 0;
      for (auto t:times) sum += t;
      return sum / double(times.size());
    }
};

TimerAvrg Fps;

cv::Mat resize(const cv::Mat &in, int width) {
  if (in.size().width <= width)
    return in;
  float yf = float(width) / float(in.size().width);
  cv::Mat im2;
  cv::resize(in, im2, cv::Size(width, static_cast<int>(in.size().height * yf)));
  return im2;
}

/************************************
 *
 *
 *
 *
 ************************************/
void setParamsFromGlobalVariables(aruco::MarkerDetector &md) {


  md.setDetectionMode((DetectionMode) iDetectMode, float(iMinMarkerSize) / 1000.);
  md.getParameters().setCornerRefinementMethod((aruco::CornerRefinementMethod) iCornerMode);

  md.getParameters().detectEnclosedMarkers(iEnclosed);
  md.getParameters().ThresHold = iThreshold;
  md.setDictionary((aruco::Dictionary::DICT_TYPES) iDictionaryIndex, float(iCorrectionRate) / 10.);  // sets the dictionary to be employed (ARUCO,APRILTAGS,ARTOOLKIT,etc)
}

void createMenu() {
  cv::createTrackbar("Dictionary", "menu", &iDictionaryIndex, 13, cvTackBarEvents);
  cv::createTrackbar("DetectMode", "menu", &iDetectMode, 2, cvTackBarEvents);
  cv::createTrackbar("CornerMode", "menu", &iCornerMode, 2, cvTackBarEvents);

  cv::createTrackbar("MinMarkerSize", "menu", &iMinMarkerSize, 1000, cvTackBarEvents);
  cv::createTrackbar("Threshold", "menu", &iThreshold, 40, cvTackBarEvents);
  cv::createTrackbar("ErrorRate", "menu", &iCorrectionRate, 10, cvTackBarEvents);
  cv::createTrackbar("Enclosed", "menu", &iEnclosed, 1, cvTackBarEvents);
  cv::createTrackbar("ShowAll", "menu", &iShowAllCandidates, 1, cvTackBarEvents);
  iThreshold = MDetector.getParameters().ThresHold;
  iCornerMode = MDetector.getParameters().cornerRefinementM;
}

void putText(cv::Mat &im, string text, cv::Point p, float size) {
  float fact = float(im.cols) / float(640);
  if (fact < 1) fact = 1;

  cv::putText(im, text, p, FONT_HERSHEY_SIMPLEX, size, cv::Scalar(0, 0, 0), 3 * fact);
  cv::putText(im, text, p, FONT_HERSHEY_SIMPLEX, size, cv::Scalar(125, 255, 255), 1 * fact);

}

void printHelp(cv::Mat &im) {
  float fs = float(im.cols) / float(1200);

  putText(im, "'m': show/hide menu", cv::Point(10, fs * 60), fs * 0.5f);
  putText(im, "'s': start/stop video capture", cv::Point(10, fs * 80), fs * 0.5f);
  putText(im, "'w': write image to file", cv::Point(10, fs * 100), fs * 0.5f);
  putText(im, "'t': do a speed test", cv::Point(10, fs * 120), fs * 0.5f);
  putText(im, "'f': saves current configuration to file 'arucoConfig.yml'", cv::Point(10, fs * 140), fs * 0.5f);
}

void printInfo(cv::Mat &im) {
  float fs = float(im.cols) / float(1200);
  putText(im, "fps=" + to_string(1. / Fps.getAvrg()), cv::Point(10, fs * 20), fs * 0.5f);
  putText(im, "'h': show/hide help", cv::Point(10, fs * 40), fs * 0.5f);
  if (bPrintHelp) printHelp(im);
}

void printMenuInfo() {
  cv::Mat image(200, 400, CV_8UC3);
  image = cv::Scalar::all(255);
  string str = "Dictionary=" + aruco::Dictionary::getTypeString((aruco::Dictionary::DICT_TYPES) iDictionaryIndex);

  cv::putText(image, str, cv::Size(10, 20), FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0, 0, 0), 1);

  str = "Detection Mode=" + MarkerDetector::Params::toString(MDetector.getParameters().detectMode);
  cv::putText(image, str, cv::Size(10, 40), FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0, 0, 0), 1);
  str = "Corner Mode=" + MarkerDetector::Params::toString(MDetector.getParameters().cornerRefinementM);;
  cv::putText(image, str, cv::Size(10, 60), FONT_HERSHEY_SIMPLEX, 0.35, cv::Scalar(0, 0, 0), 1);
  cv::imshow("menu", image);
}

cv::Mat resizeImage(cv::Mat &in, float resizeFactor) {
  if (fabs(1 - resizeFactor) < 1e-3)return in;
  float nc = float(in.cols) * resizeFactor;
  float nr = float(in.rows) * resizeFactor;
  cv::Mat imres;
  cv::resize(in, imres, cv::Size(nc, nr));
  cout << "Imagesize=" << imres.size() << endl;
  return imres;
}

/************************************
 *
 *
 *
 *
 ************************************/

void mainLoop() {
  static char key = 0;
  static int index = 0, indexSave = 0;
  TheInputImage = resizeImage(TheInputImage, resizeFactor);
  // copy image
  Fps.start();
  TheMarkers = MDetector.detect(TheInputImage, TheCameraParameters, TheMarkerSize);
  Fps.stop();
  // chekc the speed by calculating the mean speed of all iterations
  cout << "\rTime detection=" << Fps.getAvrg() * 1000 << " milliseconds nmarkers=" << TheMarkers.size() << std::endl;

  // print marker info and draw the markers in image
  TheInputImage.copyTo(TheInputImageCopy);

  if (iShowAllCandidates) {
    auto candidates = MDetector.getCandidates();
    for (auto cand:candidates)
      Marker(cand, -1).draw(TheInputImageCopy, Scalar(255, 0, 255));
  }

  for (unsigned int i = 0; i < TheMarkers.size(); i++) {
    cout << TheMarkers[i] << endl;
    TheMarkers[i].draw(TheInputImageCopy, Scalar(0, 0, 255), 2, true);
  }

  // draw a 3d cube in each marker if there is 3d info
  if (TheCameraParameters.isValid() && TheMarkerSize > 0)
    for (unsigned int i = 0; i < TheMarkers.size(); i++) {
      CvDrawingUtils::draw3dCube(TheInputImageCopy, TheMarkers[i], TheCameraParameters);
      CvDrawingUtils::draw3dAxis(TheInputImageCopy, TheMarkers[i], TheCameraParameters);
    }

  // DONE! Easy, right?
  // show input with augmented information and  the thresholded image
  printInfo(TheInputImageCopy);
  if (showMennu)printMenuInfo();
  cv::imshow("thres", resize(MDetector.getThresholdedImage(), 1024));
  cv::imshow("in", resize(TheInputImageCopy, 1024));

  key = cv::waitKey(waitTime);  // wait for key to be pressed
  if (key == 's')
    waitTime = waitTime == 0 ? 10 : 0;
/*  if (key == 'w') {//writes current input image
    string number = std::to_string(indexSave++);
    while (number.size() != 3)number = "0" + number;
    string imname = "arucoimage" + number + ".png";
    cv::imwrite(imname, TheInputImageCopy);
    cout << "saved " << imname << endl;
    imname = "orgimage" + number + ".png";
    cv::imwrite(imname, TheInputImage);
    cout << "saved " << imname << endl;
    imname = "thresimage" + number + ".png";
    cv::imwrite(imname, MDetector.getThresholdedImage());
  }*/
  if (key == 'm') {
    if (showMennu) cv::destroyWindow("menu");
    else {
      cv::namedWindow("menu", cv::WINDOW_NORMAL);
      cv::resizeWindow("menu", 640, 480);
      createMenu();
      printMenuInfo();
    }
    showMennu = !showMennu;
  }
  if (key == 'h')bPrintHelp = !bPrintHelp;

  if (key == 't') {//run a deeper speed test

    for (int t = 0; t < 30; t++) {
      // Detection of markers in the image passed
      Fps.start();
      TheMarkers = MDetector.detect(TheInputImage, TheCameraParameters, TheMarkerSize);
      Fps.stop();
      // chekc the speed by calculating the mean speed of all iterations
    }
    printInfo(TheInputImageCopy);
  }
  if (key == 'f') {
    if (config_file_save.empty())
      ROS_WARN_STREAM("Cannot save config. config_file_save is empty.");
    else {
      ROS_WARN_STREAM("Configuration saved to:" << config_file_save);
      MDetector.saveParamsToFile(config_file_save);
    }
  }
  index++;  // number of images captured

  if (isVideo)
    if (TheVideoCapturer.grab() == false) key = 27;
}

void callbackImage(sensor_msgs::ImageConstPtr msg) {
  if (msg->encoding == "mono8") {
    cv::cvtColor(cv_bridge::toCvShare(msg, msg->encoding)->image, TheInputImage, cv::COLOR_GRAY2BGR);
  } else if (msg->encoding == "rgb8") {
    TheInputImage = cv_bridge::toCvShare(msg, msg->encoding)->image;
  } else {
    ROS_WARN("[%s] Unknown Image encoding %s.", ros::this_node::getName().c_str(), msg->encoding.c_str());
    return;
  }
  // Track
  mainLoop();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, ros::this_node::getName());
  ros::NodeHandle n("~");

  n.param<std::string>("camera_parameter", camera_parameter, "");
  n.param<std::string>("config_file_save", config_file_save, "");
  n.param<std::string>("dictionary", dictionary_string, "ARUCO");
  n.param<float>("marker_size", TheMarkerSize, 100);
  n.param<float>("resize_factor", resizeFactor, 1);
  n.param<std::string>("topic_in_image", topicInImage, "/genicam/cam4"); // Video parameter for the camera

  ros::Subscriber imageSub = n.subscribe(topicInImage, 1, callbackImage);

  if (!camera_parameter.empty()) TheCameraParameters.readFromXMLFile(camera_parameter);

  if (TheCameraParameters.isValid())
    TheCameraParameters.resize(TheInputImage.size());
  iDictionaryIndex = (uint64_t) aruco::Dictionary::getTypeFromString(dictionary_string);
  MDetector.setDictionary(dictionary_string, float(iCorrectionRate) / 10.);  // sets the dictionary to be employed (ARUCO,APRILTAGS,ARTOOLKIT,etc)
  iThreshold = MDetector.getParameters().ThresHold;
  iCornerMode = MDetector.getParameters().cornerRefinementM;

  setParamsFromGlobalVariables(MDetector);

  ros::spin();

  return 0;
}

void cvTackBarEvents(int pos, void *) {
  (void) (pos);


  setParamsFromGlobalVariables(MDetector);

  // recompute
  Fps.start();
  MDetector.detect(TheInputImage, TheMarkers, TheCameraParameters);
  Fps.stop();
  // chekc the speed by calculating the mean speed of all iterations
  TheInputImage.copyTo(TheInputImageCopy);
  if (iShowAllCandidates) {
    auto candidates = MDetector.getCandidates();
    for (auto cand:candidates)
      Marker(cand, -1).draw(TheInputImageCopy, Scalar(255, 0, 255), 1);
  }

  for (unsigned int i = 0; i < TheMarkers.size(); i++) {
    cout << TheMarkers[i] << endl;
    TheMarkers[i].draw(TheInputImageCopy, Scalar(0, 0, 255), 2);
  }

  // draw a 3d cube in each marker if there is 3d info
  if (TheCameraParameters.isValid())
    for (unsigned int i = 0; i < TheMarkers.size(); i++)
      CvDrawingUtils::draw3dCube(TheInputImageCopy, TheMarkers[i], TheCameraParameters);
  cv::putText(TheInputImageCopy, "fps=" + to_string(1. / Fps.getAvrg()), cv::Point(10, 20), FONT_HERSHEY_SIMPLEX, 0.5f, cv::Scalar(125, 255, 255), 2, CV_AA);

  cv::imshow("in", TheInputImageCopy);
  cv::imshow("thres", resize(MDetector.getThresholdedImage(), 1024));
  if (showMennu)printMenuInfo();

}