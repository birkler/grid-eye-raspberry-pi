#include "amg88xx-i2c.h"


#include <opencv2/highgui/highgui.hpp>

#include <opencv2/imgproc/imgproc.hpp>

cv::Vec4b getColorSubpix(const cv::Mat& img,cv::Point2f val)
{
    cv::Mat patch;
    cv::getRectSubPix(img, cv::Size(1,1), val, patch);
    return patch.at<cv::Vec4b>(0,0);
}

int clamp(int val, int minval, int maxval) {
    return std::min(maxval,std::max(minval,val));

}
static void colorize(float minval,float maxval, const cv::Mat& in_, const cv::Mat& palette_,cv::Mat& res) {
    res.create(in_.size(),CV_8UC4);

    float palette_scale = float(palette_.cols) / ( maxval-minval);

    for (int r=0;r<res.rows;r++) {
    for (int c=0;c<res.cols;c++) {
        float val = in_.at<float>(r,c);
        val = (val - minval) * palette_scale;

        int val_0 = floor(val);
        int val_1 = floor(val+1.0);
        float w = val-val_0; 

        val_0 = clamp(val_0,0,palette_.cols);
        val_1 = clamp(val_1,0,palette_.cols);

        cv::Vec4f rgba0 = palette_.at<cv::Vec4f>(val_0);
        cv::Vec4f rgba1 = palette_.at<cv::Vec4f>(val_1);


        cv::Vec4f rgba = rgba0*(1.0 - w) + rgba1*w;
        res.at<cv::Vec4b>(r,c) = rgba*255.0;
    }
    }
}

int main(int argc, char* argv[]) {
    bool haveRGBCamera = false;
    bool haveGridEye = false;
    
    cv::VideoCapture cap(0); // open the default camera
    if(cap.isOpened())  {
        haveRGBCamera = true;
    } // check if we succeeded


    std::string windowName = "Thermal";
    cv::namedWindow(windowName,cv::WINDOW_NORMAL); // not required

 
    cv::Vec4f palette_jet_colors[] = {
        cv::Vec4f(0.6,0.0,0.0,1.0),
        cv::Vec4f(1.0,0.0,0.0,1.0),
        cv::Vec4f(0.7,0.7,0.0,1.0),
        cv::Vec4f(0.0,0.9,0.0,1.0),
        cv::Vec4f(0.0,0.7,0.7,1.0),
        cv::Vec4f(0.0,0.0,0.9,1.0),
        //cv::Vec4f(.8,0.3,0.3,1.0),
        cv::Vec4f(.7,0.7,0.7,1.0)
    };

    cv::Mat palette_jet(1,sizeof(palette_jet_colors) / sizeof(palette_jet_colors[0]),CV_32FC4,palette_jet_colors);


    cv::Mat palette_show;

    cv::resize(palette_jet,palette_show,cv::Size(200,200));


    cv::imshow(windowName,palette_show);

    cv::waitKey(-1);

    Adafruit_AMG88xx grideye;
    try {
        grideye.init();
        haveGridEye = true;
    } catch (...) {

    } 

    cv::Mat gridEyePixels(8,8,CV_32F);
    gridEyePixels *= 0.0;
    cv::Mat colorized;

    float minval = 15.0;

    float maxval = 40.0;

    cv::Mat colorized_resized;

    cv::Mat cameraFrame;
    cv::Mat cameraFrameGray;

    cv::Mat cameraFrameBlurred;
    cv::Size mergedViewSize = gridEyePixels.size()*32;


	


    do {
        if (haveRGBCamera) {
            bool newFrame = cap.read(cameraFrame);

            cv::imshow("rgb",cameraFrame);

            cv::cvtColor(cameraFrame,cameraFrameGray,CV_BGR2GRAY);

            cv::GaussianBlur(cameraFrameGray,cameraFrameBlurred,cv::Size(0,0),4.0,4.0);

            cv::Mat diff;
            cv::addWeighted(cameraFrameGray,2.0,cameraFrameBlurred,-2.0,+0.5 * 255.0,diff);

            cv::imshow("rgb-diff",diff);

        }

        if (haveGridEye) {
            grideye.readPixels(&gridEyePixels.at<float>(0,0),gridEyePixels.size().area());
        }

        colorize(minval,maxval,gridEyePixels,palette_jet,colorized);

        cv::resize(colorized,colorized_resized,mergedViewSize,0.0,0.0,cv::INTER_CUBIC);


        cv::imshow(windowName,colorized_resized);

        int ch=cv::waitKey(5);

        if (ch >= 0) break;
    } while((true));

    cap.release();
}

