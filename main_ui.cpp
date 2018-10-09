#include "amg88xx-i2c.h"


#include <opencv2/highgui/highgui.hpp>

#include <opencv2/imgproc/imgproc.hpp>

cv::Vec4b getColorSubpix(const cv::Mat& img,cv::Point2f val)
{
    cv::Mat patch;
    cv::getRectSubPix(img, cv::Size(1,1), val, patch);
    return patch.at<cv::Vec4b>(0,0);
}

static void colorize(float minval,float maxval, const cv::Mat& in_, const cv::Mat& palette_,cv::Mat& res) {
    res.create(in_.size(),CV_8UC4);

    float palette_scale = float(palette_.cols) / ( maxval-minval);

    for (int r=0;r<res.rows;r++) {
    for (int c=0;c<res.cols;c++) {
        float val = in_.at<float>(r,c);
        cv::Vec4f rgba = getColorSubpix(palette_,cv::Point2f(val,0.0));
        res.at<cv::Vec4b>(r,c) = rgba*255.0;
    }
    }
}

int main(int argc, char* argv[]) {
    std::string windowName = "Thermal";
    cv::namedWindow(windowName,cv::WINDOW_NORMAL); // not required

 
    cv::Vec4f palette_jet_colors[] = {
        cv::Vec4f(0.6,0.0,0.0,1.0),
        cv::Vec4f(1.0,0.0,0.0,1.0),
        cv::Vec4f(1.0,1.0,0.0,1.0),
        cv::Vec4f(0.0,1.0,0.0,1.0),
        cv::Vec4f(0.0,1.0,1.0,1.0),
        cv::Vec4f(0.0,0.0,1.0,1.0),
        //cv::Vec4f(1.0,0.5,0.5,1.0),
        cv::Vec4f(1.0,1.0,1.0,1.0)
    };

    cv::Mat palette_jet(1,sizeof(palette_jet_colors) / sizeof(palette_jet_colors[0]),CV_32FC4,palette_jet_colors);


    cv::Mat palette_show;

    cv::resize(palette_jet,palette_show,cv::Size(200,200));


    cv::imshow(windowName,palette_show);

    cv::waitKey(-1);

    Adafruit_AMG88xx grideye;
    grideye.init();

    float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
    cv::Mat gridEyePixels(8,8,CV_32F);
    cv::Mat colorized;

    float minval = 15.0;

    float maxval = 40.0;

    cv::Mat colorized_resized;

    do {
        grideye.readPixels(&gridEyePixels.at<float>(0,0),gridEyePixels.size().area());


        colorize(minval,maxval,gridEyePixels,palette_jet,colorized);

        cv::resize(colorized,colorized_resized,gridEyePixels.size()*16,0.0,0.0,cv::INTER_CUBIC);


        cv::imshow(windowName,colorized_resized);

        int ch=cv::waitKey(5);

        if (ch >= 0) break;
    } while((true));
}

