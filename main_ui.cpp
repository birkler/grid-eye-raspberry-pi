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
    int cameraIndex = 0;
    if (argc > 0) {
        sscanf(argv[1],"%d",&cameraIndex);
    }
    
    cv::VideoCapture cap(cameraIndex); // open the default camera
    if(cap.isOpened())  {
        haveRGBCamera = true;
    } // check if we succeeded


    std::string windowName = "Thermal";
    cv::namedWindow(windowName,cv::WINDOW_NORMAL); // not required

 
    cv::Vec4f palette_jet_colors[] = {
        cv::Vec4f(0.7,0.0,0.1,1.0),
        cv::Vec4f(1.0,0.2,0.2,1.0),
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

    cv::Mat cameraFeedoverlay;

    cameraFeedoverlay.create(mergedViewSize,CV_8UC4);
	
    cv::Matx32f RGBinv;

    float rgbFocal = 600.0;
    float rgbPPX = 320.0;
    float rgbPPY = 240.0;
    
    RGBinv(0,0) = 1.0 / rgbFocal;
    RGBinv(1,1) = 1.0 / rgbFocal;
    RGBinv(0,1) = 0.0;
    RGBinv(1,0) = 0.0;
    RGBinv(0,2) = - rgbPPX * RGBinv(0,0);
    RGBinv(1,2) = - rgbPPY * RGBinv(0,0);

    float gridEyeFOV = 60.0;

    cv::Matx23f M = cv::Matx23f::zeros();



    do {
        if (haveGridEye) {
            grideye.readPixels(&gridEyePixels.at<float>(0,0),gridEyePixels.size().area());
        }
        colorize(minval,maxval,gridEyePixels,palette_jet,colorized);

        cv::resize(colorized,colorized_resized,mergedViewSize,0.0,0.0,cv::INTER_CUBIC);

        if (haveRGBCamera) {
            bool newFrame = cap.read(cameraFrame);

            cv::imshow("rgb",cameraFrame);

            cv::cvtColor(cameraFrame,cameraFrameGray,CV_BGR2GRAY);

            cv::GaussianBlur(cameraFrameGray,cameraFrameBlurred,cv::Size(0,0),4.0,4.0);

            cv::Mat diff;
            cv::addWeighted(cameraFrameGray,2.0,cameraFrameBlurred,-2.0,+0.5 * 255.0,diff);

            cv::imshow("rgb-diff",diff);

            if (M(0,0) < 0.001) {

                float rgbFocal = 300.0 * 640.0 / float(cameraFrameGray.cols);
                float gridEyeFOV = 60.0;
                float gridEyeFocal = float(colorized_resized.cols) * 0.5 / tan(gridEyeFOV * 0.5 * CV_PI / 180.0);

                cv::Point2f from[3];
                cv::Point2f to[3];
                //Principle point to principle point

                from[0].x = cameraFrameGray.cols / 2;
                from[0].y = cameraFrameGray.rows / 2;
                to[0].x = colorized_resized.cols / 2;
                to[0].y = colorized_resized.rows / 2;

                from[1] = from[0];
                from[2] = from[0];
                to[1] = to[0];
                to[2] = to[0];

                
                from[1].x += 1.0 * rgbFocal;
                to[1].x += 1.0 * gridEyeFocal;

                from[2].y += 1.0 * rgbFocal;
                to[2].y += 1.0 * gridEyeFocal;

                M = cv::getAffineTransform(from,to);

            }

            cv::Mat diffWarped;

            int warpFlags = cv::INTER_LINEAR| cv::WARP_FILL_OUTLIERS;

            cv::warpAffine(diff,diffWarped,M,colorized_resized.size(),warpFlags,cv::BORDER_CONSTANT,cv::Scalar::all(128));

            cv::cvtColor(diffWarped,diffWarped,CV_GRAY2RGBA);

            cv::Mat combined;

            cv::addWeighted(colorized_resized,1.0,diffWarped,0.5,-0.5 * 255.0,combined);


            cv::imshow("rgb+grideye",combined);

        }



        cv::imshow(windowName,colorized_resized);

        int ch=cv::waitKey(5);

        if (ch >= 0) break;
    } while((true));

    cap.release();
}

