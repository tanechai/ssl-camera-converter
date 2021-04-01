#include "ExtractDifferences.hpp"

ExtractDifferences::ExtractDifferences(cv::Mat& standardImg_):standardImg(standardImg_){
    cv::cvtColor(standardImg,grayStandardImg,cv::COLOR_BGR2GRAY);
};

cv::Mat ExtractDifferences::getDifferences(const cv::Mat& img){
    cv::Mat grayImg;
    cv::cvtColor(img,grayImg,cv::COLOR_BGR2GRAY);
    
    cv::Mat diffImg;
    cv::absdiff(grayStandardImg,grayImg,diffImg);

    cv::Mat mask;
    cv::threshold(diffImg,mask,10,255,cv::THRESH_BINARY);

    cv::Mat maskedImg;
    cv::bitwise_and(img,img,maskedImg,mask);

    cv::Mat resultImg;
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(5,5));
    cv::morphologyEx(maskedImg,resultImg,cv::MORPH_OPEN,element);

    return maskedImg;
};