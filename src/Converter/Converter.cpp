#include "Converter.hpp"
#include "../ExtractDifferences/ExtractDifferences.hpp"

void overlayImage(cv::Mat* src, cv::Mat* overlay, const cv::Point& location);

Converter::Converter(Initializer& initializer):initializer_(initializer){}

cv::Mat Converter::imagesToImage(const std::vector<cv::Mat>& images, const cv::Size2d& fieldImageSize){
    // 合成後の画像
    cv::Mat mixedImage;

    std::vector<model::Camera> cameraParameters = initializer_.parameters();

    for (int i = 0; i < images.size(); i++) {
        
        cv::Mat undistortedImage = cv::Mat::zeros(fieldImageSize.height, fieldImageSize.width, CV_8UC3);
        cv::Mat warpedImage = cv::Mat::zeros(fieldImageSize.height, fieldImageSize.width, CV_8UC3);
        
        ExtractDifferences ext(initializer_.defaultFields().at(i));
        cv::Mat leanImage = ext.getDifferences(images.at(i));
        
        cv::undistort(leanImage, undistortedImage, cameraParameters.at(i).cameraMatrix(), cameraParameters.at(i).distCoeffs(), cameraParameters.at(i).cameraMatrix());

        cv::warpPerspective(undistortedImage, warpedImage, initializer_.homographyMatrixs().at(i), warpedImage.size());

        if(mixedImage.empty()){
            mixedImage = warpedImage;
        }else{
            // 合成割合を決める定数を決める
            double alpha = 1 / static_cast<double>(i+1);
            double beta = 1 - alpha;
            // 合成元をコピー
            cv::Mat tmp = mixedImage;
            cv::addWeighted(warpedImage, alpha, tmp, beta, 0,mixedImage);
        }
    }
    //cv::Mat affineMat = (cv::Mat_<double>(2,3)<<1,0,0,0,1,0);
    //cv::warpAffine(initializer_.field(), mixedImage, affineMat, mixedImage.size(), cv::INTER_LINEAR, cv::BORDER_TRANSPARENT);
    cv::Mat field = initializer_.field();
    cv::Mat tmp = mixedImage;
    cv::addWeighted(field, 0.3, tmp, 0.7, 0,mixedImage);

    return mixedImage;
}

void overlayImage(cv::Mat* src, cv::Mat* overlay, const cv::Point& location) {
    for (int y = std::max(location.y, 0); y < src->rows; ++y) {
        int fY = y - location.y;
        if (fY >= overlay->rows)
            break;
        for (int x = std::max(location.x, 0); x < src->cols; ++x) {
            int fX = x - location.x;
            if (fX >= overlay->cols)
                break;
            double opacity = ((double)overlay->data[fY * overlay->step + fX * overlay->channels() + 3])/255;
            for (int c = 0; opacity > 0 && c < src->channels(); ++c) {
                unsigned char overlayPx = overlay->data[fY * overlay->step + fX * overlay->channels() + c];
                unsigned char srcPx = src->data[y * src->step + x * src->channels() + c];
                src->data[y * src->step + src->channels() * x + c] = srcPx * (1. - opacity) + overlayPx * opacity;
            }
        }
    }
}