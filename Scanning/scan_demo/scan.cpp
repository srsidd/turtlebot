// from http://stackoverflow.com/questions/14132951/how-to-obtain-the-scale-and-rotation-angle-from-logpolar-transform


#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

int main()
{
    cv::Mat a = cv::imread("rect1.png", 0);
    cv::Mat b = cv::imread("rect2.png", 0);
    if (a.empty() || b.empty())
        return -1;

    cv::imshow("a", a);
    cv::imshow("b", b);

    cv::Mat pa = cv::Mat::zeros(a.size(), CV_8UC1);
    cv::Mat pb = cv::Mat::zeros(b.size(), CV_8UC1);
    IplImage ipl_a = a, ipl_pa = pa;
    IplImage ipl_b = b, ipl_pb = pb;
    cvLogPolar(&ipl_a, &ipl_pa, cvPoint2D32f(a.cols >> 1, a.rows >> 1), 40);
    cvLogPolar(&ipl_b, &ipl_pb, cvPoint2D32f(b.cols >> 1, b.rows >> 1), 40);

    cv::imshow("logpolar a", pa);
    cv::imshow("logpolar b", pb);

    cv::Mat pa_64f, pb_64f;
    pa.convertTo(pa_64f, CV_64F);
    pb.convertTo(pb_64f, CV_64F);

    cv::Point2d pt = cv::phaseCorrelate(pa_64f, pb_64f);

    std::cout << "Shift = " << pt 
              << "Rotation = " << cv::format("%.2f", pt.y*180/(a.cols >> 1)) 
              << std::endl;

    cv::waitKey(0);

    return 0;
}