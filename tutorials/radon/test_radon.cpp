#include <iostream>
#include "Radon.h"

using namespace framework;

int main(int argc, char *argv[])
{
    cv::Mat tmp[6];
    cv::Mat r[6], r_candidate;
    cv::Mat r_sig[6];
    cv::Mat candidate, r_sig_c;

    tmp[0] = cv::imread("1.jpg");
    tmp[1] = cv::imread("2.jpg");
    tmp[2] = cv::imread("3.jpg");
    tmp[3] = cv::imread("4.jpg");
    tmp[4] = cv::imread("5.jpg");
    tmp[5] = cv::imread("6.jpg");
    candidate = cv::imread("2.jpg");

//    double step = 1, range = 180;
    Radon::Instance()->initRotateMatrixLookupTable(tmp[0].cols, tmp[0].rows);

    for (int i = 0; i < 6; i++) {
        Radon::Instance()->computeRadon(r[i], tmp[i]);

        r_sig[i] = cv::Mat::zeros(1, 180, CV_32FC1);
        Radon::Instance()->computeRSignature(r_sig[i], r[i]);
//        std::cout << r_sig[i] << std::endl;
    }
    Radon::Instance()->computeRadon(r_candidate, candidate);
    r_sig_c = cv::Mat::zeros(1, 180, CV_32FC1);
    Radon::Instance()->computeRSignature(r_sig_c, r_candidate);

    cv::imshow("r1", r[0]);
    cv::imshow("r2", r[1]);
    cv::imshow("r3", r[2]);
    cv::imshow("r4", r[3]);
    cv::imshow("r5", r[4]);
    cv::imshow("r6", r[5]);

    double distance = 0.0;
    for (int i = 0; i < 6; i++) {
        Radon::Instance()->computeDistance(distance, r_sig[i], r_sig_c);
        std::cout << distance << " ";
    }
    std::cout << std::endl;
    cv::waitKey(0);
    return 0;
}
