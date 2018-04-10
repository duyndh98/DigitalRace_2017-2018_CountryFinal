#include "lane_detection.h"

/////// My function
Mat filterLane(const Mat &imgLane, bool &pop, Point &point, int check, bool &preState)
{
    pop = false;
    //point.x = 0;
    //point.y = 0;
	if (check==-1){
		point.x = 0;
		point.y = imgLane.rows/2;
	} else {
		point.x = imgLane.cols;
		point.y = imgLane.rows/2;	
	}
    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    findContours(imgLane, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, Point(0, 0));
    if (contours.size() == 0)
    {
        Mat none = Mat::zeros(imgLane.size(), CV_8UC1);
        return none;
    }
    Mat result = Mat::zeros(imgLane.size(), CV_8UC1);
    int sumX = 0;
    int sumY = 0;
    int maxArea = 0;
    int maxIndex = 0;
    for (int i = 0; i < (int)contours.size(); ++i)
    {
        int s = contourArea(contours[i]);
        if (s > maxArea)
        {
            maxArea = s;
            maxIndex = i;
        }
    }
    if (contourArea(contours[maxIndex]) < 100)
    {
        Mat none = Mat::zeros(imgLane.size(), CV_8UC1);
        return none;
    }
    int xMin = 0, yMin = 1000, xMax = 0, yMax = -1;
    /*for (int i = 0; i < contours[maxIndex].size(); i++)
    {
        // if (contours[maxIndex][i].y > yMax)
        // {
        //     yMax = contours[maxIndex][i].y;
        //     xMax = contours[maxIndex][i].x;
        // }
        // if (contours[maxIndex][i].y < yMin)
        // {
        //     yMin = contours[maxIndex][i].y;
        //     xMin = contours[maxIndex][i].x;
        // }
        sumX += contours[maxIndex][i].x;
        sumY += contours[maxIndex][i].y;
    }
	*/
    //if((xMax>=xMin) && (check==1)){
    drawContours(result, contours, maxIndex, Scalar(255), CV_FILLED);
    //point.x = sumX / contours[maxIndex].size();
    //point.y = sumY / contours[maxIndex].size();
	if(check==-1){
		point.x = 0;
		for (int i = 0; i < contours[maxIndex].size(); i++)
    		{
			if(contours[maxIndex][i].y - imgLane.rows/2>-20 && contours[maxIndex][i].y - imgLane.rows/2<20){
				if (point.x < contours[maxIndex][i].x)
					point.x = contours[maxIndex][i].x;
			}
        	//sumX += contours[maxIndex][i].x;
        	//sumY += contours[maxIndex][i].y;
    		}	
	} else {
		point.x = imgLane.cols;
		for (int i = 0; i < contours[maxIndex].size(); i++)
    		{
			if(contours[maxIndex][i].y - imgLane.rows/2>-20 && contours[maxIndex][i].y - imgLane.rows/2<20){
				if (point.x > contours[maxIndex][i].x)
					point.x = contours[maxIndex][i].x;
			}
        	//sumX += contours[maxIndex][i].x;
        	//sumY += contours[maxIndex][i].y;
    		}	
	}
	point.y = imgLane.rows/2;

    if ((point.x > imgLane.cols / 2) && (check == -1) && (preState == false))
    {
        //cout << "sum X: " << sumX << endl;
        Mat none = Mat::zeros(imgLane.size(), CV_8UC1);
        return none;
    }
    if ((point.x < imgLane.cols / 2) && (check == 1) && (preState == false))
    {
        Mat none = Mat::zeros(imgLane.size(), CV_8UC1);
        return none;
    }

    pop = true;
    return result;
    // } else
    // if((xMax<=xMin) && (check==-1)){
    //     drawContours(result, contours, maxIndex, Scalar(255), CV_FILLED);
    //     point.x = sumX / contours[maxIndex].size();
    //     point.y = sumY / contours[maxIndex].size();
    //     pop = true;
    //     return result;
    // }
    // return result;
}