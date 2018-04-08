int main()
{
	//read video
	VideoCapture cap(0);
	
	int iLowH = 70;
	int iHighH = 100;
	int iLowS = 200; 
	int iHighS = 255;
	int iLowV = 20;
	int iHighV = 130;
	int iLowArea = 10000;
	createTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
	createTrackbar("HighH", "Control", &iHighH, 179);
	createTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
	createTrackbar("HighS", "Control", &iHighS, 255);
	createTrackbar("LowV", "Control", &iLowV, 255);//Value (0 - 255)
	createTrackbar("HighV", "Control", &iHighV, 255);
	createTrackbar("LowArea", "Control", &iLowArea, 76800);

	
	while(true){
		Mat imgOriginal;
		bool success = cap.read(imgOriginal);
		
		Mat imgHSV;
		cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);
		
		Mat imgThreshold;
		
		inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
		dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
		erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );

		imshow("Thresholded Image", imgThresholded); //show the thresholded image
		imshow("Original", imgOriginal); //show the original image

        if (waitKey(30) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
		{
            break; 
		}
		
		Moments oMoments = moments(imgThresholded);
		
		double dM01 = oMoments.m01;
		double dM10 = oMoments.m10;
		double dArea = oMoments.m00;
		
		if(dArea < iLowArea) {
			cout << "Not obstacle" << endl;
		}
		else {
			int posX = dM10/dArea;
			int posY = dM01/dArea;
			
			cout << "Obstacle: " << posX << ", " << posY << endl;
		}
	}
	
	return 0;
}