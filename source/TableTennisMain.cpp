#include <opencv2\opencv.hpp>
#include <vector>
#include <iostream>
#include <opencv2\highgui.hpp>
#include <fstream>
#include <sstream>

//#define findLeftAndRight
//#define tryCal
#define Calibration
//#define VideoRecord

using namespace std;
using namespace cv;



bool getCorner(Mat& rgbImage, Mat& grayImage, Size boardSize, vector<vector<Point2f>>& corners);
void guessCameraParam(Mat& intrinsic, Mat& distortion_coeff);
void calRealPoint(vector<vector<Point3f>>& obj, int boardwidth, int boardheight, int imgNumber, int squaresize);
void outputCameraParam(const Mat& leftIntrinsic, const Mat& rightIntrinsic, const Mat& leftDistortion_coeff, const Mat& rightDistortion_coeff);
void outRT(const Mat& R, const Mat& T);
int imageRightWidth = 640;                             
int imageRightHeight = 480;
int imageLeftWidth = 640;
int imageLeftHeight = 480;
const int boardWidth = 5;                           
const int boardHeight = 5;                              
const int boardCorner = boardWidth * boardHeight;      
const int frameNumberForcalibrate = 50;               
const int squareSize = 30;                           
const Size boardSize = Size(boardWidth, boardHeight);

Mat leftIntrinsic, rightIntrinsic;                   
Mat leftDistortion_coeff, rightDistortion_coeff;    
vector<Mat> leftRvecs, rightRvecs;                                     
vector<Mat> leftTvecs, rightTvecs;                                    
vector<vector<Point2f>> leftCorners, rightCorners;
vector<vector<Point3f>> objRealPoint;                  


int main() { 
	VideoCapture leftCapture(1);
	VideoCapture rightCapture(2);
	if (!leftCapture.isOpened()) {
		cout << "leftCapture is not open!!" << endl;
		return 1;
	}
	if (!rightCapture.isOpened()) {
		cout << "rightCapture is not open!!" << endl;
		return 1;
	}
	imageLeftWidth = leftCapture.get(CV_CAP_PROP_FRAME_WIDTH);
	imageLeftHeight = leftCapture.get(CV_CAP_PROP_FRAME_HEIGHT);
	imageRightWidth = rightCapture.get(CV_CAP_PROP_FRAME_WIDTH);
	imageRightHeight = rightCapture.get(CV_CAP_PROP_FRAME_HEIGHT);
	int imageLeftFPS = leftCapture.get(CV_CAP_PROP_FPS);
	int imageRightFps = rightCapture.get(CV_CAP_PROP_FPS);
	cout << "imageLeftWidth: " << imageLeftWidth << endl;
	cout << "imageLeftHeight: " << imageLeftHeight << endl;
	cout << "imageRightWidth: " << imageRightWidth << endl;
	cout << "imageRightHeight: " << imageRightHeight << endl;
	cout << "imageLeftFPS: " << imageLeftFPS << endl;
	cout << "imageRightFps: " << imageRightFps << endl;
	Mat leftOriImage, rightOriImage;
	Mat leftGrayImage, rightGrayImage;
	//namedWindow("leftImage", WINDOW_NORMAL);
	int goodRightFrame = 0;
	int goodLeftFrame = 0;

#ifdef findLeftAndRight
	while (1) {
		leftCapture.grab();
		rightCapture.grab();
		leftCapture.retrieve(leftOriImage);
		rightCapture.retrieve(rightOriImage);
		imshow("leftOriImage", leftOriImage);
		imshow("rightOriImage", rightOriImage);
		waitKey(100);
	}
#endif

#ifdef tryCal
	int frame = 0;
	while (frame < frameNumberForcalibrate) {
		leftCapture.grab();
		leftCapture.retrieve(leftOriImage);
		Mat copyCor = leftOriImage.clone();
		imshow("leftImage", leftOriImage);
		cvtColor(leftOriImage, leftGrayImage, CV_BGR2GRAY);
		bool find = getCorner(leftOriImage, leftGrayImage, boardSize, leftCorners);
		if (find) { frame++; }
		waitKey(100);
	}
	cout << "leftCorners: " << leftCorners.size() << endl;
	guessCameraParam(leftIntrinsic, leftDistortion_coeff);
	cout << "guess successful" << endl;
	calRealPoint(objRealPoint, boardWidth, boardHeight, frameNumberForcalibrate, squareSize);
	cout << "111" << endl;
	calibrateCamera(objRealPoint, leftCorners, Size(imageLeftWidth, imageLeftHeight), leftIntrinsic, leftDistortion_coeff, leftRvecs, leftTvecs, 0);
	cout << "222" << endl;
	int aa;
	cin >> aa;

#endif 

#ifdef VideoRecord
	VideoWriter writeLeftVideo;
	VideoWriter writeRightVideo;
	string outLeftVideo = "Left30.avi";
	string outRightVideo = "Right30.avi";
	writeLeftVideo.open(outLeftVideo, -1, 30, Size(imageLeftWidth, imageLeftHeight), true);
	writeRightVideo.open(outRightVideo, -1, 30, Size(imageRightWidth, imageRightHeight), true);
	bool stop = false;
	while (!stop)
	{
		
		if (!leftCapture.read(leftOriImage) || !rightCapture.read(rightOriImage)) {
			cout << "Read Video False" << endl;
			return 1;
		}
			
		imshow("VideoLeft", leftOriImage);
		imshow("VideoRight", rightOriImage);
		
		writeLeftVideo.write(leftOriImage);
		writeRightVideo.write(rightOriImage);
		if (waitKey(10) > 0)
		{
			cout << "Stop Video Record!" << endl;
			stop = true;
		}
	}	
#endif

#ifdef Calibration
	while (goodLeftFrame < frameNumberForcalibrate || goodRightFrame <frameNumberForcalibrate) {
		leftCapture.grab();
		rightCapture.grab();
		leftCapture.retrieve(leftOriImage);
		rightCapture.retrieve(rightOriImage);
		//imshow("oriLeftImage", leftOriImage);
		//imshow("oriRightImage", rightOriImage);
		cvtColor(leftOriImage, leftGrayImage, CV_BGR2GRAY);
		cvtColor(rightOriImage, rightGrayImage, CV_BGR2GRAY);
		if (goodLeftFrame < frameNumberForcalibrate) {
			bool isLeftFind = getCorner(leftOriImage, leftGrayImage, boardSize, leftCorners);
			imshow("leftOriImage", leftOriImage);
			if (isLeftFind) { 
				
				goodLeftFrame++; 
			}
		}
		if (goodRightFrame < frameNumberForcalibrate) {
			bool isRightFind = getCorner(rightOriImage, rightGrayImage, boardSize, rightCorners);
			imshow("rightOriImage", rightOriImage);
			if (isRightFind) { 
				goodRightFrame++; 
			}
		}
		waitKey(50);
	}
	guessCameraParam(leftIntrinsic, leftDistortion_coeff);
	guessCameraParam(rightIntrinsic, rightDistortion_coeff);
	cout << "guess successful" << endl;
	calRealPoint(objRealPoint, boardWidth, boardHeight, frameNumberForcalibrate, squareSize);
	cout << "111" << endl;
	calibrateCamera(objRealPoint, leftCorners, Size(imageLeftWidth, imageLeftHeight), leftIntrinsic, leftDistortion_coeff, leftRvecs, leftTvecs, 0);
	cout << "222" << endl;
	calibrateCamera(objRealPoint, rightCorners, Size(imageRightWidth, imageRightHeight), rightIntrinsic, rightDistortion_coeff, rightRvecs, rightTvecs, 0);
	cout << "333" << endl;
	outputCameraParam(leftIntrinsic, rightIntrinsic, leftDistortion_coeff, rightDistortion_coeff);
	cout << "cal real successful" << endl;
	int a;
	cin >> a;
	rightCorners.clear();
	leftCorners.clear();
	int goodFrame = 0;
	leftCapture.grab();
	rightCapture.grab();
	leftCapture.retrieve(leftOriImage);
	rightCapture.retrieve(rightOriImage);
	Mat R, T, E, F;
	while (goodFrame < frameNumberForcalibrate) {
		cout << "inin" << endl;
		leftCapture.grab();
		rightCapture.grab();
		leftCapture.retrieve(leftOriImage);
		rightCapture.retrieve(rightOriImage);
		imshow("oriLeftImage", leftOriImage);
		imshow("oriRightImage", rightOriImage);
		cvtColor(leftOriImage, leftGrayImage, CV_BGR2GRAY);
		cvtColor(rightOriImage, rightGrayImage, CV_BGR2GRAY);
		string leftCa;
		string rightCa;
		stringstream leftCaStream;
		stringstream rightCaStream;
		vector<Point2f> leftCorner, rightCorner;
		bool isFindLeft = findChessboardCorners(leftOriImage, boardSize, leftCorner);
		bool isFindRight = findChessboardCorners(rightOriImage, boardSize, rightCorner);
		if (isFindLeft && isFindRight) {
			cout << goodFrame << endl;
			cout << leftCorner[0].x << " "  << leftCorner[0].y << " " <<leftCorner[24].x << " " << leftCorner[24].y << endl;
			cout << rightCorner[0].x << " " << rightCorner[0].y << " " << rightCorner[24].x << " " << rightCorner[24].y << endl;
			//isFindLeft = isFindLeft && (((leftCorner[24].x - leftCorner[0].x) * (rightCorner[24].x - rightCorner[0].x)) > 0);
			//cout << (leftCorner[24].x - leftCorner[0].x) * (rightCorner[24].x - rightCorner[0].x ) << endl;
			//isFindLeft = isFindLeft && (((leftCorner[24].y - leftCorner[0].y) * (rightCorner[24].y - rightCorner[0].y)) > 0);
			//cout << (leftCorner[24].y - leftCorner[0].y) * (rightCorner[24].y - rightCorner[0].y) << endl;
			isFindLeft = isFindLeft && ((leftCorner[24].x - leftCorner[0].x) < 0);
			isFindLeft = isFindLeft && ((leftCorner[24].y - leftCorner[0].y) > 0);
			isFindRight = isFindRight && ((rightCorner[24].x - rightCorner[0].x) < 0);
			isFindRight = isFindRight && ((rightCorner[24].y - rightCorner[0].y) > 0);
			
		}
		if (isFindLeft && isFindRight) {
			//cout << (leftCorner[24].x - leftCorner[0].x) * (rightCorner[24].x - rightCorner[0].x) << endl;
			//cout << (leftCorner[24].y - leftCorner[0].y) * (rightCorner[24].y - rightCorner[0].y) << endl;
			cornerSubPix(leftGrayImage, leftCorner, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
			drawChessboardCorners(leftOriImage, boardSize, leftCorner, isFindLeft);
			imshow("chessboardL", leftOriImage);
			leftCorners.push_back(leftCorner);
			cornerSubPix(rightGrayImage, rightCorner, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
			drawChessboardCorners(rightOriImage, boardSize, rightCorner, isFindRight);
			imshow("chessboardR", rightOriImage);
			cout << "rightCorner.size()" << rightCorner.size() << endl;
			rightCorners.push_back(rightCorner);
			leftCaStream << "leftCa" << goodFrame << ".bmp";
			leftCa = leftCaStream.str();
			imwrite(leftCa, leftOriImage);
			rightCaStream << "rightCa" << goodFrame << ".bmp";
			rightCa = rightCaStream.str();
			imwrite(rightCa, rightOriImage);
			goodFrame++;
			cout << "The image is good" << endl;
			waitKey(50);
		}
		else {
			cout << "The image is bad please try again" << endl;
		}
		leftCorner.clear();
		rightCorner.clear();
	}
	int ss;
	cin >> ss;
	cout << "ss" << endl;
	cout << ss << endl;
	double rms = stereoCalibrate(objRealPoint, leftCorners, rightCorners,
		leftIntrinsic, leftDistortion_coeff,
		rightIntrinsic, rightDistortion_coeff,
		Size(640, 480), R, T, E, F,
		CALIB_USE_INTRINSIC_GUESS,
		TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 100, 1e-5));
	outRT(R, T);
	cout << R.size() << endl;
	cout << T.size() << endl;
	cout << "mm" << endl;
	int mm;
	cin >> mm;
	cout << mm << endl;



#endif

	
	return 0;
}
void calRealPoint(vector<vector<Point3f>>& obj, int boardwidth, int boardheight, int imgNumber, int squaresize) {
	//  Mat imgpoint(boardheight, boardwidth, CV_32FC3,Scalar(0,0,0));  
	vector<Point3f> imgpoint;
	for (int rowIndex = 0; rowIndex < boardheight; rowIndex++) {
		for (int colIndex = 0; colIndex < boardwidth; colIndex++) {
			//imgpoint.at<Vec3f>(rowIndex, colIndex) = Vec3f(rowIndex * squaresize, colIndex*squaresize, 0);  
			imgpoint.push_back(Point3f(rowIndex * squaresize, colIndex * squaresize, 0));
		}
	}
	for (int imgIndex = 0; imgIndex < imgNumber; imgIndex++) {
		obj.push_back(imgpoint);
	}
}

bool getCorner(Mat& rgbImage, Mat& grayImage, Size boardSize, vector<vector<Point2f>>& corners) {
	vector<Point2f> corner;
	bool isFind = findChessboardCorners(rgbImage, boardSize, corner, 0);
	isFind = isFind && ((corner[24].x - corner[0].x) < 0);
	isFind = isFind && ((corner[24].y - corner[0].y) > 0);
	if (isFind == true) {
		cornerSubPix(grayImage, corner, Size(5, 5), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 20, 0.1));
		drawChessboardCorners(rgbImage, boardSize, corner, isFind);
		//imshow("chessboard", rgbImage);
		corners.push_back(corner);
		cout << "corners: " << corners.size() << endl;
		cout << "The image is good" << endl;
		return true;
	}
	else {
		cout << "The image is bad please try again" << endl;
		corner.clear();
		return false;
	}
}

void guessCameraParam(Mat& intrinsic, Mat& distortion_coeff) {
	/*�����ڴ�*/
	intrinsic.create(3, 3, CV_64FC1);
	distortion_coeff.create(5, 1, CV_64FC1);

	/*
	fx 0 cx
	0 fy cy
	0 0  1
	*/
	intrinsic.at<double>(0, 0) = 0;   //fx         
	intrinsic.at<double>(0, 2) = 0;   //cx  
	intrinsic.at<double>(1, 1) = 0;   //fy  
	intrinsic.at<double>(1, 2) = 0;   //cy  

	intrinsic.at<double>(0, 1) = 0;
	intrinsic.at<double>(1, 0) = 0;
	intrinsic.at<double>(2, 0) = 0;
	intrinsic.at<double>(2, 1) = 0;
	intrinsic.at<double>(2, 2) = 0;

	/*
	k1 k2 p1 p2 p3
	*/
	distortion_coeff.at<double>(0, 0) = 0;  //k1  
	distortion_coeff.at<double>(1, 0) = 0;  //k2  
	distortion_coeff.at<double>(2, 0) = 0;   //p1  
	distortion_coeff.at<double>(3, 0) = 0;   //p2  
	distortion_coeff.at<double>(4, 0) = 0;   //p3 
}


	

void outputCameraParam(const Mat& leftIntrinsic, const Mat& rightIntrinsic, const Mat& leftDistortion_coeff, const Mat& rightDistortion_coeff) {
	ofstream intrinsicParameters;
	intrinsicParameters.open("intrinsicParameters.txt");
	string data;
	stringstream dataStream;
	dataStream << "Left_Intrinsic:" << "\n";
	dataStream << leftIntrinsic.at<double>(0, 0) << " " << leftIntrinsic.at<double>(0, 1) << " " << leftIntrinsic.at<double>(0, 2) << "\n";
	dataStream << leftIntrinsic.at<double>(1, 0) << " " << leftIntrinsic.at<double>(1, 1) << " " << leftIntrinsic.at<double>(1, 2) << "\n";
	dataStream << leftIntrinsic.at<double>(2, 0) << " " << leftIntrinsic.at<double>(2, 1) << " " << leftIntrinsic.at<double>(2, 2) << "\n";
	dataStream << "Left_Distortion_coeff:" << "\n";
	dataStream << leftDistortion_coeff.at<double>(0, 0) << " " << leftDistortion_coeff.at<double>(0, 1) << " " << leftDistortion_coeff.at<double>(0, 2) << " " << leftDistortion_coeff.at<double>(0, 3) << " " << leftDistortion_coeff.at<double>(0, 4) << " " << leftDistortion_coeff.at<double>(0, 5) <<"\n";
	dataStream << "Right_Intrinsic:" << "\n";
	dataStream << rightIntrinsic.at<double>(0, 0) << " " << rightIntrinsic.at<double>(0, 1) << " " << rightIntrinsic.at<double>(0, 2) << "\n";
	dataStream << rightIntrinsic.at<double>(1, 0) << " " << rightIntrinsic.at<double>(1, 1) << " " << rightIntrinsic.at<double>(1, 2) << "\n";
	dataStream << rightIntrinsic.at<double>(2, 0) << " " << rightIntrinsic.at<double>(2, 1) << " " << rightIntrinsic.at<double>(2, 2) << "\n";
	dataStream << "Right_Distortion_coeff:" << "\n";
	dataStream << rightDistortion_coeff.at<double>(0, 0) << " " << rightDistortion_coeff.at<double>(0, 1) << " " << rightDistortion_coeff.at<double>(0, 2) << " " << rightDistortion_coeff.at<double>(0, 3) << " " << rightDistortion_coeff.at<double>(0, 4) << " " << rightDistortion_coeff.at<double>(0, 5) << "\n";
	data = dataStream.str();
	intrinsicParameters << data << endl;
	intrinsicParameters.close();
	return;
}
	
void outRT(const Mat& R, const Mat& T) {
	ofstream RT;
	RT.open("RT.txt");
	string data;
	stringstream dataStream;
	dataStream << "R:" << "\n";
	dataStream << R.at<double>(0, 0) << " " << R.at<double>(0, 1) << " " << R.at<double>(0, 2) << "\n";
	dataStream << R.at<double>(1, 0) << " " << R.at<double>(1, 1) << " " << R.at<double>(1, 2) << "\n";
	dataStream << R.at<double>(2, 0) << " " << R.at<double>(2, 1) << " " << R.at<double>(2, 2) << "\n";
	dataStream << "T:" << "\n";
	dataStream << T.at<double>(0, 0) << " " << T.at<double>(0, 1) << " " << T.at<double>(0, 2) << "\n";
	data = dataStream.str();
	RT << data << endl;
	cout << data << endl;
	RT.close();
	return;
}