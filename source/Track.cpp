#include <opencv2\opencv.hpp>
#include <iostream>
#include <fstream>
#include <sstream>

using namespace std;
using namespace cv;
bool findCen(const Mat& Pic, Point2f& Point);

double r1 = 0.957364, r2 = -0.0750122, r3 = -0.278977, r4 = 0.0359157, r5 = 0.989113, r6 = -0.142704;
double r7 = 0.286645, r8 = 0.1266, r9 = 0.949635;
double t1 = 465.329, t2 = 18.7391, t3 = -0.173557;
double fa = 438.431, fb = 904.265;
Point3f convert(Point2f uvLeft, Point2f uvRight);
Point3f uv2xyz(Point2f uvLeft, Point2f uvRight, const Mat& mLeftIntrinsic, const Mat& mLeftRotation, const Mat& mLeftTranslation, const Mat& mRightIntrinsic, const Mat& mRightRotation, const Mat& mRightTranslation);
int main() {
	Mat leftIntrinsic = (Mat_<double>(3, 3)  << 904.265, 0      , 309.442,
												0      , 891.259, 242.282,
												0      , 0      , 1       ); 
	Mat	rightIntrinsic = (Mat_<double>(3, 3) << 438.431, 0      , 317.82,
		                                        0      , 438.284, 248.467,
		                                        0      , 0      , 1       );
	Mat leftDistortion_coeff = (Mat_<double>(1, 5) << -0.769789, 1.88931, 0.00767651, 0.0307312, -3.71081, 0);
	Mat rightDistortion_coeff = (Mat_<double>(1, 5) << -0.287466, -0.240052, -0.00921337, -0.00638894, 1.53909, 0);
	Mat leftRvecs = (Mat_<double>(3, 3) << 1      , 0      , 0      ,
										   0      , 1      , 0      ,
		                                   0      , 0      , 1       );
	Mat rightRvecs = (Mat_<double>(3, 3) << 0.957364 , -0.0750122, -0.278977,
		                                    0.0359157, 0.989113  , -0.142704,
		                                    0.286645 , 0.1266    , 0.949635);
	Mat leftTvecs = (Mat_<double>(3, 1) << 0, 0, 0);
	Mat rightTvecs = (Mat_<double>(3, 1) << 465.329, 18.7391, -0.173557);
	VideoCapture leftCapture("Left11.avi");
	VideoCapture rightCapture("Right11.avi");
	int leftFrameNumber = leftCapture.get(CV_CAP_PROP_FRAME_COUNT);
	int rightFrameNumber = rightCapture.get(CV_CAP_PROP_FRAME_COUNT);
	cout << leftFrameNumber << " " << rightFrameNumber << endl;
	Mat leftPreFrame, leftCurFrame;
	Mat rightPreFrame, rightCurFrame;
	Mat leftShift, rightShift;
	Mat leftGrayShift, rightGrayShift;
	Mat leftBwShift, rightBwShift;
	int g_nStructElementSize = 1;
	Mat element = getStructuringElement(MORPH_ELLIPSE,
		Size(2 * g_nStructElementSize + 1, 2 * g_nStructElementSize + 1),
		Point(g_nStructElementSize, g_nStructElementSize));
	leftCapture >> leftCurFrame;
	rightCapture >> rightCurFrame;
	Mat leftCircle, rightCircle;
	vector<Point3f> ball;
	int frameCount = 0;
	leftPreFrame = leftCurFrame.clone();
	rightPreFrame = rightCurFrame.clone();
	leftCapture >> leftCurFrame;
	rightCapture >> rightCurFrame;
	leftPreFrame = leftCurFrame.clone();
	rightPreFrame = rightCurFrame.clone();
	leftCapture >> leftCurFrame;
	rightCapture >> rightCurFrame;
	ofstream fout;
	fout.open("point.txt");
	while (frameCount < 140) {
		leftPreFrame = leftCurFrame.clone();
		rightPreFrame = rightCurFrame.clone();
		leftCapture >> leftCurFrame;
		rightCapture >> rightCurFrame;
		leftShift = leftCurFrame - leftPreFrame;
		rightShift = rightCurFrame - rightPreFrame;
		cvtColor(leftShift, leftGrayShift, CV_BGR2GRAY);
		cvtColor(rightShift, rightGrayShift, CV_BGR2GRAY);
		if (!leftGrayShift.empty() && !rightGrayShift.empty()) {
			threshold(leftGrayShift, leftBwShift, 40, 255, CV_THRESH_BINARY);
			threshold(rightGrayShift, rightBwShift, 40, 255, CV_THRESH_BINARY);
			//erode(leftBwShift, leftBwShift, element, Point(-1, -1));
			//erode(rightBwShift, rightBwShift, element, Point(-1, -1));
			imshow("leftBwShift", leftBwShift);
			imshow("rightBwShift", rightBwShift);
			//cout << rightBwShift << endl;
			//dilate(leftShift, leftShift, element, Point(-1, -1));
			//dilate(rightShift, rightShift, element, Point(-1, -1));
			//circle(leftGrayShift, Point(leftCircle.at<uchar>(0, 1), leftCircle.at<uchar>(0, 2)), leftCircle.at<uchar>(0, 3), Scalar(255, 0, 0));
			//circle(rightGrayShift, Point(rightCircle.at<uchar>(0, 1), rightCircle.at<uchar>(0, 2)), rightCircle.at<uchar>(0, 3), Scalar(255, 0, 0));
			Point2f leftPoint, rightPoint;
			bool leftFind = findCen(leftGrayShift, leftPoint);
			bool rightFind = findCen(rightGrayShift, rightPoint);
			//circle(leftShift, leftPoint, 20, Scalar(0, 255, 0));
			//circle(rightShift, leftPoint, 20, Scalar(0, 255, 0));
			if (leftFind && rightFind) {
				cout << "inin" << endl;
				circle(leftShift, leftPoint, 20, Scalar(255, 0, 0));
				circle(rightShift, rightPoint, 20, Scalar(255, 0, 0));
				cout << leftPoint.x << " " << leftPoint.y << endl;
				cout << rightPoint.x << " " << rightPoint.y << endl;
				imshow("leftShift", leftShift);
				imshow("rightShift", rightShift);
				Point3f point3 = uv2xyz(leftPoint, rightPoint, leftIntrinsic, leftRvecs, leftTvecs, rightIntrinsic, rightRvecs, rightTvecs);
				cout << point3.x << " " << point3.y << " " << point3.z << endl;
				Point3f point33 = convert(leftPoint, rightPoint);
				cout << point33.x << " " << point33.y << " " << point33.z << endl;
				fout << "point:" << endl;
				fout << leftPoint.x << " " << leftPoint.y << " " << rightPoint.x << " " << rightPoint.y << endl;
				fout << point33.x << " " << point33.y << " " << point33.z << endl;
			}
			imshow("leftShift", leftGrayShift);
			imshow("rightShift", rightGrayShift);
			stringstream lname,rname,llname,rrname;
			lname << frameCount << "left.jpg";
			rname << frameCount << "right.jpg";
			llname << frameCount << "leftOriginal.jpg";
			rrname << frameCount << "rightOriginal.jpg";
			//imwrite(lname.str(), leftBwShift);
			//imwrite(rname.str(), rightBwShift);
			//imwrite(llname.str(), leftCurFrame);
			//imwrite(rrname.str(), rightCurFrame);
			cout << "_______________________________________________________________" << endl;

			/*
			HoughCircles(leftGrayShift, leftCircle, CV_HOUGH_GRADIENT, 1, 10);
			HoughCircles(rightGrayShift, rightCircle, CV_HOUGH_GRADIENT, 1, 10);
			cout << leftCircle.size() << endl;
			if (!leftCircle.empty() && !rightCircle.empty()) {
				circle(leftGrayShift, Point(leftCircle.at<uchar>(0, 1), leftCircle.at<uchar>(0, 2)), leftCircle.at<uchar>(0, 3), Scalar(255, 0, 0));
				circle(rightGrayShift, Point(rightCircle.at<uchar>(0, 1), rightCircle.at<uchar>(0, 2)), rightCircle.at<uchar>(0, 3), Scalar(255, 0, 0));
				imshow("leftShift", leftGrayShift);
				imshow("rightShift", rightGrayShift);
			}
			*/
			waitKey(100);
		}
		frameCount++;
	}
	fout.close();
	int a;
	cin >> a;
	return 0;

	
}

Point3f uv2xyz(Point2f uvLeft, Point2f uvRight, const Mat& mLeftIntrinsic, const Mat& mLeftRotation, const Mat& mLeftTranslation, const Mat& mRightIntrinsic, const Mat& mRightRotation, const Mat& mRightTranslation)
{
	//  [u1]      |X|                     [u2]      |X|  
	//Z*|v1| = Ml*|Y|                   Z*|v2| = Mr*|Y|  
	//  [ 1]      |Z|                     [ 1]      |Z|  
	//            |1|                               |1|  
	
	Mat mLeftRT = Mat(3, 4, CV_32F);//左相机M矩阵  
	hconcat(mLeftRotation, mLeftTranslation, mLeftRT);
	Mat mLeftM = mLeftIntrinsic * mLeftRT;
	//cout<<"左相机M矩阵 = "<<endl<<mLeftM<<endl;  

	Mat mRightRT = Mat(3, 4, CV_32F);//右相机M矩阵  
	hconcat(mRightRotation, mRightTranslation, mRightRT);
	Mat mRightM = mRightIntrinsic * mRightRT;
	//cout<<"右相机M矩阵 = "<<endl<<mRightM<<endl;  

	//最小二乘法A矩阵  
	Mat A = Mat(4, 3, CV_32F);
	A.at<float>(0, 0) = uvLeft.x * mLeftM.at<float>(2, 0) - mLeftM.at<float>(0, 0);
	A.at<float>(0, 1) = uvLeft.x * mLeftM.at<float>(2, 1) - mLeftM.at<float>(0, 1);
	A.at<float>(0, 2) = uvLeft.x * mLeftM.at<float>(2, 2) - mLeftM.at<float>(0, 2);

	A.at<float>(1, 0) = uvLeft.y * mLeftM.at<float>(2, 0) - mLeftM.at<float>(1, 0);
	A.at<float>(1, 1) = uvLeft.y * mLeftM.at<float>(2, 1) - mLeftM.at<float>(1, 1);
	A.at<float>(1, 2) = uvLeft.y * mLeftM.at<float>(2, 2) - mLeftM.at<float>(1, 2);

	A.at<float>(2, 0) = uvRight.x * mRightM.at<float>(2, 0) - mRightM.at<float>(0, 0);
	A.at<float>(2, 1) = uvRight.x * mRightM.at<float>(2, 1) - mRightM.at<float>(0, 1);
	A.at<float>(2, 2) = uvRight.x * mRightM.at<float>(2, 2) - mRightM.at<float>(0, 2);

	A.at<float>(3, 0) = uvRight.y * mRightM.at<float>(2, 0) - mRightM.at<float>(1, 0);
	A.at<float>(3, 1) = uvRight.y * mRightM.at<float>(2, 1) - mRightM.at<float>(1, 1);
	A.at<float>(3, 2) = uvRight.y * mRightM.at<float>(2, 2) - mRightM.at<float>(1, 2);

	//最小二乘法B矩阵  
	Mat B = Mat(4, 1, CV_32F);
	B.at<float>(0, 0) = mLeftM.at<float>(0, 3) - uvLeft.x * mLeftM.at<float>(2, 3);
	B.at<float>(1, 0) = mLeftM.at<float>(1, 3) - uvLeft.y * mLeftM.at<float>(2, 3);
	B.at<float>(2, 0) = mRightM.at<float>(0, 3) - uvRight.x * mRightM.at<float>(2, 3);
	B.at<float>(3, 0) = mRightM.at<float>(1, 3) - uvRight.y * mRightM.at<float>(2, 3);

	Mat XYZ = Mat(3, 1, CV_32F);
	//采用SVD最小二乘法求解XYZ  
	solve(A, B, XYZ, DECOMP_SVD);

	//cout<<"空间坐标为 = "<<endl<<XYZ<<endl;  

	//世界坐标系中坐标  
	Point3f world;
	world.x = XYZ.at<float>(0, 0);
	world.y = XYZ.at<float>(1, 0);
	world.z = XYZ.at<float>(2, 0);
	
	return world;
}

bool findCen(const Mat& Pic, Point2f& Point) {
	long long int sumX = 0;
	long long int sumY = 0;
	long long int sum = 0;
	for (int i = 0; i < 480; i++) {
		for (int j = 0; j < 640; j++) {
			//cout << Pic.at<double>(i, j) << endl;
			//cout << "jidjaifjajif" << endl;
			if (Pic.at<uchar>(i, j) >40) {
				sumX += i * Pic.at<uchar>(i, j);
				sumY += j * Pic.at<uchar>(i, j);
				sum += Pic.at<uchar>(i, j);
				//cout << i  << " " << j << " " << (int)(Pic.at<uchar>(i, j)) << endl;
			}
		}
	}
	if (sum != 0) {
		Point = Point2f(sumX / (double)sum, sumY / (double)sum);
		//cout << sumX / (double)sum << sumY / (double)sum << endl;
		return true;
	}
	else {
		return false;
	}

}

Point3f convert(Point2f uvLeft, Point2f uvRight) {
	double xua = uvRight.x;
	double yua = uvRight.y;
	double xub = uvLeft.x;
	double yub = uvLeft.y;
	double z = fb*(fa*t1 - xua*t3) / (xua*(xub*r7 + yub*r8 + fb*r9) - fa*(xub*r1 + yub*r2 + fb*r3));
	double x = xub*z / fb;
	double y = yub*z / fb;
	return Point3f(x, y, z);
}