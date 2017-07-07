#ifdef _DEBUG
#define _SCL_SECURE_NO_WARNINGS
#endif

#include <iostream>             //Standard Library
#include <thread>
#include <chrono>
#include <fstream>
#include <vector>

#include <Kinect.h>             //Linect for Windows SDK Header
#include <opencv2/core.hpp> //open cv
#include <opencv2/highgui.hpp> //header
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/video.hpp>

HRESULT result;
using namespace std;
bool isDetected = false;
bool has_depth = false;
bool running = true;
bool bg_captured = false;
const int SCAN_RADIUS = 5;
int neckY, neckX, waistY1, waistY2, waistY, waistX, elbowRightX, elbowRightY, shoulderRightX, shoulderRightY, armX, armY;
std::vector<std::vector<cv::Point>> contour_points;

ICoordinateMapper* mapper;

class Point {
	int x;
	int y;
	UINT16 depth;

public:
	Point(const int xin, const int yin) {
		x = xin;
		y = yin;
	}

	void setDepth(const UINT16 depthIn) {
		depth = depthIn;
	}

	int getX() {
		return x;
	}

	int getY() {
		return y;
	}

	UINT16 getDepth() {
		return depth;
	}
};
vector<Point> points;

class Timer {
	thread timerThread;
	const int COUNT_DOWN_VALUE = 3;
	const int IMG_COUNT_VALUE = 2;
	const string imgName1 = "/Users/A30286/Desktop/front.bmp";
	const string imgName2 = "/Users/A30286/Desktop/side.bmp";
	int countDown;
	int countImg;

public:
	typedef cv::Mat Img;

	void start(Img img) {
		countDown = COUNT_DOWN_VALUE;
		countImg = IMG_COUNT_VALUE;
		timerThread = thread([=]() {
			while (true) {
				if (countDown == 0) {
					if (countImg == 2) {
						cv::imwrite(imgName1, img);
						cout << "Front captured" << endl;
						countDown = COUNT_DOWN_VALUE;
						countImg--;
					}
					else {
						cv::imwrite(imgName2, img);
						cout << "Side captured" << endl;
						countImg--;
					}
				}
				else if (isDetected == true) {
					cout << countDown << endl;
					this_thread::sleep_for((std::chrono::milliseconds)1000);
					countDown--;
				}
				if (countImg == 0) {
					running = false;
					break;
				}
			}
		});
		timerThread.detach();
	}
};

/*8 direction examination*/
bool isBorder(BYTE* buffer, int x, int y, int width, int height) {
	if (x, y > 2 && x < width - 2 && y < height - 2) {
		int fixPont = buffer[x + y      * width];
		int refPoint0 = buffer[(x - 1) + (y - 1) * width];
		int refPoint1 = buffer[(x - 1) + y      * width];
		int refPoint2 = buffer[(x - 1) + (y + 1) * width];
		int refPoint3 = buffer[x + (y - 1) * width];
		int refPoint4 = buffer[x + (y + 1) * width];
		int refPoint5 = buffer[(x + 1) + (y - 1) * width];
		int refPoint6 = buffer[(x + 1) + y      * width];
		int refPoint7 = buffer[(x + 1) + (y + 1) * width];
		if ((fixPont == refPoint0) && (fixPont == refPoint1) &&
			(fixPont == refPoint2) && (fixPont == refPoint3) &&
			(fixPont == refPoint4) && (fixPont == refPoint5) &&
			(fixPont == refPoint6) && (fixPont == refPoint7)) {
			return false;
		}
		else {
			return true;
		}
	}
	else {
		return false;
	}
}


bool isDot(cv::Mat img, int x, int y, cv::Vec3b colorTable[7], int radius) {
	int whiteCount = 0;
	for (int iy = y - radius; iy <= y + radius; iy++) {
		for (int ix = x - radius; ix <= x + radius; ix++) {
			if ((ix != x || iy != y) && img.at<cv::Vec3b>(iy, ix) == colorTable[6]) {
				whiteCount++;
			}
		}
	}
	int fullCount = (radius + 1 + radius) * (radius + 1 + radius) - 1;
	if (whiteCount == fullCount) {
		return true;
	}
	else {
		return false;
	}
}

cv::Point changeCoordinatesToDepth(Joint joint[JointType::JointType_Count], int type)
{
	DepthSpacePoint depthPoint = { 0 };
	mapper->MapCameraPointToDepthSpace(joint[type].Position, &depthPoint);
	int x = static_cast<int>(depthPoint.X);
	int y = static_cast<int>(depthPoint.Y);

	return cv::Point(x, y);
}

void drawSkeleton(cv::Mat canvas, Joint joint[JointType::JointType_Count])
{
	cv::line(canvas, changeCoordinatesToDepth(joint, JointType_Head), changeCoordinatesToDepth(joint, JointType_Neck), CvScalar(0, 0, 255), 1);
	cv::line(canvas, changeCoordinatesToDepth(joint, JointType_Neck), changeCoordinatesToDepth(joint, JointType_SpineShoulder), CvScalar(0, 0, 255), 1);
	cv::line(canvas, changeCoordinatesToDepth(joint, JointType_SpineShoulder), changeCoordinatesToDepth(joint, JointType_ShoulderLeft), CvScalar(0, 0, 255), 1);
	cv::line(canvas, changeCoordinatesToDepth(joint, JointType_SpineShoulder), changeCoordinatesToDepth(joint, JointType_ShoulderRight), CvScalar(0, 0, 255), 1);
	cv::line(canvas, changeCoordinatesToDepth(joint, JointType_SpineShoulder), changeCoordinatesToDepth(joint, JointType_SpineMid), CvScalar(0, 0, 255), 1);
	cv::line(canvas, changeCoordinatesToDepth(joint, JointType_ShoulderLeft), changeCoordinatesToDepth(joint, JointType_ElbowLeft), CvScalar(0, 0, 255), 1);
	cv::line(canvas, changeCoordinatesToDepth(joint, JointType_ShoulderRight), changeCoordinatesToDepth(joint, JointType_ElbowRight), CvScalar(0, 0, 255), 1);
	cv::line(canvas, changeCoordinatesToDepth(joint, JointType_ElbowLeft), changeCoordinatesToDepth(joint, JointType_WristLeft), CvScalar(0, 0, 255), 1);
	cv::line(canvas, changeCoordinatesToDepth(joint, JointType_ElbowRight), changeCoordinatesToDepth(joint, JointType_WristRight), CvScalar(0, 0, 255), 1);
	cv::line(canvas, changeCoordinatesToDepth(joint, JointType_WristLeft), changeCoordinatesToDepth(joint, JointType_HandLeft), CvScalar(0, 0, 255), 1);
	cv::line(canvas, changeCoordinatesToDepth(joint, JointType_WristRight), changeCoordinatesToDepth(joint, JointType_HandRight), CvScalar(0, 0, 255), 1);
	cv::line(canvas, changeCoordinatesToDepth(joint, JointType_HandLeft), changeCoordinatesToDepth(joint, JointType_HandTipLeft), CvScalar(0, 0, 255), 1);
	cv::line(canvas, changeCoordinatesToDepth(joint, JointType_HandRight), changeCoordinatesToDepth(joint, JointType_HandTipRight), CvScalar(0, 0, 255), 1);
	cv::line(canvas, changeCoordinatesToDepth(joint, JointType_WristLeft), changeCoordinatesToDepth(joint, JointType_ThumbLeft), CvScalar(0, 0, 255), 1);
	cv::line(canvas, changeCoordinatesToDepth(joint, JointType_WristRight), changeCoordinatesToDepth(joint, JointType_ThumbRight), CvScalar(0, 0, 255), 1);
	cv::line(canvas, changeCoordinatesToDepth(joint, JointType_SpineMid), changeCoordinatesToDepth(joint, JointType_SpineBase), CvScalar(0, 0, 255), 1);
	cv::line(canvas, changeCoordinatesToDepth(joint, JointType_SpineBase), changeCoordinatesToDepth(joint, JointType_HipLeft), CvScalar(0, 0, 255), 1);
	cv::line(canvas, changeCoordinatesToDepth(joint, JointType_SpineBase), changeCoordinatesToDepth(joint, JointType_HipRight), CvScalar(0, 0, 255), 1);
	cv::line(canvas, changeCoordinatesToDepth(joint, JointType_HipLeft), changeCoordinatesToDepth(joint, JointType_KneeLeft), CvScalar(0, 0, 255), 1);
	cv::line(canvas, changeCoordinatesToDepth(joint, JointType_HipRight), changeCoordinatesToDepth(joint, JointType_KneeRight), CvScalar(0, 0, 255), 1);
	cv::line(canvas, changeCoordinatesToDepth(joint, JointType_KneeLeft), changeCoordinatesToDepth(joint, JointType_AnkleLeft), CvScalar(0, 0, 255), 1);
	cv::line(canvas, changeCoordinatesToDepth(joint, JointType_KneeRight), changeCoordinatesToDepth(joint, JointType_AnkleRight), CvScalar(0, 0, 255), 1);
	cv::line(canvas, changeCoordinatesToDepth(joint, JointType_AnkleLeft), changeCoordinatesToDepth(joint, JointType_FootLeft), CvScalar(0, 0, 255), 1);
	cv::line(canvas, changeCoordinatesToDepth(joint, JointType_AnkleRight), changeCoordinatesToDepth(joint, JointType_FootRight), CvScalar(0, 0, 255), 1);
}

cv::Mat subtraction(cv::Mat img, cv::Mat bg) {
	std::cout << "start bg_subtraction" << std::endl;
	cv::Mat org = img.clone();
	cv::Mat rst;
	//cvtColor(img, img, CV_BGR2GRAY);
	//cvtColor(bg, bg, CV_BGR2GRAY);
	//cv::absdiff(img, bg, rst);
	cv::subtract(bg, img, rst);
	cvtColor(rst, rst, CV_BGR2GRAY);
	double min, max;
	cv::minMaxLoc(rst, &min, &max);
	cv::Mat contoursImgRgb;
	rst.convertTo(contoursImgRgb, CV_8UC1, 255.0 / (max - min), -min);
	cv::Mat edges;
	cv::GaussianBlur(contoursImgRgb, contoursImgRgb, cv::Size(5, 5), 0);
	//cv::threshold(contoursImgRgb, contoursImgRgb, 45, 255, cv::THRESH_BINARY);
	cv::erode(contoursImgRgb, contoursImgRgb, cv::Mat(), cv::Point(0, 0), 2);
	cv::dilate(contoursImgRgb, contoursImgRgb, cv::Mat(), cv::Point(0, 0), 2);
	cv::Canny(contoursImgRgb, edges, 40, 80, 3, true);
	std::vector<std::vector<cv::Point>> contours;
	std::vector<cv::Vec4i> hierarchy;
	try
	{
		cv::findContours(edges, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	}
	catch (cv::Exception &e)
	{
		cerr << e.msg << endl;
	}
	for (int i = 0; i < contours.size(); i++) {
		double area = cv::contourArea(contours[i]);
		double minArea = 25;
		if (area > minArea) {
			contour_points.push_back(contours[i]);
			cv::Scalar color = cv::Scalar(0, 0, 150, 255);
			cv::drawContours(org, contours, i, color, 2, 8, hierarchy, 0);
		}
	}
	std::cout << "finish bg_subtration" << std::endl;
	return org;
}

int main(int argc, char** argv) {
	/*get default sensor*/
	cout << "Try to get default sensor" << endl;
	IKinectSensor* sensor = nullptr;
	if (GetDefaultKinectSensor(&sensor) != S_OK) {
		cerr << "Get sensor failed" << endl;
		return -1;
	}
	else {
		cout << "Succeed, Try to open sensor" << endl;
		if (sensor->Open() != S_OK) {
			cerr << "Can't open sensor" << endl;
			return -1;
		}
		else {
			cout << "Succeed" << endl;
		}
	}
	// TODO Replace seperated frame reader with multisourceframcereader
	// IMultiSourceFrameReader* multiSourceFrameReader = nullptr;
	// sensor->OpenMultiSourceFrameReader(
	// FrameSourceTypes_BodyIndex | FrameSourceTypes_Depth, &multiSourceFrameReader);

	/*get frame source*/
	IBodyIndexFrameSource* bodyIndexFrameSource = nullptr;
	sensor->get_BodyIndexFrameSource(&bodyIndexFrameSource);

	/*get frame description*/
	int width = 0;
	int height = 0;
	IFrameDescription* frameDescription = nullptr;
	bodyIndexFrameSource->get_FrameDescription(&frameDescription);
	frameDescription->get_Width(&width);
	frameDescription->get_Height(&height);
	frameDescription->Release();
	frameDescription = nullptr;

	/*get frame reader*/
	IBodyIndexFrameReader* bodyIndexFrameReader = nullptr;
	bodyIndexFrameSource->OpenReader(&bodyIndexFrameReader);

	/*release frame source*/
	bodyIndexFrameSource->Release();
	bodyIndexFrameSource = nullptr;

	IDepthFrameSource* depthFrameSource = nullptr;
	sensor->get_DepthFrameSource(&depthFrameSource);
	int depth_width = 0;
	int depth_height = 0;
	depthFrameSource->get_FrameDescription(&frameDescription);
	frameDescription->get_Width(&depth_width);
	frameDescription->get_Height(&depth_height);
	frameDescription->Release();
	frameDescription = nullptr;
	IDepthFrameReader* depthFrameReader = nullptr;
	UINT16 depthMin = 0, depthMax = 0;
	depthFrameSource->get_DepthMinReliableDistance(&depthMin);
	depthFrameSource->get_DepthMaxReliableDistance(&depthMax);
	depthFrameSource->OpenReader(&depthFrameReader);
	depthFrameSource->Release();
	depthFrameSource = nullptr;

	IBodyFrameSource* bodyframeSource = nullptr;
	sensor->get_BodyFrameSource(&bodyframeSource);
	INT32 bodyCount = 0;
	bodyframeSource->get_BodyCount(&bodyCount);
	IBody** bodies = new IBody*[bodyCount];
	for (int i = 0; i < bodyCount; ++i) {
		bodies[i] = nullptr;
	}
	IBodyFrameReader* bodyFrameReader = nullptr;
	bodyframeSource->OpenReader(&bodyFrameReader);
	bodyframeSource->Release();
	bodyframeSource = nullptr;

	int color_width;
	int color_height;
	IColorFrameSource* colorFrameSource = nullptr;
	sensor->get_ColorFrameSource(&colorFrameSource);
	colorFrameSource->get_FrameDescription(&frameDescription);
	frameDescription->get_Width(&color_width);
	frameDescription->get_Height(&color_height);
	frameDescription->Release();
	frameDescription = nullptr;

	IColorFrameReader* colorFrameReader = nullptr;
	colorFrameSource->OpenReader(&colorFrameReader);

	/*prepare OpenCV*/
	cv::Mat img(height, width, CV_8UC3);
	cv::Mat img_depth(depth_height, depth_width, CV_16UC1);
	cv::Mat img_depth8bit(height, width, CV_8UC1);
	cv::Mat img_color(color_height, color_width, CV_8UC4);
	cv::Mat img_color_pure(color_height, color_width, CV_8UC4);
	cv::Mat img_color_background(color_height, color_width, CV_8UC4);
	cv::namedWindow("contour detect");
	cv::namedWindow("depth image");

	/*color array*/
	cv::Vec3b colorTable[7] = {
		cv::Vec3b(255,255,255),
		cv::Vec3b(0,255,0),
		cv::Vec3b(0,0,255),
		cv::Vec3b(255,255,0),
		cv::Vec3b(255,0,255),
		cv::Vec3b(0,255,255),
		cv::Vec3b(0,0,0),
	};

	/*Coordinate Mapper*/
	sensor->get_CoordinateMapper(&mapper);

	/*set timer*/
	Timer captureTimer;
	captureTimer.start(img);

	/*stream for variables saving*/
	ofstream streamxy;
	ofstream streamxy_f;
	ofstream stream_joints;

	DepthSpacePoint* color_to_depth_points = new DepthSpacePoint[color_width * color_height];

	/*enter loop*/
	while (running) {
		//clear frames
		streamxy.open("contour_points_unfiltered.txt", ios::out | ios::trunc);
		streamxy_f.open("contour_points_filtered.txt", ios::out | ios::trunc);
		stream_joints.open("joints_position.txt", ios::out | ios::trunc);
		neckY = 0;
		waistY1 = 0;
		waistY2 = 0;
		elbowRightX = 0;
		elbowRightY = 0;
		shoulderRightX = 0;
		shoulderRightY = 0;
		points.clear();
		IBodyIndexFrame* bodyIndexFrame = nullptr;
		IDepthFrame* depthFrame = nullptr;
		IBodyFrame* bodyFrame = nullptr;
		IColorFrame* colorFrame = nullptr;
		Joint joints[JointType::JointType_Count];
		//color
		if (colorFrameReader->AcquireLatestFrame(&colorFrame) == S_OK) {
			UINT bufferSize = color_height * color_width * 4 * sizeof(BYTE);
			colorFrame->CopyConvertedFrameDataToArray(bufferSize, img_color.data, ColorImageFormat_Bgra);
			img_color_pure = img_color.clone();
			if (bg_captured == false) {
				img_color_background = img_color.clone();
				bg_captured = true;
			}
			colorFrame->Release();
		}
		//skeletons
		if (bodyFrameReader->AcquireLatestFrame(&bodyFrame) == S_OK) {
			if (bodyFrame->GetAndRefreshBodyData(bodyCount, bodies) == S_OK) {
				int trackBodyCount = 0;
				for (int i = 0; i < bodyCount; ++i) {
					IBody* body = bodies[i];
					// check if it's tracked
					BOOLEAN tracked = false;
					if ((body->get_IsTracked(&tracked) == S_OK) && tracked) {
						++trackBodyCount;

						body->GetJoints(JointType::JointType_Count, joints);
						for (int i = 0; i < 25; i++) {
							DepthSpacePoint depthPoint = { 0 };
							mapper->MapCameraPointToDepthSpace(joints[i].Position, &depthPoint);
							stream_joints << depthPoint.X << " ";
							stream_joints << depthPoint.Y << " ";
							stream_joints << joints[i].Position.Z << "\n";
							if (i == JointType::JointType_Neck) {
								neckY = depthPoint.Y - 6;
								neckX = depthPoint.X;
							}
							else if (i == JointType::JointType_SpineMid) {
								waistY1 = depthPoint.Y;
								waistX = depthPoint.X;
							}
							else if (i == JointType::JointType_SpineBase) {
								waistY2 = depthPoint.Y;
							}
							else if (i == JointType::JointType_HandRight) {
								elbowRightX = depthPoint.X;
								elbowRightY = depthPoint.Y;
							}
							else if (i == JointType::JointType_ShoulderRight) {
								shoulderRightX = depthPoint.X;
								shoulderRightY = depthPoint.Y;
							}
						}
					}
				}
			}
			waistY = (waistY1 + waistY2) / 2;
			armX = (shoulderRightX + elbowRightX) / 2;
			armY = (shoulderRightY + elbowRightY) / 2;
			stream_joints.close();
			bodyFrame->Release();
		}
		//bodyidx
		if (bodyIndexFrameReader->AcquireLatestFrame(&bodyIndexFrame) == S_OK) {
			/*fill image*/
			UINT size = 0;
			BYTE *buffer = nullptr;
			bodyIndexFrame->AccessUnderlyingBuffer(&size, &buffer);
			for (int x = SCAN_RADIUS; x < width - SCAN_RADIUS; ++x) {
				for (int y = SCAN_RADIUS; y < height - SCAN_RADIUS; ++y) {
					int bodyIdx = buffer[x + y * width];
					if (bodyIdx < 6) { //human body 
						if (!isDetected) { //start countdown
							cout << "Body detected" << endl;
							isDetected = true;
						}
						if (isBorder(buffer, x, y, width, height) == true) { //unfiltered contour
							img.at<cv::Vec3b>(y, x) = colorTable[0];
							Point tmp(x, y);
							points.push_back(tmp);
							streamxy << x << " " << y << "\n";
						}
						else {
							img.at<cv::Vec3b>(y, x) = colorTable[6];
						}
						if (isBorder(buffer, x, y, width, height) == true &&
							isDot(img, x, y, colorTable, SCAN_RADIUS) == true) { //filtered contour with custom radius
																				 //img.at<cv::Vec3b>(y, x) = colorTable[0];
							streamxy_f << x << " " << y << "\n";
						}
						else {
							//img.at<cv::Vec3b>(y, x) = colorTable[6];
						}
					}
					else { //background
						img.at<cv::Vec3b>(y, x) = colorTable[6];
					}
				}
			}
			//draw joints
			for (int type = 0; type < JointType::JointType_Count; type++) {
				cv::Point jointPoint = changeCoordinatesToDepth(joints, type);
				if ((jointPoint.x >= 0) && (jointPoint.y >= 0) && (jointPoint.x < depth_width) && (jointPoint.y < depth_height)) {
					cv::circle(img, jointPoint, 3, CvScalar(0, 0, 255), -1, CV_AA);
				}
			}
			//draw skeletons
			drawSkeleton(img, joints);
			cv::imshow("contour detect", img);

			bodyIndexFrame->Release();
			streamxy.close();
			streamxy_f.close();
		}
		//depth
		if (depthFrameReader->AcquireLatestFrame(&depthFrame) == S_OK) {
			UINT depth_size = 0;
			UINT16* frameData = nullptr;
			UINT16* depthBuffer = nullptr;
			depthFrame->AccessUnderlyingBuffer(&depth_size, &depthBuffer);
			depthFrame->CopyFrameDataToArray(depth_width * depth_height,
				reinterpret_cast<UINT16*>(img_depth.data));
			depthFrame->CopyFrameDataToArray(depth_width * depth_height, frameData);
			int countW = 0;
			int countA = 0;
			int countN = 0;
			int depthRef, aY1, aY2, nX1, nX2, wX1, wX2 = 0;
			float diff;
			for (Point &p : points) {
				p.setDepth(depthBuffer[p.getX() + depth_width * p.getY()]);
				DepthSpacePoint depthPoint = { static_cast<float>(p.getX()), static_cast<float>(p.getY()) };
				ColorSpacePoint colorPoint = { 0.0f, 0.0f };
				mapper->MapDepthPointToColorSpace(depthPoint, depthBuffer[p.getX() + depth_width * p.getY()], &colorPoint);
				int colorX = static_cast<int>(std::floor(colorPoint.X + 0.5f));
				int colorY = static_cast<int>(std::floor(colorPoint.Y + 0.5f));
				cv::circle(img_color, cv::Point(colorX, colorY), 1, CvScalar(0, 0, 255), -1, CV_AA);
				if (p.getX() == armX) {
					countA++;
					if (countA == 1) {
						aY1 = p.getY();
					}
					else if (countA == 2) {
						aY2 = p.getY();
					}
				}
				if (p.getY() == neckY) {
					countN++;
					if (countN == 1) {
						nX1 = p.getX();
					}
					else if (countN == 2) {
						nX2 = p.getX();
					}
				}
				if (p.getY() == waistY) {
					countW++;
					if (countW == 1) {
						wX1 = p.getX();
					}
					else if (countW == 2) {
						wX2 = p.getX();
					}
				}
			}
			depthRef = depthBuffer[waistX + depth_width * waistY1];
			//kinectv2 depth camera-> 512 * 424, 70.6 * 60 degrees
			float pixelSizeWidth = (0.708 * depthRef * 0.1 * 2) / 512;
			float pixelSizeHeight = (0.57735 * depthRef * 0.1 * 2) / 424;
			if (countA == 2) {
				diff = depthBuffer[armX + depth_width * aY1] -
					depthBuffer[armX + depth_width * armY];
				if (diff > 0) {
					//cout << "arm's depth: " << diff * 0.2 << "cm";
					//cout << ", arm's width: " << (aY2 - aY1) * pixelSizeHeight << "cm" << endl;
				}
			}
			if (countN == 2) {
				diff = depthBuffer[nX1 + depth_width * neckY] -
					depthBuffer[neckX + depth_width * neckY];
				if (diff > 0) {
					//cout << "neck's depth: " << diff * 0.2 << "cm";
					//cout << ", neck's width: " << (nX2 - nX1) * pixelSizeWidth << "cm" << endl;
				}
			}
			if (countW == 2) {
				diff = depthBuffer[wX1 + depth_width * waistY] -
					depthBuffer[waistX + depth_width * waistY];
				if (diff > 0) {
					//cout << "waist's depth: " << diff * 0.2 << "cm";
					//cout << ", waist's width: " << (wX2 - wX1) * pixelSizeWidth << "cm" << endl;
				}
			}
			//cout << "neck depth: " << depthBuffer[neckX + depth_width * neckY] << endl;
			img_depth.convertTo(img_depth8bit, CV_8U, 255.0f / depthMax);
			cv::resize(img_depth8bit, img_depth8bit, cv::Size(img_depth8bit.cols * 2.85, img_depth8bit.rows * 3));
			cv::imshow("depth image", img_depth8bit);
			cv::imshow("color with depth contour", img_color);

			result = mapper->MapColorFrameToDepthSpace(depth_width * depth_height, depthBuffer, color_width * color_height, color_to_depth_points);

			depthFrame->Release();
		}
		if (cv::waitKey(30) == VK_ESCAPE) {
			cv::imwrite("/Users/A30286/Desktop/object.bmp", img);
			break;
		}
	}
	img_depth.convertTo(img_depth, CV_8UC3, 255.0f / depthMax);
	cvtColor(img_depth, img_depth, CV_GRAY2BGR);
	cvDestroyAllWindows();
	cv::Mat color_sub = subtraction(img_color_pure, img_color_background);
	for (Point p : points) {
		DepthSpacePoint depthPoint = { static_cast<float>(p.getX()), static_cast<float>(p.getY()) };
		ColorSpacePoint colorPoint = { 0.0f, 0.0f };
		mapper->MapDepthPointToColorSpace(depthPoint, p.getDepth(), &colorPoint);
		int colorX = static_cast<int>(std::floor(colorPoint.X + 0.5f));
		int colorY = static_cast<int>(std::floor(colorPoint.Y + 0.5f));
		cv::circle(color_sub, cv::Point(colorX, colorY), 2, CvScalar(255, 0, 0), -1, CV_AA);
	}

	//color contour map to depth image
	std::vector<cv::Point> tmp = {};
	if (SUCCEEDED(result)) {
		for (int i = 0; i < contour_points.size(); i++) {
			tmp.clear();
			for (cv::Point p : contour_points[i]) {
				if (color_to_depth_points[p.x + p.y * color_width].X >= 0 &&
					color_to_depth_points[p.x + p.y * color_width].X < depth_width &&
					color_to_depth_points[p.x + p.y * color_width].Y >= 0 &&
					color_to_depth_points[p.x + p.y * color_width].Y < depth_height) {
					int color_to_depth_X = static_cast<int>(std::floor(color_to_depth_points[p.x + p.y * color_width].X + 0.5f));
					int color_to_depth_Y = static_cast<int>(std::floor(color_to_depth_points[p.x + p.y * color_width].Y + 0.5f));
					tmp.push_back(cv::Point(color_to_depth_X, color_to_depth_Y));
					cout << p.x << ", " << p.y << endl;
				}
			}
			cv::polylines(img_depth, tmp, true, CvScalar(255, 255, 0), 1, CV_AA);
		}
		cv::imshow("ColorFrameToDepthSpace", img_depth);
	}
	else {
		cout << "Error code: " << result << endl;
	}
	cv::imshow("subs", color_sub);
	cvWaitKey(0);

	/*release frame reader*/
	bodyIndexFrameReader->Release();
	bodyIndexFrameReader = nullptr;
	depthFrameReader->Release();
	depthFrameReader = nullptr;
	bodyFrameReader->Release();
	bodyFrameReader = nullptr;
	/*close sensor*/
	sensor->Close();
	sensor->Release();
	sensor = nullptr;

	return 0;
}​