#include <iostream> //Standard Library
#include <array>
#include <thread>
#include <chrono>
#include <Kinect.h> //Linect for Windows SDK Header
#include <opencv2/core.hpp>		//open cv
#include <opencv2/highgui.hpp>	//header
#include <opencv2/imgproc.hpp>

using namespace std;
bool isDetected = false;

class Timer {
	thread timerThread;
	const int COUNT_DOWN_VALUE = 5;
	const int IMG_COUNT_VALUE = 2;
	const string imgName1 = "C:\\front.jpg";
	const string imgName2 = "C:\\side.jpg";
	int countDown;
	int countImg;

public:
	//typedef std::chrono::milliseconds Interval;
	//typedef std::function<void(void)> Timeout;
	typedef cv::Mat Img;

	void start(Img img) {
		countDown = COUNT_DOWN_VALUE;
		countImg = IMG_COUNT_VALUE;
		timerThread = thread([=]() {
			while (countImg != 0) {
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
				} else if (isDetected == true) {
					cout << countDown << endl;
					this_thread::sleep_for((std::chrono::milliseconds)1000);
					countDown--;
				}
			}
		});
		timerThread.detach();
	}
};

/*8 direction examination*/
bool isBorder(BYTE* buffer, int x, int y, int width, int height) {
	if (x, y > 2 && x < width - 2 && y < height - 2) {
		int fixPont   = buffer[ x      +  y      * width];
		int refPoint0 = buffer[(x - 1) + (y - 1) * width];
		int refPoint1 = buffer[(x - 1) +  y      * width];
		int refPoint2 = buffer[(x - 1) + (y + 1) * width];
		int refPoint3 = buffer[ x      + (y - 1) * width];
		int refPoint4 = buffer[ x      + (y + 1) * width];
		int refPoint5 = buffer[(x + 1) + (y - 1) * width];
		int refPoint6 = buffer[(x + 1) +  y      * width];
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

	/*get frame source*/
	IBodyIndexFrameSource* bodyFrameSource = nullptr;
	//IDepthFrameSource* depthFrameSource = nullptr;
	sensor->get_BodyIndexFrameSource(&bodyFrameSource);
	//sensor->get_DepthFrameSource(&depthFrameSource);

	/*get frame description*/
	int width = 0;
	int height = 0;
	IFrameDescription* frameDescription = nullptr;
	bodyFrameSource->get_FrameDescription(&frameDescription);
	frameDescription->get_Width(&width);
	frameDescription->get_Height(&height);
	frameDescription->Release();
	frameDescription = nullptr;

	/*get frame reader*/
	IBodyIndexFrameReader* bodyFrameReader = nullptr;
	IDepthFrameReader* depthFrameReader = nullptr;
	bodyFrameSource->OpenReader(&bodyFrameReader);
	//depthFrameSource->OpenReader(&depthFrameReader);
	/*release frame source*/
	bodyFrameSource->Release();
	//depthFrameSource->Release();
	bodyFrameSource = nullptr;
	//depthFrameSource = nullptr;

	/*prepare OpenCV*/
	cv::Mat img(height, width, CV_8UC3);
	///cv::Mat depthImg(height, width, CV_16UC1);
	///cv::Mat img8bit(height, width, CV_8UC1);
	cv::namedWindow("contour detect");

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

	/*set timer*/
	Timer captureTimer;
	captureTimer.start(img);

	/*enter loop*/
	while (true) {
		/*get last frame*/
		IBodyIndexFrame* bodyFrame = nullptr;
		//IDepthFrame* depthFrame = nullptr;
		if (bodyFrameReader->AcquireLatestFrame(&bodyFrame) == S_OK) {
			///copy the depth map to image
			///frame->CopyFrameDataToArray(width * height, reinterpret_cast<UINT16*>(depthImg.data));
			/*fill image*/
			UINT size = 0;
			BYTE *buffer = nullptr;
			bodyFrame->AccessUnderlyingBuffer(&size, &buffer);
			for (int y = 0; y < height; ++y) {
				for (int x = 0; x < width; ++x) {
					int bodyIdx = buffer[x + y * width];
					if (bodyIdx < 6) {
						if (!isDetected) {
							cout << "Body detected" << endl;
							isDetected = true;
						}
						if (isBorder(buffer, x, y, width, height) == true) {
							img.at<cv::Vec3b>(y, x) = colorTable[0];
							/*switch (bodyIdx) {
							case 0:img.at<cv::Vec3b>(y, x) = colorTable[0];
							break;
							case 1:img.at<cv::Vec3b>(y, x) = colorTable[1];
							break;
							case 2:img.at<cv::Vec3b>(y, x) = colorTable[2];
							break;
							case 3:img.at<cv::Vec3b>(y, x) = colorTable[3];
							break;
							case 4:img.at<cv::Vec3b>(y, x) = colorTable[4];
							break;
							case 5:img.at<cv::Vec3b>(y, x) = colorTable[5];
							break;
							}*/
						}
						else {
							img.at<cv::Vec3b>(y, x) = colorTable[6];
						}
					}
					else {
						img.at<cv::Vec3b>(y, x) = colorTable[6];
					}

				}
			}
			///convert from 16bit to 8bit
			///depthImg.convertTo(img8bit, CV_8U, 255.0f / depthMax);
			cv::imshow("contour detect", img);
			//release frame
			bodyFrame->Release();
			if (cv::waitKey(30) == VK_ESCAPE) {
				break;
			}
		}
	}
	/*release frame reader*/
	bodyFrameReader->Release();
	bodyFrameReader = nullptr;
	/*close sensor*/
	sensor->Close();
	sensor->Release();
	sensor = nullptr;

	return 0;
}