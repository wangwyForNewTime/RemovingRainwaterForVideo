#include "stdafx.h"
#include <time.h>
#include "gargnayar.h"
//#pragma comment(lib,"opencv_img_hash344d.lib")
#pragma comment(lib,"opencv_world344.lib")
using namespace std;
using namespace cv;

//GargNayar除雨器
GargNayarRainRemover::GargNayarRainRemover(std::string inputVideo, std::string outputFolder, GNRainParameters rainParams)
{
	this->inputVideo = inputVideo;
	this->outputFolder = outputFolder;
	this->rainParams = rainParams;

	computeDirectionalMasks(); // Initialise the directional masks初始化定向掩膜
}
//使用参数来自Rain Removal in Traffic Surveillance: Does it Matter?这篇文章
//! Retrieves default parameters as used in the article "Rain Removal in Traffic Surveillance: Does it Matter?"
/*!
\return struct of default parameters

*/
GNRainParameters GargNayarRainRemover::getDefaultParameters()//获取默认参数
{
	GNRainParameters defaultParams;
	defaultParams.c = 3;//c是阈值表示在噪声存在下可以检测到的下降所引起的最小强度变化
	defaultParams.betaMax = 0.039;//斜率最大值
	defaultParams.neighborhoodSize = 11;//领域范围
	defaultParams.numCorrelationFrames = 30;//总相关帧数
	defaultParams.minDirectionalCorrelation.push_back(0.40);//最小定向相关
	defaultParams.maxDirectionalSpread.push_back(3);//最大定向分散
	defaultParams.numFramesReplacement = 2;//数字帧替换

	defaultParams.saveDiffImg = false;//存储差异图像
	defaultParams.saveOverviewImg = false; //存储概述图像
	defaultParams.useMedianBlur = false;//使用中值模糊
	defaultParams.noGNProcessing = false;//没有加工处理

	return defaultParams;
}

//! Loads parameters saved in a OpenCV FileStorage compatible format
//加载以OpenCV文件存储兼容格式保存的参数
/*!
\param filePath load the parameters from this path, either relative or full参数文件路径从此路径加载参数，无论是相对的还是绝对的
\return struct containing the loaded parameters包含加载参数的返回结构

*/
GNRainParameters GargNayarRainRemover::loadParameters(std::string filePath)//加载参数
{
	GNRainParameters newParams = getDefaultParameters();//获取默认参数
	FileStorage fs(filePath, FileStorage::READ);//文件存储

	if (fs.isOpened()) {
		int tmpInt;
		fs["c"] >> tmpInt;
		if (tmpInt != 0) {
			newParams.c = tmpInt;
		}

		double tmpDouble;
		fs["betaMax"] >> tmpDouble;
		if (tmpDouble != 0.) {
			newParams.betaMax = tmpDouble;
		}

		fs["neighborhoodSize"] >> tmpInt;
		if (tmpInt != 0) {
			newParams.neighborhoodSize = tmpInt;
		}

		fs["numCorrelationFrames"] >> tmpInt;
		if (tmpInt != 0) {
			newParams.numCorrelationFrames = tmpInt;
		}

		vector<float> tmpFloatVec;
		fs["minDirectionalCorrelation"] >> tmpFloatVec;
		if (!tmpFloatVec.empty()) {
			newParams.minDirectionalCorrelation = tmpFloatVec;
		}

		vector<int> tmpIntVec;
		fs["maxDirectionalSpread"] >> tmpIntVec;
		if (!tmpIntVec.empty()) {
			newParams.maxDirectionalSpread = tmpIntVec;
		}

		fs["numFramesReplacement"] >> tmpInt;
		if (tmpInt != 0) {
			newParams.numFramesReplacement = tmpInt;
		}

		fs["saveOverviewImg"] >> newParams.saveOverviewImg;
		fs["saveDiffImg"] >> newParams.saveDiffImg;
		fs["useMedianBlur"] >> newParams.useMedianBlur;
		fs["verbose"] >> newParams.verbose;
	}

	return newParams;
}

//! 以OpenCV文件存储兼容格式保存当前参数
/*!
\param filePath将参数保存到此路径，可以是相对的，也可以是完整的
\如果操作成功，则返回0，否则返回1
*/
int GargNayarRainRemover::saveParameters(std::string filePath)
{
	FileStorage fs(filePath, FileStorage::WRITE);

	if (fs.isOpened()) {
		fs << "c" << rainParams.c;
		fs << "betaMax" << rainParams.betaMax;
		fs << "neighborhoodSize" << rainParams.neighborhoodSize;
		fs << "numCorrelationFrames" << rainParams.numCorrelationFrames;
		fs << "minDirectionalCorrelation" << rainParams.minDirectionalCorrelation;
		fs << "maxDirectionalSpread" << rainParams.maxDirectionalSpread;
		fs << "numFramesReplacement" << rainParams.numFramesReplacement;

		fs << "saveOverviewImg" << rainParams.saveOverviewImg;
		fs << "saveDiffImg" << rainParams.saveDiffImg;
		fs << "useMedianBlur" << rainParams.useMedianBlur;
		fs << "verbose" << rainParams.verbose;

		return 0;
	}
	else {
		return 1;
	}
}

//! Retrieves next image from video and handles bookkeeping of current and previous frames
//从视频中检索下一个图像并处理当前帧和上一帧的记帐
/*!
\param cap handle to opened VideoCapture container参数cap为打开VideoCapture容器的句柄
\param prevImgs vector of previous images, containing at least four elements参数prevImgs为上一图像的参数向量，至少包含四个元素
\param currentImg matrix returned as the "current", time-shifted image参数currentImg为“当前”时移图像返回的矩阵
\param nextImgs vector of "next" images. New frames from the video capture are placed here, and propagated through the current image and previous images
\“下一个”图像的参数nextImgs向量。来自视频捕获的新帧被放在这里，并在当前图像和以前的图像中传播
\return if return == 0, the frame was retrieved successfully. If return == 1, no frame was retrieved
\return如果return==0，则成功检索帧。如果return==1，则没有检索到帧
*/

//找到下一帧
int GargNayarRainRemover::fetchNextImage(cv::VideoCapture &cap, std::map<int, std::vector<cv::Mat> > &prevImgs,
	std::map<int, cv::Mat> &currentImg, std::map<int, std::vector<cv::Mat> > &nextImgs)
{
	// Handle bookkeeping of frames处理框架记账
	prevImgs[Modality::color][3] = prevImgs[Modality::color][2].clone();
	prevImgs[Modality::grayscale][3] = prevImgs[Modality::grayscale][2].clone();
	prevImgs[Modality::color][2] = prevImgs[Modality::color][1].clone();
	prevImgs[Modality::grayscale][2] = prevImgs[Modality::grayscale][1].clone();
	prevImgs[Modality::color][1] = prevImgs[Modality::color][0].clone();
	prevImgs[Modality::grayscale][1] = prevImgs[Modality::grayscale][0].clone();

	prevImgs[Modality::color][0] = currentImg[Modality::color].clone();
	prevImgs[Modality::grayscale][0] = currentImg[Modality::grayscale].clone();
	currentImg[Modality::color] = nextImgs[Modality::color][0].clone();
	currentImg[Modality::grayscale] = nextImgs[Modality::grayscale][0].clone();

	nextImgs[Modality::color][0] = nextImgs[Modality::color][1].clone();
	nextImgs[Modality::grayscale][0] = nextImgs[Modality::grayscale][1].clone();

	nextImgs[Modality::color][1] = nextImgs[Modality::color][2].clone();
	nextImgs[Modality::grayscale][1] = nextImgs[Modality::grayscale][2].clone();

	cap >> nextImgs[Modality::color][2];

	if (nextImgs[Modality::color][2].empty()) {
		return 1;
	}

	cv::cvtColor(nextImgs[Modality::color][2], nextImgs[Modality::grayscale][2], COLOR_BGR2GRAY);

	return 0;
}

//! The main processing loop of the rain removal algorithm. Operates on the parameters given in rainParams of the class
//雨水去除算法的主处理回路。对类的rainParams中给定的参数进行操作
//主要处理循环消雨算法。操作类rainParams中给定的参数
int GargNayarRainRemover::removeRain()
{
	//把处理完成的视频存储起来
	VideoWriter writer1("C:\\Users\\Admin\\Desktop\\222\\test1.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 25, Size(1280, 720), 1);
	VideoWriter writer2("C:\\Users\\Admin\\Desktop\\222\\test2.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 25, Size(1280, 720), 1);
	VideoWriter writer3("C:\\Users\\Admin\\Desktop\\222\\test3.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 25, Size(1280, 720), 1);
	VideoWriter writer4("C:\\Users\\Admin\\Desktop\\222\\test4.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 25, Size(1280, 720), 1);
	// Open video打开视频文件
	VideoCapture cap(inputVideo);

	map<int, vector<GNRainImage>> rainImgs;//从最外层看，这是个map容器类型，它的键为int类型，值为vector<int>类型

	for (int method = GNMethod::fullMethod; method <= GNMethod::overview; ++method) {
		vector<GNRainImage> gnImages;

		string basePath;
		switch (method)
		{
		case GNMethod::fullMethod:
		{
			// The full method as described in Garg and Nayar in the paper 
			//本文中Garg和Nayar中描述的完整方法
			// "Detection and Removal of Rain from Videos"“从视频中检测和清除雨水”
			basePath = outputFolder + "/Full";
			break;
		}
		case GNMethod::candidatePixels://候选雨像素
		{
			// We consider all the candidate rain pixels to be rain pixels.
			//我们认为所有候选雨像素都是雨像素。
			// Conforms to step (a) of Figure 6 in page 5 in the paper 符合论文第5页图6步骤（a）
			basePath = outputFolder + "/Candidate/";
			break;
		}
		case GNMethod::photometricConstraint://光度约束
		{
			// We apply the photometric constraint to the candidate pixels
			// 我们对候选像素应用光度约束
			// Conforms to step (a) + (b) of Figure 6.
			basePath = outputFolder + "/Photometric/";
			break;
		}
		case GNMethod::correlationMagnitude://时空相关度
		{
			// We apply the spatio-temporal correlation magnitude on top of a
			// collection of detected rain streaks found by the photometric constraint
			// Conforms to step (a) + (b) + (c) + (d) of Figure 6. 
			// If we add step (e), the direction of correlation, we will get the full 
			// method
			//我们在a上应用时空相关幅度
			//收集根据光度限制发现的雨纹
			//符合图6中的步骤(a) + (b) + (c) + (d)。
			//如果我们加上步骤(e)，方向的相关性，我们将得到the full method

			basePath = outputFolder + "/STCorr/";
			break;
		}
		case GNMethod::overview:
		{
			//Produces a neat overview image of the computational steps listed above.
			//生成上面列出的计算步骤的简单概述图像
			basePath = outputFolder + "/Overview/";
			break;
		}
		}


		if (method == GNMethod::fullMethod) {
			// Insert (possibly) different range of rain parameters in the rainImage struct
			//在rainImage结构中插入（可能）不同范围的rain参数
			for (auto minDirCorr : rainParams.minDirectionalCorrelation) {
				for (auto maxDirSpread : rainParams.maxDirectionalSpread) {
					GNRainImage gnImage;
					gnImage.prevRainImgs.resize(2);
					gnImage.nextRainImgs.resize(2);
					gnImage.customParams = rainParams;

					gnImage.customParams.minDirectionalCorrelation.clear();
					gnImage.customParams.maxDirectionalSpread.clear();

					gnImage.customParams.minDirectionalCorrelation.push_back(minDirCorr);
					gnImage.customParams.maxDirectionalSpread.push_back(maxDirSpread);

					stringstream ssMinDirCorr, maxDir;
					ssMinDirCorr << setw(3) << setfill('0') << std::setprecision(2) << minDirCorr;

					gnImage.outputFolder = basePath + "dirCorr-" + ssMinDirCorr.str() +
						"-maxDir-" + to_string(maxDirSpread) + "/";

					gnImages.push_back(gnImage);
				}
			}

		}
		else {
			// Insert the modified rainImage with default parameters. Only one image struct is needed
			//使用默认参数插入修改后的rainImage。只需要一个图像结构
			GNRainImage gnImage;
			gnImage.prevRainImgs.resize(2);
			gnImage.nextRainImgs.resize(2);
			gnImage.customParams = rainParams;
			gnImage.outputFolder = basePath;
			gnImages.push_back(gnImage);
		}

		rainImgs[method] = gnImages;
	}



	for (int method = GNMethod::fullMethod; method <= GNMethod::overview; ++method) {
		for (auto&& param : rainImgs[method]) {
			CreateDirectoryA(param.outputFolder.c_str(), NULL);
		}
	}

	if (rainParams.useMedianBlur) {
		string dir = outputFolder + "/Median/";
		CreateDirectoryA(dir.c_str(), NULL);
	}

	if (cap.isOpened()) {
		cout << "Opening video: " << inputVideo << endl;

		// Extract the first five frames
		//提取前五帧
		map<int, Mat> currentImg;
		map<int, vector<Mat>> prevImgs, nextImgs;
		prevImgs[Modality::grayscale].resize(4);
		prevImgs[Modality::color].resize(4);
		nextImgs[Modality::grayscale].resize(3);
		nextImgs[Modality::color].resize(3);

		// Convert images to greyscale
		//转换图像到灰度
		cap >> prevImgs[Modality::color][2]; cv::cvtColor(prevImgs[Modality::color][2], prevImgs[Modality::grayscale][2], COLOR_BGR2GRAY);
		cap >> prevImgs[Modality::color][1]; cv::cvtColor(prevImgs[Modality::color][1], prevImgs[Modality::grayscale][1], COLOR_BGR2GRAY);
		cap >> prevImgs[Modality::color][0]; cv::cvtColor(prevImgs[Modality::color][0], prevImgs[Modality::grayscale][0], COLOR_BGR2GRAY);
		cap >> currentImg[Modality::color]; cv::cvtColor(currentImg[Modality::color], currentImg[Modality::grayscale], COLOR_BGR2GRAY);
		cap >> nextImgs[Modality::color][0]; cv::cvtColor(nextImgs[Modality::color][0], nextImgs[Modality::grayscale][0], COLOR_BGR2GRAY);
		cap >> nextImgs[Modality::color][1]; cv::cvtColor(nextImgs[Modality::color][1], nextImgs[Modality::grayscale][1], COLOR_BGR2GRAY);
		cap >> nextImgs[Modality::color][2]; cv::cvtColor(nextImgs[Modality::color][2], nextImgs[Modality::grayscale][2], COLOR_BGR2GRAY);

		deque<Mat> binaryFieldImages, candidatePixelImages, magnitudeImages;

		clock_t start0 = clock();

		// Process the needed number of frames in order to allow the computation of the spatio-temporal correlation
		// 处理所需的帧数，以便计算时空相关性
		for (auto i = 0; i < rainParams.numCorrelationFrames; ++i) {
			if (i != 0) {
				int retVal = fetchNextImage(cap, prevImgs, currentImg, nextImgs);

				if (retVal != 0) {
					cout << "Could not retrive frame " << i << ". Aborting." << endl;
					return 1;
				}
			}


			Mat candidatePixels; Mat binaryField;

			findCandidatePixels(prevImgs[Modality::grayscale][0], currentImg[Modality::grayscale], nextImgs[Modality::grayscale][0], candidatePixels);
			candidatePixelImages.push_back(candidatePixels);

			enforcePhotometricConstraint(candidatePixels, prevImgs[Modality::grayscale][0], currentImg[Modality::grayscale], binaryField);
			waitKey(0);

			binaryFieldImages.push_back(binaryField);
		}


		clock_t ends0 = clock();

		//leo
		cout << "处理所需的帧数 Running Time : " << ((double)(ends0 - start0) / CLOCKS_PER_SEC) * 1000 << endl;

		// Process the first five frames outside the great for-loop to populate the next/prev rain
		//处理大for循环之外的前五帧来填充下一个/prev rain
		// images  
		clock_t start1 = clock();
		for (auto i = 0; i < (rainParams.numFramesReplacement * 2 + 1); ++i) {
			Mat candidatePixels, currentMagnitudeImg;

			computeSTCorrelation(binaryFieldImages, currentMagnitudeImg, GNOptions::STZerothTimeLag);

			magnitudeImages.push_back(currentMagnitudeImg);

			for (int method = GNMethod::fullMethod; method <= GNMethod::correlationMagnitude; ++method) {
				for (auto&& param : rainImgs[method]) {
					param.prevRainImgs[1] = param.prevRainImgs[0].clone();
					param.prevRainImgs[0] = param.currentRainImg.clone();
					param.currentRainImg = param.nextRainImgs[0].clone();
					param.nextRainImgs[0] = param.nextRainImgs[1].clone();
				}
			}

			rainImgs[GNMethod::candidatePixels][0].nextRainImgs[1] = candidatePixelImages.back();
			rainImgs[GNMethod::photometricConstraint][0].nextRainImgs[1] = binaryFieldImages.back();
			rainImgs[GNMethod::correlationMagnitude][0].nextRainImgs[1] = magnitudeImages.back();

			for (auto&& param : rainImgs[GNMethod::fullMethod]) {
				computeCorrelationDirection(currentMagnitudeImg, param.nextRainImgs[1],
					param.customParams.minDirectionalCorrelation[0], param.customParams.maxDirectionalSpread[0]);
			}

			//Fetch a new image, and start over
			//获取一个新图像，然后重新开始

			int retVal = fetchNextImage(cap, prevImgs, currentImg, nextImgs);

			if (retVal != 0) {
				cout << "Could not retrive frame " << i + rainParams.numCorrelationFrames << ". Aborting." << endl;
				return 1;
			}

			Mat binaryField;
			findCandidatePixels(prevImgs[Modality::grayscale][0], currentImg[Modality::grayscale], nextImgs[Modality::grayscale][0], candidatePixels);
			candidatePixelImages.push_back(candidatePixels);
			candidatePixelImages.pop_front();

			enforcePhotometricConstraint(candidatePixels, prevImgs[Modality::grayscale][0], currentImg[Modality::grayscale], binaryField);
			binaryFieldImages.push_back(binaryField);
			binaryFieldImages.pop_front();
		}

		clock_t ends1 = clock();

		//leo
		cout << "处理大for循环之外的前五帧 Running Time : " << ((double)(ends1 - start1) / CLOCKS_PER_SEC) * 1000 << endl;



		int numImages = static_cast<int>(cap.get(CAP_PROP_FRAME_COUNT));

		// We now have enough processed images to remove the rain from the frames
		//我们现在有足够的处理后的图像来去雨了

		for (auto i = 0; i < (numImages - rainParams.numCorrelationFrames - (rainParams.numFramesReplacement * 2 + 1)); ++i) {
			int displayedFrameNumber = i + (rainParams.numFramesReplacement + 1) * 2 + rainParams.numCorrelationFrames;
			stringstream outFrameNumber;
			outFrameNumber << setw(5) << setfill('0') << displayedFrameNumber;
			Mat candidatePixels, currentMagnitudeImg;
			//leo
			cout << "Processing frame " << outFrameNumber.str() << endl;

			if (!rainParams.noGNProcessing)
			{
				clock_t startstc = clock();


				computeSTCorrelation(binaryFieldImages, currentMagnitudeImg, GNOptions::STZerothTimeLag);

				magnitudeImages.push_back(currentMagnitudeImg);
				magnitudeImages.pop_front();

				clock_t endsstc = clock();

				//leo
				cout << "computeSTCorrelation函数 Running Time : " << ((double)(endsstc - startstc) / CLOCKS_PER_SEC) * 1000 << endl;


				clock_t start2 = clock();

				for (int method = GNMethod::fullMethod; method <= GNMethod::correlationMagnitude; ++method) {
					for (auto&& param : rainImgs[method]) {
						param.prevRainImgs[1] = param.prevRainImgs[0].clone();
						param.prevRainImgs[0] = param.currentRainImg.clone();
						param.currentRainImg = param.nextRainImgs[0].clone();
						param.nextRainImgs[0] = param.nextRainImgs[1].clone();
					}
				}
				clock_t ends2 = clock();

				//leo
				cout << "两个for循环 Running Time : " << ((double)(ends2 - start2) / CLOCKS_PER_SEC) * 1000 << endl;

				rainImgs[GNMethod::candidatePixels][0].nextRainImgs[1] = candidatePixelImages.back();
				rainImgs[GNMethod::photometricConstraint][0].nextRainImgs[1] = binaryFieldImages.back();
				rainImgs[GNMethod::correlationMagnitude][0].nextRainImgs[1] = magnitudeImages.back();


				clock_t start3 = clock();

				for (auto&& param : rainImgs[GNMethod::fullMethod]) {
					computeCorrelationDirection(currentMagnitudeImg, param.nextRainImgs[1],
						param.customParams.minDirectionalCorrelation[0], param.customParams.maxDirectionalSpread[0]);
				}

				clock_t ends3 = clock();

				//leo
				cout << "for循环执行computeCorrelationDirection函数 Running Time: " << ((double)(ends3 - start3) / CLOCKS_PER_SEC) * 1000 << endl;

				// Remove detected rain. In order to remove the rain from the image, we require 
				// 删去检测到的雨
				// that rain has been detected in: frame-2, frame-1, frame, frame+1, frame+2, 
				//在第2帧、第1帧、第1帧、第 + 1帧、第2帧中检测到了降雨
				// or more/less controlled by the numFramesReplacement parameter.
				//或多或少由numFramesReplacement参数控制
				// If numFramesReplacement != 2 however, the code below must be changed to 
				//如果numFramesReplacement！=2但是，下面的代码必须改为
				// copy the vectors accordingly
				// 相应地复制矢量
				// The mapping between the images and the rain images is the following:
				//图像与雨图像之间的映射如下:
				// currentImage: nextRainImgs[1]
				// prevImgs[0]: nextRainImgs[0]
				// prevImgs[1]: currentRainImg
				// prevImgs[2]: prevRainImgs[0]or
				// prevImgs[3]: prevRainImgs[1]
				// We must remap the images accordingly in order to remove the rain
				//我们必须相应地重新映射图像，以便排除雨水
				std::vector<Mat> tmpPrevImgs, tmpNextImgs;
				tmpPrevImgs.push_back(prevImgs[Modality::color][2]);
				tmpPrevImgs.push_back(prevImgs[Modality::color][3]);
				tmpNextImgs.push_back(prevImgs[Modality::color][0]);
				tmpNextImgs.push_back(prevImgs[Modality::color][1]);
				map<int, Mat> rainRemovedImg;


				//使用完整的方法去除检测到的雨，候选体，大小图像，光度限制


				clock_t start4 = clock();

				for (int method = GNMethod::fullMethod; method <= GNMethod::correlationMagnitude; ++method) {
					for (auto&& param : rainImgs[method]) {
						removeDetectedRain(tmpPrevImgs, prevImgs[Modality::color][1], tmpNextImgs, param, rainRemovedImg[method]);
						cv::imwrite(param.outputFolder + outFrameNumber.str() + ".png", rainRemovedImg[method]);
						//imshow("dst", rainRemovedImg[method]);
						if (method == GNMethod::fullMethod)
							writer1.write(rainRemovedImg[method]);
						if (method == GNMethod::candidatePixels)
							writer2.write(rainRemovedImg[method]);
						if (method == GNMethod::photometricConstraint)
							writer3.write(rainRemovedImg[method]);
						if (method == GNMethod::correlationMagnitude)
							writer4.write(rainRemovedImg[method]);


						if (rainParams.saveDiffImg && (method != GNMethod::fullMethod)) {


							//计算中间输出的差分图像

							Mat diff;
							absdiff(rainRemovedImg[GNMethod::fullMethod], rainRemovedImg[method], diff);
							cv::imwrite(param.outputFolder + "diff-" + outFrameNumber.str() + ".png", diff * 255);
						}


					}
				}

				clock_t ends4 = clock();
				//leo
				cout << "计算差分图像 Running Time : " << ((double)(ends4 - start4) / CLOCKS_PER_SEC) * 1000 << endl;


				if (rainParams.saveOverviewImg) {
					Mat combinedImg = Mat(Size(currentImg[Modality::grayscale].cols * 3, currentImg[Modality::grayscale].rows * 2), currentImg[Modality::grayscale].type()); // CV_8UC1
					Mat upperLeft(combinedImg, Rect(0, 0, currentImg[Modality::grayscale].cols, currentImg[Modality::grayscale].rows));
					prevImgs[Modality::grayscale][1].copyTo(upperLeft);

					Mat upperMid(combinedImg, Rect(currentImg[Modality::grayscale].cols, 0, currentImg[Modality::grayscale].cols, currentImg[Modality::grayscale].rows));
					Mat rainRemovedImgGray;
					cv::cvtColor(rainRemovedImg[GNMethod::fullMethod], rainRemovedImgGray, COLOR_BGR2GRAY);
					rainRemovedImgGray.copyTo(upperMid);

					Mat upperRight(combinedImg, Rect(currentImg[Modality::grayscale].cols * 2, 0, currentImg[Modality::grayscale].cols, currentImg[Modality::grayscale].rows));
					rainImgs[GNMethod::fullMethod][0].currentRainImg.convertTo(upperRight, CV_8UC1, 100);

					Mat lowerLeft(combinedImg, Rect(0, currentImg[Modality::grayscale].rows, currentImg[Modality::grayscale].cols, currentImg[Modality::grayscale].rows));
					candidatePixelImages[candidatePixelImages.size() - 3].copyTo(lowerLeft);

					Mat lowerMid(combinedImg, Rect(currentImg[Modality::grayscale].cols, currentImg[Modality::grayscale].rows, currentImg[Modality::grayscale].cols, currentImg[Modality::grayscale].rows));
					binaryFieldImages[binaryFieldImages.size() - 3].copyTo(lowerMid);

					Mat lowerRight(combinedImg, Rect(currentImg[Modality::grayscale].cols * 2, currentImg[Modality::grayscale].rows, currentImg[Modality::grayscale].cols, currentImg[Modality::grayscale].rows));
					magnitudeImages[magnitudeImages.size() - 3].convertTo(lowerRight, CV_8UC1, 100);
					//prevMagnitudeImgs[1].convertTo(lowerRight, CV_8UC1, 100);
					cv::imwrite(rainImgs[GNMethod::overview][0].outputFolder + outFrameNumber.str() + ".png", combinedImg);
				}
			}


			if (rainParams.useMedianBlur) {
				Mat medBlur;
				medianBlur(prevImgs[Modality::color][1], medBlur, 3);
				cv::imwrite(outputFolder + "/Median/" + outFrameNumber.str() + ".png", medBlur);
			}

			//抓取一个新帧，重新开始处理

			int retVal = fetchNextImage(cap, prevImgs, currentImg, nextImgs);

			if (retVal != 0) {
				cout << "Could not retrive frame " << i + rainParams.numCorrelationFrames + rainParams.numFramesReplacement * 2 + 1 << ". Aborting." << endl;
				return 0;
			}

			if (!rainParams.noGNProcessing)
			{
				Mat binaryField;

				findCandidatePixels(prevImgs[Modality::grayscale][0], currentImg[Modality::grayscale], nextImgs[Modality::grayscale][0], candidatePixels);
				candidatePixelImages.push_back(candidatePixels);
				candidatePixelImages.pop_front();

				//imwrite("Out/CandidatePixels-" + std::to_string(i + 1 + rainParams.numCorrelationFrames) + ".png", candidatePixels);

				enforcePhotometricConstraint(candidatePixels, prevImgs[Modality::grayscale][0], currentImg[Modality::grayscale], binaryField);
				//imwrite("Out/PConstraint-" + std::to_string(i + 1 + rainParams.numCorrelationFrames) + ".png", binaryField);

				binaryFieldImages.push_back(binaryField);
				binaryFieldImages.pop_front();
			}
		}

	}
	else {
		cout << "Could not open video " << inputVideo << endl;
		return 1;
	}

	return 0;
}

//! Find candidate rain pixels from the grayscale input images
//从灰度输入图像中寻找候选雨像素
/*!
\param prevImg previous image, frame n - 1
\param currentImg current image, frame n
\param nextImg next image, frame n + 1
\param outImg returned image with the found candidate rain pixels
*/
void GargNayarRainRemover::findCandidatePixels(cv::Mat prevImg, cv::Mat currentImg, cv::Mat nextImg, cv::Mat &outImg)
{//找候选雨像素
	assert(prevImg.size() == currentImg.size());
	assert(nextImg.size() == currentImg.size());

	outImg = Mat::zeros(currentImg.size(), CV_8UC1);

	Mat prevDiff, nextDiff;
	subtract(currentImg, prevImg, prevDiff);
	subtract(currentImg, nextImg, nextDiff);

	//如果之前的差值等于下一个像素级的差值，
	// prevDiff/nextDiff中的像素值大于一个参数，
	//把这个标记为雨水候选

	Mat diffEquality, invDiffEquality;
	absdiff(prevDiff, nextDiff, diffEquality); // if prevDiff == nextDiff, 0
	threshold(diffEquality, invDiffEquality, 0, 255, THRESH_BINARY_INV); // if prevDiff == nextDiff, 255


																		 // 只保留prevDiff中等于nextDiff的值

	Mat maskedDiff;
	prevDiff.copyTo(maskedDiff, invDiffEquality);

	// Check prevDiff >= rainParams.c.
	// If this applies, the intensity change just occurs for this frame and is 
	// bright enough to be detected as a rain chandidate.  如果这适用的话，强度变化只会发生在这个帧上，并且亮度足够高，可以被探测到是一场雨。
	// Mark this in the out image, outImg.at<uchar>(y, x) = 255，在输出图像中标记这个
	inRange(maskedDiff, rainParams.c, 255, outImg);
}


//线性光度约束

/*!
\param prevImg previous image, frame n - 1
\param currentImg current image, frame n
\param nextImg next image, frame n + 1
\param outImg returned image with the extracted rain streaks
*/
void GargNayarRainRemover::enforcePhotometricConstraint(cv::Mat inBinaryImg, cv::Mat prevImg, cv::Mat currentImg, cv::Mat &outImg)
{
	// Enforce the linear photometric constraint introduced by Garg and Nayar, 2004


	//查找二元图像的连接组件(单个雨条纹)

	Mat inputImage = inBinaryImg.clone();
	vector<vector<Point> > contours;

	findContours(inputImage, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);


	//通过一阶线性模型计算像素的匹配度来过滤连接的单个雨条纹。

	outImg = inBinaryImg.clone();
	int deletedContours = 0;

	for (auto i = 0; i < contours.size(); ++i) {
		if (contours[i].size() > 1) {

			//如果轮廓线大于1像素，检查
			//在这种情况下，解线性方程组(可能是超定的)
			//对于轮廓中的每个像素:
			//   deltaI = -beta * prevImg + alpha
			//   where:
			//     deltaI = currentImg - prevImg
			//     beta, alpha = parameters that should be estimated
			//
			// We can write the linear equation in a matrix form:
			//              ( beta  )
			// (-prevImg 1) ( alpha ) = deltaI
			// 
			// or: 
			//   ( beta  )
			// A ( alpha ) = deltaI
			// 
			// where:
			//    A      = (-prevImg 1), a j,2 matrix
			//    deltaI : a j,1 matrix
			//
			// The equation is solved by least squares by:
			// ( beta )
			// ( alpha ) = (A^T A)^(-1) A^T deltaI

			Mat A = Mat::ones(Size(2, static_cast<int>(contours[i].size())), CV_32FC1);
			Mat deltaI = Mat::ones(Size(1, static_cast<int>(contours[i].size())), CV_32FC1);


			//用图像中的值填充矩阵A和deltaI
			for (auto j = 0; j < contours[i].size(); ++j) {
				A.at<float>(j, 0) = -static_cast<float>(prevImg.at<uchar>(contours[i][j].y, contours[i][j].x));

				int prevDiff = currentImg.at<uchar>(contours[i][j].y, contours[i][j].x) -
					prevImg.at<uchar>(contours[i][j].y, contours[i][j].x);
				deltaI.at<float>(j, 0) = static_cast<float>(prevDiff);
			}

			//找最合适的

			// Compute(A^T A)^(-1) A^T deltaI
			Mat estimate = (A.t() * A).inv() * A.t() * deltaI;
			float beta = estimate.at<float>(0, 0);

			// If the estimated beta is above the threshold, we should discard the current streak.
			//如果估计的beta值高于阈值，我们应该丢弃当前的连续记录。
			// Otherwise, keep the streak
			if (estimate.at<float>(0, 0) > rainParams.betaMax) {
				// Make sure the entire contour is deleted from the output image
				//确保从输出图像中删除整个轮廓
				for (auto j = 0; j < contours[i].size(); ++j) {
					outImg.at<uchar>(contours[i][j].y, contours[i][j].x) = 0;
				}

				++deletedContours;
			}
		}

		// If the contour just consists of one point, there is no point in checking, and we might thus
		// let the contour pass the test. Because we have copied the binary input image, we need not to do anything        
	    //如果轮廓只由一个点组成，则没有检查点，因此我们可以让轮廓通过测试。因为我们已经复制了二进制输入图像，所以我们不需要做任何事情
	}

	if (rainParams.verbose) {
		cout << "PMConstraint: Found " << contours.size() << " contours in rain candidate image, and deleted " << deletedContours << " of those " << endl;
	}
}

//! Compute the spatio-temporal correlation of a collection of rain streak images
//计算一组雨斑图像的时空相关性
/*!
\param binaryFieldHistory extracted rain streaks under the photometric constraint from the previous n frames
\param outImg filtered rain image (binary) subject to the spatio-temporal correlation of the provided history
\param method Time lag method. Use GNOptions:STZerothTimeLag to reproduce the original results by Garg and Nayar
*/
void GargNayarRainRemover::computeSTCorrelation(std::deque<cv::Mat> binaryFieldHistory, cv::Mat &outImg, int method)
{
	// Compute the spatio-temporal correlation of the binary field history
	//计算二元场历史的时空相关性
	assert(static_cast<int>(binaryFieldHistory.size()) == rainParams.numCorrelationFrames);

	if (rainParams.numCorrelationFrames <= 0) {
		return;
	}

	outImg = Mat::zeros(binaryFieldHistory.front().size(), CV_32FC1);

	// Compute the spatial correlation for each binary field, and add them to 
	// the temporary spatio-temporal image
	//计算每个二值场的空间相关性，并将其添加到临时时空图像中
	for (auto it = binaryFieldHistory.cbegin(); it != binaryFieldHistory.cend(); ++it) {
		Mat bField;

		if (method == GNOptions::STZerothTimeLag) {
			// Add the spatial correlation to the output, spatio-temporal image to produce the 
			// zero'th order temporal correlation as presented in the original paper
			//将空间相关性加入到输出的时空图像中，产生如原论文所述的零阶时间相关
			bField = *it;
		}
		else if (method == GNOptions::STVaryingTimeLag) {
			// Use an alternative interpretation of the the spatio-temporal correspondence 
			// what uses a varying time lag - thus, use the binary field of the 'current' image
			// to multiply in the end of the for loop
			//使用时空对应关系的另一种解释，即使用可变的时间滞后——所以，使用“当前”图像的二进制字段在for循环的末尾进行乘法
			bField = binaryFieldHistory.back();
		}


		// Create a scaled version of the binary field where the only values are 0,1
		//创建二进制字段的缩放版本，其中值仅为0,1
		Mat sBField;
		threshold(bField, sBField, 254, 1, THRESH_BINARY); // If val > threshold, then 1, else 0

														   // Compute the spatial correlation for all pixels of the current image using a box filter
														   // The correlation matrix is used subsequently to compute the correlation Rb
		                                                   //使用盒形滤波器计算当前图像的所有像素的空间相关性随后使用相关矩阵来计算相关性Rb
		Mat filteredImg = Mat::zeros(bField.size(), bField.type());
		int window = rainParams.neighborhoodSize;

		boxFilter(sBField, filteredImg, -1, Size(window, window), Point(-1, -1), false);

		// Perform a per-element multiplication of the scaled binary field and the 
		// correlation matrix to compute Rb for the particular instance of the binary field
		//执行缩放二进制字段和相关矩阵的每元素乘法，以计算二进制字段的特定实例的Rb
		Mat spatialCorr;
		multiply(sBField, filteredImg, spatialCorr);

		add(spatialCorr, outImg, outImg, noArray(), outImg.depth());
	}

	// Average output image with a 3x3 mean filter
	//使用3x3平均滤波器平均输出图像
	Mat nonBlurImg = outImg.clone();
	blur(nonBlurImg, outImg, Size(3, 3), Point(-1, -1), BORDER_REPLICATE);

	// We do not normalize the output by the term 1/L as presented in equation 3 in (Garg, Nayar).
	// However, this only affects the magnitude of the spatio-temporal correlation linearly, which 
	// is compensated by adjusting the rainParams.minDirectionalCorrelation accordingly
	//我们没有用1/L项对输出进行归一化，如（Garg，Nayar）中的等式3所示。然而，这只会线性地影响时空相关性的大小，通过调整rainParams.mindirectional相关因此
}


//! Compute the directional correlation of the spatio-temporal correlation. This will exclude non-streak-like rain streaks
//计算时空相关的方向相关。这将排除非条纹状雨条纹
/*!
Prior to calling this method, the directional masks should have been generated by calling computeDirectionalMasks()

\param ingImg binary image as output by computeSTCorrelation
\param outImg filtered rain image
\param minDirectionalCorrelation a rain pixel should have a directional correlation over this threshold in order to remain a rain streak
\param maxDirectionalSpread the directional correlation of a rain pixel should not vary more than this threshold

计算时空相关的方向相关。这将排除非条纹状的雨条纹。
在调用此方法之前，应该通过调用computedirectionalmask()生成方向掩码
将param ingImg二进制图像作为computeSTCorrelation的输出
param outImg过滤雨图像，一个雨像素应该有一个超过这个阈值的方向相关，以便保持一个雨条纹
降雨像素的方向相关性不应该超过这个阈值
*/
void GargNayarRainRemover::computeCorrelationDirection(cv::Mat inImg, cv::Mat &outImg,
	float minDirectionalCorrelation, int maxDirectionalSpread)
{
	// Find the correlation of the spatio-temporal image with the directional masks
	//找出时空图像与方向掩模的相关性
	// Use template mathing for each pixel/area in the input image
	//对输入图像中的每个像素/区域使用模板mathing

	vector<Mat> matchResults;

	clock_t start7 = clock();
	for (auto i = 0; i < directionalMasks.size(); ++i) {
		Mat result;
		matchTemplate(inImg, directionalMasks[i], result, TM_CCORR_NORMED);
		matchResults.push_back(result);
		//cv::imwrite("matchRes-" + std::to_string(i) + ".png", result*100);
	}
	clock_t ends7 = clock();
	//cout << "computeCorrelationDirection里第一个for循环Running Time : " << ((double)(ends7 - start7) / CLOCKS_PER_SEC) * 1000 << endl;

	// Go through every pixel in the matched images and find areas where:
	// a) The correlation is weak (defined by arbitrary parameter)
	// b) There is consistency in the directional correlation
	// 遍历匹配图像中的每个像素，找到以下区域:
	// a)相关性较弱(由任意参数定义)
	// b)方向相关具有一致性

	outImg = inImg.clone();

	int discardedRainPixels = 0;

	int templateBoundary = (rainParams.neighborhoodSize - 1) / 2;
	//neighborhoodSize用于计算时间相关性的pixels


	clock_t start8 = clock();
	for (auto x = templateBoundary; x < inImg.cols - templateBoundary; x += 2) {
		clock_t start11 = clock();
		for (auto y = templateBoundary; y < inImg.rows - templateBoundary; y += 2) {
			if (inImg.at<float>(y, x) != 0) {
				// Only proceed for candidate rain pixels
				// 只处理候选雨像素
				map<int, float> correlations; float maxCorrelation = 0;
				int maxCorrelationIndex = -1;
				int matchX = x - templateBoundary;
				int matchY = y - templateBoundary;

				for (auto i = 0; i < directionalMasks.size(); ++i) {
					if (matchResults[i].at<float>(matchY, matchX) > minDirectionalCorrelation) {
						correlations[i] = matchResults[i].at<float>(matchY, matchX);

						if (matchResults[i].at<float>(matchY, matchX) > maxCorrelation) {
							maxCorrelation = matchResults[i].at<float>(matchY, matchX);
							maxCorrelationIndex = i;
						}
					}
				}

				// From the correlations, figure out if this pixel does not correlate to a rain pixel
				//从相关性中，找出这个像素是否与雨像素不相关
				bool rainPixel = true;

				if (maxCorrelation == 0) {
					// No correlation was above the minimum criteria. Delete this pixel
					// 没有相关性高于最低标准。删除这个像素
					rainPixel = false;
				}
				else {
					// The correlation was above the criteria. Check if there is consistency 
					// in the directions. This means, that any other correlation in the 
					// correlations map must be within (param)*10 degrees of the largest 
					// correlation value
					// Go through the correlation maps and check for this
					/*
					相关性高于标准。检查是否有一致性的方向。这意味着，在
					相关性图必须在(参数)*10度的最大相关值通过相关映射进行检查
					*/

					for (auto const &ent1 : correlations) {
						if (std::abs(ent1.first - maxCorrelationIndex) > maxDirectionalSpread) {
							rainPixel = false;
							break;
						}
					}

				}

				if (!rainPixel)
				{
					// The current pixel is disqualified as a rain pixel. Delete it from the output map
					//当前像素为不合格的雨像素。从输出映射中删除它
					outImg.at<float>(y, x) = 0;
					++discardedRainPixels;
				}

				// If the current pixel is qualified as a rain pixel (hurray!), we don't need to do anything, 
				// 如果当前像素被限定为雨像素，我们不需要做任何事情，
				// as we have cloned the input image
			}
		}
		clock_t ends11 = clock();
		//cout << "computeCorrelationDirection循环5Running Time : " << ((double)(ends11 - start11) / CLOCKS_PER_SEC) * 1000 << endl;
	}

	clock_t ends8 = clock();
	//cout << "computeCorrelationDirection里第二个for循环（多层）Running Time : " << ((double)(ends8 - start8) / CLOCKS_PER_SEC) * 1000  << endl;

	if (rainParams.verbose) {
		cout << "Discarded " << discardedRainPixels << " pixels from ST rain image" << endl;
	}


}

//! Compute the directional masks.计算方向掩模 
void GargNayarRainRemover::computeDirectionalMasks()
{
	// Construct 18 (the paper says 17, but the range from {0, 10, ... 170} indicates that we need 18)
	// oriented binary masks that helps compute the correlation direction
	//构造18（论文说17，但是范围是{0，10。。。170}表示我们需要18）
	//有助于计算相关方向的定向二进制掩码
	Mat horizontalMask = Mat::zeros(Size(rainParams.neighborhoodSize, rainParams.neighborhoodSize), CV_32FC1);
	cv::line(horizontalMask, Point(0, (rainParams.neighborhoodSize - 1) / 2), Point(rainParams.neighborhoodSize, (rainParams.neighborhoodSize - 1) / 2), Scalar(255));
	directionalMasks.push_back(horizontalMask);

	// Rotate the horizontal mask by steps of 10 degrees
	//将水平遮罩旋转10度
	for (auto i = 1; i < 18; ++i) {
		Mat rot = getRotationMatrix2D(Point((rainParams.neighborhoodSize - 1) / 2, (rainParams.neighborhoodSize - 1) / 2), i * 10, 1);
		Mat rotMask;

		warpAffine(horizontalMask, rotMask, rot, horizontalMask.size());
		directionalMasks.push_back(rotMask);
	}
}

//去除检测到的雨像素
//! Remove rain from the current image if rain is detected using the current rain image. 
//! If there is rain detected at a pixel in the current rain image, look at the previous and next rain image. 
//! If there is no rain in these, use the averaged values
//! of these images to replace the rain-affected pixel.
//如果使用当前雨图像检测到下雨，请从当前图像中移除雨水。如果在当前雨图像中的某个像素处检测到雨水，请查看上一个和下一个雨图像。如果这些图像中没有雨水，请使用这些图像的平均值来替换受雨水影响的像素。
void GargNayarRainRemover::removeDetectedRain(std::vector<cv::Mat> prevImgs, cv::Mat currentImg, std::vector<cv::Mat> nextImgs,
	GNRainImage rainImage, cv::Mat &rainRemovedCurrentImg)
{


	assert(prevImgs.size() == nextImgs.size() && rainImage.prevRainImgs.size() == rainImage.nextRainImgs.size());

	for (auto i = 0; i < prevImgs.size(); ++i) {
		assert(prevImgs[i].size().height == currentImg.size().height && nextImgs[i].size().height == rainImage.currentRainImg.size().height);
		assert(prevImgs[i].size().width == currentImg.size().width && nextImgs[i].size().width == rainImage.currentRainImg.size().width);
		assert(rainImage.prevRainImgs[i].size().height == rainImage.currentRainImg.size().height && rainImage.currentRainImg.size().height == rainImage.nextRainImgs[i].size().height);
		assert(rainImage.prevRainImgs[i].size().width == rainImage.currentRainImg.size().width && rainImage.currentRainImg.size().width == rainImage.nextRainImgs[i].size().width);
	}

	currentImg.copyTo(rainRemovedCurrentImg);

	// Beware; Duplicate code due to different type specifiers!

	if (rainImage.currentRainImg.type() == CV_8UC1) {
		for (auto x = 0; x < currentImg.cols; ++x) {
			for (auto y = 0; y < currentImg.rows; ++y) {
				if (rainImage.currentRainImg.at<uchar>(y, x) != 0) {
					// Rain has been detected in the current pixel在当前像素中检测到雨
					// Find a previous + next pixel to replace it

					// Make sensible defaults if all previous/next pixels are affected by rain 
					//如果所有上一个/下一个像素都受雨水影响，请设置合理的默认值
					Vec3b replacementPrev = prevImgs[0].at<Vec3b>(y, x);
					Vec3b replacementNext = nextImgs[0].at<Vec3b>(y, x);

					for (auto i = 0; i < rainImage.prevRainImgs.size(); ++i) {
						if (rainImage.prevRainImgs[i].at<uchar>(y, x) == 0) {
							// No rain detected. Use this pixel没有下雨。使用此像素
							replacementPrev = prevImgs[i].at<Vec3b>(y, x);
							break;
						}
					}

					for (auto i = 0; i < rainImage.nextRainImgs.size(); ++i) {
						if (rainImage.nextRainImgs[i].at<uchar>(y, x) == 0) {
							// No rain detected. Use this pixel没有下雨。使用此像素
							replacementNext = nextImgs[i].at<Vec3b>(y, x);
							break;
						}
					}

					// Average the proposed previous and next pixels and replace the current pixel
					//平均建议的前一个和下一个像素，并替换当前像素
					rainRemovedCurrentImg.at<Vec3b>(y, x)[0] = (replacementPrev[0] + replacementNext[0]) / 2;
					rainRemovedCurrentImg.at<Vec3b>(y, x)[1] = (replacementPrev[1] + replacementNext[1]) / 2;
					rainRemovedCurrentImg.at<Vec3b>(y, x)[2] = (replacementPrev[2] + replacementNext[2]) / 2;
				}
			}
		}
	}
	else if (rainImage.currentRainImg.type() == CV_32FC1) {
		for (auto x = 0; x < currentImg.cols; ++x) {
			for (auto y = 0; y < currentImg.rows; ++y) {
				if (rainImage.currentRainImg.at<float>(y, x) != 0) {
					// Rain has been detected in the current pixel在当前像素中检测到雨
					// Find a previous + next pixel to replace it

					// Make sensible defaults if all previous/next pixels are affected by rain
					//如果所有上一个/下一个像素都受雨水影响，请设置合理的默认
					Vec3b replacementPrev = prevImgs[0].at<Vec3b>(y, x);
					Vec3b replacementNext = nextImgs[0].at<Vec3b>(y, x);

					for (auto i = 0; i < rainImage.prevRainImgs.size(); ++i) {
						if (rainImage.prevRainImgs[i].at<float>(y, x) == 0) {
							// No rain detected. Use this pixel
							replacementPrev = prevImgs[i].at<Vec3b>(y, x);
							break;
						}
					}

					for (auto i = 0; i < rainImage.nextRainImgs.size(); ++i) {
						if (rainImage.nextRainImgs[i].at<float>(y, x) == 0) {
							// No rain detected. Use this pixel
							replacementNext = nextImgs[i].at<Vec3b>(y, x);
							break;
						}
					}

					// Average the proposed previous and next pixels and replace the current pixel
					//平均建议的前一个和下一个像素，并替换当前像素
					rainRemovedCurrentImg.at<Vec3b>(y, x)[0] = (replacementPrev[0] + replacementNext[0]) / 2;
					rainRemovedCurrentImg.at<Vec3b>(y, x)[1] = (replacementPrev[1] + replacementNext[1]) / 2;
					rainRemovedCurrentImg.at<Vec3b>(y, x)[2] = (replacementPrev[2] + replacementNext[2]) / 2;
				}
			}
		}
	}

}


int main(int argc, char** argv)
{
	clock_t start = clock(); //clock_t是一个长整形数，clock() 是C/C++中的计时函数，而与其相关的数据类型是clock_t
	//简单而言，就是该程序从启动到函数调用占用CPU的时间。这个函数返回从“开启这个程序进程”到“程序中调用clock()函数”时之间的CPU时钟计时单元（clock tick）数，在MSDN中称之为挂钟时间（wal-clock）；若挂钟时间不可取，则返回-1。其中 clock_t 是用来保存时间的数据类型。
	
	const string keys =
		"{help h            |       | Print help message}"
		"{fileName fn       |       | Input video file}"
		"{outputFolder of   |       | Output folder of processed files}"//已处理文件的输出文件夹
		"{settingsFile      |       | File from which settings are loaded/saved}"//从中加载/保存设置的文件
		"{saveSettings s    | 0     | Save settings to settingsFile (0,1)}"//将设置保存到设置文件（0.1）
		"{overviewImg oi    | 0     | Generate overview images of deraining process (0,1)}"//生成减少雨滴过程的大概图像
		"{diffImg di        | 0     | Generate difference frames from intermediate steps of algorithm (0,1)}"//从算法中间步骤生成差分帧
		"{medianBlur mb     | 0     | Generate derained frames using basic mean blur (3x3 kernel) (0,1)}"//使用基本平均模糊（3x3核）（0,1）生成减少雨滴帧
		"{noGNProcessing    | 0     | Disables the entire Garg-Nayar processing and only allows median blur (0,1)}"//禁用整个Garg-Nayar处理，只允许中值模糊
		"{verbose v         | 0     | Write additional debug information to console (0,1)}"//将其他调试信息写入控制台
		;

	cv::CommandLineParser cmd(argc, argv, keys);//命令行解析类
	cmd.about("Reimplementation of the method from Garg and Nayar in \"Detection and Removal of Rain From Videos\".");
	//Garg和Nayar方法在“视频雨水检测与去除”中的再实现
	/*
	if (argc <= 1 || (cmd.has("help"))) {
	std::cout << "Usage: " << argv[0] << " [options]" << std::endl;
	std::cout << "Available options:" << std::endl;
	cmd.printMessage();
	return 1;
	}
	*/

	//	std::string filename = cmd.get<std::string>("C:\\Users\\Administrator\\Desktop\\mag.avi");
	//	std::string outputFolder = cmd.get<std::string>("C:\\Users\\Administrator\\Desktop\\");

	//std::string filename = cmd.get<std::string>("fileName");
	std::string outputFolder = cmd.get<std::string>("outputFolder");
	//从命令行中获得outputFolder的字符串。命令行中有个outputFolder的参数，用这一句可以得到outputFolder的数据。

	//filename = "mag.avi";
	outputFolder = "C:\\Users\\Admin\\Desktop\\222";//处理后的文件存储在此

	GNRainParameters defaultParams = GargNayarRainRemover::getDefaultParameters();//获取默认参数

	// Load settings if settingsFile exists如果设置文件存在，则加载设置
	if (cmd.has("settingsFile")) {
		defaultParams = GargNayarRainRemover::loadParameters(cmd.get<std::string>("settingsFile"));//加载参数
	}

	defaultParams.saveOverviewImg = cmd.get<int>("overviewImg") != 0;
	defaultParams.useMedianBlur = cmd.get<int>("medianBlur") != 0;
	defaultParams.saveDiffImg = cmd.get<int>("diffImg") != 0;
	defaultParams.verbose = cmd.get<int>("verbose") != 0;
	defaultParams.noGNProcessing = cmd.get<int>("noGNProcessing") != 0;

	string s = "C:\\Users\\Admin\\Desktop\\rain2.mp4";//加载需要处理的视频
	GargNayarRainRemover gNRainRemover(s/*"rain2.mp4"*/, outputFolder, defaultParams);//进行处理

	// Save final settings保存最终设置
	if (cmd.has("settingsFile") && (cmd.get<int>("saveSettings") != 0)) {
		gNRainRemover.saveParameters(cmd.get<std::string>("settingsFile"));
	}

	clock_t ends = clock();

	cout << "Running Time1 : " << (double)(ends - start) / CLOCKS_PER_SEC << endl;

	gNRainRemover.removeRain();//进行处理




}