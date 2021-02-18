#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>  

using namespace std;
using namespace cv;


void writeToText(ofstream& fileout, Mat imageToWrite);

void createXYZ(ofstream& fileout, Mat imageToWrite, int layer);

void createEdgeConnectedCrackObj(ofstream& fileout, std::vector<std::vector<Point>> surface, int z, Point* cornerPoints);

Mat getGreenMachinedBorder(Mat imageToWrite, Point* cornerPoints);

std::vector<Point> removeDuplicatesFromContour(std::vector<Point> input, Mat& contouredMat);

int main()
	{
	
	int id = 1;
	std::string crackPath = "railCleansed/L" + to_string(id) + ".png";

	std::string borderPath = "BorderedCracks/L" + to_string(id) + ".png";

	std::string outputPath = to_string(id);
	cv::Mat input = imread(("Images/" + crackPath).c_str());
	std::ofstream fOut("Images/xyzLayers/Layer " + outputPath + ".obj");

	cv::Mat final, thresh, inverted, contouredFinal;

	//BIBLUR METHOD MATS
	cv::Mat blue, biblur, at, canny, morphed, difference;

	//Mask which stores the rectangular machined area and the 4 corner points
	cv::Mat borderMask;
	Point borderCorners[4];

	int sigma = 3;
	int kernel = 2 * ceil(2 * sigma) + 1;

	//first vector determines layer, 2nd vector determines which of the cracks on that layer
	//3rd vector determines of the selected crack on the specific layer coordinates.
	std::vector<std::vector<std::vector<Point>>> cracks;

	vector<std::vector<Point>> layerContours;


	while (!input.empty()) 
	{

		// Insert here
		borderMask = imread(("Images/" + borderPath).c_str());
		cvtColor(borderMask, borderMask, COLOR_BGR2HSV);

		cvtColor(input, input, COLOR_BGR2HSV);
		cv::imshow("input", input);

		borderMask = getGreenMachinedBorder(borderMask, borderCorners);
		cv::imshow("border", borderMask);
		cv::waitKey();

		inRange(input, Scalar(0, 220, 190), Scalar(179, 255, 255), thresh);
		cv::imshow("base", thresh);

		// Get only the edge pixels using morph operations
		bitwise_not(thresh, inverted);
		cv::imshow("inverted", inverted);
		cv::waitKey();

		morphologyEx(thresh, final, MORPH_DILATE, getStructuringElement(MORPH_CROSS, Size(3, 3)));
		cv::imshow("morph", final);

		bitwise_and(inverted, final, final);
		bitwise_and(borderMask, final, final);
		cv::waitKey();
		//Enclose the cracks with 8-connectivity using findcontours
		findContours(final, layerContours, RETR_EXTERNAL, CHAIN_APPROX_NONE);

		//For visualization purposes/comparing can see the enclosing contour
		cv::cvtColor(final, contouredFinal, COLOR_GRAY2BGR);
		
		//bitwise_or(final, borders, final);
		cv::imshow("final", final);
		cv::waitKey();
		// End here
		for (size_t i = 0; i < layerContours.size(); i++)
		{
			layerContours.at(i) = removeDuplicatesFromContour(layerContours.at(i), final);
		}

		cv::drawContours(contouredFinal, layerContours, -1, Scalar(255, 255, 0), 1);
		cv::imshow("contouredFinal", contouredFinal);

		//createXYZ(fOut, borders, (id-1)*5);
		createEdgeConnectedCrackObj(fOut, layerContours, (id - 1) * 6, borderCorners);
		fOut.close();
		bitwise_and(inverted, borderMask, borderMask);
		cv::imshow("wepio", borderMask);


		//For creating the layers
		fOut = ofstream("Images/xyzLayers/Layer " + outputPath + ".xyz");
		createXYZ(fOut, borderMask, 0);

		fOut.close();

		id++;

		crackPath = "railCleansed/L" + to_string(id) + ".png";
		borderPath = "BorderedCracks/L" + to_string(id) + ".png";

		outputPath = to_string(id);

		fOut = std::ofstream("Images/xyzLayers/Layer " + outputPath + ".obj");
		input = imread(("Images/" + crackPath).c_str());
		
	}

	cv::waitKey();
	}


void writeToText(ofstream& fileout, Mat imageToWrite)
{
	int rows = imageToWrite.rows - 1;
	int cols = imageToWrite.cols - 1;
	for (int i = 0; i <= rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			fileout << (int)imageToWrite.at<uchar>(i, j) << ",";
		}
		fileout << "; " << endl;
	}
}

void createXYZ(ofstream& fileout, Mat imageToWrite, int layer)
{
	int rows = imageToWrite.rows - 1;
	int cols = imageToWrite.cols - 1;
	// Set the 4 corners of the image pixels
	/*
	fileout << 0 << " " << 0 << " " << to_string(layer) << endl;
	fileout << 0 << " " << rows<< " " << to_string(layer) << endl;
	fileout << cols << " " << 0 << " " << to_string(layer) << endl;
	fileout << cols << " " << rows << " " << to_string(layer) << endl;
	*/

	for (int i = 0; i < rows; i++)
	{
		for (int j = 0; j < cols; j++)
		{
			if ((int)imageToWrite.at<uchar>(i, j) == 255) 
			{ fileout << to_string(j) << " " << to_string(-i) << " " << to_string(layer) << endl;  }
		}
	}
}

void createEdgeConnectedCrackObj(ofstream& fileout, std::vector<std::vector<Point>> surface, int z, Point* cornerPoints)
{
	//for now assume start is top left and end is bottom right, this iterator will loop through the points that make each crack
	std::vector<Point>::iterator pointIt;
	// A vector of vector of points, i.e all the cracks on a particular surface, this iterator will loop through the different cracks on a surface
	std::vector<std::vector<Point>>::iterator surfaceIt;

	for (surfaceIt = surface.begin(); surfaceIt != surface.end(); surfaceIt++) //Loop between the cracks
	{
		for (pointIt = surfaceIt->begin(); pointIt != surfaceIt->end(); pointIt++) //Loop through the crack points and output
		{
			//Blender has the Y-Z coordinates mixed up for some reason
			fileout << "v " << to_string(pointIt->x) << " " << to_string(z) << " " << to_string(-(pointIt->y)) << endl;
		}
	}

	//Add the corner points always should be 4...
	for (int i = 0; i < 4; i++)
	{
		fileout << "v " << cornerPoints[i].x << " " << to_string(z) << " " << -cornerPoints[i].y << endl;
	}

	//Loop between the cracks again for line connectivity, have to do after vertices unfortunately hence 2 loops.
	// for example, the first crack represented by the 5 first vertices from the above
	// need to connect vertex 1->2, 2->3, 3->4, 4->5, 5->1 (5 edges -> 5vertices as we want a closed loop)
	uint64 numberOfPoints; //size of vector CAN return up to an unsigned 64bit number
	uint64 index = 1; // first point within the .obj vertexes will be vertex 1, .obj files are indexed from 1
	uint64 sizedIndex;
	for (uint64 i = 0; i < surface.size(); i++)
	{
		numberOfPoints = surface.at(i).size();
		sizedIndex = (numberOfPoints + index) - 1;
		for (uint64 j = index; j < sizedIndex; j++)
		{
			fileout << "l " << to_string(j) << " " << to_string(j + 1) << endl;
		}
		// reached the end, now connect the first point to the last
		fileout << "l " << to_string(sizedIndex) << " " << to_string(index) << endl;

		index = sizedIndex + 1; //New start
	}

}

std::vector<Point> removeDuplicatesFromContour(std::vector<Point> input, Mat& contouredMat)
{
	std::vector<Point> cleaned;
	cleaned.reserve(input.size());

	for (uint64 i = 0; i < input.size(); i++)
	{
		if ((int)contouredMat.at<uchar>(input.at(i).y, input.at(i).x) == 255)
		{
			cleaned.push_back(input.at(i));
			contouredMat.at<uchar>(input.at(i)) = 0;
		}
	}
	cleaned.shrink_to_fit();
	return cleaned;
}

Mat getGreenMachinedBorder(Mat imageToWrite, Point* cornerPoints)
{
	//Border represented in green.
	//Input has to be HSV and the reectangle green for now.
	Mat thresh, corneredBorder, final;
	inRange(imageToWrite, Scalar(50, 200, 20), Scalar(80, 255, 255), thresh);
	vector<std::vector<Point>> borderContours;

	cv::cvtColor(thresh, corneredBorder, COLOR_GRAY2BGR);

	findContours(thresh, borderContours, RETR_EXTERNAL, CHAIN_APPROX_NONE);

	cv::RotatedRect rotRec = minAreaRect(borderContours.at(0));
	cv::Point2f temp[4];
	rotRec.points(temp);

	for (int i = 0; i < 4; i++) {
		cornerPoints[i] = temp[i];
	}

	fillConvexPoly(corneredBorder, cornerPoints, 4, Scalar(255, 255, 255));

	cv:cvtColor(corneredBorder, final, COLOR_BGR2GRAY);

	cv::drawContours(corneredBorder, borderContours, -1, Scalar(0, 255, 0), 1);
	cv::imshow("final", corneredBorder);

	return final;
}


/*

		
*/

/*

		extractChannel(input, blue, 0);
		imshow("input", blue);

		bilateralFilter(blue, biblur, 20, 50, 80);
		imshow("biblur", biblur);

		bitwise_not(biblur, biblur);
		imshow("bitwisenot", biblur);

		difference = biblur + blue;
		bitwise_not(difference, difference);
		imshow("difference", difference);

		adaptiveThreshold(biblur, at, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, 7, 2);
		imshow("at1", at);
		morphologyEx(at, morphed, MORPH_OPEN, cv::getStructuringElement(MORPH_RECT, Size(3, 3)));
		imshow("morphed", morphed);

*/