

#include "fertilized/fertilized.h"
#include <chrono>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <Eigen/Core>

#include <memory>
#include <vector>
#include <functional>
#include <string>
#include <fstream>
#include <algorithm>

#include "segmentation/libseg.h"

#define KERNEL_SIZE 3
#define BACKGROUND_DEPTH 3000
# define GET_CLOSER_TO_SENSOR 600
# define SRC_COLS 80
# define SRC_ROWS 60
# define N_FEAT 8.0
// how far away should we shoot when computing features - should be proportional to N_FEAT 
# define DELTA 12000.0

# define N_THREADS 16
static int D_width = 640;
static int D_height = 480;

# define THRESHOLD 0.7
# define THRESHOLD_WRIST 0.6
# define DILATION_SIZE 9

static auto soil = fertilized::Soil<float, float, fertilized::uint, fertilized::Result_Types::probabilities>();
static auto forest = soil.ForestFromFile("ff_handsegmentation.ff");

void hand_segmentation(cv::Mat& depth, cv::Mat& color, cv::Mat &sensor_silhouette)
{
	
	
	
	int n_features = (2 * N_FEAT + 1)*(2 * N_FEAT + 1);
	
	cv::Mat sensor_depth = depth.clone();
	int downsampling_factor = 2;
	int ds = 4;
	cv::Mat sensor_depth_ds = cv::Mat(cv::Size(D_width / (downsampling_factor*ds), 
		D_height / (downsampling_factor * ds)), CV_16UC1, cv::Scalar(0));


	cv::medianBlur(sensor_depth, sensor_depth, KERNEL_SIZE);

	cv::resize(sensor_depth, sensor_depth_ds, 
		cv::Size(D_width / (downsampling_factor*ds), 
		D_height / (downsampling_factor * ds)), 0, 0, cv::INTER_NEAREST);


	double min;
	double max;
	cv::minMaxIdx(sensor_depth, &min, &max);
	sensor_depth.setTo(cv::Scalar(max), sensor_depth == 0);
	cv::Mat normalized_depth;
	sensor_depth.convertTo(normalized_depth, CV_8UC1, 255.0 / (max - min), -min);
	cv::Mat color_map;
	cv::applyColorMap(normalized_depth, color_map, cv::COLORMAP_COOL);

	//depth.setTo(cv::Scalar(BACKGROUND_DEPTH), depth == 0);
	sensor_depth.setTo(cv::Scalar(BACKGROUND_DEPTH), sensor_depth == 0);
	sensor_depth_ds.setTo(cv::Scalar(BACKGROUND_DEPTH), sensor_depth_ds == 0);
	cv::Mat src_X;
	sensor_depth_ds.convertTo(src_X, CV_32F);
	float* ptr = (float*)src_X.data;
	size_t elem_step = src_X.step / sizeof(float);

	std::vector<cv::Point> locations;
	std::vector<std::vector< cv::Point> > contours;
	std::vector<cv::Vec4i> hierarchy;

	cv::findNonZero(src_X < GET_CLOSER_TO_SENSOR, locations);
	int n_samples = locations.size();


	fertilized::Array<float, 2, 2> new_data = fertilized::allocate(n_samples, n_features);
	{
		// Extract the lines serially, since the Array class is not thread-safe (yet)
		std::vector<fertilized::Array<float, 2, 2>::Reference> lines;
		for (int i = 0; i < n_samples; ++i)
		{
			lines.push_back(new_data[i]);
		}
#pragma omp parallel for num_threads(N_THREADS) \
			//default(none) /* Require explicit spec. */\
			shared(ptr,new_data) \
			schedule(static)
		for (int j = 0; j < n_samples; j++)
		{
			// depth of current pixel
			//Array<float, 2, 2> line = allocate(1, n_features);
			std::vector<float> features;
			float d = (float)ptr[elem_step*locations[j].y + locations[j].x];
			for (int k = 0; k < (2 * N_FEAT + 1); k++)
			{
				int idx_x = locations[j].x + (int)(DELTA / d) * ((k - N_FEAT) / N_FEAT);
				for (int l = 0; l < (2 * N_FEAT + 1); l++)
				{
					int idx_y = locations[j].y + (int)(DELTA / d) * ((l - N_FEAT) / N_FEAT);
					// read data
					if (idx_x < 0 || idx_x > SRC_COLS || idx_y < 0 || idx_y > SRC_ROWS)
					{
						features.push_back(BACKGROUND_DEPTH - d);
						continue;
					}
					float d_idx = (float)ptr[elem_step*idx_y + idx_x];
					features.push_back(d_idx - d);
				}
			}
			std::copy(features.begin(), features.end(), lines[j].getData());
		}
	}

	// predict data
	fertilized::Array<double, 2, 2> predictions = forest->predict(new_data, N_THREADS);

	// build probability maps for current frame
	// hand
	cv::Mat probabilityMap = cv::Mat::zeros(SRC_ROWS, SRC_COLS, CV_32F);
	for (size_t j = 0; j < locations.size(); j++)
	{
		probabilityMap.at<float>(locations[j]) = predictions[j][1];
	}

	//wrist
	cv::Mat probabilityMap_w = cv::Mat::zeros(SRC_ROWS, SRC_COLS, CV_32F);
	for (size_t j = 0; j < locations.size(); j++)
	{
		probabilityMap_w.at<float>(locations[j]) = predictions[j][2];
	}

	cv::Mat mask_ds = probabilityMap > THRESHOLD;
	// find biggest blob, a.k.a. hand 
	cv::findContours(mask_ds, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	int idx = 0, largest_component = 0;
	double max_area = 0;
	for (; idx >= 0; idx = hierarchy[idx][0])
	{
		double area = fabs(cv::contourArea(cv::Mat(contours[idx])));
		if (area > max_area)
		{
			max_area = area;
			largest_component = idx;
		}
	}


	cv::Mat mask_ds_biggest_blob = cv::Mat::zeros(mask_ds.size(), CV_8U);
	cv::drawContours(mask_ds_biggest_blob, contours, largest_component, cv::Scalar(255), CV_FILLED, 8, hierarchy);

	std::pair<float, int> avg;
	for (int row = 0; row < mask_ds_biggest_blob.rows; ++row)
	{
		for (int col = 0; col < mask_ds_biggest_blob.cols; ++col)
		{
			float depth_wrist = sensor_depth_ds.at<ushort>(row, col);
			if (mask_ds_biggest_blob.at<uchar>(row, col) == 255)
			{
				avg.first += depth_wrist;
				avg.second++;
			}
		}
	}
	ushort depth_hand = (avg.second == 0) ? BACKGROUND_DEPTH : avg.first / avg.second;


	cv::Mat probabilityMap_us;
	cv::Mat probabilityMap_w_us;

	// UPSAMPLE USING RESIZE: advantages of joint bilateral upsampling are already exploited 
	cv::resize(probabilityMap, probabilityMap_us, sensor_depth.size());
	cv::resize(probabilityMap_w, probabilityMap_w_us, sensor_depth.size());

	cv::Mat mask = probabilityMap_us > THRESHOLD;
	cv::Mat mask_wrist = probabilityMap_w_us > THRESHOLD_WRIST;


	// Extract pixels at depth range on hand only
	ushort depth_range = 100;
	cv::Mat range_mask;
	cv::inRange(sensor_depth, depth_hand - depth_range, depth_hand + depth_range, range_mask);

	// POSTPROCESSING: APPLY SOME DILATION and SELECT BIGGEST BLOB
	cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, 
		cv::Size(2 * DILATION_SIZE + 1, 2 * DILATION_SIZE + 1));

	cv::dilate(mask, mask, element);
	mask.setTo(cv::Scalar(120), mask_wrist > 0);

	cv::Mat pp;
	mask.copyTo(pp);

	cv::findContours(pp, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	idx = 0, largest_component = 0;
	max_area = 0;
	for (; idx >= 0; idx = hierarchy[idx][0])
	{
		double area = fabs(cv::contourArea(cv::Mat(contours[idx])));
		//std::cout << area << std::endl;
		if (area > max_area)
		{
			max_area = area;
			largest_component = idx;
		}
	}
	cv::Mat dst = cv::Mat::zeros(mask.size(), CV_8U);
	cv::drawContours(dst, contours, largest_component, cv::Scalar(255), CV_FILLED, 8, hierarchy);
	dst.setTo(cv::Scalar(0), range_mask == 0);
	mask.setTo(cv::Scalar(0), dst == 0);
	dst.copyTo(sensor_silhouette);
}
