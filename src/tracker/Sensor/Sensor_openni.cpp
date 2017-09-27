#include "Sensor.h"
#include "util/mylogger.h"
#include "util/Sleeper.h"

#include "tracker/Types.h"
#include "tracker/Data/DataFrame.h"
#include "tracker/Data/Camera.h"

#ifndef WITH_OPENNI
void openni_hard_quit(){ std::cout << "!!!ERROR: Cannot use SensorOpenNI" << std::endl; }
SensorOpenNI::SensorOpenNI(Camera* camera) : Sensor(camera) { openni_hard_quit(); }
SensorOpenNI::~SensorOpenNI(){ openni_hard_quit(); }
bool SensorOpenNI::spin_wait_for_data(Scalar timeout_seconds){ openni_hard_quit(); return false; }
bool SensorOpenNI::fetch_streams(DataFrame& frame){ openni_hard_quit(); return false; }
int SensorOpenNI::initialize(){ openni_hard_quit(); return 0; }
#else
#include "OpenNI.h"
#include <QObject>
#include <QElapsedTimer>
#include <QEventLoop>
#include <QApplication>

#ifdef __APPLE__
    namespace kinect = openni;
#endif
#ifdef _WIN32
    namespace kinect = openni;
#endif

/// Device
kinect::Device device;

/// Streams
kinect::VideoStream g_depthStream;
kinect::VideoStream g_colorStream;

/// Frames
kinect::VideoFrameRef g_depthFrame;
kinect::VideoFrameRef g_colorFrame;

SensorOpenNI::SensorOpenNI(Camera *camera) : Sensor(camera) {
    if(camera->mode() != QVGA)
        LOG(FATAL) << "OpenNI sensor needs QVGA camera mode";
}

int SensorOpenNI::initialize()
{
    LOG(INFO) << "Initializing OpenNI";
    ///< force shutdown before starting!!
    kinect::OpenNI::shutdown();

    kinect::Status rc;
    rc = kinect::STATUS_OK;

    /// Fetch the device URI to pass to Device::open()
    const char* deviceURI = kinect::ANY_DEVICE;

    /// Initialize the device
    rc = kinect::OpenNI::initialize();
    if(rc!=kinect::STATUS_OK)
    {
        qDebug()<<"Initialization Errors (if any): "<< kinect::OpenNI::getExtendedError();
        kinect::OpenNI::shutdown();
        exit(0);
    }

    /// Open the device using the previously fetched device URI
    rc = device.open(deviceURI);
    if (rc != kinect::STATUS_OK)
    {
        qDebug()<<"Device open failed: "<<kinect::OpenNI::getExtendedError();
        kinect::OpenNI::shutdown();
        exit(0);
    }

    /// Create the depth stream
    rc = g_depthStream.create(device, kinect::SENSOR_DEPTH);
    if (rc == kinect::STATUS_OK)
    {
        /// start the depth stream, if its creation was successful
        rc = g_depthStream.start();

        if (rc != kinect::STATUS_OK)
        {
            qDebug()<<"Couldn't start depth stream: "<<kinect::OpenNI::getExtendedError();
            g_depthStream.destroy();
            exit(0);
        }
    }
    else
    {
        qDebug()<<"Couldn't find depth stream: "<<kinect::OpenNI::getExtendedError();
        exit(0);
    }

    if (!g_depthStream.isValid())
    {
        qDebug()<<"No valid depth streams. Exiting";
        kinect::OpenNI::shutdown();
        exit(0);
    }

    /// Create the color stream
    rc = g_colorStream.create(device, kinect::SENSOR_COLOR);

    if (rc == kinect::STATUS_OK)
    {
        /// start the color stream, if its creation was successful
        rc = g_colorStream.start();

        if (rc != kinect::STATUS_OK)
        {
            qDebug()<<"Couldn't start color stream: "<<kinect::OpenNI::getExtendedError();
            g_colorStream.destroy();
            exit(0);
        }
    }
    else
    {
        qDebug()<<"Couldn't find color stream: "<<kinect::OpenNI::getExtendedError();
        exit(0);
    }

    if (!g_colorStream.isValid())
    {
        qDebug()<<"No valid color streams. Exiting";
        kinect::OpenNI::shutdown();
        exit(0);
    }

    /// Configure resolutions
    {
        /// Attempt to set for depth
        {
            kinect::VideoMode mode = g_depthStream.getVideoMode();
            if(((int)camera->FPS())==60)
                mode.setFps(60);
            else
                mode.setFps(30);
            mode.setResolution(camera->width(), camera->height());
            rc = g_depthStream.setVideoMode(mode);
            if (rc != kinect::STATUS_OK)
                std::cerr << "error setting video mode (depth)" << std::endl;
        }
        /// Attempt to set for color
        {
            kinect::VideoMode mode = g_colorStream.getVideoMode();
            if(((int)camera->FPS())==60)
                mode.setFps(60);
            else
                mode.setFps(30);
            mode.setFps(30); ///< @todo check!!!
            mode.setResolution(camera->width(), camera->height());
            rc = g_colorStream.setVideoMode(mode);
            if (rc != kinect::STATUS_OK)
                std::cerr << "error setting video mode (color)" << std::endl;
        }
    }


#ifdef THIS_CAUSES_INIT_STALLS
    /// Enable depth/color frame synchronization
    rc = device.setDepthColorSyncEnabled(true);
    if (rc != kinect::STATUS_OK)
    {
        qDebug()<<"Could not synchronise device";
        // VGA Kinect always seems to shut down here
        kinect::OpenNI::shutdown();
        exit(0);
    }
#endif

    /// Camera settings
    kinect::CameraSettings* settings = g_colorStream.getCameraSettings();
    settings->setAutoExposureEnabled(true);
    settings->setAutoWhiteBalanceEnabled(true);

    /// Fetch the camera intrinsics
#if 0
        float w = g_depthStream.getVideoMode().getResolutionX();protected:
        Camera*const camera;
        /// Device
        kinect::Device device;
        bool initialized;

        /// Streams
        kinect::VideoStream g_depthStream;
        kinect::VideoStream g_colorStream;

        /// Frames
        kinect::VideoFrameRef g_depthFrame;
        kinect::VideoFrameRef g_colorFrame;
        float fov_h = g_depthStream.getHorizontalFieldOfView();
        float fl_h = .5*w / tan(.5*fov_h);
        float h = g_depthStream.getVideoMode().getResolutionY();
        float fov_v = g_depthStream.getVerticalFieldOfView();
        float fl_v = .5*h / tan(.5*fov_v);
        std::cout << "cameras focal lengths" << fl_h << fl_v;
#endif

    initialized = true;
    return true;
}

SensorOpenNI::~SensorOpenNI()
{
    if(initialized){
        LOG(INFO) << "Shutting down Kinect...";
        flush(std::cout);
        g_depthStream.destroy();
        g_colorStream.destroy();
        device.close();
        kinect::OpenNI::shutdown();
    }
}

bool SensorOpenNI::spin_wait_for_data(Scalar timeout_seconds)
{
    DataFrame frame(-1);
    QElapsedTimer chrono;
    chrono.start();
    while(fetch_streams(frame)==false){
        LOG(INFO) << "Waiting for data.. " << chrono.elapsed();
        Sleeper::msleep(500);
        QApplication::processEvents(QEventLoop::AllEvents);
        if( chrono.elapsed() > 1000*timeout_seconds )
            return false;
    }

    return true;
}

#include <thread>
#include <mutex>
#include <condition_variable>


int D_width = 640;
int D_height = 480;
const int BACK_BUFFER = 1;
const int FRONT_BUFFER = 0;
cv::Mat color_array[2];
cv::Mat full_color_array[2];
cv::Mat depth_array[2];

std::vector<int> sensor_indicator_array[2];
int num_sensor_points_array[2];

bool wristband_found_buffer;
Vector3 wristband_center_buffer;
Vector3 wristband_direction_buffer;
cv::Mat sensor_silhouette_buffer;

std::thread sensor_thread;
std::mutex swap_mutex;
std::condition_variable condition;
bool main_released = true;
bool thread_released = false;

int i = 1;

int sensor_frame = 0;
int tracker_frame = 0;

bool SensorOpenNI::concurrent_fetch_streams(
	DataFrame &frame,
	HandFinder & other_handfinder, cv::Mat & full_color)
{
	std::unique_lock<std::mutex> lock(swap_mutex);
	condition.wait(lock, [] {return thread_released; });
	main_released = false;

	frame.color = color_array[FRONT_BUFFER].clone();
	frame.depth = depth_array[FRONT_BUFFER].clone();
	if (real_color) full_color = full_color_array[FRONT_BUFFER].clone();

	other_handfinder.sensor_silhouette = sensor_silhouette_buffer.clone();
	other_handfinder._wristband_found = wristband_found_buffer;
	other_handfinder._wband_center = wristband_center_buffer;
	other_handfinder._wband_dir = wristband_direction_buffer;

	other_handfinder.num_sensor_points = num_sensor_points_array[FRONT_BUFFER];
	for (size_t i = 0; i < other_handfinder.num_sensor_points; i++)
		other_handfinder.sensor_indicator[i] = sensor_indicator_array[FRONT_BUFFER][i];

	//cout << "tracker_frame = " << tracker_frame << endl;

	main_released = true;
	lock.unlock();
	condition.notify_one();
	return true;
}




bool SensorOpenNI::run()
{
	return true;
}


void SensorOpenNI::start()
{

}


void SensorOpenNI::stop()
{

}


bool SensorOpenNI::fetch_streams(DataFrame &frame)
{
    if(initialized==false)
        this->initialize();

    /// @note WE DO IT HERE OTHERWISE IT GETS IGNORED IF WE DO IT IN INITIALIZATION :(
    /// Depth image is transformed to have the same apparent vantage point as the RGB image
    kinect::Status m_rc = device.setImageRegistrationMode(kinect::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
    if (m_rc != kinect::STATUS_OK)
    {
        qDebug()<<"Could not set Image Registration Mode";
        kinect::OpenNI::shutdown();
        exit(0);
    }

    kinect::VideoStream* streams[] = {&g_depthStream, &g_colorStream};
    int changedIndex = -1;
    kinect::Status rc = kinect::STATUS_OK;
    while (rc == kinect::STATUS_OK){
        rc = kinect::OpenNI::waitForAnyStream(streams, 2, &changedIndex, 0);

        if (rc == kinect::STATUS_OK){
            switch (changedIndex){
            case 0:
                // timer.restart();
                g_depthStream.readFrame(&g_depthFrame);
                // qDebug() << "depth.readFrame" << timer.restart();
                break;
            case 1:
                g_colorStream.readFrame(&g_colorFrame);
                // qDebug() << "color.readFrame" << timer.restart();
                break;
            default:
                printf("Error in wait\n");
            }
        }
    }

    // qDebug() << "\n Frames: ";
    // if(g_colorFrame.isValid())   qDebug("color: %d",g_colorFrame.getFrameIndex());
    // if(g_depthFrame.isValid())   qDebug("depth: %d",g_depthFrame.getFrameIndex());

    if( !g_colorFrame.isValid() ) return false;
    if( !g_depthFrame.isValid() ) return false;

    //--- DEBUG: identify bottleneck
    // qDebug() << "D - frameID: " << g_depthFrame.getFrameIndex();
    // qDebug() << "C - frameID: " << g_colorFrame.getFrameIndex();

    /// @note this DOES NOT copy memory!
    const kinect::RGB888Pixel* color_buffer = (const kinect::RGB888Pixel*) g_colorFrame.getData();
    const kinect::DepthPixel*  depth_buffer = (const kinect::DepthPixel*) g_depthFrame.getData();
    
	/*
		cv::Mat frame_color = cv::Mat(camera->height(), camera->width(), CV_8UC3);
		cv::Mat frame_depth = cv::Mat(camera->height(), camera->width(), CV_16UC1, cv::Scalar(0));
		*/
	//cv::Mat tmp_depth = cv::Mat(camera->height(), camera->width(), CV_16UC1, (void*)depth_buffer);

	//const uint16_t* data =depth_buffer;
	//for (int y = 0, y_sub = 0; y_sub < camera->height(); y += 2, y_sub++) {
	//	for (int x = 0, x_sub = 0; x_sub < camera->width(); x += 2, x_sub++) {
	//		if (x == 0 || y == 0) {
	//			//frame_depth.at<uint16_t>(y_sub, x_sub) = data[y*D_width + (D_width - x - 1)];
	//			continue;
	//		}
	//		std::vector<int> neighbors = {
	//			data[(y - 1)* D_width + (D_width - (x - 1) - 1)],
	//			data[(y + 0)* D_width + (D_width - (x - 1) - 1)],
	//			data[(y + 1)* D_width + (D_width - (x - 1) - 1)],
	//			data[(y - 1)* D_width + (D_width - (x + 0) - 1)],
	//			data[(y + 0)* D_width + (D_width - (x + 0) - 1)],
	//			data[(y + 1)* D_width + (D_width - (x + 0) - 1)],
	//			data[(y - 1)* D_width + (D_width - (x + 1) - 1)],
	//			data[(y + 0)* D_width + (D_width - (x + 1) - 1)],
	//			data[(y + 1)* D_width + (D_width - (x + 1) - 1)],
	//		};
	//		std::sort(neighbors.begin(), neighbors.end());
	//		frame_depth.at<unsigned short>(y_sub, x_sub) = neighbors[4];
	//	}
	//}
	//
	//

	//cv::Mat sync_color_pxc = projection->CreateColorImageMappedToDepth(sample->depth, sample->color);


	cv::Mat cvRawImg16U(g_depthFrame.getHeight(), 
		g_depthFrame.getWidth(), 
		CV_16UC1, (void*)g_depthFrame.getData());
	cv::Mat cvDepthImg, cvBGRImg, cvFusionImg;
	cvRawImg16U.convertTo(cvDepthImg, CV_8U, 255.0 / (g_depthStream.getMaxPixelValue()));
	flip(cvDepthImg, cvDepthImg, 1);//水平翻转  


	cv::Mat cvRGBImg(g_colorFrame.getHeight(), g_colorFrame.getWidth(), CV_8UC3, (void*)g_colorFrame.getData());
	cv::cvtColor(cvRGBImg, cvBGRImg, CV_RGB2BGR);
	flip(cvBGRImg, cvBGRImg, 1);//水平翻转

	cv::cvtColor(cvDepthImg, cvFusionImg, CV_GRAY2BGR);
	addWeighted(cvBGRImg, 0.5, cvFusionImg, 0.5, 0, cvFusionImg);
	flip(cvFusionImg, cvFusionImg, 1);//水平翻转  


	frame.color = cv::Mat(camera->height(), camera->width(), CV_8UC3, (void*)color_buffer);
	frame.depth = cv::Mat(camera->height(), camera->width(), CV_16UC1, (void*)depth_buffer);

	    // qDebug() << "*stream->add_frame()" << timer.restart();
    return true;
}

#endif


#ifdef DOES_NOT_WORK
public slots:
    void force_reinit(){
        cout << "====> FORCING DEVICE RE-INITIALIZATION" << endl;
        this->shutdown();
        this->initialized = false;
        Sleeper::msleep(500/*ms*/); //< give it time... sigh
        hidden::kinect::OpenNI::initialize();
        cout << "====> (DONE) FORCING DEVICE RE-INITIALIZATION" << endl;
    }
#endif

#ifdef DISABLED
public:
    void auto_white_balance(bool flag){
        openni::CameraSettings* settings = g_colorStream.getCameraSettings();
        settings.setAutoWhiteBalanceEnabled(flag);
    }
    void auto_exposure(bool flag){
        openni::CameraSettings* settings = g_colorStream.getCameraSettings();
        settings.setAutoExposureEnabled(flag);
    }

public:
    void force_reinit(){
        initialized = false;
        openni::OpenNI::shutdown();
        initialize();
    }
#endif
