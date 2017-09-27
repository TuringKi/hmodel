
//#include "cudax/CudaHelper.h"
//#include <vld.h>

#define zz ok

#ifdef  zz

#include <iostream>
#include <QApplication>

#include "tracker/Sensor/Sensor.h"
#include "tracker/Data/Camera.h"

#include "tracker/Tracker.h"
#include "tracker/GLWidget.h"


int main(int argc, char* argv[]) {
	bool htrack = false;
	bool test = false; //J' * J on CPU
	bool real_color = false;
	bool save_rastorized_model = false;

	bool benchmark = true;
	bool playback = false;
	int user_name = 0;

	int devID = 0;
	//cudaError status = cudaGLSetGLDevice(devID);
	//cudaFree(0);

	std::string sequence_path = "F:/HandPose_Depth/tpHModel/x64/teaser/";
	std::string data_path = "F:/HandPose_Depth/tpHModel/src/data/";
	std::string sequence_name = "teaser";

	Q_INIT_RESOURCE(shaders);
	QApplication app(argc, argv);

	Camera camera(QVGA, 60);
	SensorOpenNI sensor(&camera);

	DataStream datastream(&camera);
	SolutionStream solutions;

	Worker worker(&camera, test, benchmark, save_rastorized_model, user_name, data_path);

	{
		worker.settings->termination_max_iters = 8;

		worker.E_fitting.settings->fit2D_enable = true;
		worker.E_fitting.settings->fit2D_weight = 0.7;

		worker.E_fitting.settings->fit3D_enable = true;

		worker.E_limits.jointlimits_enable = true;

		worker.E_pose._settings.enable_split_pca = true;
		worker.E_pose._settings.weight_proj = 4 * 10e2;

		worker.E_collision._settings.collision_enable = true;
		worker.E_collision._settings.collision_weight = 1e3;

		worker.E_temporal._settings.temporal_coherence1_enable = true;
		worker.E_temporal._settings.temporal_coherence2_enable = true;
		worker.E_temporal._settings.temporal_coherence1_weight = 0.05;
		worker.E_temporal._settings.temporal_coherence2_weight = 0.05;

		worker.E_damping._settings.abduction_damping = 1500000;
		worker._settings.termination_max_rigid_iters = 1;
	}

	GLWidget glwidget(&worker, &datastream, &solutions, playback, false /*real_color*/, data_path);
	worker.bind_glwidget(&glwidget);
	glwidget.show();

	Tracker tracker(&worker, camera.FPS(), sequence_path + sequence_name + "/", real_color);
	tracker.sensor = &sensor;
	tracker.datastream = &datastream;
	tracker.solutions = &solutions;

	///--- Starts the tracking
	tracker.toggle_tracking(!benchmark && !playback);
	tracker.toggle_benchmark(benchmark);
	//tracker.toggle_playback(playback);

	return app.exec();
}
#else  
#include <iostream>
#include <QDebug>
#include <QApplication>
#include <QDir>

#include "util/gl_wrapper.h"
#include "util/OpenGL32Format.h"
#include <QGLWidget>
#include "tracker/AntTweakBarEventFilter.h"

#include "tracker/ForwardDeclarations.h"
#include "tracker/HandFinder/HandFinder.h"
#include "tracker/Sensor/Sensor.h"
#include "tracker/Data/Camera.h"
#include "tracker/Calibration/Calibration.h"
#include "tracker/OpenGL/KinectDataRenderer/KinectDataRenderer.h"
#include "tracker/OpenGL/CylindersRenderer/Cylinders_renderer.h"
#include "tracker/hTracker.h"

class GLWidget : public QGLWidget{
public:
	Worker*const worker;
	DataStream*const datastream;
	SolutionStream*const solutions;

	Camera*const _camera;
	KinectDataRenderer kinect_renderer;
	Cylinders_renderer mrenderer;

public:
	GLWidget(Worker* worker, DataStream * datastream, SolutionStream * solution) :
		QGLWidget(OpenGL32Format()),
		worker(worker),
		datastream(datastream),
		solutions(solutions),
		_camera(worker->camera),
		mrenderer(worker->cylinders)
	{
		std::cout << "Started OpenGL " << this->format().majorVersion() << "." << this->format().minorVersion() << std::endl;
		this->installEventFilter(new AntTweakBarEventFilter(this)); ///< all actions pass through filter
	}

	~GLWidget(){
		worker->cleanup_graphic_resources();
		tw_settings->tw_cleanup();
	}

	void initializeGL(){
		std::cout << "GLWidget::initializeGL()" << std::endl;
		initialize_glew();
		tw_settings->tw_init(this->width(), this->height()); ///< FIRST!!

		glEnable(GL_DEPTH_TEST);

		kinect_renderer.init(_camera);
		mrenderer.init(Cylinders_renderer::NORMAL);
		mrenderer.init_geometry(); // thick model

		///--- Initialize other graphic resources
		this->makeCurrent();
		worker->init_graphic_resources();

		///--- Setup with data from worker
		kinect_renderer.setup(worker->sensor_color_texture->texid(), worker->sensor_depth_texture->texid());
	}

	void paintGL() {
		glViewport(0, 0, this->width(), this->height());
		glClearColor(1, 1, 1, 1);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		///--- Rendering
		Eigen::Matrix4f view_projection = _camera->view_projection_matrix();
		Eigen::Matrix4f view = _camera->view_matrix();

		if (worker->handfinder->wristband_found())
		{
			kinect_renderer.enable_colormap(true);
			kinect_renderer.set_zNear(worker->handfinder->wristband_center()[2] - 150);
			kinect_renderer.set_zFar(worker->handfinder->wristband_center()[2] + 150);
		}
		kinect_renderer.set_uniform("view_projection", view_projection);
		kinect_renderer.render();

		mrenderer.set_uniform("view", view);
		mrenderer.set_uniform("view_projection", view_projection);
		mrenderer.render();

		tw_settings->tw_draw();
	}

	void reload_model(){
		/// This recomputes segments lengths
		worker->cylinders->recomputeLengths();
		/// This accepts the new joint translations
		worker->skeleton->setInitialTranslations();
		/// This generates VBO from segments
		mrenderer.init_geometry();
		/// After change, show what's happened
		this->updateGL();
	}

private:
	void keyPressEvent(QKeyEvent *event){
		GLWidget* qglviewer = this;
		switch (event->key()){
		case Qt::Key_Escape:
			this->close();
			break;
		case Qt::Key_1:
			// make_hand_thinner();
			worker->skeleton->scaleWidth(-5);
			Calibration::update_radius_helper(worker->cylinders, worker->skeleton);
			qglviewer->reload_model();
			break;
		case Qt::Key_2:
			// make_hand_wider();
			worker->skeleton->scaleWidth(5);
			Calibration::update_radius_helper(worker->cylinders, worker->skeleton);
			qglviewer->reload_model();
			break;
		case Qt::Key_3:
			// make_hand_shorter();
			worker->skeleton->scaleHeight(-1);
			Calibration::update_radius_helper(worker->cylinders, worker->skeleton);
			qglviewer->reload_model();
			break;
		case Qt::Key_4:
			// make_hand_longer();
			worker->skeleton->scaleHeight(1);
			Calibration::update_radius_helper(worker->cylinders, worker->skeleton);
			qglviewer->reload_model();
			break;
		case Qt::Key_5:
			// make_hand_smaller();
			worker->skeleton->scale(0.99f);
			qglviewer->reload_model();
			break;
		case Qt::Key_6:
			// make_hand_bigger();
			worker->skeleton->scale(1.01f);
			qglviewer->reload_model();
			break;
		default:
			QGLWidget::keyPressEvent(event);
		}
	}
};

int main(int argc, char* argv[]){
	Q_INIT_RESOURCE(shaders); ///< http://qt-project.org/wiki/QtResources
	QApplication app(argc, argv);
	std::cout << "htrack starting" << std::endl;
	std::cout << "--Execution path: " << QDir::currentPath().toStdString() << std::endl;

#if defined(SOFTKIN) && !defined(__APPLE__)
	Camera camera(Intel, 60 /*FPS*/);
	SensorSoftKin sensor(&camera);
#endif

#if defined(DEPTHSENSEGRABBER) && !defined(__APPLE__)
	Camera camera(Intel, 60 /*FPS*/);
	SensorDepthSenseGrabber sensor(&camera);
#endif

#if defined(WITH_OPENNI)
	Camera camera(QVGA, 60 /*FPS*/);
	// Camera camera(QVGA, 30 /*FPS*/);
	SensorOpenNI sensor(&camera);
#endif

#if defined(REALSENSE)
	Camera camera(QVGA, 60 /*FPS*/);
	SensorRealSense sensor(&camera);
#endif

#if defined(LIBREALSENSE)
	Camera camera(Intel, 60 /*FPS*/);
	SensorLibRealSense sensor(&camera);
#endif

	DataStream datastream(&camera);
	SolutionStream solutions;

	hWorker worker(&camera);
	GLWidget glarea(&worker, &datastream, &solutions);
	glarea.resize(640 * 2, 480 * 2); ///< force resize
	worker.bind_glwidget(&glarea); ///< TODO: can we avoid this?

	///--- Load calibration
	Calibration(&worker).autoload();
	// glarea->reload_model(); ///< TODO


	glarea.show(); ///< calls GLWidget::InitializeGL

	hTracker tracker(&worker, camera.FPS());
	tracker.sensor = &sensor;
	tracker.datastream = &datastream;
	tracker.solutions = &solutions;

	///--- Starts the tracking
	tracker.toggle_tracking(true);

	return app.exec();
}
#endif