/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/network.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/kinematic_calibration_gui/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kinematic_calibration_gui {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
    {init();}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
    ros::init(init_argc,init_argv,"qtest"); std::cout << "111" << std::endl;
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
    measurementSubscriber = n.subscribe("/kinematic_calibration/measurement_data", 1000,
                                        &QNode::measurementCb, this);
    connect(&measurements_model,
            SIGNAL(itemChanged(QStandardItem*)),
            this,
            SLOT(measurementItemChanged(QStandardItem*)));

//    connect(selectionModel(),
//            SIGNAL(selectionChanged(QItemSelection,QItemSelection)),
//            this,
//            SLOT(measurementSelectionChanged(QItemSelection)));


	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(1);
	int count = 0;
	while ( ros::ok() ) {
        std::cout << "run" << std::endl;
        ros::getGlobalCallbackQueue()->callAvailable();
        loop_rate.sleep();

        /*
		std_msgs::String msg;
		std::stringstream ss;
		ss << "hello world " << count;
		msg.data = ss.str();
		chatter_publisher.publish(msg);
		log(Info,std::string("I sent: ")+msg.data);
		ros::spinOnce();
		loop_rate.sleep();
		++count;

        ros::getGlobalCallbackQueue()->callAvailable();*/
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QNode::log( const LogLevel &level, const std::string &msg) {
	logging_model.insertRows(logging_model.rowCount(),1);
	std::stringstream logging_model_msg;
	switch ( level ) {
		case(Debug) : {
				ROS_DEBUG_STREAM(msg);
				logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Info) : {
				ROS_INFO_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Warn) : {
				ROS_WARN_STREAM(msg);
				logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Error) : {
				ROS_ERROR_STREAM(msg);
				logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
				break;
		}
		case(Fatal) : {
				ROS_FATAL_STREAM(msg);
				logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
				break;
		}
	}
	QVariant new_row(QString(logging_model_msg.str().c_str()));
	logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
	Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

void QNode::measurementCb(const measurementDataConstPtr& msg) {
    std::cout << "measurementCb" << std::endl;
    measurements.push_back(*msg);
    measurements_model.insertRows(measurements_model.rowCount(),1);
    std::stringstream id;
    id << msg->id;
    //QVariant new_row(QString(id.str().c_str()));
    QStandardItem *Item = new QStandardItem();
    Item->setData(QString(id.str().c_str()));
    Item->setText(QString(id.str().c_str()));
    Item->setCheckable( true );
    Item->setCheckState( Qt::Checked );
    measurements_model.setItem( measurements_model.rowCount()-1, Item );

    Q_EMIT measurementReceived();
}

void QNode::measurementItemChanged(QStandardItem * item) {
    std::cout << "measurementItemChanged" << std::endl;
    if(Qt::Checked == item->checkState()) {
        checkedMeasurements.insert(item);
        uncheckedMeasurements.remove(item);
    } else if(Qt::Unchecked == item->checkState()) {
        uncheckedMeasurements.insert(item);
        checkedMeasurements.remove(item);
    }

    QSetIterator<QStandardItem*> i(uncheckedMeasurements);
    while (i.hasNext()) {
        QStandardItem* item = i.next();
        std::cout << "unchecked: " << item->text().toStdString() << std::endl;
    }
}

namespace enc = sensor_msgs::image_encodings;

void QNode::measurementSelectionChanged(const QModelIndex& index) {
    std::cout << "current changed: "  << std::endl;
    std::cout << index.row() << std::endl;
    measurementData md = measurements[index.row()];
    cv_bridge::CvImageConstPtr cv_ptr;
    cv_ptr = cv_bridge::toCvCopy(md.image, enc::BGR8);
    cv::Mat imageBGR = cv_ptr->image;
    cv::Mat imageRGB;

    // draw the marker position
    std::vector<cv::Point2f> corners;
    corners.push_back(cv::Point2f(md.marker_data[0], md.marker_data[1]));
    cv::drawChessboardCorners(imageBGR, cv::Size(1, 1), corners, true);

    // copy image to new image, converting pixel data from OpenCV's default BGR format to Qt's RGB format
    cv::cvtColor(imageBGR, imageRGB, CV_BGR2RGB);

    // create a this newly converted RGB pixel data with a QImage
    QImage* qImg = new QImage((uchar *)imageRGB.data, imageRGB.cols, imageRGB.rows, QImage::Format_RGB888);
    Q_EMIT measurementImageUpdated(qImg);
}

}  // namespace qtest
