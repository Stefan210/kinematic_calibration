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
#include <std_msgs/Empty.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <string>
#include <std_srvs/Empty.h>
#include <sstream>
#include "../include/kinematic_calibration_gui/qnode.hpp"
#include <kdl_parser/kdl_parser.hpp>

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
    ros::init(init_argc,init_argv,"qtest");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
    this->nodeHandle = new ros::NodeHandle();

	// Add your ros communications here.
    measurementSubscriber = nodeHandle->subscribe("/kinematic_calibration/measurement_data", 1000,
                                        &QNode::measurementCb, this);
    connect(&measurements_model,
            SIGNAL(itemChanged(QStandardItem*)),
            this,
            SLOT(measurementItemChanged(QStandardItem*)));

    this->robotModel.initParam("/robot_description");
    kdl_parser::treeFromUrdfModel(this->robotModel, this->robotTree);

	start();
	return true;
}

void QNode::run() {
	ros::Rate loop_rate(1);
	int count = 0;
	while ( ros::ok() ) {
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

    // update information
    MeasurementInformation information;
    information.chainName = QString::fromUtf8(md.chain_name.c_str());
    information.chainRoot = QString::fromUtf8(md.chain_root.c_str());
    information.chainTip = QString::fromUtf8(md.chain_tip.c_str());

    KDL::Chain currentChain;
    this->robotTree.getChain(md.chain_root, md.chain_tip, currentChain);
    for(int i = 0; i < currentChain.getNrOfSegments(); i++) {
        if(currentChain.getSegment(i).getJoint().getType() == KDL::Joint::None)
            continue;
        std::string jointName = currentChain.getSegment(i).getJoint().getName();
        double lower = this->robotModel.getJoint(jointName)->limits->lower;
        double upper = this->robotModel.getJoint(jointName)->limits->upper;
        double value = md.jointState.position[std::find(md.jointState.name.begin(),md.jointState.name.end(), jointName) - md.jointState.name.begin()];
        std::stringstream ss;
        ss << jointName << ": " << value << " (" << lower  << "/" << upper << ")";
        information.jointStates.push_back(tr(ss.str().c_str()));
    }

    Q_EMIT measurementInformationUpdated(information);
}

void QNode::updateIgnoredMeasurements() {
    std::vector<std::string> ids;
    QSetIterator<QStandardItem*> i(uncheckedMeasurements);
    while (i.hasNext()) {
        QStandardItem* item = i.next();
        ids.push_back(item->text().toStdString());
    }
    this->nodeHandle->setParam("ignore_measurements", ids);
}

void QNode::startOptimization() {
    ros::ServiceClient client = this->nodeHandle->serviceClient<std_srvs::Empty>("/kinematic_calibration/start_optimization");
    std_srvs::Empty msg;
    client.call(msg.request, msg.response);
}

void QNode::updateIgnoredMeasurementsAndstartOptimization() {
    updateIgnoredMeasurements();
    startOptimization();
}

void QNode::clearMeasurements() {
    this->checkedMeasurements.clear(); // checked list
    this->uncheckedMeasurements.clear(); // unchecked lsit
    this->measurements.clear(); // measurement object list
    this->measurements_model.clear(); // measurement model/view list
    Q_EMIT measurementReceived();
}

}  // namespace qtest
