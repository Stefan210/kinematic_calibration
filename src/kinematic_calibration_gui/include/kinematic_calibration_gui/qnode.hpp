/**
 * @file /include/qtest/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date February 2011
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef qtest_QNODE_HPP_
#define qtest_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <QStandardItemModel>
#include <QSet>
#include <kinematic_calibration/measurementData.h>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kinematic_calibration_gui {

using namespace kinematic_calibration;

/*****************************************************************************
** Class
*****************************************************************************/

class QNode : public QThread {
    Q_OBJECT
public:
	QNode(int argc, char** argv );
	virtual ~QNode();
    bool init();
	void run();

	/*********************
	** Logging
	**********************/
	enum LogLevel {
	         Debug,
	         Info,
	         Warn,
	         Error,
	         Fatal
	 };

	QStringListModel* loggingModel() { return &logging_model; }
    QStandardItemModel* measurementsModel() { return &measurements_model; }
	void log( const LogLevel &level, const std::string &msg);

    void executePose(int pos) {
        // TODO
        std::cout << "pos: " << pos << std::endl;
    }

Q_SIGNALS:
	void loggingUpdated();
    void rosShutdown();
    void measurementReceived();
    void measurementImageUpdated(QImage* image);

public Q_SLOTS:
    void measurementItemChanged(QStandardItem * item);

    void measurementSelectionChanged(const QModelIndex& index);

    void updateIgnoredMeasurements();

    void startOptimization();

    void updateIgnoredMeasurementsAndstartOptimization();

    void clearMeasurements();

protected:
    void measurementCb(const measurementDataConstPtr& msg);

private:
	int init_argc;
	char** init_argv;
    ros::NodeHandle* nodeHandle;
    QStringListModel logging_model;
    ros::Subscriber measurementSubscriber;
    QStandardItemModel measurements_model;
    QSet<QStandardItem*> checkedMeasurements, uncheckedMeasurements;
    std::vector<measurementData> measurements;
};

}  // namespace qtest

#endif /* qtest_QNODE_HPP_ */
