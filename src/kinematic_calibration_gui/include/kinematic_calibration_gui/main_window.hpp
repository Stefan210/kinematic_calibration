/**
 * @file /include/qtest/main_window.hpp
 *
 * @brief Qt based gui for qtest.
 *
 * @date November 2010
 **/
#ifndef qtest_MAIN_WINDOW_H
#define qtest_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace kinematic_calibration_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
	void on_actionAbout_triggered();

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void updateMeasurementImage(QImage* image);
    void updateMeasruementInformation(MeasurementInformation information);

private:
	Ui::MainWindowDesign ui;
	QNode qnode;
};

}  // namespace qtest

#endif // qtest_MAIN_WINDOW_H
