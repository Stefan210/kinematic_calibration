/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/kinematic_calibration_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace kinematic_calibration_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    ui.listView_measurements->setModel(qnode.measurementsModel());
    ui.listView_measurements->setEditTriggers(QAbstractItemView::NoEditTriggers);

    connect(ui.listView_measurements->selectionModel(),
            SIGNAL(currentChanged(QModelIndex,QModelIndex)),
            &qnode,
            SLOT(measurementSelectionChanged(QModelIndex)));

    connect(&qnode, SIGNAL(measurementImageUpdated(QImage*)),
                           this, SLOT(updateMeasurementImage(QImage*)));

    connect(&qnode, SIGNAL(measurementInformationUpdated(MeasurementInformation)),
                           this, SLOT(updateMeasruementInformation(MeasurementInformation)));

    connect(ui.pushButton_startOptimization,
            SIGNAL(clicked()),
            &qnode,
            SLOT(updateIgnoredMeasurementsAndstartOptimization()));

    connect(ui.pushButton_clear,
            SIGNAL(clicked()),
            &qnode,
            SLOT(clearMeasurements()));


    //ui.tab_manager->setCurrentIndex(2);
    //ui.listView_measurements->setSelectionMode(QAbstractItemView::ExtendedSelection);
    //QObject::connect(&qnode, SIGNAL(measurementReceived()), this, SLOT(updateLoggingView()));

}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

void MainWindow::updateMeasurementImage(QImage *image) {
    QGraphicsPixmapItem item( QPixmap::fromImage(*image)); image->save("/tmp/tmp.jpg");
    QGraphicsScene* scene = new QGraphicsScene(this);
    std::cout << item.pixmap().width() << "x" << item.pixmap().height() << std::endl;
    //scene->addItem(&item);
    scene->addPixmap(QPixmap::fromImage(*image));
    //QPainter p(image);
    //scene->render(&p);
    ui.graphicsView->setScene(scene);
    ui.graphicsView->show();
}

void MainWindow::updateMeasruementInformation(MeasurementInformation information) {
    ui.label_chain_name_info->setText(information.chainName);
    ui.label_chain_root_info->setText(information.chainRoot);
    ui.label_chain_tip_info->setText(information.chainTip);
    ui.listWidget_joints->clear();
    for(int i = 0; i < information.jointStates.size(); i++) {
        new QListWidgetItem(information.jointStates[i], ui.listWidget_joints); std::cout << "oak" << std::endl;
    }
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
	QMainWindow::closeEvent(event);
}

}  // namespace qtest

