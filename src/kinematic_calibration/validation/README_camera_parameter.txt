1) include/optimization/CameraIntrinsicsVertex.h:

Templateparameter für die Anzahl der Parameter entsprechend ändern bei
"class CameraIntrinsicsVertex: public BaseVertex<5, sensor_msgs::CameraInfo>"

2) src/optimization/CameraIntrinsicsVertex.cpp:

void CameraIntrinsicsVertex::oplusImpl(const double* delta): "updates" entsprechend ein/auskommentieren
void CameraIntrinsicsVertex::setToOriginImpl(): nicht optimierte Parameter auf sinnvollen Wert setzen