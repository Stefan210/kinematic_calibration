/*
 * TemperatureNode.cpp
 *
 *  Created on: 12.12.2013
 *      Author: stefan
 */

#include "../../include/data_capturing/TemperatureNode.h"

#include <alcommon/albroker.h>
#include <alerror/alerror.h>
#include <alvalue/alvalue.h>
#include <alcommon/almodule.h>
#include <boost/program_options/options_description.hpp>
#include <boost/program_options/parsers.hpp>
#include <boost/program_options/value_semantic.hpp>
#include <boost/program_options/variables_map.hpp>
#include <boost/smart_ptr/shared_ptr.hpp>
#include <ros/console.h>
#include <ros/init.h>
#include <rosconsole/macros_generated.h>
#include <clocale>
#include <iostream>
#include <string>

namespace kinematic_calibration {

TemperatureNode::TemperatureNode(boost::shared_ptr<AL::ALBroker> broker,
		const std::string& name) :
		AL::ALModule(broker, name), m_broker(broker) {
	if (!connectProxy()) {
		ROS_ERROR("Error!");
		throw std::exception();
	}

	BIND_METHOD(TemperatureNode::temperatureCallback);
}

TemperatureNode::~TemperatureNode() {
// TODO Auto-generated destructor stub
}

void TemperatureNode::run() {
// TODO

	string eventName = "HotJointDetected";
	string callbackModule = "TemperatureNode";
	string callbackMessage = "TemperatureNode";
	string callbackMethod = "temperatureCallback";
	this->m_memoryProxy->subscribeToMicroEvent(eventName, callbackModule,
			callbackMessage, callbackMethod);
}

void TemperatureNode::temperatureCallback(const string& key,
		const ALValue& value, const ALValue& message) {
	// TODO
}

bool TemperatureNode::connectProxy() {
	if (!m_broker) {
		ROS_ERROR("Broker is not ready. Have you called connectNaoQi()?");
		return false;
	}

	try {
		m_memoryProxy = boost::shared_ptr<AL::ALMemoryProxy>(
				new AL::ALMemoryProxy(m_broker));
	} catch (const AL::ALError& e) {
		ROS_ERROR("Could not create ALMemoryProxy.");
		return false;
	}
	ROS_INFO("Proxy to ALMemory ready.");
	return true;
}

} /* namespace kinematic_calibration */

using namespace kinematic_calibration;

bool connectNaoQi();
void parse_command_line(int argc, char ** argv);

std::string m_pip;
std::string m_ip;
int m_port;
int m_pport;
std::string m_brokerName;
boost::shared_ptr<AL::ALBroker> m_broker;

void parse_command_line(int argc, char ** argv) {
	std::string pip;
	std::string ip;
	int pport;
	int port;
	boost::program_options::options_description desc("Configuration");
	desc.add_options()("help", "show this help message")("ip",
			boost::program_options::value<std::string>(&ip)->default_value(
					m_ip), "IP/hostname of the broker")("port",
			boost::program_options::value<int>(&port)->default_value(m_port),
			"Port of the broker")("pip",
			boost::program_options::value<std::string>(&pip)->default_value(
					m_pip), "IP/hostname of parent broker")("pport",
			boost::program_options::value<int>(&pport)->default_value(m_pport),
			"port of parent broker");
	boost::program_options::variables_map vm;
	boost::program_options::store(
			boost::program_options::parse_command_line(argc, argv, desc), vm);
	boost::program_options::notify(vm);
	m_port = vm["port"].as<int>();
	m_pport = vm["pport"].as<int>();
	m_pip = vm["pip"].as<std::string>();
	m_ip = vm["ip"].as<std::string>();
	cout << "pip is " << m_pip << endl;
	cout << "ip is " << m_ip << endl;
	cout << "port is " << m_port << endl;
	cout << "pport is " << m_pport << endl;

	if (vm.count("help")) {
		std::cout << desc << "\n";
		return;
	}
}

bool connectNaoQi() {
// Need this to for SOAP serialization of floats to work
	setlocale(LC_NUMERIC, "C");
// A broker needs a name, an IP and a port:
// listen port of the broker (here an anything)
	try {
		m_broker = ALBroker::createBroker(m_brokerName, m_ip, m_port, m_pip,
				m_pport, false);
	} catch (const ALError& e) {
		ROS_ERROR("Failed to connect broker to: %s:%d", m_pip.c_str(), m_port);

		return false;
	}
	cout << "broker ready." << endl;
	return true;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "TemperatureNode");
// TODO
	m_brokerName = "TemperatureNodeBroker";
	parse_command_line(argc, argv);
	connectNaoQi();
	TemperatureNode node(m_broker, m_brokerName);
	return 0;
}
