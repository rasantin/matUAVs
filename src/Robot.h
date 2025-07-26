/*
 * Robot.h
 *
 *  Created on: 5 de jun de 2017
 *      Author: rsantin
 */

#ifndef ROBOT_H_
#define ROBOT_H_

#include "Node.h"
#include "Configuration.h"
#include <string>

class Robot {

private:
	int robotId;
	std::string robotName;
	std::string configName;
	int getId();
	double maxFuel;
	double maxVel;
	double prop;
	int * baseId;
	int bID;

public:


	Robot();
	Robot(std::string name);
	Robot(std::string name, double vel, double fuel, double p);
	Robot(std::string name, Configuration config);


	virtual ~Robot();

	void setMaxFuel(double fuel);
	void setMaxVel(double vel);
	void setProp(double p);
	void setRobotName(std::string name);
	void setRobotBaseId(int* id);
	void setRobotBID(int bID);

	int getRobotId();
	std::string getRobotName();

	int getRobotBaseId();
	int getRobotBID();
	double getMaxFuel();
	double getMaxVel();
	double getProp();

	int *getBasePtr();

	std::string getRobotConfigName();



};

#endif /* ROBOT_H_ */
