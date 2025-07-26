/*
 * Configuration.h
 *
 *  Created on: 26 de jun de 2017
 *      Author: rsantin
 */

#ifndef CONFIGURATION_H_
#define CONFIGURATION_H_

#include <string>


class Configuration {
private:
	std::string configName;
	double maxFuel;
	double maxVel;
	double prop;
public:
	Configuration();
	Configuration(std::string name, double vel, double fuel, double p);
	std::string getConfigName();
	double getMaxVel();
	double getMaxFuel();
	double getProp();

	virtual ~Configuration();
};

#endif /* CONFIGURATION_H_ */
