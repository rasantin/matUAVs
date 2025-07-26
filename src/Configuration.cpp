/*
 * Configuration.cpp
 *
 *  Created on: 26 de jun de 2017
 *      Author: rsantin
 */

#include "Configuration.h"



Configuration::Configuration(std::string name, double vel, double fuel, double p){
	configName = name;
	maxFuel=fuel;
	maxVel=vel;
	prop=p;
}

Configuration::~Configuration() {
	// TODO Auto-generated destructor stub
}

std::string Configuration::getConfigName(){
	return configName;
}
double Configuration::getMaxFuel(){
	return maxFuel;
}
double Configuration::getMaxVel(){
	return maxVel;
}
double Configuration::getProp(){
	return prop;
}
