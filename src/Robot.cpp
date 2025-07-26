/*
 * Robot.cpp
 *
 *  Created on: 5 de jun de 2017
 *      Author: rsantin
 */

#include "Robot.h"

int idCount =0;

Robot::Robot() {
	// TODO Auto-generated constructor stub
	robotId=getId();
	robotName.clear();
	maxFuel=0;;
	maxVel=0;
    prop=0;
    bID =0;
}

Robot::Robot(std::string name, double vel, double fuel, double p) {
	robotName = name;
	robotId=getId();
	maxFuel=fuel;
	maxVel=vel;
	prop=p;
	baseId = NULL;
	bID =-1;
}


Robot::Robot(std::string name, Configuration config) {
	robotName = name;
	robotId=getId();
	maxFuel=config.getMaxFuel();
	maxVel=config.getMaxVel();
	prop=config.getProp();
	configName =config.getConfigName();
	baseId = NULL;
	bID=-1;
}


Robot::~Robot() {
	// TODO Auto-generated destructor stub
}


int Robot::getRobotId(){
	return robotId;
}

std::string Robot::getRobotName(){
	return robotName;
}


std::string Robot::getRobotConfigName(){
	return configName;
}


int Robot::getId(){
	return idCount++;
}

void Robot::setMaxFuel(double fuel){
	maxFuel=fuel;
}

void Robot::setMaxVel(double vel){
	maxVel=vel;
}

void Robot::setRobotName(std::string name){
	robotName = name;
}

void Robot::setProp(double p){
	prop = p;
}


void Robot::setRobotBID(int id){
	bID = id;
}


void Robot::setRobotBaseId(int* id){
	baseId = id;
}

int Robot::getRobotBID(){
	return bID;
}

int Robot::getRobotBaseId(){
	if(baseId!=NULL)
		return *baseId;

	else return -1;
}

double Robot::getProp(){
	return prop;
}

double Robot::getMaxFuel(){
	return maxFuel;
}

double Robot::getMaxVel(){
	return maxVel;
}

int *Robot::getBasePtr(){
	return baseId;

}


