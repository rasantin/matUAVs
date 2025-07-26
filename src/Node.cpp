/*
 * Node.cpp
 *
 *  Created on: 2 de jun de 2017
 *      Author: rsantin
 */

#include "Node.h"

int countId=0;

Node::Node(){
	x_coord =0;
	y_coord =0;
   nodeTypeId=0;
   nodeId =0;

}

Node::Node(double x, double y, std::string type){
	x_coord = x;
	y_coord = y;
	nodeId = getId();
	nodeType = type;
	nodeBaseName.clear();

	if(nodeType.compare("depot")==0)
		nodeTypeId=0;

	else if(nodeType.compare("target")==0)
		nodeTypeId=2;


}

Node::Node(double x, double y, std::string type, std::string baseName){
	x_coord = x;
	y_coord = y;
	nodeId = getId();
	nodeType = type;
	nodeBaseName= baseName;
	nodeTypeId=1;

}


void Node::setXY(double x, double y){
	x_coord = x;
	y_coord = y;
}

void Node::setNodeId(int id){
	nodeId = id;
}

void Node::setNodeBaseName(std::string baseName){
	nodeBaseName = baseName;
}

void Node::setNodeType(std::string type){
	nodeType = type;

	if(nodeType.compare("depot")==0)
		nodeTypeId=0;

	else if(nodeType.compare("base")==0)
		nodeTypeId=1;

	else if(nodeType.compare("target")==0)
		nodeTypeId=2;

}

/*double Node::getX(){
	return x_coord;
}

double  Node::getY(){
	return y_coord;
}*/

int Node::getId(){
	return countId++;
}


Node::~Node() {
	// TODO Auto-generated destructor stub
}



