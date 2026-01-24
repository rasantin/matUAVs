/*
 * Node.h
 *
 *  Created on: 2 de jun de 2017
 *      Author: rsantin
 */

#ifndef NODE_H_
#define NODE_H_

#include <iostream>
#include <string>

class Node {

private:
	double x_coord;
	double y_coord;
	std::string nodeType;
	std::string nodeBaseName;
	int getId();
	int nodeTypeId;

public:

	Node();
	Node(double x, double y,std::string type);
	Node(double x, double y,std::string type,std::string baseName);
	int nodeId;
	void setXY(double x, double y);
	void setNodeId(int id);
	void setNodeBaseName(std::string baseName);
	void setNodeType(std::string type);
	double getX() const {return x_coord;} ;
	double  getY() const {return y_coord;};
	int getNodeId() const {return nodeId;};
	int* getNodeIdPtr()  {return &nodeId;};
	int getNodeTypeId() const{return nodeTypeId;};
	std::string getNodeType() const {return nodeType;};
	std::string getNodeBaseName() const {return nodeBaseName;};

	virtual ~Node();
};

#endif /* NODE_H_ */
