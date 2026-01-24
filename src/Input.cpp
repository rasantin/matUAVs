/*
 * Input.cpp
 *
 *  Created on: 3 de jun de 2017
 *      Author: rsantin
 */

#include "Input.h"

Input::Input(std::string filename)
{
	readFile(filename);
	this->file_name = filename;
}
Input::Input()
{
}

void Input::readFile(std::string fileName)
{

	std::ifstream ifs;
	std::string line;
	std::string robotName;
	std::string configName;
	std::string baseName;

	int firstBaseNodeId = -1;
	int lastBaseNodeId = -1;
	int robotInfo = 0;

	int configInfo = 0;

	double vel;
	double fuel;
	double prop;

	std::vector<std::string> base;
	std::map<std::string, int *> basemap;

	std::map<std::string, Robot> robotsA;
	std::map<std::string, Configuration> configMap;

	std::pair<std::map<std::string, int *>::iterator, bool> ret;
	std::map<std::string, int *>::iterator it;
	std::map<std::string, Robot>::iterator itRobot;
	std::map<std::string, Configuration>::iterator itConfig;

	baseNum = 0;
	depotNum = 0;
	targetNum = 0;

	bool missingBases = false;

	std::string::size_type sz;

	ifs.open(fileName, std::ifstream::in);

	if (ifs.is_open())
	{
		std::string sectionType;

		while (ifs.good())
		{

			getline(ifs, line);

			// define operation and continue
			if (line.compare("@target") == 0)
			{
				sectionType = "target";
				continue;
			}

			else if (line.compare("@depot") == 0)
			{
				sectionType = "depot";
				continue;
			}

			else if (line.compare("@base") == 0)
			{
				sectionType = "base";
				continue;
			}
			else if (line.compare("@robots_configuration") == 0)
			{
				sectionType = "robots_configuration";
				continue;
			}

			else if (line.compare("@robot") == 0)
			{
				sectionType = "robot";
				continue;
			}
			else if (line.compare("@robot_base") == 0)
			{
				// sort node types in following depot,base and target sequence
				sortNodes(true);
				for (std::vector<Node>::iterator itNodes = nodes.begin(); itNodes != nodes.end(); ++itNodes)
				{
					if (itNodes->getNodeType().compare("base") == 0 && firstBaseNodeId < 0)
					{
						firstBaseNodeId = itNodes->getNodeId();
					}
					else if (itNodes->getNodeType().compare("target") == 0 && lastBaseNodeId < 0)
					{
						lastBaseNodeId = itNodes->getNodeId() - 1;
					}
				}
				sectionType = "robot_base";
				continue;
			}
			else if (line.compare("@param") == 0)
			{
				sectionType = "param";
				continue;
			}

			// check if the line is not empty or isn't the label line.
			if (!line.empty() && !sectionType.empty() &&
				!line.compare(sectionType) == 0 && !line.compare(" ") == 0)
			{

				// get the robots id and his base position on //base node section(@base)
				if (sectionType.compare("base") == 0)
				{
					baseName = line.substr(0, line.find("@"));
					line.assign(line.substr(line.find("@") + 1));

					try
					{
						double x = std::stod(line, &sz);
						double y = std::stod(line.substr(sz));
						nodes.emplace_back(x, y, sectionType, baseName);
						baseNum++;
						depotNum++;
					}
					catch (const std::invalid_argument &e)
					{
						std::cerr << " The function stod() could not convert the arguments " << line << " to <double, double> " << '\n';
					}
				}
				else if (sectionType.compare("robot_base") == 0)
				{ // robots section(@robots)
					robotName = line.substr(0, line.find("@"));
					baseName = (line.substr(line.find("@") + 1));
					// insert on map only base's names defined on base section (@base)
					for (int i = firstBaseNodeId; i <= lastBaseNodeId; i++)
					{
						if (nodes[i].getNodeType().compare("base") == 0 && nodes[i].getNodeBaseName().compare(baseName) == 0)
						{
							basemap.insert(std::pair<std::string, int *>(baseName, nodes[i].getNodeIdPtr()));
						}
					}

					// maps robot's name into base
					it = basemap.find(baseName);
					if (it != basemap.end())
					{
						itRobot = robotsA.find(robotName);
						if (itRobot != robotsA.end())
						{
							itRobot->second.setRobotBaseId(it->second);
						}

						for (unsigned int i = 0; i < robots.size(); i++)
						{
							if (robots[i].getRobotName().compare(robotName) == 0)
							{
								// atribui
								robots[i].setRobotBaseId(it->second);
								robots[i].setRobotBID(*(it->second));
							}
						}
					}
					continue;
				}

				// get node position  (@depot) or @target) section
				else if (sectionType.compare("target") == 0 || sectionType.compare("depot") == 0)
				{
					try
					{
						double x = std::stod(line, &sz);
						double y = std::stod(line.substr(sz));
						nodes.emplace_back(x, y, sectionType);
						if (sectionType.compare("target") == 0)
							targetNum++;
						else
							depotNum++;
					}
					catch (const std::invalid_argument &e)
					{
						std::cerr << " The function stod() could not convert the arguments " << line << " to <double, double> " << '\n';
					}
				}

				// get information about robot @robots_configuration section
				else if (sectionType.compare("robots_configuration") == 0)
				{
					if (line.find("Id") != std::string::npos)
					{
						configName.assign(line.substr(line.find(":") + 1));
						configInfo++;
					}
					else if (line.find("Vel") != std::string::npos)
					{
						line.assign(line.substr(line.find(":") + 1));
						try
						{
							vel = std::stod(line, &sz);
							configInfo++;
						}
						catch (const std::invalid_argument &e)
						{
							std::cerr << " The function stod() could not convert the arguments " << line << " to <double> " << '\n';
						}
					}
					else if (line.find("Fuel") != std::string::npos)
					{
						line.assign(line.substr(line.find(":") + 1));
						try
						{
							fuel = std::stod(line, &sz);
							configInfo++;
						}
						catch (const std::invalid_argument &e)
						{
							std::cerr << " The function stod() could not convert the arguments " << line << " to <double> " << '\n';
						}
					}
					else if (line.find("prop") != std::string::npos)
					{
						line.assign(line.substr(line.find(":") + 1));
						try
						{
							prop = std::stod(line, &sz);
							configInfo++;
						}
						catch (const std::invalid_argument &e)
						{
							std::cerr << " The function stod() could not convert the arguments " << line << " to <double> " << '\n';
						}
					}
					if (configInfo == 4)
					{
						configInfo = 0;
						Configuration configs(configName, vel, fuel, prop);
						configMap.insert(std::pair<std::string, Configuration>(configName, configs));
					}
				}
				else if (sectionType.compare("robot") == 0)
				{
					if (line.find("robot") != std::string::npos)
					{
						robotName.assign(line.substr(line.find(":") + 1));
						robotInfo++;
					}
					else if (line.find("config") != std::string::npos)
					{
						configName.assign(line.substr(line.find(":") + 1));
						robotInfo++;
					}
					if (robotInfo == 2)
					{
						itConfig = configMap.find(configName);
						itRobot = robotsA.find(robotName);

						robotInfo = 0;
						if (itConfig != configMap.end() && itRobot == robotsA.end())
						{
							Robot robot(robotName, itConfig->second);
							robots.emplace_back(robotName, itConfig->second);
							robotsA.insert(std::pair<std::string, Robot>(robotName, robot));
						}
					}
				}

				else if (sectionType.compare("param") == 0)
				{
					if (line.find("nexec") != std::string::npos)
					{
						line.assign(line.substr(line.find(":") + 1));
						try
						{
							nexec = std::stoi(line, &sz);
						}
						catch (const std::invalid_argument &e)
						{
							std::cerr << " The function stod() could not convert the arguments " << line << " to <int> " << '\n';
						}
					}
					else if (line.find("m") != std::string::npos)
					{
						line.assign(line.substr(line.find(":") + 1));
						try
						{
							m = std::stoi(line, &sz);
						}
						catch (const std::invalid_argument &e)
						{
							std::cerr << " The function stod() could not convert the arguments " << line << " to <int> " << '\n';
						}
					}
					else if (line.find("n") != std::string::npos)
					{
						line.assign(line.substr(line.find(":") + 1));
						try
						{
							n = std::stoi(line, &sz);
						}
						catch (const std::invalid_argument &e)
						{
							std::cerr << " The function stod() could not convert the arguments " << line << " to <int> " << '\n';
						}
					}
					else if (line.find("cvl_subset") != std::string::npos)
					{
						line.assign(line.substr(line.find(":") + 1));
						try
						{
							max_cvl_subset = std::stoi(line, &sz);
						}
						catch (const std::invalid_argument &e)
						{
							std::cerr << " The function stod() could not convert the arguments " << line << " to <int> " << '\n';
						}
					}
				}
			}
		}
	}
	else
	{
		std::cout << "The file" << fileName << " could not be opened" << "\n";
	}

	// change missing bases in to depot
	for (int i = firstBaseNodeId; i <= lastBaseNodeId; i++)
	{
		it = basemap.find(nodes[i].getNodeBaseName());
		if (it == basemap.end())
		{
			nodes[i].setNodeBaseName("");
			nodes[i].setNodeType("depot");
			missingBases = true;
			baseNum--;
		}
	}
	// sort vector insert the missing bases as depot;
	if (missingBases)
	{
		sortNodes(true);
		// update basemap to new position id
		for (unsigned int i = 0; i < nodes.size(); i++)
		{
			it = basemap.find(nodes[i].getNodeBaseName());
			if (it != basemap.end())
				*it->second = nodes[i].getNodeId();
		}
	}
	// remove robot's without bases
	for (unsigned int i = 0; i < robots.size(); i++)
	{
		if (robots[i].getRobotBID() < 0)
		{
			robots.erase(robots.begin() + i);
		}
	}

	// criar vetores com os indices de targets e depots.
	nodesIndexes();
	insertDepotsOnTargets();
}

void Input::nodesIndexes()
{

	nodesDepotsIndexes.clear();
	nodesTargetsIndexes.clear();

	for (Node i : nodes)
	{
		// se for target
		if (i.getNodeTypeId() == 2)
			nodesTargetsIndexes.emplace_back(i.getNodeId());
		else
			nodesDepotsIndexes.emplace_back(i.getNodeId());
	}

	sort(nodesDepotsIndexes.begin(), nodesDepotsIndexes.end());
	sort(nodesTargetsIndexes.begin(), nodesTargetsIndexes.end());
}

void Input::printNodes()
{
	for (unsigned int i = 0; i < nodes.size(); i++)
	{
		std::cout << nodes[i].getNodeType() << " " << nodes[i].getNodeId() << " ==> ( " << nodes[i].getX() << "," << nodes[i].getY() << ") \n";
	}
}

void Input::printRobots()
{
	for (unsigned int i = 0; i < robots.size(); i++)
	{
		std::cout << "Robot: " << robots[i].getRobotName() << " at base: " << robots[i].getRobotBID() << "\n";
	}
}

bool Input::nodesEmpty()
{
	return nodes.empty();
}

// ordenar em relação ao tipo e posição
void Input::sortNodes(bool changeID)
{

	int id = 0;

	auto sortRuleLambda = [](const Node &n1, const Node &n2) -> bool
	{
		if (n1.getNodeTypeId() == n2.getNodeTypeId())
			if (n1.getX() == n2.getX())
				return (n1.getY() < n2.getY());
			else
				return (n1.getX() < n2.getX());
		else
			return (n1.getNodeTypeId() < n2.getNodeTypeId());
	};

	std::sort(nodes.begin(), nodes.end(), sortRuleLambda);

	// sort nodes id
	if (changeID)
	{
		for (Node &n : nodes)
		{
			n.setNodeId(id);
			id++;
		}
	}
}

// inserir nós e não alterar os ids, changeID = false
int Input::insertNodes(double x, double y, std::string type)
{

	// insert on nodes
	nodes.emplace_back(x, y, type);

	// sort considering position(x,y), not change the id of new node
	sortNodes(false);

	// reconstruir os vetores de indices de targets e depots.
	// nodesIndexes();

	// new node id
	int nodeId = nodes.size() - 1;

	if (type.compare("target") == 0)
	{
		targetNum++;
		nodesTargetsIndexes.emplace_back(nodeId);
	}

	else
	{
		depotNum++;
		depotsInserted++;

		nodesDepotsIndexes.emplace_back(nodeId);

		//		vector<int> robotsIndex;
		//		vector<int>:: iterator itR;

		/*	//update robot pointer to baseID, find baseid address at nodes and insert next pointer into robots baseid address
			for(uint r = 0; r<robots.size();r++)
				robotsIndex.push_back(r);

			//make only k robots iterations
			while(!robotsIndex.empty()){
				itR = robotsIndex.begin();
				auto rptr = robots[*itR].getBasePtr();
				//start from the end, get the baseIDs first.
				for(int i = depotNum-1 ; i >= 0; i--){
					auto nptr = nodes[i].getNodeIdPtr();

					//if the nodes baseid and robot's baseid address are the same, then point the robot's baseid to the next node's baseid.
					if(nptr == rptr){
						robots[*itR].setRobotBaseId(nodes[i+1].getNodeIdPtr());
						robotsIndex.erase(itR);
						break;
					}
				}
				itR++;
			}*/
	}

	return nodeId;
}

// adicionar um depot na mesma posição dos targets;
void Input::insertDepotsOnTargets()
{
	Node n;

	int depotId;
	for (int t : nodesTargetsIndexes)
	{
		n = getNode(t);
		depotId = insertNodes(n.getX(), n.getY(), "depots");
		mapTargetDepot.insert(std::pair<int, int>(t, depotId));
	}
}

int Input::getDepotIdOnTarget(int id)
{
	auto it = mapTargetDepot.find(id);
	if(it != mapTargetDepot.end()){
		return it->second;
	}
	return -1;
}

bool Input::isTarget(int id)
{
	auto itMap = mapTargetDepot.find(id);

	if (itMap != mapTargetDepot.end())
		return true;

	return false;
}

Input::~Input()
{
	// TODO Auto-generated destructor stub
}

int Input::getTargetNum()
{
	return targetNum;
}

int Input::getDepotNum()
{
	return depotNum;
}

int Input::getBaseNum()
{
	return baseNum;
}

int Input::getNodesNum()
{
	return nodes.size();
}

int Input::getRobotNum()
{
	return robots.size();
}

double Input::getRobotFuel(int robot)
{
	return robots[robot].getMaxFuel();
}

double Input::getRobotVel(int robot)
{
	return robots[robot].getMaxVel();
}

double Input::getRobotProp(int robot)
{
	return robots[robot].getProp();
}

double Input::euclidianDistance(Node &a, Node &b)
{
	double dx = a.getX() - b.getX();
	double dy = a.getY() - b.getY();

	return std::sqrt((dx * dx) + (dy * dy));
}

/*void Input::initMCost(std::vector<std::vector<cell>> &G)
{
	for (unsigned int i = 0; i < nodes.size(); i++)
	{
		G.push_back(std::vector<cell>());
		for (unsigned int j = 0; j < nodes.size(); j++)
		{
			G[i].push_back(make_pair(euclidianDistance(nodes[i], nodes[j]), false));
		}
	}
}*/

void Input::initMCost(std::vector<std::vector<cell>>& G) 
{
    const size_t node_count = nodes.size();
    if (node_count == 0) return;  // Early return para caso vazio
    
    G.clear();  // Limpa qualquer conteúdo existente
    G.reserve(node_count);  // Pré-aloca as linhas
    
    for (size_t i = 0; i < node_count; ++i) {
        G.emplace_back();  // Adiciona nova linha
        G.back().reserve(node_count);  // Pré-aloca colunas
        
        for (size_t j = 0; j < node_count; ++j) {
            const double distance = euclidianDistance(nodes[i], nodes[j]);
            G[i].emplace_back(distance, false);  // Construção no local
        }
    }
}

// create a vxv matrix with all edges costs
/*void Input::calculeFuelCosts(){

	maxFuelCost.resize(getRobotNum());
	constM.resize(getRobotNum());

	for(unsigned int i =0; i<nodes.size();i++){
		F.push_back(std::vector<std::vector<double> >());
		for(unsigned int j =0; j<nodes.size();j++){
			F[i].push_back(std::vector<double> ());
			for(unsigned int k =0; k<robots.size();k++){
				F[i][j].push_back(euclidianDistance(nodes[i],nodes[j])/robots[k].getMaxVel());
				if(F[i][j][k] > maxFuelCost[k]){
					maxFuelCost[k] = F[i][j][k];
					constM[k]=robots[k].getMaxFuel()+ maxFuelCost[k];
				}
			}
		}
	}
}*/

void Input::calculeFuelCosts()
{
	const size_t node_count = nodes.size();
	const size_t robot_count = robots.size();

	// Initialize with proper sizes and zeros
	F.resize(node_count, std::vector<std::vector<double>>(node_count, 
              std::vector<double>(robot_count, 0.0)));
    
    maxFuelCost.assign(robot_count, 0.0);
    constM.assign(robot_count, 0.0);

    // Calculate all costs first
    for (size_t i = 0; i < node_count; i++) {
		for (size_t j = 0; j < node_count; j++)
		{
			for (size_t k = 0; k < robot_count; k++)
			{
				F[i][j][k] = euclidianDistance(nodes[i], nodes[j]) / robots[k].getMaxVel();
			}
		}
    }

    // Then find maximums for each robot
    for (size_t k = 0; k < robot_count; k++) {
		for (size_t i = 0; i < node_count; i++)
		{
			for (size_t j = 0; j < node_count; j++)
			{
				if (F[i][j][k] > maxFuelCost[k])
				{
					maxFuelCost[k] = F[i][j][k];
				}
			}
		}
		constM[k] = robots[k].getMaxFuel() + maxFuelCost[k];
    }
}

Node Input::getNode(int nodeId)
{
	Node node;

	for (Node n : nodes)
	{
		if (n.getNodeId() == nodeId)
		{
			node = n;
			break;
		}
	}
	return node;
}

double Input::getF(int i, int j, int k)
{
	return F[i][j][k];
}

int Input::getRobotBaseId(int k)
{
	// return robots[k].getRobotBaseId();
	return robots[k].getRobotBID();
}

// esse
double Input::getDistance(int i, int j)
{
	return euclidianDistance(nodes[i], nodes[j]);
}

double Input::getDistance(Node i, Node j)
{
	return euclidianDistance(i, j);
}

int Input::getDepotsInserted()
{
	return depotsInserted;
}

std::vector<int> Input::getDepotsIndexes()
{
	return nodesDepotsIndexes;
}

std::vector<int> Input::getTargetsIndexes()
{
	return nodesTargetsIndexes;
}

std::string Input::getRobotConfigName(int k)
{
	return robots[k].getRobotConfigName();
}

int Input::getMaxCVLSubSet()
{
	return max_cvl_subset;
}

int Input::getNExec()
{
	return nexec;
}

int Input::getN()
{
	return n;
}

int Input::getM()
{
	return m;
}
