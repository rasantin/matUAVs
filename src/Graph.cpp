/*
 * Graph.cpp
 *
 *  Created on: 18 de mai de 2018
 *      Author: rsantin
 */

#include "Graph.h"
#include <cassert>


	Graph::~Graph()
	{
		// TODO Auto-generated destructor stub
	}

	void Graph::reset()
	{
		// limpar todas as estruturas de dados
		graph.clear();
		nodeIdToIndex.clear();
		indexToNodeId.clear();
		graphDepotsIndexes.clear();
		graphTargetsIndexes.clear();
		mapNodesTypes.clear();
		mapRobotGroup.clear();
		mapRobotGroup_Bckup.clear();
		mapGroupRobot.clear();
		mapGroupRobot_Bckup.clear();
		map_cvset_id_to_node_id.clear();
		min_fuel.clear();
		nGraphs.clear();
		coverageSets.clear();
		coverage_set.first.clear();
		coverage_set.second = graphInfo();
		cLines = 0;
		nRobots = 0;
		nLines = 0.0;
		nodesX.clear();
		cvl_subset_num = 5;
		nodesSets.clear();
		nodesSets_Bckup.clear();
		link_nid_to_ninfo.clear();
		map_nodes_on_cl.clear();
	}

	// build initial complete graph
	void Graph::buildGraph()
	{
		graph.clear();
		unsigned int vertices = input.getNodesNum();

		// Criar a matriz NxN
		graph.resize(vertices, std::vector<double>(vertices, 0.0));

		int index_n1 = 0;
		for (const Node &n1 : input.nodes)
		{
			// Classificar o tipo do nó
			switch (n1.getNodeTypeId())
			{
			case 0: // Depot
				graphDepotsIndexes.emplace_back(n1.getNodeId());
				mapNodesTypes.emplace(std::make_pair(n1.getNodeId(),0));
				break;
			case 1: // Base
				mapNodesTypes[n1.getNodeId()] = 1;
				break;
			case 2: // Target
				graphTargetsIndexes.emplace_back(n1.getNodeId());
				mapNodesTypes.emplace(std::make_pair(n1.getNodeId(),2));				
				break;
			}
			index_n1++;
		}

		// Preencher os custos entre todos os pares (grafo completo)
		for (const Node &n1 : input.nodes)
		{
			for (const Node &n2 : input.nodes)
			{
				int i = n1.getNodeId();
				int j = n2.getNodeId();
				graph[i][j] = input.getDistance(n1, n2);
			}
		}

		std::sort(graphTargetsIndexes.begin(), graphTargetsIndexes.end());
		std::sort(graphDepotsIndexes.begin(), graphDepotsIndexes.end());
	}

	// print the initial complete graph
	void Graph::printGraph()
	{
		for (unsigned int i = 0; i < graph.size(); i++)
		{
			for (unsigned int j = 0; j < graph.size(); j++)
			{
				std::cout << graph[i][j] << " ";
			}
			std::cout << "\n";
		}
	}

	void Graph::splitGraph()
	{

		// Declaring a set that will store the pairs using above comparison
		std::map<int, double> mapOfLines;

		nRobots = input.getRobotNum();
		cLines = input.getTargetNum() / 2;
		nLines = (cLines / nRobots);

		// dividir a quantidade de linhas de cobertura (cLines) por cada robô disponível (nRobots)
		// com a menor variação das quantidades entre os grupos

		// vetor com número de linhas por robô
		std::vector<int> nLGroup;

		// quantidade de robôs que receberão o maior número linhas
		int nLinesVMax = cLines - (floor(nLines) * nRobots);
		// maior valor para grupo de linhas que tem quantidade nLinesVMax
		int vMax = ceil(nLines);
		// menor valor para grupo de linhas que tem quantidade nLinesVMin
		int vMin = floor(nLines);
		// definido a quantidade de robôs utilizando o maior número de linhas
		// determinamos o número de linhas restante (resto) para os robôs
		// que terão os grupos de menor quantidade de linhas
		int resto = cLines - (nLinesVMax * ceil(nLines));
		// calcular o número de robô para a menor quantidade de linhas
		int nLinesVMin = resto / floor(nLines);

		for (int i = 0; i < nLinesVMax; i++)
			nLGroup.push_back(vMax);

		for (int i = 0; i < nLinesVMin; i++)
			nLGroup.push_back(vMin);

		int split = 0;
		int ind = 0;

		// indices dos nós
		std::vector<int> index;

		// create vector of all targets indices
		for (int i = input.getDepotNum(); i < input.getNodesNum(); i = i + 2)
			index.push_back(input.nodes[i].nodeId);

		// agrupar todas as linhas
		while (!index.empty())
		{
			mapOfLines.clear();
			// linha de cobertura de referência, começando da linha mais à esquerda
			std::vector<int>::iterator i = index.begin();

			// obter as distâncias dos nós i e i+1 da linha de referência  em relação aos demais nós.
			mapOfLines.insert(std::pair<int, double>(*i, 0));
			for (std::vector<int>::iterator j = index.begin() + 1; j != index.end(); ++j)
			{

				double value = (graph[*i][*j] + graph[*i + 1][*j + 1]);

				// build a map of lines
				mapOfLines.insert(std::pair<int, double>(*j, value));
			}
			// cout <<"-------------------------\n";

			// criar o conjunto ordenado (crescente) das distâncias entre a linha representada
			// pelo indice i
			std::set<std::pair<int, double>, Comparator> setOfLines(
				mapOfLines.begin(), mapOfLines.end(), compFunctor);

			// splitting setOfLines in n nearest lines(nlines).
			// each lines is represented for one node index.
			// divisão do conjunto stLines em n linhas mais próximas
			nodesSets.push_back(Set());

			// obter a quantidad de linhas do grupo
			split = nLGroup.back();

			// remover a quantidade de linhas do grupo do vetor
			nLGroup.pop_back();
			int sum = 0;
			for (std::set<std::pair<int, double>, Comparator>::iterator it = setOfLines.begin(); it != setOfLines.end(); ++it)
			{
				if (sum < split)
				{
					// encontrar o índice da linha mais próxima da linha de referência e removê-la
					//  de index que controla a rodada de busca.
					for (std::vector<int>::iterator vi = index.begin(); vi != index.end(); vi++)
					{
						if ((*it).first == *vi)
						{
							// adicionar o índice da linha ao grupo
							nodesSets[ind].cvLines.push_back((*it).first);
							index.erase(vi);
							break;
						}
					}
				}
				sum++;
			}
			setOfLines.clear();
			ind++;
		}

		// ordenar  nós relativo a posição X de cada nó em nodesX;
		sortNodesX();

		// obter apenas os depots que estão entre as linhas de cobertura
		for (unsigned int i = 0; i < nodesSets.size(); i++)
		{
			auto result = minmax_element(nodesSets[i].cvLines.begin(), nodesSets[i].cvLines.end());
			nodesSets[i].depots = getDepotsBetweenNodes(*result.first, *result.second);
		}

		std::cout << "Numero de grupos: " << nodesSets.size() << "\n";
		for (unsigned int i = 0; i < nodesSets.size(); i++)
		{
			std::cout << "Grupo: " << i << std::endl;

			std::cout << "Targets: " << "=>";
			for (unsigned int j = 0; j < nodesSets[i].cvLines.size(); j++)
			{
				std::cout << nodesSets[i].cvLines[j] << " ";
			}
			std::cout << "\n";

			std::cout << "Depots: " << "=>";
			for (unsigned int j = 0; j < nodesSets[i].depots.size(); j++)
			{
				std::cout << nodesSets[i].depots[j] << " ";
			}
			std::cout << "\n";
		}
		std::vector<std::pair<int, double>> setOfAreas;

		setOfAreas = getSetsArea();

		std::vector<std::pair<int, double>> setOfRobots;

		for (int i = 0; i < input.getRobotNum(); i++)
			setOfRobots.push_back(std::make_pair(i, input.getRobotProp(i)));

		// ordena o robôs do pior desempenho ao melhor, ou seja do maior prop(menor tempo de voo) para o maior
		std::sort(setOfRobots.begin(), setOfRobots.end(), compFunctor2);

		for (unsigned int i = 0; i < setOfAreas.size(); i++)
		{
			int setID = setOfAreas[i].first;
			int robotID = setOfRobots[i].first;

			nodesSets[setID].robotID = robotID;

			std::cout << "area id:" << setID << " robotID: " << robotID << " prop: " << setOfRobots[i].second << " area: " << setOfAreas[i].second << std::endl;
		}
	}
	void Graph::splitHGraph()
	{

		// Declaring a set that will store the pairs using above comparison
		// set<pair<int, double>,Comparator> setOfLines;
		std::map<int, double> mapOfLines;

		std::vector<Set> teste;

		double cvLinesSum = 0;
		double length = 0;

		std::vector<std::pair<int, double>> cvLength;

		// indices dos nós
		std::vector<int> index;

		// calcular a distância total ao percorrer todas as linhas de cobertura.
		// Não consideramos o deslocamento para a linha

		// targets on graph
		int t1, t2 = 0;
		for (uint i = 0; i < graphTargetsIndexes.size(); i = i + 2)
		{
			t1 = graphTargetsIndexes[i];

			// se o t1 for o últino target não é possível constituir uma linha de cobertura
			if (t1 < graphTargetsIndexes.back())
				t2 = graphTargetsIndexes[i + 1];
			else
				break;

			// create vector of all targets indices
			index.push_back(t1);

			// obter o tamanho da linha
			double length = (graph[t1][t2]);

			// somatório dos tamanhos das linhas de coberturas.
			cvLinesSum += length;
		}

		// inicializa um vetor com o pair do índice do robô e a sua proporção
		std::vector<std::pair<int, double>> setOfRobots;

		// insere o indice e a proporção do robô
		for (int i = 0; i < input.getRobotNum(); i++)
			setOfRobots.push_back(std::make_pair(i, input.getRobotProp(i)));

		// ordena o robôs do pior  desempenho ao melhor, ou seja do maior prop(menor tempo de voo) para o maior
		std::sort(setOfRobots.begin(), setOfRobots.end(), compFunctor);

		// vetor com o desempenho de cobertura dos robôs relacioados ao melhor robô (menor prop)
		std::vector<std::pair<int, double>> robotsCVPerformance;

		// vetor com o desempenho dos robôs relacioados ao melhor robô (menor prop)
		std::vector<std::pair<int, double>> robotsPropPerformance;

		// obter a melhor performance
		double bestPropPerformance = (*setOfRobots.begin()).second;

		int robotID = 0;
		int set_id = 0;
		double prop = 0;
		double propSum = 0;

		// obter a relação da performance do melhor robô em relação a todos os robôs
		for (uint i = 0; i < setOfRobots.size(); i++)
		{
			robotID = setOfRobots[i].first;
			prop = (1 / (setOfRobots[i].second / bestPropPerformance));
			robotsPropPerformance.push_back(std::make_pair(robotID, prop));
			propSum += prop;
		}

		// calcula a distância que o melhor robô deverá percorrer das linhas de cobertura
		double totalCVLenghtBestRobot = cvLinesSum / propSum;
		double totalCVLenghtRobot = 0;

		// calcula a distância que cada robô percorrerá das linha de cobertura
		for (unsigned int i = 0; i < robotsPropPerformance.size(); i++)
		{
			totalCVLenghtRobot = totalCVLenghtBestRobot * robotsPropPerformance[i].second;
			robotID = robotsPropPerformance[i].first;
			robotsCVPerformance.push_back(std::make_pair(robotID, totalCVLenghtRobot));
		}

		std::vector<std::pair<int, double>>::iterator itRobots = robotsCVPerformance.begin();
		std::vector<int>::iterator itIndex = index.begin();

		// inserir a linha e o seu tamanho no vetor
		for (int i : index)
		{
			double length = (graph[i][i + 1]);
			cvLength.push_back(std::make_pair(i, length));
		}

		// verificar se as linhas são congruentes
		std::pair<std::vector<std::pair<int, double>>::iterator, std::vector<std::pair<int, double>>::iterator> bounds;

		bounds = std::equal_range(cvLength.begin(), cvLength.end(), cvLength.front(), [](const std::pair<int, double> &p1, const std::pair<int, double> &p2)
								   { return p1.second < p2.second; });

		// se todas as linha forem congruentes
		if (cvLength.size() == uint(bounds.second - cvLength.begin()))
		{
			while (!cvLength.empty())
			{

				// apontar para o vetor de tamanhos das linhas de cobertura
				std::vector<std::pair<int, double>>::iterator itLength = cvLength.begin();

				// iniciar bestCapacity com um número pequeno. Esta variável armazenará a melhor capacidade disponível para a frota.
				double bestCapacity = std::numeric_limits<double>::min();

				// iterator para encontrar o robô com melhor capacidade disponível
				std::vector<std::pair<int, double>>::iterator bestRobot;

				// entre todos os robô verificar qual possui melhor capacidade
				for (std::vector<std::pair<int, double>>::iterator itR = robotsCVPerformance.begin(); itR != robotsCVPerformance.end(); ++itR)
				{

					if (bestCapacity < (*itR).second)
					{
						bestRobot = itR;
						bestCapacity = (*itR).second;
					}
				}

				// encontrada a maior capacidade disponível, verificar se o robô pode sobrevoar a maior linha de cvLength.
				// caso não haja capacidade disponível, alocá-la ao melhor robô.
				if (bestCapacity < (*itLength).second)
				{

					// procurar a posição do nodesSet que contenha o ID do robô que irá recebê-la
					std::vector<Set>::iterator itNodeSet;
					for (std::vector<Set>::iterator itSet = nodesSets.begin(); itSet != nodesSets.end(); ++itSet)
					{
						if ((*bestRobot).first == (*itSet).robotID)
						{
							itNodeSet = itSet;
							break;
						}
					}

					// se o robô ainda não recebeu nenhuma CV, criar um conjunto vazio e inserir as informações no final
					if (itNodeSet == nodesSets.end())
					{
						nodesSets.push_back(Set());
						// inserir as informações no nodesSet
						nodesSets.back().cvLines.push_back((*itLength).first);
						nodesSets.back().robotID = (*bestRobot).first;
						nodesSets.back().length += (*itLength).second;

						// inserir o índice do grupo
						nodesSets.back().set_id = set_id++;
					}
					// caso o grupo já tenha recebido alguma linha
					else
					{
						// verificar se última se a linha restante pode ser anexada ao grupo, dado a capacidade do robô
						int id_cl_on_ns = (*itNodeSet).cvLines.back();
						double dist = graph[id_cl_on_ns][(*itLength).first];
						double max_dist_robot = input.getRobotFuel((*bestRobot).first) * input.getRobotVel(((*bestRobot).first));

						// inserir ao robô se for viável
						if (dist < max_dist_robot)
						{
							// inserir as informações no nodesSet
							(*itNodeSet).cvLines.push_back((*itLength).first);
							(*itNodeSet).robotID = (*bestRobot).first;
							(*itNodeSet).length += (*itLength).second;
							// diminuir a capacidade do robô
						}
						// caso contrário inserir no grupo que possui a linha mais próxima

						else
						{
							nodesSets.back().cvLines.push_back((*itLength).first);
							nodesSets.back().length += (*itLength).second;
							// apontar o bestRobot para o robô do último grupo para que seja possível reduzir a sua capacidade
							for (std::vector<std::pair<int, double>>::iterator itR = robotsCVPerformance.begin(); itR != robotsCVPerformance.end(); ++itR)
							{
								if (itR->first == nodesSets.back().robotID)
								{
									bestRobot = itR;
									break;
								}
							}
						}
					}
					(*bestRobot).second = (*bestRobot).second - (*itLength).second;
					// a linha é removida do vetor.
					cvLength.erase(itLength);
					continue;
				}

				// Se os robôs não receberam linhas de cobertura. Inserir a sequencia de linhas
				if (nodesSets.size() < robotsCVPerformance.size())
				{
					// Para cada robô disponível, inserir a maior linha de acordo com a sua capacidade
					for (std::vector<std::pair<int, double>>::iterator itR = robotsCVPerformance.begin(); itR != robotsCVPerformance.end(); ++itR)
					{

						// se a capacidade do robô está esgotada, pegar o próximo robô
						if ((*itR).second < 0)
							continue;

						// insertir um conjunto vazio
						nodesSets.push_back(Set());
						nodesSets.back().robotID = (*itR).first;
						nodesSets.back().set_id = set_id++;

						auto itLength = cvLength.begin();
						// para cada linha a ser cobertura, começando pela maior linha cv
						while (itLength != cvLength.end())
						{

							// se a capacidade do robô for suficiente para a linha cv
							if ((*itR).second >= (*itLength).second)
							{
								// inseria o índice da linha, id do robot e o tamanho da linha
								nodesSets.back().cvLines.push_back((*itLength).first);
								nodesSets.back().length += (*itLength).second;
								// diminuir a capacidade do robô no vetor
								(*itR).second = (*itR).second - (*itLength).second;
								itLength = cvLength.erase(itLength);
							}
							else
								break;
						}
					}
				}
			}
		}

		else
		{ // se as linhas forem diferentes, ordenar as linhas em relação a extensão e alocar as k primeiras linhas para os k vants,
			// posteriormeter alocar de acordo com a distância entre a linha atribuída ao grupo.
			// ordenar da maior linha à menor
			std::sort(cvLength.begin(), cvLength.end(), [](const std::pair<int, double> &p1, const std::pair<int, double> &p2)
				 {
			if(isDefinitelyGreaterThan(p1.second,p2.second,1.0))
				return true;
			else if(isApproximatelyEqual(p1.second,p2.second))
				return p1.first < p2.first;
			return false; });

			// alocar  todas as linhas para os robôs
			while (!cvLength.empty())
			{

				// apontar para o vetor de tamanhos das linhas de cobertura
				std::vector<std::pair<int, double>>::iterator itLength = cvLength.begin();

				// iniciar bestCapacity com um número pequeno. Esta variável armazenará a melhor capacidade disponível para a frota.
				double bestCapacity = std::numeric_limits<double>::min();

				// iterator para encontrar o robô com melhor capacidade disponível
				std::vector<std::pair<int, double>>::iterator bestRobot;

				// entre todos os robô verificar qual possui melhor capacidade
				for (std::vector<std::pair<int, double>>::iterator itR = robotsCVPerformance.begin(); itR != robotsCVPerformance.end(); ++itR)
				{

					if (bestCapacity < (*itR).second)
					{
						bestRobot = itR;
						bestCapacity = (*itR).second;
					}
				}
				// encontrada a maior capacidade disponível, verificar se o robô pode sobrevoar a maior linha de cvLength.
				// caso não haja capacidade disponível, alocá-la ao melhor robô.
				if (bestCapacity < (*itLength).second)
				{

					// procurar a posição do nodesSet que contenha o ID do robô que irá recebê-la
					std::vector<Set>::iterator itNodeSet;
					for (std::vector<Set>::iterator itSet = nodesSets.begin(); itSet != nodesSets.end(); ++itSet)
					{
						if ((*bestRobot).first == (*itSet).robotID)
						{
							itNodeSet = itSet;
							break;
						}
					}
					// se o robô ainda não recebeu nenhuma CV, criar um conjunto vazio e inserir as informações no final
					if (itNodeSet == nodesSets.end())
					{
						nodesSets.push_back(Set());
						// inserir as informações no nodesSet
						nodesSets.back().cvLines.push_back((*itLength).first);
						nodesSets.back().robotID = (*bestRobot).first;
						nodesSets.back().length += (*itLength).second;

						// inserir o índice do grupo
						nodesSets.back().set_id = set_id++;
					}
					// caso o grupo já tenha recebido algum linha
					else
					{
						// inserir as informações no nodesSet
						(*itNodeSet).cvLines.push_back((*itLength).first);
						(*itNodeSet).robotID = (*bestRobot).first;
						(*itNodeSet).length += (*itLength).second;
					}
					// diminuir a capacidade do robô
					(*bestRobot).second = (*bestRobot).second - (*itLength).second;

					// a linha é removida do vetor.
					cvLength.erase(itLength);
					continue;
				}

				// Se os robôs não receberam linhas de cobertura. Inserir a maior linha possível
				// para cada robô
				if (nodesSets.size() < robotsCVPerformance.size())
				{
					// Para cada robô disponível, inserir a maior linha de acordo com a sua capacidade
					for (std::vector<std::pair<int, double>>::iterator itR = robotsCVPerformance.begin(); itR != robotsCVPerformance.end(); ++itR)
					{

						// se a capacidade do robô está esgotada, pegar o próximo robô
						if ((*itR).second < 0)
							continue;

						// para cada linha a ser cobertura, começando pela maior linha cv
						for (std::vector<std::pair<int, double>>::iterator itLength = cvLength.begin(); itLength != cvLength.end(); itLength++)
						{

							// se a capacidade do robô for suficiente para a linha cv
							if ((*itR).second >= (*itLength).second)
							{
								// insertir um conjunto vazio
								nodesSets.push_back(Set());

								// inseria o índice da linha, id do robot e o tamanho da linha
								nodesSets.back().cvLines.push_back((*itLength).first);
								nodesSets.back().robotID = (*itR).first;
								nodesSets.back().length += (*itLength).second;
								nodesSets.back().set_id = set_id++;
								// diminuir a capacidade do robô no vetor
								(*itR).second = (*itR).second - (*itLength).second;
								cvLength.erase(itLength);
								break;
							}
						}
					}
				}
				// para cada linha inserida no nodesSet procurar a linha mais próxima que possar ser atribuída ao robô
				else
				{

					int cvIndex = 0;

					// para cada grupo, obter a linha inserida.
					for (std::vector<Set>::iterator itNodes = nodesSets.begin(); itNodes != nodesSets.end(); ++itNodes)
					{

						// obter o id do robô do grupo
						int robotID = (*itNodes).robotID;

						// iterador para indicar qual é a performance do robô
						std::vector<std::pair<int, double>>::iterator itRobot;

						// encontrar a performance do robô alocado no grupo
						for (std::vector<std::pair<int, double>>::iterator itR = robotsCVPerformance.begin(); itR != robotsCVPerformance.end(); ++itR)
						{
							// apontar para a performance do robô
							if (robotID == (*itR).first)
							{
								itRobot = itR;
								break;
							}
						}

						// verificar se o robô possui capacidade suficiente para que outra linhas
						// possa ser atribuída a ele.
						if ((*itRobot).second > 0)
						{
							cvIndex = (*itNodes).cvLines.front();
							mapOfLines.clear();

							// obter as distâncias dos nós i e i+1 da linha do grupo  em relação às demais linhas
							for (std::vector<std::pair<int, double>>::iterator j = cvLength.begin(); j != cvLength.end(); ++j)
							{

								int nextCv = (*j).first;
								double value = (graph[cvIndex][nextCv] + graph[cvIndex + 1][nextCv + 1]);

								// build a map of lines
								mapOfLines.insert(std::pair<int, double>(nextCv, value));
							}

							// criar o conjunto ordenado (crescente) das distâncias entre a linha
							/*set<pair<int,double>,Comparator> setOfLines(
								mapOfLines.begin(), mapOfLines.end(), compFunctor);

						//verificar a partir da mais próxima, qual linha pode ser atribuída ao robô
						for(set<pair<int,double>, Comparator>::iterator it=setOfLines.begin();
								it!= setOfLines.end();++it){
							 */

							std::vector<std::pair<int, double>> setOfLines;
							setOfLines.insert(setOfLines.begin(), mapOfLines.begin(), mapOfLines.end());
							std::sort(setOfLines.begin(), setOfLines.end(), compFunctor);

							// verificar a partir da mais próxima, qual linha pode ser atribuída ao robô
							for (std::vector<std::pair<int, double>>::iterator it = setOfLines.begin();
								 it != setOfLines.end(); ++it)
							{

								// calcular o tamanho da linha
								length = graph[(*it).first][(*it).first + 1];

								// se o tamanho da linha for menor ou igual  a capacidade do robô
								if (length <= (*itRobot).second)
								{
									// adicionar o índice da linha ao grupo
									(*itNodes).cvLines.push_back((*it).first);
									(*itNodes).length += length;
									// dim*vi+1inuir a capacidade do robô
									(*itRobot).second = (*itRobot).second - length;

									// atualizar o vetor de tamanhos de linhas de cobertura
									for (std::vector<std::pair<int, double>>::iterator itLength = cvLength.begin(); itLength != cvLength.end(); ++itLength)
									{
										if ((*it).first == (*itLength).first)
										{
											cvLength.erase(itLength);
											break;
										}
									}
								}
							}
							setOfLines.clear();
						}
					}
				}
			}
		}

		std::cout << "Numero de grupos: " << nodesSets.size() << "\n";
		for (unsigned int i = 0; i < nodesSets.size(); i++)
		{
			std::cout << "Grupo: " << i << std::endl;

			std::cout << "Targets: " << "=>";
			for (unsigned int j = 0; j < nodesSets[i].cvLines.size(); j++)
			{
				std::cout << nodesSets[i].cvLines[j] << " ";
			}
			std::cout << "\n";

			std::cout << "Depots: " << "=>";
			for (unsigned int j = 0; j < nodesSets[i].depots.size(); j++)
			{
				std::cout << nodesSets[i].depots[j] << " ";
			}
			std::cout << "\n";
		}

		// sort node ids considering the position
		sortNodesX();

		input.maxFuelCost.resize(nodesSets.size());
		input.constM.resize(nodesSets.size());

		for (Node n : nodesX)
			link_nid_to_ninfo.emplace(n.nodeId, n);
	}

	// encontrado o nodeset, dividí-lo
	void Graph::SplitSubGraph()
	{
		float total_coverage_lines_num;
		float group_coverage_lines_num;

		float groups_num;
		int max_coverage_lines_num = cvl_subset_num;
		int groups_max_cl_num;
		int groups_min_cl_num;
		int resto;
		int vMax;
		int vMin;
		int split;
		std::map<int, double> mapOfLines;
		std::vector<int> ordered_index;
		std::vector<int> sub_set_size;

		if (max_coverage_lines_num > 1)
		{

			int nodes_set_id = 0;
			auto it_ns = nodesSets.begin();
			while (it_ns != nodesSets.end())
			{

				total_coverage_lines_num = it_ns->cvLines.size();
				groups_num = ceil(total_coverage_lines_num / max_coverage_lines_num);
				group_coverage_lines_num = total_coverage_lines_num / groups_num;

				vMax = ceil(group_coverage_lines_num);
				vMin = floor(group_coverage_lines_num);

				groups_max_cl_num = total_coverage_lines_num - (vMin * groups_num);

				resto = total_coverage_lines_num - (groups_max_cl_num * vMax);
				groups_min_cl_num = resto / vMin;

				sub_set_size.clear();
				for (int i = 0; i < groups_max_cl_num; i++)
					sub_set_size.push_back(vMax);

				for (int i = 0; i < groups_min_cl_num; i++)
					sub_set_size.push_back(vMin);

				std::cout << "divisão de nodeSet: " << nodes_set_id++ << std::endl;
				for (int i : sub_set_size)
					std::cout << " " << i;
				std::cout << std::endl;

				if (sub_set_size.size() > 1)
				{
					// indices dos nós
					std::vector<int> v_ids;

					// create vector of all targets indices
					v_ids = it_ns->cvLines;

					it_ns->sub_set.clear();

					// first sub_set index on vector
					auto i = v_ids.begin();

					// agrupar todas as linhas
					while (!v_ids.empty())
					{

						// i é um iterator para o v_id
						i = v_ids.begin();

						// obter a quantidad de linhas do sub grupo
						split = sub_set_size.back();
						// remover a quantidade de linhas do sub grupo do vetor
						sub_set_size.pop_back();

						// obter as distâncias dos nós i e i+1 da linha de referência  em relação aos demais nós.
						mapOfLines.insert(std::pair<int, double>(*i, 0));
						for (std::vector<int>::iterator j = v_ids.begin() + 1; j != v_ids.end(); ++j)
						{

							double value = (graph[*i][*j] + graph[*i + 1][*j + 1]);

							// build a map of lines
							mapOfLines.insert(std::pair<int, double>(*j, value));
						}
						// criar o conjunto ordenado (crescente) das distâncias entre a linha representada

						// pelo indice i
						std::vector<std::pair<int, double>> setOfLines;
						setOfLines.insert(setOfLines.begin(), mapOfLines.begin(), mapOfLines.end());
						std::sort(setOfLines.begin(), setOfLines.end(), compFunctor);

						if (setOfLines.size() != mapOfLines.size())
							std::cout << "problema subset" << std::endl;

						ordered_index.clear();
						for (std::pair<int, double> el : setOfLines)
							ordered_index.emplace_back(el.first);

						// inicializar um novo sub_set
						it_ns->sub_set.push_back(sub_set_data());

						// inserir a quantidade de índices no sub-grupo
						auto end_id = ordered_index.begin();

						// avançar o iterator para o split do vetor ordered
						std::advance(end_id, split);

						std::vector<int> temp = ordered_index;

						try
						{
							temp.resize(split);
							temp.shrink_to_fit();
							it_ns->sub_set.back().cvLines = temp;
							it_ns->sub_set.back().depots = it_ns->depots;
							// remover de index os índices inseridos no sub grupo
							ordered_index.erase(ordered_index.begin(), end_id);
						}
						catch (const std::exception &x)
						{
							std::cerr << "Problem  on Subset  update!!" << '\n';
						}

						v_ids = ordered_index;

						mapOfLines.clear();
					}
				}
				++it_ns;
			}
		}
	}

	// função para atualizar o subset de um nodeset particular.
	// é necessário recalcular a quantidade de linhas, visto que as funções de vizinhança podem alterar a quantidade de linhas do nodeset (shift e swap)
	void Graph::UpdateSubSet(int nodeset)
	{

		float total_coverage_lines_num;
		float group_coverage_lines_num;

		float groups_num;
		int max_coverage_lines_num = cvl_subset_num;
		int groups_max_cl_num;
		int groups_min_cl_num;
		int resto;
		int vMax;
		int vMin;
		int split;
		std::map<int, double> mapOfLines;
		std::vector<int> sub_set_size;
		std::vector<int> ordered_index;
		std::vector<int> global_depots;

		if (max_coverage_lines_num > 1)
		{

			// iterator para um set particular
			auto it_ns = nodesSets.begin() + nodeset;

			if (it_ns != nodesSets.end())
			{

				total_coverage_lines_num = it_ns->cvLines.size();

				if (total_coverage_lines_num <= max_coverage_lines_num)
				{
					it_ns->sub_set.clear();
					return;
				}

				groups_num = ceil(total_coverage_lines_num / max_coverage_lines_num);
				group_coverage_lines_num = total_coverage_lines_num / groups_num;

				vMax = ceil(group_coverage_lines_num);
				vMin = floor(group_coverage_lines_num);

				groups_max_cl_num = total_coverage_lines_num - (vMin * groups_num);

				resto = abs(total_coverage_lines_num - (groups_max_cl_num * vMax));

				// caso não tenhamos um mínimo suficiente
				if (vMin < 1)
				{
					it_ns->sub_set.clear();
					return;
				}

				groups_min_cl_num = resto / vMin;

				sub_set_size.clear();
				for (int i = 0; i < groups_max_cl_num; i++)
					sub_set_size.push_back(vMax);

				for (int i = 0; i < groups_min_cl_num; i++)
					sub_set_size.push_back(vMin);

				if (sub_set_size.size() > 1)
				{
					// indices dos nós
					std::vector<int> v_ids;

					// create vector of all targets indices
					v_ids = it_ns->cvLines;

					it_ns->sub_set.clear();

					// first sub_set index on vector
					auto i = v_ids.begin();

					// agrupar todas as linhas
					while (!v_ids.empty())
					{

						// i é um iterator para o v_id
						i = v_ids.begin();

						// obter a quantidad de linhas do sub grupo
						split = sub_set_size.back();
						// remover a quantidade de linhas do sub grupo do vetor
						sub_set_size.pop_back();

						// obter as distâncias dos nós i e i+1 da linha de referência  em relação aos demais nós.
						mapOfLines.insert(std::pair<int, double>(*i, 0));
						for (std::vector<int>::iterator j = v_ids.begin() + 1; j != v_ids.end(); ++j)
						{

							double value = (graph[*i][*j] + graph[*i + 1][*j + 1]);

							// build a map of lines
							mapOfLines.insert(std::pair<int, double>(*j, value));
						}
						// criar o conjunto ordenado (crescente) das distâncias entre a linha representada

						// pelo indice i
						std::vector<std::pair<int, double>> setOfLines;
						setOfLines.insert(setOfLines.begin(), mapOfLines.begin(), mapOfLines.end());
						std::sort(setOfLines.begin(), setOfLines.end(), compFunctor);

						ordered_index.clear();
						for (std::pair<int, double> el : setOfLines)
							ordered_index.emplace_back(el.first);

						// inicializar um novo sub_set
						it_ns->sub_set.push_back(sub_set_data());

						// inserir a quantidade de índices no sub-grupo
						auto end_id = ordered_index.begin();

						// avançar o iterator para o split do vetor ordered
						std::advance(end_id, split);

						std::vector<int> temp = ordered_index;

						try
						{
							temp.resize(split);
							temp.shrink_to_fit();
							it_ns->sub_set.back().cvLines = temp;
							it_ns->sub_set.back().depots = it_ns->depots;
							// remover de index os índices inseridos no sub grupo
							if (end_id <= ordered_index.end())
								ordered_index.erase(ordered_index.begin(), end_id);
						}
						catch (const std::exception &x)
						{
							std::cerr << "Problem  on Subset  update!!" << '\n';
						}

						v_ids = ordered_index;

						mapOfLines.clear();
					}
				}

				++it_ns;
			}
		}
	}

	void Graph::mapRobotTypeGroups()
	{
		std::vector<std::string> robotType_temp;
		std::string sType;
		uint ndiff = 0;
		int robotID;
		int type = 0;
		for (uint i = 0; i < nodesSets.size(); i++)
		{
			ndiff = 0;
			robotID = nodesSets[i].robotID;
			sType = input.getRobotConfigName(robotID);

			auto itMap = mapRobotGroup.end();

			if (robotType_temp.empty())
			{
				robotType_temp.emplace_back(sType);
				itMap = mapRobotGroup.emplace_hint(itMap, type, std::set<int>());
				itMap->second.insert(i);
				;
			}

			else
			{
				// inserir os robôs de tipos diferentes
				// for(auto itTemp = robotType_temp.begin();itTemp!=robotType_temp.end();++itTemp){
				for (uint typeID = 0; typeID < robotType_temp.size(); ++typeID)
				{
					if (sType.compare(robotType_temp[typeID]) == 0)
					{
						itMap = mapRobotGroup.emplace_hint(itMap, typeID, std::set<int>());
						itMap->second.insert(i);
						break;
					}

					else
						ndiff++;
				}
				if (ndiff == robotType_temp.size())
				{
					type = robotType_temp.size();
					robotType_temp.emplace_back(sType);
					itMap = mapRobotGroup.emplace_hint(itMap, type, std::set<int>());
					itMap->second.insert(i);
				}
			}
		}
		// imprimir o mapa de tipos de robôs e seus respectivos grupos
		for (std::pair<int, std::set<int>> p : mapRobotGroup)
		{
			std::cout << "Tipo Robo: " << p.first << " Grupo: ";
			for (int grupo : p.second)
			{
				std::cout << ' ' << grupo;
			}

			std::cout << "\n";
		}
	}

	// inserir postos nas localizações dos targets
	void Graph::insertDepotsOnNodesSets()
	{
		for (Set &set : nodesSets)
		{
			for (int cvL : set.cvLines)
			{
				set.depots.emplace_back(input.getDepotIdOnTarget(cvL));
				set.depots.emplace_back(input.getDepotIdOnTarget(cvL) + 1);
			}
		}
	}

	std::vector<std::pair<int, double>> Graph::getSetsArea()
	{

		int node_a, node_b, prev_a, prev_b;
		prev_a = prev_b = -1;
		double sum;
		std::vector<std::pair<int, double>> setsArea;
		for (unsigned int i = 0; i < nodesSets.size(); i++)
		{

			for (unsigned int j = 0; j < nodesSets[i].cvLines.size(); j++)
			{
				node_a = nodesSets[i].cvLines[j];
				node_b = nodesSets[i].cvLines[j] + 1;

				// obter o tamanho da linha de cobertura do inicio e fim de cada grupo
				if ((prev_a < 0 && prev_b < 0) || (j == nodesSets[i].cvLines.size() - 1))
				{
					sum = sum + input.getDistance(node_a, node_b);
					std::cout << "node_a: " << node_a << " node_b: " << node_b << " " << std::endl;
				}

				if (prev_a >= 0 && prev_b >= 0)
				{
					sum = sum + input.getDistance(node_a, prev_a);
					sum = sum + input.getDistance(node_b, prev_b);
				}

				prev_a = node_a;
				prev_b = node_b;
			}
			// insere o vetor
			setsArea.push_back(std::make_pair(i, sum));
			prev_a = prev_b = -1;
			sum = 0;
		}

		// ordena da menor para maior
		std::sort(setsArea.begin(), setsArea.end(), compFunctor);

		return setsArea;
	}

	// ordenar em relação a posição dos nós no eixo X.
	void Graph::sortNodesX()
	{
		// Declaring the type of Predicate that accepts 2 nodes and return a bool
		typedef std::function<bool(Node, Node)> CompNode;

		// Defining a lambda function to compare two nodes. It will compare two nodes using node's position (X position).
		CompNode compX =
			[](Node node1, Node node2)
		{
			return node1.getX() < node2.getX();
		};

		// assign nodes on input to nodesX
		nodesX = input.nodes;

		// sort vector of nodes nodesX
		std::sort(nodesX.begin(), nodesX.end(), compX);
	}

	// get all depots between extremes coverages lines
	std::vector<int> Graph::getDepotsBetweenNodes(int n1, int n2)
	{
		std::vector<Node>::iterator begin;
		std::vector<Node>::iterator end;

		std::vector<int> depots;

		if (n1 == n2)
			return depots;

		double n2_x;
		double n1_x;

		std::vector<Node>::iterator it_n1 = nodesX.end();
		std::vector<Node>::iterator it_n2 = nodesX.end();

		for (std::vector<Node>::iterator it = nodesX.begin(); it != nodesX.end(); ++it)
		{
			if ((*it).getNodeId() == n1)
			{
				n1_x = it->getX();
				it_n1 = it;

				if (it_n2 != nodesX.end())
					break;
			}
			else if ((*it).getNodeId() == n2)
			{
				n2_x = it->getX();
				it_n2 = it;
				if (it_n1 != nodesX.end())
					break;
			}
		}

		if (n1_x < n2_x)
		{
			begin = it_n1;
			end = it_n2;
		}
		else
		{
			begin = it_n2;
			end = it_n1;
		}

		// ajustar o ponteiro ao primeiro elemento que aparece no vetor de posição
		for (std::vector<Node>::iterator it = begin; it != nodesX.begin(); --it)
		{
			if (isApproximatelyEqual(begin->getX(), it->getX()))
			{
				begin = it;
			}
			else
				break;
		}

		// ajustar o ponteiro ao último elemento que aparece no vetor de posição
		for (std::vector<Node>::iterator it = end; it != nodesX.end(); ++it)
		{
			if (isApproximatelyEqual(end->getX(), it->getX()))
			{
				end = it;
			}
			else
				break;
		}

		for (std::vector<Node>::iterator it = begin; it <= end; ++it)
		{
			if ((*it).getNodeType().compare("depots") == 0)
			{
				depots.push_back((*it).getNodeId());
			}
		}
		return depots;
	}

	// remover os depots associados a linha de cobertura
	void Graph::removeDepots(int n1, std::vector<int> &depots)
	{

		std::vector<Node>::iterator cvLine;
		std::vector<int>::iterator itDepots;
		std::vector<int> depotsToremove;

		// encontrar a linha de cobertura que será removida no vetor de nós (nodesX)
		for (std::vector<Node>::iterator it = nodesX.begin(); it != nodesX.end(); ++it)
		{
			if ((*it).getNodeId() == n1)
			{
				cvLine = it;
				break;
			}
		}

		// apontar para o próximo depot a direita da cvLine
		while ((*cvLine).getNodeType().compare("target") == 0 && cvLine != nodesX.end())
		{
			++cvLine;
		}
		// caso tenhamos mais depots associado a linha, adicionamos no vetor depotsToremove
		// obter todos os depots a direita da linha de cobertura
		while ((*cvLine).getNodeType().compare("depot") == 0)
		{
			depotsToremove.push_back((*cvLine).getNodeId());
			cvLine++;
		}

		std::cout << "Remover: ";
		for (int i : depotsToremove)
		{
			std::cout << " " << i;
		}
		std::cout << std::endl;

		std::cout << "grupo k2 : ";
		for (int i : depots)
		{
			std::cout << " " << i;
		}
		std::cout << std::endl;

		// percorrermos o vetor de depots até encontrar o primeiro depot que deverá ser removido
		itDepots = depots.begin();

		std::cout << "nodes removed: ";
		// enquanto a lista não for vazia: //remover os depots seguintes
		while (!depotsToremove.empty() && itDepots != depots.end())
		{
			if (*itDepots == depotsToremove.front())
			{
				std::cout << *itDepots;
				depots.erase(itDepots);
				depotsToremove.erase(depotsToremove.begin());
			}
			itDepots++;
		}
		std::cout << std::endl;
	}

	// verificar a possibilidade(custo) para inserir a linha targetId (pertencente a k2) no grupo k1.
	// k2 grupo da linha de cobertura identificada por targetID que se deseja inserir no grupo k1
	bool Graph::insertNewCVLine(int k1, int k2, int targetID, std::vector<int> &depotsK1, std::vector<int> &depotsK2)
	{

		std::vector<Node>::iterator groupNodesEdge;
		std::vector<Node>::iterator newCVLineEdge;

		if (depotsK1.empty())
			return (false);

		std::vector<int> newDepots;

		double costToLast = std::numeric_limits<double>::max();
		double costToFirst = std::numeric_limits<double>::max();

		int robotID = nodesSets[k1].robotID;

		std::cout << "k1: " << k1 << " K2: " << k2 << std::endl;
		std::cout << std::numeric_limits<double>::max() << std::endl;

		// obter os depots que estão entre as linhas de cobertura do grupo k1
		auto result = minmax_element(nodesSets[k1].cvLines.begin(), nodesSets[k1].cvLines.end());

		// se a linha de cobertura l2 a ser inserida(linha representada por cvLineID do grupo k2) estiver a direita do grupo k1
		// em relação a última linha de cobertura à direita do grupo k1(*result.second)
		if (targetID > *result.second)
		{
			// calcular distância entre o depot à direita do grupo k1 e à esquerda da linha inserida

			// varrer o vetor de nós ordenados em relação ao eixo x para encontrar os índices das linhas
			for (std::vector<Node>::iterator it = nodesX.begin(); it != nodesX.end(); ++it)
			{
				// se o iterador apontar para a linha de cobertura targetId
				if ((*it).getNodeId() == targetID)
				{
					// aponte o iterador de cobertura para targetID
					newCVLineEdge = it;
					// encontra o primeiro depot anterior à linha de cobertura. Para isso, retorne aos à esquerda enquanto
					//  os nós forem targets e não seja o início do vetor de nós (nodesX)
					while ((*newCVLineEdge).getNodeType().compare("target") == 0 && newCVLineEdge != nodesX.begin())
					{
						--newCVLineEdge;
					}

					// inserir todos os depots antes de targetID até a primeira linha de cobertura à esquerda de targetID
					while ((*newCVLineEdge).getNodeType().compare("depot") == 0 && newCVLineEdge != nodesX.begin())
					{
						newDepots.push_back((*newCVLineEdge).getNodeId());
						--newCVLineEdge;
					}

					break;
				}
			}
		}
		// caso a linha a ser inserida esteja a esquerda da linha mais esquerda do grupo k1
		else if (targetID < *result.first)
		{
			// distância entre o depot à esquerda do grupo e o depot à direita da linha inserida
			for (std::vector<Node>::iterator it = nodesX.begin(); it != nodesX.end(); ++it)
			{
				if ((*it).getNodeId() == targetID)
				{

					// apontar novamente para a cvLine selecionada
					newCVLineEdge = it;
					// obter o próximo depot à direita
					while ((*newCVLineEdge).getNodeType().compare("target") == 0 && newCVLineEdge != nodesX.end())
						++newCVLineEdge;
					// inserir no vetor de vizinhança todos os depots até a próxima cvl à direita.
					while ((*newCVLineEdge).getNodeType().compare("depot") == 0 && newCVLineEdge != nodesX.end())
					{
						newDepots.push_back((*newCVLineEdge).getNodeId());
						++newCVLineEdge;
					}
					break;
				}
			}
		}

		// inserir os depósitos associados a k2 na lista de depot do grupo k1
		if (!newDepots.empty())
		{

			std::cout << "newDepots: ";
			for (int i : newDepots)
				std::cout << i << " ";
			std::cout << std::endl;

			std::cout << "depots: ";
			for (int i : depotsK1)
				std::cout << i << " ";
			std::cout << std::endl;

			int lastDepotID = newDepots.back();
			int firstDepotID = newDepots.front();

			std::vector<int>::iterator itDepots;
			std::vector<int>::iterator pointInsertion;

			if (firstDepotID < depotsK1.front() && lastDepotID < depotsK1.front())
			{
				costToFirst = getFlightTime(input.getDistance(firstDepotID, depotsK1.front()), input.getRobotVel(robotID));
				pointInsertion = depotsK1.begin();
			}
			else if (firstDepotID > depotsK1.back() && lastDepotID > depotsK1.back())
			{
				costToLast = getFlightTime(input.getDistance(lastDepotID, depotsK1.back()), input.getRobotVel(robotID));
				pointInsertion = depotsK1.end();
			}
			else if (firstDepotID > depotsK1.front() && lastDepotID < depotsK1.back())
			{
				itDepots = depotsK1.begin();
				while (itDepots != depotsK1.end() - 1)
				{
					if (firstDepotID > *itDepots && lastDepotID < *(itDepots + 1))
					{
						costToFirst = getFlightTime(input.getDistance(firstDepotID, *itDepots), input.getRobotVel(robotID));
						costToLast = getFlightTime(input.getDistance(lastDepotID, (*itDepots) + 1), input.getRobotVel(robotID));
						pointInsertion = itDepots + 1;
						break;
					}
					++itDepots;
				}
			}

			if (costToFirst < input.getRobotFuel(robotID) || costToLast < input.getRobotFuel(robotID))
			{
				depotsK1.insert(pointInsertion, newDepots.begin(), newDepots.end());
				removeDepots(targetID, depotsK2);
				std::cout << "depots_inserted: ";
				for (int i : depotsK1)
					std::cout << i << " ";
				std::cout << std::endl;

				return true;
			}
		}
		return false;
	}

	void Graph::setAllNodesCosts()
	{
		int nTargets, nDepots, robotID = 0;

		// para cada grupo
		for (unsigned int k = 0; k < nodesSets.size(); ++k)
		{
			double cost = 0;
			nTargets = nDepots = 0;

			robotID = nodesSets[k].robotID;

			nDepots = nodesSets[k].depots.size();

			input.maxFuelCost.resize(input.getRobotNum());
			input.constM.resize(input.getRobotNum());

			// unir os vetores de depots com linhas de cobetura(cvLines)
			std::vector<int> nodes = nodesSets[k].depots;
			// nodes.insert(nodes.end(),nodesSets[k].cvLines.begin(),nodesSets[k].cvLines.end());
			nodes.insert(nodes.begin() + nDepots, input.getRobotBaseId(robotID));

			// A base inserida conta com depot
			nDepots = nDepots + 1;

			// inserir todos os targets. Cada cvLine possui dois targets
			for (int i : nodesSets[k].cvLines)
			{
				nodes.push_back(i);
				nodes.push_back(i + 1);
				nTargets = nTargets + 2;
			}

			// inicializa o conjunto
			coverageSets.push_back(std::pair<std::vector<std::pair<std::vector<double>, int>>, graphInfo>());

			// adiciona as quantidades de deposts e targets ao grupo k
			coverageSets[k].second.D = nDepots;
			coverageSets[k].second.T = nTargets;
			coverageSets[k].second.baseID = nDepots - 1;
			coverageSets[k].second.robotID = robotID;

			// atribuir os custos entre todos os nós
			for (uint i = 0; i < nodes.size(); i++)
			{
				// inicializa o par vetor de distância e id do nó.
				coverageSets[k].first.push_back(std::pair<std::vector<double>, int>());
				coverageSets[k].first[i].second = nodes[i];
				for (uint j = i; j < nodes.size(); j++)
				{
					cost = getFlightTime(graph[nodes[i]][nodes[j]], input.getRobotVel(robotID));
					coverageSets[k].first[i].first.push_back(cost);
					if (cost > input.maxFuelCost[k])
					{
						input.maxFuelCost[k] = cost;
						input.constM[k] = input.getRobotFuel(robotID) + input.maxFuelCost[k];
					}
				}
			}
		}
	}

	// atualiza o coverageSets(operador do grupo de cobertura) com base nas informações dos nodesSet (operador de linhas em grupo)
	void Graph::updateCoverageSet(int k)
	{
		int nTargets = 0;
		int nDepots = 0;
		int robotID = 0;
		double cost = 0;

		// nTargets = nodesSets[k].cvLines.size()*2;
		nDepots = nodesSets[k].depots.size();

		// id do robô atribuído ao grupo k
		robotID = nodesSets[k].robotID;

		// unir os vetores de depots com linhas de cobetura(cvLines)
		std::vector<int> nodes = nodesSets[k].depots;

		// inserir a base
		nodes.insert(nodes.begin() + nDepots, input.getRobotBaseId(robotID));

		// A base inserida conta como depot
		nDepots = nDepots + 1;

		// inserir todos os  id dos targets em nodeSets. Para cada cvline inserir dois nós em nodes. Os node serão inseridos em coverageSets;
		for (int i : nodesSets[k].cvLines)
		{
			nodes.push_back(i);
			nodes.push_back(i + 1);
			nTargets = nTargets + 2;
		}

		// delete old set
		coverageSets[k].first.clear();

		// adiciona as quantidades de depots e targets ao grupo
		coverageSets[k].second.D = nDepots;
		coverageSets[k].second.T = nTargets;
		coverageSets[k].second.robotID = robotID;

		// id da base é nDepot -1.
		coverageSets[k].second.baseID = nDepots - 1;

		// reset max fuel cost at sets k1 and k2
		input.maxFuelCost[k] = 0;

		// calculate new  costs, maxFuel and  constM to k1
		// atribuir as distância entre todos os nós
		for (uint i = 0; i < nodes.size(); i++)
		{
			// inicializa o par vetor de distância e id do nó.
			coverageSets[k].first.push_back(std::pair<std::vector<double>, int>());

			// inserir o indice id do nó em cada pair <vetor, inteiro>. Nesse caso, o vetor armazena
			//  as distância do nó id aos outro nós.
			coverageSets[k].first[i].second = nodes[i];

			// inserir o custo do node[i] (identificador) a todos os outros nodes no vetor
			for (uint j = i; j < nodes.size(); j++)
			{
				cost = getFlightTime(graph[nodes[i]][nodes[j]], input.getRobotVel(robotID));
				coverageSets[k].first[i].first.push_back(cost);
				if (cost > input.maxFuelCost[k])
				{
					input.maxFuelCost[k] = cost;
					input.constM[k] = input.getRobotFuel(robotID) + input.maxFuelCost[k];
				}
			}
		}
	}

	// atualiza o coverageSets(operador do grupo de cobertura) com base nas informações dos nodesSet (operador de linhas em grupo)
	void Graph::Convert_NS_to_CS(Set nodes_set)
	{

		int nTargets = 0;
		int nDepots = 0;
		int robotID = 0;
		double cost = 0;


		// 🔴 ASSERT 1 — set_id válido (ANTES de usar input.maxFuelCost)
    	assert(nodes_set.set_id >= 0);
   		assert(nodes_set.set_id < input.maxFuelCost.size());
    	assert(nodes_set.set_id < input.constM.size());

		nDepots = nodes_set.depots.size();

		
    	// 🔴 ASSERT 2 — robotID válido
		// id do robô atribuído ao grupo k
   	 	robotID = nodes_set.robotID;
    	assert(robotID >= 0);
   		assert(robotID < input.getRobotNum());

		// unir os vetores de depots com linhas de cobetura(cvLines)
		std::vector<int> nodes = nodes_set.depots;

		// 🔴 ASSERT 3 — depots válidos no grafo
    	for (int d : nodes){
        	assert(d >= 0);
        	assert(d < graph.size());
    	}

		// inserir a base

		int baseID = input.getRobotBaseId(robotID);
    	assert(baseID >= 0);
    	assert(baseID < graph.size());
		nodes.insert(nodes.begin() + nDepots, baseID);

		// A base inserida conta como depot
		nDepots = nDepots + 1;

		// inserir todos os id dos targets em nodeSets. Para cada cvline inserir dois nós em nodes. Os node serão inseridos em coverageSets;
		for (int i : nodes_set.cvLines)
		{
			//🔴 ASSERT 4 — cvLines válidos
        	assert(i >= 0);
        	assert(i + 1 < graph.size());

			nodes.push_back(i);
			nodes.push_back(i + 1);
			nTargets = nTargets + 2;
		}

		// delete old set
		coverage_set.first.clear();

		// delete old cv index;
		map_cvset_id_to_node_id.clear();

		// adiciona as quantidades de depots e targets ao grupo
		coverage_set.second.D = nDepots;
		coverage_set.second.T = nTargets;
		coverage_set.second.robotID = robotID;

		// id da base é nDepot -1.
		coverage_set.second.baseID = nDepots - 1;

		// reset max fuel cost at sets k1 and k2
		input.maxFuelCost[nodes_set.set_id] = 0;

		// calculate new  costs, maxFuel and  constM to k1
		// atribuir as distância entre todos os nós
		for (uint i = 0; i < nodes.size(); i++)
		{

			// 🔴 ASSERT 5 — índice i válido
        	assert(i < nodes.size());
        	assert(nodes[i] >= 0);
        	assert(nodes[i] < graph.size());

			// atribui valor máximo para o identificado do menor combustível
			// para atingir algum posto
			// inicializa o par vetor de distância e id do nó.
			coverage_set.first.push_back(std::pair<std::vector<double>, int>());

			// inserir o indice id do nó em cada pair <vetor, inteiro>. Nesse caso, o vetor armazena
			//  as distância do nó id aos outro nós.
			coverage_set.first[i].second = nodes[i];

			// criar o map para obter do indice do grafo o indice do coverage_set
			// 🔴 ASSERT 6 — map consistente
			auto [it, inserted] = 
			map_cvset_id_to_node_id.emplace(nodes[i], i);
			assert(inserted); // garantir que a inserção foi realizada com sucesso

			// inserir o custo do node[i] (identificador) a todos os outros nodes no vetor
			for (uint j = i; j < nodes.size(); j++)
			{

				// 🔴 ASSERT 7 — índice j válido
            	assert(nodes[j] >= 0);
          	  	assert(nodes[j] < graph.size());

				cost = getFlightTime(graph[nodes[i]][nodes[j]], input.getRobotVel(robotID));

				// 🔴 ASSERT 8 — custo válido
    			assert(std::isfinite(cost));
            	assert(cost >= 0.0);

				coverage_set.first[i].first.push_back(cost);
				if (cost > input.maxFuelCost[nodes_set.set_id])
				{
					input.maxFuelCost[nodes_set.set_id] = cost;
					input.constM[nodes_set.set_id] = input.getRobotFuel(robotID) + input.maxFuelCost[nodes_set.set_id];
				}
			}
		}

		min_fuel.clear();
		set_min_fuel_2_depot();
	}

	// contabiliza a identidade de cada nó (target ou depot).
	// utilizamos os índices (index) de cada nós, armazenado em nGraphs[k].first[i].second para acessar a identificação.
	void Graph::setGraphInfo()
	{
		if (!nGraphs.empty())
		{
			int index = 0;
			for (unsigned int k = 0; k < nGraphs.size(); k++)
			{
				for (unsigned int i = 0; i < nGraphs[k].first.size(); ++i)
				{
					index = nGraphs[k].first[i].second;
					if (input.nodes[index].getNodeType().compare("base") == 0 || input.nodes[index].getNodeType().compare("depot") == 0)
						nGraphs[k].second.D++;
					else if (input.nodes[index].getNodeType().compare("target") == 0)
					{
						nGraphs[k].second.T++;
					}
				}
			}
		}
	}

	// calcula o tempo de voo
	double Graph::getFlightTime(double distance, double vel)
	{
		return static_cast<double>(distance) / static_cast<double>(vel);
	}

	void Graph::swapCLine(uint k1, uint k2, int l1, int l2)
	{
		int lTemp = -1;

		lTemp = nodesSets[k1].cvLines[l1];
		nodesSets[k1].cvLines[l1] = nodesSets[k2].cvLines[l2];
		// verificar a necessidade de atualizar o subset de k1, caso tenha
		UpdateSubSet(k1);

		nodesSets[k2].cvLines[l2] = lTemp;
		// verificar a necessidade de atualizar o subset de k2, caso tenha
		UpdateSubSet(k2);
	}

	// remove a linha iLine de k1 e a insere em k2
	void Graph::shiftCLine(uint k1, uint k2, uint iLine)
	{
		int line = nodesSets[k1].cvLines[iLine];

		nodesSets[k2].cvLines.push_back(line);
		// verificar a necessidade de atualizar o subset de k1, caso tenha
		UpdateSubSet(k2);

		uint nodeBefore = nodesSets[k1].cvLines.size();
		nodesSets[k1].cvLines.erase(nodesSets[k1].cvLines.begin() + iLine);
		// verificar a necessidade de atualizar o subset de k1, caso tenha

		// cout << "sizeNodeset before: " << nodeBefore << " index line: " << iLine << " line removed: " << line <<endl;
		// cout << "sizeNodeset after: " << nodesSets[k1].cvLines.size() <<endl;

		if (nodeBefore != nodesSets[k1].cvLines.size() + 1)
			std::cout << "problema shiftCline" << std::endl;

		UpdateSubSet(k1);
	}

	void Graph::removeCLine(uint k1, uint iLine)
	{
		nodesSets[k1].cvLines.erase(nodesSets[k1].cvLines.begin() + iLine);
		UpdateSubSet(k1);
	}

	// backup data sets of coverage lines and matrix of costs
	void Graph::copyNSets()
	{
		// set back up  of costs coverage Sets
		coverageSets_Bckup.clear();
		coverageSets_Bckup = coverageSets;

		// set the back up of nodes lines
		nodesSets_Bckup.clear();
		nodesSets_Bckup = nodesSets;

		// set backup of the robot to group map
		mapRobotGroup_Bckup = mapRobotGroup;
		mapGroupRobot_Bckup = mapGroupRobot;
	}

	// restore coverage  and nodes sets
	void Graph::restoreNSets()
	{
		coverageSets.clear();
		coverageSets = coverageSets_Bckup;
		nodesSets.clear();
		nodesSets = nodesSets_Bckup;
		mapRobotGroup = mapRobotGroup_Bckup;
		mapGroupRobot = mapGroupRobot_Bckup;
	}

	// retorna o custo para ir de x a y no grafo k
	double Graph::getCost(unsigned int k, unsigned int x, unsigned int y)
	{
		double cost = 0;
		// se o índice k é um grupo válido
		if (k < coverageSets.size())
		{

			uint minID, maxID = 0;
			minID = std::min(x, y);
			maxID = std::max(x, y);

			if (minID < coverageSets[k].first.size() && maxID < coverageSets[k].first.size())
			{
				cost = (coverageSets[k].first[minID].first[maxID - minID]);
			}
		}
		return cost;
	}

	// retorna o custo para ir de x a y no grafo k
	double Graph::getCost(unsigned int x, unsigned int y)
	{
		double cost = 0;
		// se o índice k é um grupo válido

		uint minID, maxID = 0;

		minID = std::min(x, y);
		maxID = std::max(x, y);

		if (minID < coverage_set.first.size() && maxID < coverage_set.first.size())
		{
			cost = (coverage_set.first[minID].first[maxID - minID]);
		}

		return cost;
	}

	// get a node id. This id is assigned for the initial global graph.
	// This keep a possible to get a original reference to the node.
	int Graph::getIndex(int k, int i)
	{
		return coverageSets[k].first[i].second;
	}

	// get a node id. This id is assigned for the initial global graph.
	// This keep a possible to get a original reference to the node.
	/*int Graph::getIndex(int i)
	{
		auto it = indexToNodeId.find(i);
		int test_id = coverage_set.first[i].second;
		if (it != indexToNodeId.end()){
			if(test_id != it->second)
				cout << "Problema nos indices !\n";

			return it->second;
		}	
		else
		{
			std::cerr << "Erro: índice " << i << " não encontrado em indexToNodeId.\n";
			return -1; // ou lançar uma exceção se preferir
		}
	}*/

	int Graph::getIndex(int i)
	{
		return coverage_set.first[i].second;
	}
	
	


// get a coverrage line id. This id is assigned for the input node.
// This keep a possible to get a original reference to the node.
int Graph::getCVLIndex(int k, int i)
{
	return nodesSets[k].cvLines[i];
}

// return the number of groups
int Graph::getNumberOfGraphs()
{
	return nGraphs.size();
}

// return the number of groups
int Graph::getNumberOfSets()
{
	return nodesSets.size();
}

// return number of targets from group k
int Graph::getTargetNum(int k)
{
	if (k < getNumberOfSets())
		return coverageSets[k].second.T;
	else
		return 0;
}

// return number of targets from group k
int Graph::getTargetNum()
{
	return coverage_set.second.T;
}

// return depots num from group k
int Graph::getDepotNum(int k)
{
	if (k < getNumberOfSets())
		return coverageSets[k].second.D;
	else
		return 0;
}

// return depots num from group k
int Graph::getDepotNum()
{
	return coverage_set.second.D;
}

// return number of coverage lines from group k
int Graph::getNumberOfLines(int k)
{
	if (k < getNumberOfSets())
	{
		// verficar se existe linha de cobertura
		if (!nodesSets[k].cvLines.empty())
			return nodesSets[k].cvLines.size();
		else
			return 0;
	}
	else
		return 0;
}

// print coverage lines from all groups.
void Graph::printGroupOfLines()
{
	for (uint k = 0; k < nodesSets.size(); k++)
	{
		std::cout << "Group: " << k << " " << std::endl;
		for (uint i = 0; i < nodesSets[k].cvLines.size(); i++)
		{
			std::cout << nodesSets[k].cvLines[i] << " ";
		}
		std::cout << "\n";
	}
	std::cout << "\n";
}

int Graph::getBaseID(int k)
{
	return coverageSets[k].second.baseID;
}

int Graph::getBaseID()
{
	return coverage_set.second.baseID;
}

void Graph::insertNSDepots(int id, std::vector<int> depots)
{
	nodesSets[id].depots.insert(nodesSets[id].depots.begin(), depots.begin(), depots.end());
	std::sort(nodesSets[id].depots.begin(), nodesSets[id].depots.end());
	auto it = unique(nodesSets[id].depots.begin(), nodesSets[id].depots.end());
	nodesSets[id].depots.resize(distance(nodesSets[id].depots.begin(), it));
}

void Graph::UpdateDepots(int id, std::vector<int> depots)
{
	nodesSets[id].depots.clear();
	nodesSets[id].depots = depots;

	auto it_subset = nodesSets[id].sub_set.begin();

	while (it_subset != nodesSets[id].sub_set.end())
	{
		it_subset->depots.clear();
		it_subset->depots = depots;
		++it_subset;
	}
}

// update nodesSets and CoverageSets
void Graph::updateNodesSets(std::vector<Set> ns)
{
	nodesSets = ns;

	for (uint id = 0; id < nodesSets.size(); ++id)
		UpdateSubSet(id);
}

// update nodesSets and CoverageSets
void Graph::updateCoverageSets(std::vector<Set> ns)
{
	nodesSets = ns;
	for (uint id = 0; id < nodesSets.size(); ++id)
	{
		updateCoverageSet(id);
		UpdateSubSet(id);
	}
}

void Graph::swapRobotsNodesSets(int g1, int g2)
{
	int tempID = nodesSets[g1].robotID;
	nodesSets[g1].robotID = nodesSets[g2].robotID;

	nodesSets[g2].robotID = tempID;
}

int Graph::getMapRobotGroupSize()
{
	return mapRobotGroup.size();
}

std::set<int> Graph::getRobotGroups(int k)
{
	return mapRobotGroup.find(k)->second;
}

// change map robot type between path g1 and g2 and update nodesSets and converageSets;
void Graph::swapRobotsGroups(int k1, int g1, int k2, int g2)
{
	// swapRobots between path  g1 and g2;
	swapRobotsNodesSets(g1, g2);

	// get sets k1 and k2 from mapRobotGroup;
	std::set<int> s1 = mapRobotGroup.find(k1)->second;
	std::set<int> s2 = mapRobotGroup.find(k2)->second;

	// erase path g1 from set of robots types
	s1.erase(g1);

	// erase path g2 from set of robots types
	s2.erase(g2);

	// insert new path g2 into s1, robots with the same type
	s1.insert(g2);
	// insert new path g1 into s2, robots with the same type
	s2.insert(g1);

	// update mapRobots sets
	mapRobotGroup.find(k1)->second.clear();
	mapRobotGroup.find(k1)->second.insert(s1.begin(), s1.end());
	mapRobotGroup.find(k2)->second.clear();
	mapRobotGroup.find(k2)->second.insert(s2.begin(), s2.end());

	mapGroupRobot.find(g1)->second = k2;
	mapGroupRobot.find(g2)->second = k1;
}

int Graph::getTargetsGraphIndexNum()
{
	return graphTargetsIndexes.size();
}

double Graph::getCostOnGraph(unsigned int robotID, unsigned int a, unsigned int b)
{
	if (a < graph.size() && b < graph.front().size())
		return getFlightTime(graph[a][b], input.getRobotVel(robotID));
	return -1;
}

std::map<int, int> Graph::GetALLCLines()
{

	std::map<int, int> map_nodes_cl;
	std::vector<int> targets = input.getTargetsIndexes();
	int node1, node2;
	for (uint i = 0; i < targets.size(); i = i + 2)
	{
		node1 = targets[i];
		node2 = targets[i + 1];
		map_nodes_cl.emplace(node1, node2);
	}
	return map_nodes_cl;
}

bool Graph::IsCLine(int node1, int node2)
{

	if (map_nodes_on_cl.empty())
		return false;

	auto it1 = map_nodes_on_cl.find(node1);
	if (it1 != map_nodes_on_cl.end() && it1->second == node2)
		return true;

	auto it2 = map_nodes_on_cl.find(node2);
	if (it2 != map_nodes_on_cl.end() && it2->second == node1)
		return true;

	return false;
}

/*double Graph::get_min_fuel_2_depot(int i){
	int depot = getDepotNum();
	double min_fuel = numeric_limits<double> :: max();
	double fuel;
	for(int j = 0;j < depot;j++){
		fuel = getCost(i,j);
		if(fuel < min_fuel)
			min_fuel = fuel;
	}
	return min_fuel;
}*/

void Graph::set_min_fuel_2_depot()
{
	int n_depots = getDepotNum();
	int n_targets = getTargetNum();
	int n_nodes = n_depots + n_targets;

	for (int i = n_depots; i < n_nodes; ++i)
	{
		double min_f = std::numeric_limits<double>::max();
		double fuel = 0;
		for (int j = 0; j < n_depots; j++)
		{
			fuel = getCost(i, j);
			if (fuel < min_f)
				min_f = fuel;
		}
		min_fuel.emplace(i, min_f);
	}
}

double Graph::get_min_fuel_2_depot(int i)
{
	double fuel = 0;
	auto it_min_fuel = min_fuel.find(i);
	if (it_min_fuel != min_fuel.end())
		fuel = it_min_fuel->second;
	return fuel;
}

// retorna o o tipo do robô
int Graph::GetGroupOfRobot(int p_id)
{
	int group = 0;
	auto itmap = mapGroupRobot.find(p_id);
	if (itmap != mapGroupRobot.end())
		group = itmap->second;
	return group;
}

void Graph::setMapGroupOfRobot()
{
	// criar mapGroupRobot para recuperar o grupo do robô dado o caminho
	for (std::pair<int, std::set<int>> type : mapRobotGroup)
	{
		for (int p_id : type.second)
		{
			mapGroupRobot.emplace(p_id, type.first);
		}
	}
}
