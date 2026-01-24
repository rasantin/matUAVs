/*
 * Rand.h
 *
 *  Created on: 29 de jul de 2018
 *      Author: rsantin
 */

#ifndef SRC_RAND_H_
#define SRC_RAND_H_

#include <vector>
#include <random>
#include <algorithm>
#include <numeric>

class Rand {
public:
	Rand();
	virtual ~Rand();

	// Create random engine generator
	std::mt19937 eng{ std::random_device{}() };

	std::vector<int> randVector(int i);
	void sufferVector(std::vector<int>* v);
	int randNum(int i);
	int randNum(int a, int b);
};


#endif /* SRC_RAND_H_ */
