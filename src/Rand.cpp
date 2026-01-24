/*
 * Rand.cpp
 *
 *  Created on: 29 de jul de 2018
 *      Author: rsantin
 */

#include "Rand.h"

Rand::Rand() {
	// TODO Auto-generated constructor stub

}

Rand::~Rand() {
	// TODO Auto-generated destructor stub
}

std::vector<int> Rand::randVector(int i){

	std::vector<int> container(i);
	// Fill it with numbers 0, 1, ..., max_num - 1)
	iota(container.begin(), container.end(), 0);
	shuffle(container.begin(), container.end(), eng);

	return (container);
}
void Rand::sufferVector(std::vector<int>* v){
	std::shuffle(v->begin(), v->end(), eng);
}

int Rand::randNum(int i){
	std::uniform_int_distribution<> distr(0, i); // define the range
	return distr(eng);

}

int Rand::randNum(int a,int b){
	std::uniform_int_distribution<> distr(a, b); // define the range
	return distr(eng);

}




