//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//


/*
 * This file contains the basic data types to use throughout the program
 */
 
#include <vector>
 
#ifndef TSPD_TYPES_H
#define TSPD_TYPES_H

namespace tspd
{

constexpr unsigned int MAX_N = 127;
using VertexSet = std::bitset<MAX_N>;

using Time = double;
using DualVector = std::vector<double>;

enum class Direction 
{
    Forward,
    Backward
};
inline constexpr int idx(Direction d) {return static_cast<int>(d);}
inline constexpr Direction op(Direction d) {return Direction(1-idx(d));}
inline constexpr int opidx(Direction d) {return idx(op(d));}

} //namespace tspd

#endif //TSPD_TYPES_H
