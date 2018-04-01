#ifndef GREEN_ARM_DYN_H
#define GREEN_ARM_DYN_H

#include <iostream>
#include <fstream>
#include <ctime>
#include <cmath>
#include <tnt.h>
#include <stdlib.h>
#include <stdio.h>
#include <string>
#include "inverse.h"
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>

using namespace std;
using namespace TNT;
using namespace boost::numeric::odeint;

typedef boost::array<double, 8 > state_type;
typedef runge_kutta4< state_type > rk4;


//Matrix<double> load_traj(string str,int rows);
Array2D<double> PID_control_gold(const state_type &r, const double t);
Array2D<double> PID_control_green(const state_type &r, const double t);
void sys_dyn_gold(const state_type &r, state_type &drdt , double t);
void sys_dyn_green(const state_type &r, state_type &drdt , double t);
void write_sys(const state_type &x);

#endif
