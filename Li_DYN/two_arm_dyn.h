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
#include "cmath"
#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>

using namespace std;
using namespace TNT;
using namespace boost::numeric::odeint;

typedef boost::array<double, 12 > state_type;
typedef runge_kutta4< state_type > rk4;

const double PI= 3.141592654;

void sys_states(const state_type &x);
void sys_dyn_gold(const state_type &x, state_type &dxdt, double t);
void sys_dyn_gold_euler(state_type &x, double dt);
int sys_dyn_gold_euler_detect(state_type &x, double dt,int iter_time);

#endif
