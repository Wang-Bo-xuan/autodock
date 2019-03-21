#ifndef __INCLUDE_LS_AUTODOCK_COMMON_H__
#define __INCLUDE_LS_AUTODOCK_COMMON_H__

#include <iostream>
#include <list>
#include <vector>
using namespace std;

namespace LS_SLAM
{
  typedef struct lineFit_struct
  {
    double k;
    double b;
    double length;
    double start_index_x;
    double start_index_y;
    double end_index_x;
    double end_index_y;
  }LINEFIT,*PLINEFIT;

  typedef list<LINEFIT> LINELIST;
  typedef list<PLINEFIT> PLINELIST;

  typedef struct charge
  {
    int tag_num;
    double x;
    double y;
    double angle;
  }CHARGE,*PCHARGE;

  typedef list<CHARGE> charge_list;
  typedef list<CHARGE>::iterator charge_iterator;
}

#endif
