/*
 *  Copyright AIST-CNRS Joint Robotics Laboratory
 *  Author: Nicolas Perrin
 */

#include "newSliderPG/newSliderPG.h"

#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <vector>
#include <string>

class Cslide : public slidingClass
{
    public:
	Cslide() {}

	~Cslide() {}
	
	bool isValid(instantFeatures & i) {
	    return true;  
	}
};

int main (int argc, char *argv[]) {

  filebuf fb;
  fb.open (argv[1],ios::in);
  istream is(&fb);
	
  string bufstring;

  CnewSliderPG * NSPG;
 
  Cslide * S = new Cslide();
  NSPG = new CnewSliderPG(S); 
  
  float body_height;
  char left_or_right;
  float t_start, t_total;
  float incrTime, gravity;
  float hDistance, maxHeight;
  string gnuplot_steps, gnuplot_comzmpx, gnuplot_comzmpy, gnuplot_height;
  is >> bufstring >> body_height; 
  is >> bufstring >> left_or_right;
  is >> bufstring >> incrTime;
  is >> bufstring >> gravity;
  is >> bufstring >> hDistance;
  is >> bufstring >> maxHeight;
  is >> bufstring >> t_start;
  is >> bufstring >> t_total;
  is >> bufstring >> gnuplot_steps;
  is >> bufstring >> gnuplot_comzmpx;
  is >> bufstring >> gnuplot_comzmpy;	
  is >> bufstring >> gnuplot_height;
  is >> bufstring;

  vector<float> stepsVect;

  float z;
  char zc;
  while (1) { 
    is >> zc;    
    if (zc == ')') break;
    is >> z;
    stepsVect.push_back(z);
    cout << z << endl;
  }  

  ofstream ofst_steps (gnuplot_steps.c_str());
  ofstream ofst_comzmpx (gnuplot_comzmpx.c_str());
  ofstream ofst_comzmpy (gnuplot_comzmpy.c_str());
  ofstream ofst_height (gnuplot_height.c_str());

  NSPG->set_incrTime(incrTime);
  
  if(left_or_right == 'L') NSPG->set_first_support_foot(LEFT);
  else NSPG->set_first_support_foot(RIGHT);
  
  viaPointConfig vpc;
  vpc.hDistance = hDistance;
  vpc.maxHeight = maxHeight;
  NSPG->set_vp_config(vpc);
  
  feetDimensions fdim;
  fdim.width = 0.13,
  fdim.length = 0.23;
  NSPG->set_ft_dim(fdim);
  
  hsConstants hsc;
  hsc.g = gravity;
  hsc.standard_height = body_height;
  hsc.t_start = t_start;
  hsc.t_total = t_total;
  NSPG->set_constants(hsc);
  
  trajFeatures t;
  t.size = 0;
  t.incrTime = incrTime;
  
  NSPG->produceTraj(t, stepsVect);
  
  NSPG->drawTraj(ofst_steps, t, stepsVect);
  NSPG->plot_com_zmp_along_x(ofst_comzmpx, t);
  NSPG->plot_com_zmp_along_y(ofst_comzmpy, t);
  NSPG->plot_feet_height(ofst_height, t);

  delete NSPG;

}
