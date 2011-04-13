/*
 *  Copyright AIST-CNRS Joint Robotics Laboratory
 *  Author: Nicolas Perrin
 *
 */

#ifndef NEWSLIDERPG_H
#define NEWSLIDERPG_H

#include <sys/time.h>
#include <stdlib.h>
#include <time.h>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <vector>
#include <set>

#include "newSliderPG/halfStep_creation.h"
#include "newSliderPG/slidingFunctions.h"

#ifndef PI
# define PI 3.14159265359
#endif

using namespace std;

class CnewSliderPG
{

	public:

		/*!
		 * Constructor.
		 */
		CnewSliderPG(slidingClass * slidingObject);

		/*!
		 * Destructor.
		 */
		~CnewSliderPG();
		
		slidingClass * slider;
		void set_incrTime(float itime) {incrTime = itime;}
		void set_first_support_foot(LoR fsf) {first_support_foot = fsf;}
		void set_vp_config(const viaPointConfig & vpc) {vp_config = vpc;}
		void set_ft_dim(const feetDimensions & ftd) {ft_dim = ftd;}
		void set_constants(const hsConstants & csts) {constants = csts;}
		
		/*!
		 * produceTraj generates the trajectories of the end-effectors of
		 * the robot lower body, for a sequence of steps defined by vect_input.
		 * The function must not be executed before setting all the internal 
		 * parameters (using set_incrTime, set_vp_config, set_ft_dim, set_constants).
		 * @param t The result will be stored in t.
		 * @param vect_input Describes the sequence of steps.\n
		 *  HERE IS A PRECISE DESCRIPTION OF THE FORMAT USED FOR vect_input:\n
		 *           First remark: all angles are in degrees.\n
		 *           Let's denote by "FStF" the CENTER OF THE FOOT which will be
		 *           stable during first step: the first stable foot.\n
		 *           The first 6 parameters are the following:\n
		 *                         We consider a plan (the floor) with a x
		 *                       axis and a y axis. The orientation of the x
		 *                       axis in this plan is 0.\n
		 *                         The (0,0) origin of this plan corresponds to
		 *                       the position of the waist and the CoM of
		 *                       the robot (they are supposed identical).\n
		 *                         The 3 first parameters correspond to the
		 *                       FStF: \n
		 *                       they are respectively the x position, y
		 *                       position and (absolute) orientation of the
		 *                       FStF. \n
		 *                         The orientation of the FStF will always be
		 *                       supposed equal to ZERO ! Hence, the third
		 *                       parameter (vect_input[2]) is superfluous:
		 *                       it should always be 0 !!  \n
		 *                         The 3 next parameters correspond to the
		 *                       CENTER OF THE first swinging foot (FSwF): 
		 *                       x position, y position, (absolute) orientation.\n
		 *                         Let's denote by x, y, t, x', y', t', the
		 *                       first 6 parameters.\n
		 *                         We suppose also that the position of the
		 *                       CoM is at the BARYCENTER OF THE INITIAL
		 *                       POSITIONS OF THE CENTER OF THE FEET; therefore we have:
		 *                       -x' = x , and -y' = y. HENCE, the 6 first
		 *                       parameters can in fact be generated with
		 *                       only 3 parameters, and :\n
		 *                         x,y,t,x',y',t' = -x',-y',0,x',y',t'\n
		 *                           So, we need only the initial position
		 *                         and orientation of the FSwF to generate
		 *                         the 6 parameters.\n
		 *           The first 6 parameters are all ABSOLUTE coordinates,
		 *           but the following parameters are all RELATIVE
		 *           coordinates ! What follows is a list of groups of
		 *           parameters, each group corresponding to sequences of half-steps. 
		 *           Each group has 5 parameters. and corresponds to 2
		 *           half-steps: one up, and one down. \n
		 *           The two first parameters are the "sliding negative times" 
		 *           (in s.), for respectively the first and second half-step.
		 *           For example, a parameter of -0.3s. means that the next half-step, 
		 *           instead of starting after the previous half-step has been completed,
		 *           will start 0.3s. before the end of the previous half-step.
		 *           The last 0.3s. of the previous half-step will thus
		 *           become a mixture between a finishing and a starting half-step. 
		 *           This technique is used to speed up the walk and make it smoother. \n
		 *           !!IMPORTANT!!: since there is nothing before the first
		 *           half step, the first slide parameter in vect_input must always be 0.\n
		 *           The next 3 parameters describe the RELATIVE position of the last 
		 *           foot that was put on the ground. For example, if G_i = (x_i, y_i, th_i) is the 
		 *           ith such group, then (x_i, y_i) is the couple of
		 *           coordinates (in meters) of the vector between the
		 *           center of the stable foot and the swinging foot 
		 *           just after the ith step is finished (i.e. when the
		 *           swinging foot touches the ground). Finally, th_i is the 
		 *           RELATIVE orientation (in DEGREES) of the final position
		 *           of the swinging foot compared to the orientation of stable
		 *           foot. For example, since the orientation of the first
		 *           stable foot is 0, the parameter th_1 is the absolute
		 *           orientation of the FSwF when the first step is just finished.\n             
		 */
		void produceTraj(trajFeatures & t, const vector<float> & vect_input);
		
		/*!
		 * This time we define also the initial absolute position of the com AND the initial absolute orientation of the first support foot:
		 */
		void produceTraj(trajFeatures & t, const vector<float> & vect_input, float x_com_init, float y_com_init, float theta_supportfoot_init);
		
		/*! drawSteps outputs a file intended for gnuplot.
		 * Let's say that the result of drawSteps is in the 
		 * file "data.gnuplot".
		 * Please write the following script file "toto.txt":
		 * set size ratio 1
		 * unset key
		 * set term postscript eps enhanced
		 * set output "totoOutput.eps"
		 * plot "data.gnuplot" index 0 w l lt 1 lw 0.5 lc 1, 
		 * "data.gnuplot" index 1 w l lt 2 lw 0.5 lc 2, "data.gnuplot"
		 *  index 2 w l lt 2 lw 0.5 lc 2, "data.gnuplot" index 3 w l
		 * lt 1 lw 0.5 lc 3, "data.gnuplot" index 4 w l lt 1 lw 0.5 lc 4
		 * Then, gnuplot> load 'toto.txt' will produce the eps file
		 * "totoOutput.eps".
		 * @param fb The ofstream corresponding to the file where the data
		 *           will be written. 
		 * @param t The trajectory to draw, usually produced by produceTraj.
		 * @param vect_input The vector of steps is also taken into account 
		 *                   because the footprints are drawn.
		 */
		void drawTraj(ofstream & fb, const trajFeatures & t, const vector<float> & vect_input);
		
		/*!
		 * Function to plot only the CoM and ZMP trajectories along the x axis.
		 */
		void plot_com_zmp_along_x(ofstream & fb, const trajFeatures & t);
		
		/*!
		 * Function to plot only the CoM and ZMP trajectories along the y axis.
		 */
		void plot_com_zmp_along_y(ofstream & fb, const trajFeatures & t);

		/*!
		 * Function to plot the feet height in function of the time.
		 */
		void plot_feet_height(ofstream & fb, const trajFeatures & t);

		
	private:
	    
		float incrTime;
		LoR first_support_foot;
		viaPointConfig vp_config;
		feetDimensions ft_dim;    
		hsConstants constants;
};
#endif
