/*
 *  Copyright AIST-CNRS Joint Robotics Laboratory
 *  Author: Nicolas Perrin
 *
 */

#ifndef SLIDINGFUNCTIONS_H
#define SLIDINGFUNCTIONS_H

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

#ifndef PI
# define PI 3.14159265359
#endif

using namespace std;

/*!
 * The class used to check the validity of C-space configurations, based on the 
 * description by an instantFeatures object. 
 * Must be implemented by the user.
 */
// class CheckValidity {
//     public:
// 	virtual bool isValid(instantFeatures & i) { return true; }
// };

class slidingClass {
    public:
	/*!
	 * Constructor.
	 */
	slidingClass();
	/*!
	 * Destructor.
	*/
	~slidingClass();
	
	virtual bool isValid(instantFeatures & i) { return true; }
	
	/*!
	* A function that takes in input an object trajFeatures corresponding to a sequence of
	* half-steps (possibly alreaded slided) ending with an UPWARD half-step, and 
	* an object trajFeatures corresponding to a downward half-step.
	* The last config of the first trajFeatures MUST be the same as the first config
	* of the second trajFeatures.
	* The integer returned is the index in t when the new half-step will start.
	*/
	int slideUpDownMAX(trajFeatures & t, trajFeatures & downward_halfstep);
	
	/*!
	 * Similar to slideUpDownMAX, but slide using the negative time neg_time (in s.), so validity checks are NOT done. 
	 */
	int slideUpDownCOEF(trajFeatures & t, float neg_time, float reduction, trajFeatures & downward_halfstep);


	/*!
	* A function that takes in input an object trajFeatures corresponding to a sequence of
	* half-steps (possibly alreaded slided) ending with a DOWNWARD half-step, and 
	* an object trajFeatures corresponding to an upward half-step.
	* The last config of the first trajFeatures MUST be the same as the first config
	* of the second trajFeatures.
	* The integer returned is the index in t when the new half-step will start.
	*/
	int slideDownUpMAX(trajFeatures & t, trajFeatures & upward_halfstep);
	
	/*!
	 * Similar to slideDownUpMAX, but slide using the negative time neg_time (in s.), so validity checks are NOT done. 
	 */
	int slideDownUpCOEF(trajFeatures & t, float neg_time, const trajFeatures & upward_halfstep);
	/*!
	 * The object used to check the validity of configurations. It might for 
	 * example include self-collision checks.
	 */
// 	CheckValidity validator;
	
    private:
	
	bool check_slideUpDown(const trajFeatures & t1,float neg_time,float reduction,const trajFeatures & t2,
	    const int & prestartindex,const int & startindex,const int & endindex,const int & postendindex);
	bool check_slideDownUp(const trajFeatures & t1,float neg_time,const trajFeatures & t2);

};

#endif