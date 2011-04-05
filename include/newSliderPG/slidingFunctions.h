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
class CheckValidity {
    public:
	virtual bool isValid(instantFeatures & i) { return true; }
};

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
	
	/*!
	* A function that takes in input an object trajFeatures corresponding to a sequence of
	* half-steps (possibly alreaded slided) ending with an UPWARD half-step, and 
	* an object trajFeatures corresponding to a downward half-step.
	* The last config of the first trajFeatures MUST be the same as the first config
	* of the second trajFeatures.
	*/
	void slideUpDownMAX(trajFeatures & t, trajFeatures & downward_halfstep);
	
	/*!
	 * Similar to slideUpDownMAX, but slide using the negative time neg_time (in s.), so validity checks are NOT done. 
	 */
	void slideUpDownCOEF(trajFeatures & t, float neg_time, float reduction, trajFeatures & downward_halfstep);


	/*!
	* A function that takes in input an object trajFeatures corresponding to a sequence of
	* half-steps (possibly alreaded slided) ending with a DOWNWARD half-step, and 
	* an object trajFeatures corresponding to an upward half-step.
	* The last config of the first trajFeatures MUST be the same as the first config
	* of the second trajFeatures.
	*/
	void slideDownUpMAX(trajFeatures & t, trajFeatures & upward_halfstep);
	
	/*!
	 * Similar to slideDownUpMAX, but slide using the negative time neg_time (in s.), so validity checks are NOT done. 
	 */
	void slideDownUpCOEF(trajFeatures & t, float neg_time, float reduction, trajFeatures & downward_halfstep);
	
    private:
	/*!
	 * The object used to check the validity of configurations. It might for 
	 * example include self-collision checks.
	 */
	CheckValidity validator;
};

#endif