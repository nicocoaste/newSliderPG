/*
* Copyright 2010, 2011
*
* Nicolas Perrin,
* Olivier Stasse,
* Florent Lamiraux,
* Eiichi Yoshida
*
*
* JRL/LAAS, CNRS/AIST
*
* This file is part of newSliderPG.
* newSliderPG is a free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* newSliderPG is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Lesser Public License for more details.
* You should have received a copy of the GNU Lesser General Public License
* along with newSliderPG. If not, see <http://www.gnu.org/licenses/>.
*
* Research carried out within the scope of the Associated
* International Laboratory: Joint Japanese-French Robotics
* Laboratory (JRL)
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
	
	/*!
	 * Redefined by the user; this function is used to implement the collision checks: 
	 * it decides whether a configuration (defined by an instantFeatures) is valid or not.
	 * This function is called by check_slideDownUp() and check_slideUpDown().
	 */
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
	void slideUpDownMAX(trajFeatures & t, trajFeatures & downward_halfstep, int & index_return, float & negative_overlap, float & reduction_coef);
	
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
	void slideDownUpMAX(trajFeatures & t, trajFeatures & upward_halfstep, int & index_return, float & negative_overlap);
	
	/*!
	 * Similar to slideDownUpMAX, but slide using the negative time neg_time (in s.), so validity checks are NOT done. 
	 */
	int slideDownUpCOEF(trajFeatures & t, float neg_time, const trajFeatures & upward_halfstep);
// 	/*!
// 	 * The object used to check the validity of configurations. It might for 
// 	 * example include self-collision checks.
// 	 */
// 	CheckValidity validator;
	
    private:
	
	bool check_slideUpDown(const trajFeatures & t1,float neg_time,float reduction,const trajFeatures & t2,
	    const int & prestartindex,const int & startindex,const int & endindex,const int & postendindex);
	bool check_slideDownUp(const trajFeatures & t1,float neg_time,const trajFeatures & t2);

};

#endif
