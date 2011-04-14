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

#include "newSliderPG/slidingFunctions.h"

//During the smoothing, the foot can start to move horizontally very early, 
//but we still want the tae-off and the landing to be vertical, so we define 
//a minimum height (in m.) for the foot to move horizontally:
#define MIN_FOOT_HEIGHT_FOR_HORIZONTAL_DISPLACEMENT 0.001

//The minimum duration of the double support (in s.):
#define MIN_DOUBLE_SUPPORT_TIME 0.2

//The minimum coefficient for the trajectory smoothing (for example, with 1.0 the 
//trajectory is not modified, and for 0.0 the feet height will always be 0):
#define MIN_PERCENT_REDUCTION 0.30

//The precision of time search during the dichotomy (in s):
#define T_STOP_DICHO 0.05

//The precision of the reduction percentage during the dichotomy:
#define P_STOP_DICHO 0.05

#ifndef PI
# define PI 3.14159265359
#endif

#ifndef MIN
# define MIN(X, Y)  ((X) < (Y) ? (X) : (Y))
#endif

#ifndef MAX
# define MAX(X, Y)  ((X) < (Y) ? (Y) : (X))
#endif

slidingClass::slidingClass()
{
    
}

slidingClass::~slidingClass() 
{
    
}

int addStepFeaturesWithSlide(
	trajFeatures & t1,
	const trajFeatures & t2,
	float negativeSlideTime
	) 
{
	if(t1.size != 0 && t2.size != 0) {
	    
	    unsigned int delayInt = (int) (abs(negativeSlideTime)/t2.incrTime);

	    //PHASE 2: add the new t2 to t1
	    for(unsigned int count = 0 ; count < t2.size ; count++) {

		if(count < delayInt) {

		    t1.traj[t1.size - delayInt + count].comX =
			    (t1.traj[t1.size - delayInt + count].comX + t2.traj[count].comX)
			    -t1.traj[t1.size - 1].comX;
		    t1.traj[t1.size - delayInt + count].zmpX =
			    (t1.traj[t1.size - delayInt + count].zmpX + t2.traj[count].zmpX)
			    -t1.traj[t1.size - 1].zmpX;
			    
		    t1.traj[t1.size - delayInt + count].comY =
			    (t1.traj[t1.size - delayInt + count].comY + t2.traj[count].comY)
			    -t1.traj[t1.size - 1].comY;
		    t1.traj[t1.size - delayInt + count].zmpY =
			    (t1.traj[t1.size - delayInt + count].zmpY + t2.traj[count].zmpY)
			    -t1.traj[t1.size - 1].zmpY;
			    
		    t1.traj[t1.size - delayInt + count].leftfootX =
			    (t1.traj[t1.size - delayInt + count].leftfootX + t2.traj[count].leftfootX)
			    -t1.traj[t1.size - 1].leftfootX;
		    t1.traj[t1.size - delayInt + count].leftfootY =
			    (t1.traj[t1.size - delayInt + count].leftfootY + t2.traj[count].leftfootY)
			    -t1.traj[t1.size - 1].leftfootY;
			    
		    t1.traj[t1.size - delayInt + count].leftfootOrient =
			    (t1.traj[t1.size - delayInt + count].leftfootOrient + t2.traj[count].leftfootOrient)
			    -t1.traj[t1.size - 1].leftfootOrient;
			    
		    t1.traj[t1.size - delayInt + count].leftfootHeight =
			    (t1.traj[t1.size - delayInt + count].leftfootHeight + t2.traj[count].leftfootHeight)
			    -t1.traj[t1.size - 1].leftfootHeight;
			    
		    t1.traj[t1.size - delayInt + count].rightfootX =
			    (t1.traj[t1.size - delayInt + count].rightfootX + t2.traj[count].rightfootX)
			    -t1.traj[t1.size - 1].rightfootX;
		    t1.traj[t1.size - delayInt + count].rightfootY =
			    (t1.traj[t1.size - delayInt + count].rightfootY + t2.traj[count].rightfootY)
			    -t1.traj[t1.size - 1].rightfootY;
			    
		    t1.traj[t1.size - delayInt + count].rightfootOrient =
			    (t1.traj[t1.size - delayInt + count].rightfootOrient + t2.traj[count].rightfootOrient)
			    -t1.traj[t1.size - 1].rightfootOrient;
			    
		    t1.traj[t1.size - delayInt + count].rightfootHeight =
			    (t1.traj[t1.size - delayInt + count].rightfootHeight + t2.traj[count].rightfootHeight)
			    -t1.traj[t1.size - 1].rightfootHeight;
		    
		    t1.traj[t1.size - delayInt + count].waistOrient =
			    (t1.traj[t1.size - delayInt + count].waistOrient + t2.traj[count].waistOrient)
			    -t1.traj[t1.size - 1].waistOrient;
			    
		    t1.traj[t1.size - delayInt + count].comHeight =
			    (t1.traj[t1.size - delayInt + count].comHeight + t2.traj[count].comHeight)
			    -t1.traj[t1.size - 1].comHeight;

		} else {

		    instantFeatures instFt;
		    
		    instFt.comX = t2.traj[count].comX;
		    instFt.zmpX = t2.traj[count].zmpX;
		    
		    instFt.comY = t2.traj[count].comY;
		    instFt.zmpY = t2.traj[count].zmpY;
		    
		    instFt.leftfootX = t2.traj[count].leftfootX;
		    instFt.leftfootY = t2.traj[count].leftfootY;
		    
		    instFt.leftfootOrient = t2.traj[count].leftfootOrient;
		    instFt.leftfootHeight = t2.traj[count].leftfootHeight;
		    
		    instFt.rightfootX = t2.traj[count].rightfootX;
		    instFt.rightfootY = t2.traj[count].rightfootY;
		    
		    instFt.rightfootOrient = t2.traj[count].rightfootOrient;
		    instFt.rightfootHeight = t2.traj[count].rightfootHeight;
		    
		    instFt.waistOrient = t2.traj[count].waistOrient;
		    instFt.comHeight = t2.traj[count].comHeight;
		    
		    t1.traj.push_back(instFt);
		}

	    }

	    t1.size = t1.size + t2.size - delayInt;
	    return t1.size - delayInt;
	}
	else if(t1.size == 0) {
	    t1 = t2;    
	    return 0;
	}
	return 0;
}



void findUpDownindexes(const trajFeatures & t, const trajFeatures & downward_halfstep, int & prestartindex, int & startindex, int & endindex, int & postendindex) {
       
    int ind = t.size-1;
    while(t.traj[ind].rightfootHeight > 0.000000001 || t.traj[ind].leftfootHeight > 0.000000001) {
	if(t.traj[ind].rightfootHeight > MIN_FOOT_HEIGHT_FOR_HORIZONTAL_DISPLACEMENT || t.traj[ind].leftfootHeight > MIN_FOOT_HEIGHT_FOR_HORIZONTAL_DISPLACEMENT) 
	    startindex = ind;
	
	prestartindex = ind;
	ind--;
    }
    ind = 0;
    while(downward_halfstep.traj[ind].rightfootHeight > 0.000000001 || downward_halfstep.traj[ind].leftfootHeight > 0.000000001) {
	
	if(downward_halfstep.traj[ind].rightfootHeight > MIN_FOOT_HEIGHT_FOR_HORIZONTAL_DISPLACEMENT || downward_halfstep.traj[ind].leftfootHeight > MIN_FOOT_HEIGHT_FOR_HORIZONTAL_DISPLACEMENT)
	    endindex = ind;
	
	postendindex = ind;
	ind++;
    }
    
}

int slideUpDownCOEF_withindexes(trajFeatures & t, float neg_time, float reduction, trajFeatures & downward_halfstep, const int & prestartindex, const int & startindex, const int & endindex, const int & postendindex) {    

    float leftXstart, rightXstart, leftYstart, rightYstart;
    float leftXfinal, rightXfinal, leftYfinal, rightYfinal;
    
    for(int i = prestartindex; i < (int) t.size; i++) {
	t.traj[i].rightfootHeight *= MAX(reduction, MIN_PERCENT_REDUCTION);
	t.traj[i].leftfootHeight *= MAX(reduction, MIN_PERCENT_REDUCTION);
    }
    for(int i = 0; i <= postendindex; i++) {
	downward_halfstep.traj[i].rightfootHeight *= MAX(reduction, MIN_PERCENT_REDUCTION);
	downward_halfstep.traj[i].leftfootHeight *= MAX(reduction, MIN_PERCENT_REDUCTION);
    }    
    
    float bound_slide = -t.incrTime * (t.size - startindex);
    
    leftXstart = t.traj[prestartindex].leftfootX;
    leftYstart = t.traj[prestartindex].leftfootY;
    rightXstart = t.traj[prestartindex].rightfootX;
    rightYstart = t.traj[prestartindex].rightfootY;          
    leftXfinal = downward_halfstep.traj[postendindex].leftfootX;
    leftYfinal = downward_halfstep.traj[postendindex].leftfootY;
    rightXfinal = downward_halfstep.traj[postendindex].rightfootX;
    rightYfinal = downward_halfstep.traj[postendindex].rightfootY;          
    float T = (float) t.size - startindex + endindex + 1;
    float delta3 = pow(T, 3);
    float delta2 = pow(T, 2);
   
    for(int i = startindex; i < (int) t.size ; i++) {
	float tmp_leftX = leftXstart + (-2/delta3 * pow(i-startindex,3) + 3/delta2 * pow(i-startindex,2)) * (leftXfinal - leftXstart);
	float tmp_leftY = leftYstart + (-2/delta3 * pow(i-startindex,3) + 3/delta2 * pow(i-startindex,2)) * (leftYfinal - leftYstart);
	float tmp_rightX = rightXstart + (-2/delta3 * pow(i-startindex,3) + 3/delta2 * pow(i-startindex,2)) * (rightXfinal - rightXstart);
	float tmp_rightY = rightYstart + (-2/delta3 * pow(i-startindex,3) + 3/delta2 * pow(i-startindex,2)) * (rightYfinal - rightYstart);	
	t.traj[i].leftfootX = MAX(reduction, MIN_PERCENT_REDUCTION)*(t.traj[i].leftfootX - tmp_leftX) + tmp_leftX;
	t.traj[i].leftfootY = MAX(reduction, MIN_PERCENT_REDUCTION)*(t.traj[i].leftfootY - tmp_leftY) + tmp_leftY;
	t.traj[i].rightfootX = MAX(reduction, MIN_PERCENT_REDUCTION)*(t.traj[i].rightfootX - tmp_rightX) + tmp_rightX;
	t.traj[i].rightfootY = MAX(reduction, MIN_PERCENT_REDUCTION)*(t.traj[i].rightfootY - tmp_rightY) + tmp_rightY;
    }

    for(int i = 0; i < endindex ; i++) {
	float tmp_leftX = leftXstart + (-2/delta3 * pow(i+t.size-startindex,3) + 3/delta2 * pow(i+t.size-startindex,2)) * (leftXfinal - leftXstart);
	float tmp_leftY = leftYstart + (-2/delta3 * pow(i+t.size-startindex,3) + 3/delta2 * pow(i+t.size-startindex,2)) * (leftYfinal - leftYstart);
	float tmp_rightX = rightXstart + (-2/delta3 * pow(i+t.size-startindex,3) + 3/delta2 * pow(i+t.size-startindex,2)) * (rightXfinal - rightXstart);
	float tmp_rightY = rightYstart + (-2/delta3 * pow(i+t.size-startindex,3) + 3/delta2 * pow(i+t.size-startindex,2)) * (rightYfinal - rightYstart);
	downward_halfstep.traj[i].leftfootX = MAX(reduction, MIN_PERCENT_REDUCTION)*(downward_halfstep.traj[i].leftfootX - tmp_leftX) + tmp_leftX;
	downward_halfstep.traj[i].leftfootY = MAX(reduction, MIN_PERCENT_REDUCTION)*(downward_halfstep.traj[i].leftfootY - tmp_leftY) + tmp_leftY;
	downward_halfstep.traj[i].rightfootX = MAX(reduction, MIN_PERCENT_REDUCTION)*(downward_halfstep.traj[i].rightfootX - tmp_rightX) + tmp_rightX;
	downward_halfstep.traj[i].rightfootY = MAX(reduction, MIN_PERCENT_REDUCTION)*(downward_halfstep.traj[i].rightfootY - tmp_rightY) + tmp_rightY;
    }      
    
    return addStepFeaturesWithSlide(t,downward_halfstep, MAX(neg_time, bound_slide) ); 
        
}

int slidingClass::slideUpDownCOEF(trajFeatures & t, float neg_time, float reduction, trajFeatures & downward_halfstep) {
      
    int prestartindex, startindex, endindex, postendindex;
    findUpDownindexes(t, downward_halfstep, prestartindex, startindex, endindex, postendindex);
    return slideUpDownCOEF_withindexes(t, neg_time, reduction, downward_halfstep, prestartindex, startindex, endindex, postendindex);
    
}

bool slidingClass::check_slideUpDown(
	const trajFeatures & t1,
	float neg_time, 
	float reduction,
	const trajFeatures & t2,
	const int & prestartindex,
	const int & startindex,
	const int & endindex,
	const int & postendindex) {
    
    if(t1.size != 0 && t2.size != 0) {
	    
	    unsigned int delayInt = t1.size - prestartindex;

	    trajFeatures t1bis;
	    t1bis.traj.resize(delayInt);
	    for(unsigned int i = 0; i < delayInt; i++) {
		t1bis.traj[i] = t1.traj[t1.size - delayInt + i];
	    }
	    t1bis.incrTime = t1.incrTime;
	    t1bis.size = delayInt;
	    
	    trajFeatures t2bis;
	    t2bis.traj.resize(t2.size);
	    for(unsigned int i = 0; i < t2.size; i++) {
		t2bis.traj[i] = t2.traj[i];
	    }
	    t2bis.incrTime = t2.incrTime;
	    t2bis.size = t2.size;
	    
	    slideUpDownCOEF_withindexes(t1bis, neg_time, reduction, t2bis, 0, startindex - prestartindex, endindex, postendindex);
	    
	    for(unsigned int i = 0; i < t1bis.size; i+=20) {    
		if(!isValid(t1bis.traj[i])) {
		    return false;
		}
	    }
    }    
    return true;    
}

int slidingClass::slideUpDownMAX(trajFeatures & t, trajFeatures & downward_halfstep) {
    
    int prestartindex, startindex, endindex, postendindex;
    findUpDownindexes(t, downward_halfstep, prestartindex, startindex, endindex, postendindex);

    float bound_slide = -t.incrTime * (t.size - startindex);
            
    float max_reduction_coef = 1.0;
    float min_reduction_coef = MIN_PERCENT_REDUCTION;
    float current_reduction_coef = MIN_PERCENT_REDUCTION;
    
    float max_neg_time = 0.0;
    float min_neg_time = bound_slide;
    float current_neg_time = bound_slide;
    
    bool altern = true;
    
    while(max_neg_time - min_neg_time > T_STOP_DICHO || max_reduction_coef - min_reduction_coef > P_STOP_DICHO) 
    {	
	bool yn = check_slideUpDown(t, current_neg_time, current_reduction_coef, downward_halfstep, prestartindex, startindex, endindex, postendindex);
	
	if(yn && altern) {
	    max_neg_time = current_neg_time;
	    current_neg_time = (max_neg_time + min_neg_time)/2.0;
	    altern = !altern;
	}
	else if (yn && !altern) {
	    max_reduction_coef = current_reduction_coef;
	    current_reduction_coef = (max_reduction_coef + min_reduction_coef)/2.0;
	    altern = !altern;   
	}
	else if (!yn && altern) {
	    min_neg_time = current_neg_time;
	    current_neg_time = (max_neg_time + min_neg_time)/2.0;
	    altern = !altern;
	} 
	else {
	    min_reduction_coef = current_reduction_coef;
	    current_reduction_coef = (max_reduction_coef + min_reduction_coef)/2.0;
	    altern = !altern;
	}
    }    
    
/*    while(max_reduction_coef - min_reduction_coef > P_STOP_DICHO) 
    {
	bool yn = check_slideUpDown(t, current_neg_time, current_reduction_coef, downward_halfstep, prestartindex, startindex, endindex, postendindex);
	
	if(yn) {
	    max_reduction_coef = current_reduction_coef;
	    current_reduction_coef = (max_reduction_coef + min_reduction_coef)/2.0;
	}
	else {
	    min_reduction_coef = current_reduction_coef;
	    current_reduction_coef = (max_reduction_coef + min_reduction_coef)/2.0;
	}
    }*/    
    
    return slideUpDownCOEF(t, current_neg_time, current_reduction_coef, downward_halfstep);    
}




void findDownUpindexes(const trajFeatures & t, const trajFeatures & upward_halfstep, int & prestartindex, int & postendindex) {
    
    int ind = t.size-1;
    if(t.size != 0) {
	while(t.traj[ind].rightfootHeight < 0.0000001 && t.traj[ind].leftfootHeight < 0.0000001) {
	    prestartindex = ind;
	    ind--;
	}
    } else {
	prestartindex = 0;
    }
    
    ind = 0;
    while(upward_halfstep.traj[ind].rightfootHeight < 0.0000001 && upward_halfstep.traj[ind].leftfootHeight < 0.0000001) {	
	postendindex = ind;
	ind++;
    } 
    
}

int slideDownUpCOEF_withindexes(trajFeatures & t, float neg_time, const trajFeatures & upward_halfstep, const int & prestartindex, const int & postendindex) {    

    float bound_slide = MIN(-t.incrTime * (t.size - prestartindex) - t.incrTime * (postendindex) + MIN_DOUBLE_SUPPORT_TIME, 0.0);    
    return addStepFeaturesWithSlide(t,upward_halfstep, MAX(neg_time, bound_slide) );
    
}
    
int slidingClass::slideDownUpCOEF(trajFeatures & t, float neg_time, const trajFeatures & upward_halfstep) {        
       
    int prestartindex, postendindex;
    findDownUpindexes(t, upward_halfstep, prestartindex, postendindex);
    return slideDownUpCOEF_withindexes(t, neg_time, upward_halfstep, prestartindex, postendindex);
}

bool slidingClass::check_slideDownUp(
	const trajFeatures & t1,
	float neg_time, 
	const trajFeatures & t2) {
    
    if(t1.size != 0 && t2.size != 0) {
	    
	    unsigned int delayInt = (int) (abs(neg_time)/t2.incrTime);

	    trajFeatures t1bis;
	    t1bis.traj.resize(delayInt);
	    for(unsigned int i = 0; i < delayInt; i++) {
		t1bis.traj[i] = t1.traj[t1.size - delayInt + i];
	    }
	    t1bis.incrTime = t1.incrTime;
	    t1bis.size = delayInt;
	    
	    slideDownUpCOEF(t1bis, neg_time, t2);
	    
	    for(unsigned int i = 0; i < t1bis.size; i+=20) {
		if(!isValid(t1bis.traj[i])) return false;
	    }
    }    
    return true;    
}

int slidingClass::slideDownUpMAX(trajFeatures & t, trajFeatures & upward_halfstep) {
    
    int prestartindex, postendindex;
    findDownUpindexes(t, upward_halfstep, prestartindex, postendindex);
    
    float bound_slide = MIN(-t.incrTime * (t.size - prestartindex) - t.incrTime * (postendindex) + MIN_DOUBLE_SUPPORT_TIME, 0.0);
    
    float max_neg_time = 0.0;
    float min_neg_time = bound_slide;
    float current_neg_time = bound_slide;
    
    while(max_neg_time - min_neg_time > T_STOP_DICHO) 
    {
	bool yn = check_slideDownUp(t, current_neg_time, upward_halfstep);
	
	if(yn) {
	    max_neg_time = current_neg_time;
	    current_neg_time = (max_neg_time + min_neg_time)/2.0;
	}
	else {
	    min_neg_time = current_neg_time;
	    current_neg_time = (max_neg_time + min_neg_time)/2.0;
	}
    }    
    
    return slideDownUpCOEF(t, current_neg_time, upward_halfstep);        
}
