//  Copyright AIST-CNRS Joint Robotics Laboratory
//  Author: Nicolas Perrin

#include "newSliderPG/slidingFunctions.h"

//During the smoothing, the foot can start to move horizontally very early, 
//but we still want the tae-off and the landing to be vertical, so we define 
//a minimum height (in m.) for the foot to move horizontally:
#define MIN_FOOT_HEIGHT_FOR_HORIZONTAL_DISPLACEMENT 0.001

//The minimum duration of the double support (in s.):
#define MIN_DOUBLE_SUPPORT_TIME 0.1

//The minimum coefficient for the trajectory smoothing (for example, with 1.0 the 
//trajectory is not modified, and for 0.0 the feet height will always be 0):
#define MIN_PERCENT_REDUCTION 0.1

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
	trajFeatures & t2,
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

int slidingClass::slideUpDownMAX(trajFeatures & t, trajFeatures & downward_halfstep) {
    addStepFeaturesWithSlide(t,downward_halfstep,0.0);    
}

int slidingClass::slideUpDownCOEF(trajFeatures & t, float neg_time, float reduction, trajFeatures & downward_halfstep) {
    
    int ind = t.size-1;
    int startindex, endindex, prestartindex, postendindex;
    float leftXstart, rightXstart, leftYstart, rightYstart;
    float leftXfinal, rightXfinal, leftYfinal, rightYfinal;
    while(t.traj[ind].rightfootHeight > 0.0000001 || t.traj[ind].leftfootHeight > 0.0000001) {
	t.traj[ind].rightfootHeight *= MAX(reduction, MIN_PERCENT_REDUCTION);
	t.traj[ind].leftfootHeight *= MAX(reduction, MIN_PERCENT_REDUCTION);
	
	if(t.traj[ind].rightfootHeight > MIN_FOOT_HEIGHT_FOR_HORIZONTAL_DISPLACEMENT || t.traj[ind].leftfootHeight > MIN_FOOT_HEIGHT_FOR_HORIZONTAL_DISPLACEMENT) 
	    startindex = ind;
	
	prestartindex = ind;
	ind--;
    }
    ind = 0;
    while(downward_halfstep.traj[ind].rightfootHeight > 0.0000001 || downward_halfstep.traj[ind].leftfootHeight > 0.0000001) {	
	downward_halfstep.traj[ind].rightfootHeight *= MAX(reduction, MIN_PERCENT_REDUCTION);
	downward_halfstep.traj[ind].leftfootHeight *= MAX(reduction, MIN_PERCENT_REDUCTION);
	
	if(downward_halfstep.traj[ind].rightfootHeight > MIN_FOOT_HEIGHT_FOR_HORIZONTAL_DISPLACEMENT || downward_halfstep.traj[ind].leftfootHeight > MIN_FOOT_HEIGHT_FOR_HORIZONTAL_DISPLACEMENT)
	    endindex = ind;
	
	postendindex = ind;
	ind++;
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

int slidingClass::slideDownUpMAX(trajFeatures & t, trajFeatures & upward_halfstep) {
    return addStepFeaturesWithSlide(t,upward_halfstep,0.0);        
}

int slidingClass::slideDownUpCOEF(trajFeatures & t, float neg_time, float reduction, trajFeatures & upward_halfstep) {        
       
    int ind = t.size-1;
    int prestartindex, postendindex;
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
    
    float bound_slide = MIN(-t.incrTime * (t.size - prestartindex) - t.incrTime * (postendindex) + MIN_DOUBLE_SUPPORT_TIME, 0.0);    
    
    return addStepFeaturesWithSlide(t,upward_halfstep, MAX(neg_time, bound_slide) );
}