//  Copyright AIST-CNRS Joint Robotics Laboratory
//  Author: Nicolas Perrin

#include "newSliderPG/slidingFunctions.h"

#ifndef PI
# define PI 3.14159265359
#endif

#ifndef MIN
# define MIN(X, Y)  ((X) < (Y) ? (X) : (Y))
#endif

#ifndef MAX
# define MAX(X, Y)  ((X) < (Y) ? (Y) : (X))
#endif

slidingClass::slidingClass(float updo_max, float doup_max) : 
	maxSlideUpDown(updo_max), maxSlideDownUp(doup_max)
{
    
}

slidingClass::~slidingClass() 
{
    
}

void addStepFeaturesWithSlide(
	trajFeatures & stepF1,
	trajFeatures & stepF2,
	float negativeSlideTime
	) 
{
	if(stepF1.size != 0 && stepF2.size != 0) {
	    
	    unsigned int delayInt = (int) (abs(negativeSlideTime)/stepF2.incrTime);

	    //PHASE 2: add the new stepF2 to stepF1
	    for(unsigned int count = 0 ; count < stepF2.size ; count++) {

		if(count < delayInt) {

		    stepF1.traj[stepF1.size - delayInt + count].comX =
			    (stepF1.traj[stepF1.size - delayInt + count].comX + stepF2.traj[count].comX)
			    -stepF1.traj[stepF1.size - 1].comX;
		    stepF1.traj[stepF1.size - delayInt + count].zmpX =
			    (stepF1.traj[stepF1.size - delayInt + count].zmpX + stepF2.traj[count].zmpX)
			    -stepF1.traj[stepF1.size - 1].zmpX;
			    
		    stepF1.traj[stepF1.size - delayInt + count].comY =
			    (stepF1.traj[stepF1.size - delayInt + count].comY + stepF2.traj[count].comY)
			    -stepF1.traj[stepF1.size - 1].comY;
		    stepF1.traj[stepF1.size - delayInt + count].zmpY =
			    (stepF1.traj[stepF1.size - delayInt + count].zmpY + stepF2.traj[count].zmpY)
			    -stepF1.traj[stepF1.size - 1].zmpY;
			    
		    stepF1.traj[stepF1.size - delayInt + count].leftfootX =
			    (stepF1.traj[stepF1.size - delayInt + count].leftfootX + stepF2.traj[count].leftfootX)
			    -stepF1.traj[stepF1.size - 1].leftfootX;
		    stepF1.traj[stepF1.size - delayInt + count].leftfootY =
			    (stepF1.traj[stepF1.size - delayInt + count].leftfootY + stepF2.traj[count].leftfootY)
			    -stepF1.traj[stepF1.size - 1].leftfootY;
			    
		    stepF1.traj[stepF1.size - delayInt + count].leftfootOrient =
			    (stepF1.traj[stepF1.size - delayInt + count].leftfootOrient + stepF2.traj[count].leftfootOrient)
			    -stepF1.traj[stepF1.size - 1].leftfootOrient;
			    
		    stepF1.traj[stepF1.size - delayInt + count].leftfootHeight =
			    (stepF1.traj[stepF1.size - delayInt + count].leftfootHeight + stepF2.traj[count].leftfootHeight)
			    -stepF1.traj[stepF1.size - 1].leftfootHeight;
			    
		    stepF1.traj[stepF1.size - delayInt + count].rightfootX =
			    (stepF1.traj[stepF1.size - delayInt + count].rightfootX + stepF2.traj[count].rightfootX)
			    -stepF1.traj[stepF1.size - 1].rightfootX;
		    stepF1.traj[stepF1.size - delayInt + count].rightfootY =
			    (stepF1.traj[stepF1.size - delayInt + count].rightfootY + stepF2.traj[count].rightfootY)
			    -stepF1.traj[stepF1.size - 1].rightfootY;
			    
		    stepF1.traj[stepF1.size - delayInt + count].rightfootOrient =
			    (stepF1.traj[stepF1.size - delayInt + count].rightfootOrient + stepF2.traj[count].rightfootOrient)
			    -stepF1.traj[stepF1.size - 1].rightfootOrient;
			    
		    stepF1.traj[stepF1.size - delayInt + count].rightfootHeight =
			    (stepF1.traj[stepF1.size - delayInt + count].rightfootHeight + stepF2.traj[count].rightfootHeight)
			    -stepF1.traj[stepF1.size - 1].rightfootHeight;
		    
		    stepF1.traj[stepF1.size - delayInt + count].waistOrient =
			    (stepF1.traj[stepF1.size - delayInt + count].waistOrient + stepF2.traj[count].waistOrient)
			    -stepF1.traj[stepF1.size - 1].waistOrient;
			    
		    stepF1.traj[stepF1.size - delayInt + count].comHeight =
			    (stepF1.traj[stepF1.size - delayInt + count].comHeight + stepF2.traj[count].comHeight)
			    -stepF1.traj[stepF1.size - 1].comHeight;

		} else {

		    instantFeatures instFt;
		    
		    instFt.comX = stepF2.traj[count].comX;
		    instFt.zmpX = stepF2.traj[count].zmpX;
		    
		    instFt.comY = stepF2.traj[count].comY;
		    instFt.zmpY = stepF2.traj[count].zmpY;
		    
		    instFt.leftfootX = stepF2.traj[count].leftfootX;
		    instFt.leftfootY = stepF2.traj[count].leftfootY;
		    
		    instFt.leftfootOrient = stepF2.traj[count].leftfootOrient;
		    instFt.leftfootHeight = stepF2.traj[count].leftfootHeight;
		    
		    instFt.rightfootX = stepF2.traj[count].rightfootX;
		    instFt.rightfootY = stepF2.traj[count].rightfootY;
		    
		    instFt.rightfootOrient = stepF2.traj[count].rightfootOrient;
		    instFt.rightfootHeight = stepF2.traj[count].rightfootHeight;
		    
		    instFt.waistOrient = stepF2.traj[count].waistOrient;
		    instFt.comHeight = stepF2.traj[count].comHeight;
		    
		    stepF1.traj.push_back(instFt);
		}

	    }

	    stepF1.size = stepF1.size + stepF2.size - delayInt;
	}
	else if(stepF1.size == 0) {
	    stepF1 = stepF2;    
	}
}

void slidingClass::slideUpDownMAX(trajFeatures & t, trajFeatures & downward_halfstep) {
    addStepFeaturesWithSlide(t,downward_halfstep,0.0);    
}

void slidingClass::slideUpDownCOEF(trajFeatures & t, float neg_time, trajFeatures & downward_halfstep) {
    addStepFeaturesWithSlide(t,downward_halfstep,neg_time);
    float leftXfinal = downward_halfstep.traj[downward_halfstep.size-1].leftfootX;
    float leftYfinal = downward_halfstep.traj[downward_halfstep.size-1].leftfootY;    
    float rightXfinal = downward_halfstep.traj[downward_halfstep.size-1].rightfootX;
    float rightYfinal = downward_halfstep.traj[downward_halfstep.size-1].rightfootY;
    float leftXstart, rightXstart, leftYstart, rightYstart;
    int trigger = false;        
    int trigger2 = false;
    int endindex, startindex;       
    
    for(unsigned int i = t.contact_indexes[t.contact_indexes.size()-1]; i < t.size; i++) {
	if(trigger) {

	    t.traj[i].rightfootHeight *= 0.3;
	    t.traj[i].leftfootHeight *= 0.3;
	    
	    if(t.traj[i].rightfootHeight > 0.02 || t.traj[i].leftfootHeight > 0.02) { //This height is to be tuned
		trigger2 = true;
		startindex = i;
	    }
	    if(trigger2 && t.traj[i].rightfootHeight < 0.015 && t.traj[i].leftfootHeight < 0.015) { //This height is to be tuned
		endindex = i;
	    }
	}
	if(!trigger && t.traj[i].rightfootHeight < 0.001 && t.traj[i].leftfootHeight < 0.001) {
	    trigger = true;
	    leftXstart = t.traj[i].leftfootX;
	    leftYstart = t.traj[i].leftfootY;
	    rightXstart = t.traj[i].rightfootX;
	    rightYstart = t.traj[i].rightfootY;    
	}
    }
    
    float delta3 = pow(endindex - startindex + 1,3);
    float delta2 = pow(endindex - startindex + 1,2);
    
    for(int i = startindex; i <= endindex ; i++) {
	float tmp_leftX = leftXstart + (-2/delta3 * pow(i-startindex,3) + 3/delta2 * pow(i-startindex,2)) * (leftXfinal - leftXstart);
	float tmp_leftY = leftYstart + (-2/delta3 * pow(i-startindex,3) + 3/delta2 * pow(i-startindex,2)) * (leftYfinal - leftYstart);
	float tmp_rightX = rightXstart + (-2/delta3 * pow(i-startindex,3) + 3/delta2 * pow(i-startindex,2)) * (rightXfinal - rightXstart);
	float tmp_rightY = rightYstart + (-2/delta3 * pow(i-startindex,3) + 3/delta2 * pow(i-startindex,2)) * (rightYfinal - rightYstart);
	
	t.traj[i].leftfootX = 0.5*(t.traj[i].leftfootX - tmp_leftX) + tmp_leftX;
	t.traj[i].leftfootY = 0.5*(t.traj[i].leftfootY - tmp_leftY) + tmp_leftY;
	t.traj[i].rightfootX = 0.5*(t.traj[i].rightfootX - tmp_rightX) + tmp_rightX;
	t.traj[i].rightfootY = 0.5*(t.traj[i].rightfootY - tmp_rightY) + tmp_rightY;
    }

//     float delta3 = pow(t.contact_indexes[t.contact_indexes.size()-1] - t.size,3);
//     float delta2 = pow(endindex - startindex + 1,2);
//     
//     for(int i = t.contact_indexes[t.contact_indexes.size()-1]; i < t.size ; i++) {
// 	float tmp_leftX = leftXstart + (-2/delta3 * pow(i-startindex,3) + 3/delta2 * pow(i-startindex,2)) * (leftXfinal - leftXstart);
// 	float tmp_leftY = leftYstart + (-2/delta3 * pow(i-startindex,3) + 3/delta2 * pow(i-startindex,2)) * (leftYfinal - leftYstart);
// 	float tmp_rightX = rightXstart + (-2/delta3 * pow(i-startindex,3) + 3/delta2 * pow(i-startindex,2)) * (rightXfinal - rightXstart);
// 	float tmp_rightY = rightYstart + (-2/delta3 * pow(i-startindex,3) + 3/delta2 * pow(i-startindex,2)) * (rightYfinal - rightYstart);
// 	
// 	t.traj[i].leftfootX *= 0.0*(t.traj[i].leftfootX - tmp_leftX) + tmp_leftX;
// 	t.traj[i].leftfootY *= 0.0*(t.traj[i].leftfootY - tmp_leftY) + tmp_leftY;
// 	t.traj[i].rightfootX *= 0.0*(t.traj[i].rightfootX - tmp_rightX) + tmp_rightX;
// 	t.traj[i].rightfootY *= 0.0*(t.traj[i].rightfootY - tmp_rightY) + tmp_rightY;
//     }
    
    t.contact_indexes.push_back(t.size-1);
}

void slidingClass::slideDownUpMAX(trajFeatures & t, trajFeatures & upward_halfstep) {
    addStepFeaturesWithSlide(t,upward_halfstep,0.0);        
}

void slidingClass::slideDownUpCOEF(trajFeatures & t, float neg_time, trajFeatures & upward_halfstep) {        
    addStepFeaturesWithSlide(t,upward_halfstep,neg_time);    
    t.contact_indexes.push_back(t.size-upward_halfstep.size);
}