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
    
    int ind = t.size-1;
    int startindex, endindex;
    float leftXstart, rightXstart, leftYstart, rightYstart;
    float leftXfinal, rightXfinal, leftYfinal, rightYfinal;
    while(t.traj[ind].rightfootHeight > 0.01 || t.traj[ind].leftfootHeight > 0.01) {
	t.traj[ind].rightfootHeight *= 0.3;
	t.traj[ind].leftfootHeight *= 0.3;
	
	startindex = ind;
	ind--;
    }
    ind = 0;
    while(downward_halfstep.traj[ind].rightfootHeight > 0.01 || downward_halfstep.traj[ind].leftfootHeight > 0.01) {	
	downward_halfstep.traj[ind].rightfootHeight *= 0.3;
	downward_halfstep.traj[ind].leftfootHeight *= 0.3;
	
	endindex = ind;
	ind++;
    }   
    
    leftXstart = t.traj[startindex].leftfootX;
    leftYstart = t.traj[startindex].leftfootY;
    rightXstart = t.traj[startindex].rightfootX;
    rightYstart = t.traj[startindex].rightfootY;          
    leftXfinal = downward_halfstep.traj[endindex].leftfootX;
    leftYfinal = downward_halfstep.traj[endindex].leftfootY;
    rightXfinal = downward_halfstep.traj[endindex].rightfootX;
    rightYfinal = downward_halfstep.traj[endindex].rightfootY;          
    float T = (float) t.size - startindex + endindex + 1;
    float delta3 = pow(T, 3);
    float delta2 = pow(T, 2);
   
    for(int i = startindex; i < t.size ; i++) {
	float tmp_leftX = leftXstart + (-2/delta3 * pow(i-startindex,3) + 3/delta2 * pow(i-startindex,2)) * (leftXfinal - leftXstart);
	float tmp_leftY = leftYstart + (-2/delta3 * pow(i-startindex,3) + 3/delta2 * pow(i-startindex,2)) * (leftYfinal - leftYstart);
	float tmp_rightX = rightXstart + (-2/delta3 * pow(i-startindex,3) + 3/delta2 * pow(i-startindex,2)) * (rightXfinal - rightXstart);
	float tmp_rightY = rightYstart + (-2/delta3 * pow(i-startindex,3) + 3/delta2 * pow(i-startindex,2)) * (rightYfinal - rightYstart);	
	t.traj[i].leftfootX = 0.5*(t.traj[i].leftfootX - tmp_leftX) + tmp_leftX;
	t.traj[i].leftfootY = 0.5*(t.traj[i].leftfootY - tmp_leftY) + tmp_leftY;
	t.traj[i].rightfootX = 0.5*(t.traj[i].rightfootX - tmp_rightX) + tmp_rightX;
	t.traj[i].rightfootY = 0.5*(t.traj[i].rightfootY - tmp_rightY) + tmp_rightY;
    }

    for(int i = 0; i < endindex ; i++) {
	float tmp_leftX = leftXstart + (-2/delta3 * pow(i+t.size-startindex,3) + 3/delta2 * pow(i+t.size-startindex,2)) * (leftXfinal - leftXstart);
	float tmp_leftY = leftYstart + (-2/delta3 * pow(i+t.size-startindex,3) + 3/delta2 * pow(i+t.size-startindex,2)) * (leftYfinal - leftYstart);
	float tmp_rightX = rightXstart + (-2/delta3 * pow(i+t.size-startindex,3) + 3/delta2 * pow(i+t.size-startindex,2)) * (rightXfinal - rightXstart);
	float tmp_rightY = rightYstart + (-2/delta3 * pow(i+t.size-startindex,3) + 3/delta2 * pow(i+t.size-startindex,2)) * (rightYfinal - rightYstart);
	downward_halfstep.traj[i].leftfootX = 0.5*(downward_halfstep.traj[i].leftfootX - tmp_leftX) + tmp_leftX;
	downward_halfstep.traj[i].leftfootY = 0.5*(downward_halfstep.traj[i].leftfootY - tmp_leftY) + tmp_leftY;
	downward_halfstep.traj[i].rightfootX = 0.5*(downward_halfstep.traj[i].rightfootX - tmp_rightX) + tmp_rightX;
	downward_halfstep.traj[i].rightfootY = 0.5*(downward_halfstep.traj[i].rightfootY - tmp_rightY) + tmp_rightY;
    }      
    
    addStepFeaturesWithSlide(t,downward_halfstep,neg_time);   
}

void slidingClass::slideDownUpMAX(trajFeatures & t, trajFeatures & upward_halfstep) {
    addStepFeaturesWithSlide(t,upward_halfstep,0.0);        
}

void slidingClass::slideDownUpCOEF(trajFeatures & t, float neg_time, trajFeatures & upward_halfstep) {        
    addStepFeaturesWithSlide(t,upward_halfstep,neg_time);
}