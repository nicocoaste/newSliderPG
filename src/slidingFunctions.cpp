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

// 	    //PHASE 1: modify stepF2 according to the change of origin
// 	    float lastwaistX = stepF1.traj[stepF1.size - 1].comX;
// 	    float lastwaistY = stepF1.traj[stepF1.size - 1].comY; 	
// 	    float radlastwaistOrient = stepF1.traj[stepF1.size - 1].waistOrient*PI/180;
// 
// 	    for(unsigned int count = 0 ; count < stepF2.size ; count++) {
// 
// 		float newcomX = (stepF2.traj[count].comX)*cos(radlastwaistOrient)
// 				-(stepF2.traj[count].comY)*sin(radlastwaistOrient)+lastwaistX;
// 		float newcomY = (stepF2.traj[count].comX)*sin(radlastwaistOrient)
// 				+(stepF2.traj[count].comY)*cos(radlastwaistOrient)+lastwaistY;
// 		float newzmpX = (stepF2.traj[count].zmpX)*cos(radlastwaistOrient)
// 				-(stepF2.traj[count].zmpY)*sin(radlastwaistOrient)+lastwaistX;
// 		float newzmpY = (stepF2.traj[count].zmpX)*sin(radlastwaistOrient)
// 				+(stepF2.traj[count].zmpY)*cos(radlastwaistOrient)+lastwaistY;
// 		float newlfX = (stepF2.traj[count].leftfootX)*cos(radlastwaistOrient)
// 				-(stepF2.traj[count].leftfootY)*sin(radlastwaistOrient)+lastwaistX;	
// 		float newlfY = (stepF2.traj[count].leftfootX)*sin(radlastwaistOrient)
// 				+(stepF2.traj[count].leftfootY)*cos(radlastwaistOrient)+lastwaistY;	
// 		float newrfX = (stepF2.traj[count].rightfootX)*cos(radlastwaistOrient)
// 				-(stepF2.traj[count].rightfootY)*sin(radlastwaistOrient)+lastwaistX;
// 		float newrfY = (stepF2.traj[count].rightfootX)*sin(radlastwaistOrient)   
// 				+(stepF2.traj[count].rightfootY)*cos(radlastwaistOrient)+lastwaistY;    
// 
// 		stepF2.traj[count].comX = newcomX;
// 		stepF2.traj[count].zmpX = newzmpX;
// 
// 		stepF2.traj[count].comY = newcomY;
// 		stepF2.traj[count].zmpY = newzmpY;
// 
// 		stepF2.traj[count].leftfootX = newlfX;
// 		stepF2.traj[count].leftfootY = newlfY;
// 
// 		stepF2.traj[count].leftfootOrient += stepF1.traj[stepF1.size - 1].waistOrient;
// 
// 		stepF2.traj[count].rightfootX = newrfX;
// 		stepF2.traj[count].rightfootY = newrfY;
// 
// 		stepF2.traj[count].rightfootOrient += stepF1.traj[stepF1.size - 1].waistOrient;
// 
// 		stepF2.traj[count].waistOrient += stepF1.traj[stepF1.size - 1].waistOrient;
// 
// 	    }
	    
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
}

void slidingClass::slideDownUpMAX(trajFeatures & t, trajFeatures & upward_halfstep) {
    addStepFeaturesWithSlide(t,upward_halfstep,0.0);        
}

void slidingClass::slideDownUpCOEF(trajFeatures & t, float neg_time, trajFeatures & upward_halfstep) {
    addStepFeaturesWithSlide(t,upward_halfstep,neg_time);        
}