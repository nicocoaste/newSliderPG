//  Copyright AIST-CNRS Joint Robotics Laboratory
//  Author: Nicolas Perrin

#ifndef PI
# define PI 3.14159265359
#endif

#ifndef MIN
# define MIN(X, Y)  ((X) < (Y) ? (X) : (Y))
#endif

#ifndef MAX
# define MAX(X, Y)  ((X) < (Y) ? (Y) : (X))
#endif

// r_param controls "how fast" the zmp will shift.
// A few possible values:
// v_100:    1.015179
// v_99_9:   1.347141
// v_99_5:   1.558471
// v_99:     1.692279
// v_97:     1.988992
// v_95:     2.177190
// v_90:     2.507291
// v_85:     2.754411
// v_80:     2.962078
// v_75:     3.146264
// #define r_PARAM 1.347141
#define r_PARAM 1.692279

//before using the quickly shifting zmp curve, we introduce a discrepancy between the 
//com position and the zmp position. Thanks to that, the initial speed required for
//the com can be not too large... This discrepancy will be realized at time t_start (for an upward half-step)
//and the p_GAP is used to define it: if the radius of the largest circle inside the foot is r,
//the discrepancy will be equal to p_GAP * r
#define p_GAP 0.15

//During half-steps, if the foot is in the air during s seconds, the horizontal motion lasts less
//than s seconds. We only move the foot horizontally for a duration h_PER * s
//Thanks to this the foot has time to first rise almost vertically, and then move horizontally.
//This gives a better obstacle clearance.
#define h_PER 0.70

//Percentage of variation of the com height, relatively to the standard com height:
#define COEF_VARIATION_COM_HEIGHT 0.02

#include "newSliderPG/halfStep_creation.h"

//a handy function that takes a global SE2 vector vref (in the reference frame), and a SE2 vector vrel relative to vref,
//and constructs the global SE2 vector corresponding to the configuration described by vrel, but given in the reference frame.
void composeSE2( SE2 & resultVect, const SE2 & globalVect, const SE2 & relativeVect) {
    resultVect.x = globalVect.x + relativeVect.x * cos(globalVect.theta) - relativeVect.y * sin(globalVect.theta);
    resultVect.y = globalVect.y + relativeVect.x * sin(globalVect.theta) + relativeVect.y * cos(globalVect.theta);
    resultVect.theta = globalVect.theta + relativeVect.theta;    
}

float A( float x ) {    
    return x*x*(2-x)*(2-x);               
}

float B( float r, float x ) {    
    return (1+r)*x-r*x*x;                
}

//This function y obtained with wxmaxima is the main solution of the differential equation that gives the com in function of the zmp
// K = z_com / g
// val = the value aimed at (this is the value towards which the ZMP goes) (in m.)
// T = the total time after which the function should be equal to val, like the ZMP. (in s.)
// r = a parameter of the ZMP curve. The standard value is 1.015179, but better results
//     can be obtained for larger values of r.
// position = the start point of the com, i.e. the y function (but the zmp starts at zero.
// speed = the initial speed of the com
// x is actually the time, so the main variable.
// 
// 	There are 2 way to use y:
// 	(CASE 1:) without change: it corresponds to starting at 'position' and aiming at 'val', 
// 	          moving the zmp towards 'val' as soon as possible.
// 	(CASE 2:) using    val - y(K,r,val,T,position,speed,T-x):
// 	          it also corresponds to starting at 'position' and aiming at 'val', but this
// 	          time the zmp stays near 'position' as long as possible.
// 
// 	In the 2 cases, the zmp curves are obtained:
// 	(CASE 1): val*A(B(r,x/T))
// 	(CASE 2): val - val*A(B(r,1-x/T))
float y( float K, float r, float val, float T, float position, double speed, float x) {
    
    float x2 = x*x;
    float x3 = x2 * x;
    float x4 = x3 * x;
    float x5 = x4 * x;
    float x6 = x5 * x;
    float x7 = x6 * x;
    float x8 = x7 * x;
  
    float T2 = T*T;
    float T3 = T2 * T;
    float T4 = T3 * T;
    float T5 = T4 * T;
    float T6 = T5 * T;
    float T7 = T6 * T;
    float T8 = T7 * T;

    float r2 = r*r;
    float r3 = r2 * r;
    float r4 = r3 * r;
    
    float K2 = K*K;
    float K3 = K2 * K;
    float K4 = K3 * K;
        
    float result = (exp(x/sqrt(K))*((speed*sqrt(K)+position)*T8+(-8*r2-16*r-8)*val*K*T6+val*(24*r3*sqrt(K)+120*r2*sqrt(K)+120*r*sqrt(K)+24*sqrt(K))*K*T5+
(-24*r4-384*r3-816*r2-384*r-24)*val*K2*T4+val*(480*r4*sqrt(K)+2880*r3*sqrt(K)+2880*r2*sqrt(K)+480*r*sqrt(K))*K2*T3+(-4320*r4-11520*r3-4320*r2)*
val*K3*T2+val*(20160*r4*sqrt(K)+20160*r3*sqrt(K))*K3*T-40320*r4*val*K4))/(2*T8)+(exp(-x/sqrt(K))*((position-speed*sqrt(K))*T8+(-8*r2-16*r-8)*val*K*T6
+val*(-24*r3*sqrt(K)-120*r2*sqrt(K)-120*r*sqrt(K)-24*sqrt(K))*K*T5+(-24*r4-384*r3-816*r2-384*r-24)*val*K2*T4+val*
(-480*r4*sqrt(K)-2880*r3*sqrt(K)-2880*r2*sqrt(K)-480*r*sqrt(K))*K2*T3+(-4320*r4-11520*r3-4320*r2)*val*K3*T2+val*(-20160*r4*sqrt(K)-20160*r3*sqrt(K))*K3*T-
40320*r4*val*K4))/(2*T8)+(((8*r2+16*r+8)*val*K+(4*r2+8*r+4)*val*x2)*T6+
((-24*r3-120*r2-120*r-24)*val*x*K+(-4*r3-20*r2-20*r-4)*val*x3)*T5+
((24*r4+384*r3+816*r2+384*r+24)*val*K2+(12*r4+192*r3+408*r2+192*r+12)*val*x2*K+(r4+16*r3+34*r2+16*r+1)*val*x4)*T4+
((-480*r4-2880*r3-2880*r2-480*r)*val*x*K2+(-80*r4-480*r3-480*r2-80*r)*val*x3*K+(-4*r4-24*r3-24*r2-4*r)*val*x5)*T3+
((4320*r4+11520*r3+4320*r2)*val*K3+(2160*r4+5760*r3+2160*r2)*val*x2*K2+(180*r4+480*r3+180*r2)*val*x4*K+(6*r4+16*r3+6*r2)*val*x6)
*T2+((-20160*r4-20160*r3)*val*x*K3+(-3360*r4-3360*r3)*val*x3*K2+(-168*r4-168*r3)*val*x5*K+(-4*r4-4*r3)*val*x7)*T+40320*r4*val*K4+
20160*r4*val*x2*K3+1680*r4*val*x4*K2+56*r4*val*x6*K+r4*val*x8)/(T8);

    return result;

};

//This function y_adjust is used to adjust the speed parameter in the function y
double y_adjust(float K, float r, float val, float T, float position) {
    
    double MIN_SPEED, MAX_SPEED;
    if( val > position ) {
	MAX_SPEED = 1.0; //it is inconceivable to start at more than 1m/s
	MIN_SPEED = 0.0;
    }
    else {
	MAX_SPEED = 0.0;
	MIN_SPEED = -1.0;
    }
    
    double CURRENT_SPEED = (MAX_SPEED + MIN_SPEED) / 2.0;
    float result = y(K, r, val, T, position, CURRENT_SPEED, T);
    
    while( abs(result - val) > 0.0001 ) {
	if(result > val) {
	    MAX_SPEED = CURRENT_SPEED;
	    CURRENT_SPEED = (MAX_SPEED + MIN_SPEED) / 2.0;
	    result = y(K, r, val, T, position, CURRENT_SPEED, T);
	} else {
	    MIN_SPEED = CURRENT_SPEED;
	    CURRENT_SPEED = (MAX_SPEED + MIN_SPEED) / 2.0;
	    result = y(K, r, val, T, position, CURRENT_SPEED, T);
	}
    }
    
    return CURRENT_SPEED;   
}

//Once the init speed has been found with y_adjust, we need to reach the intitial position 'position'.
//To do so we use this function which corresponds to the com for a zmp staying at zero:
float yinit( float K, double speedinit, float x) {
   float sqrtK = sqrt(K);
   float expp = exp(x/sqrtK);
   return speedinit*sqrtK/2*(expp - 1/expp);  
}

//but we need to find the good value speedinit which depends on the time we let (tref), and the
//value of 'position'
double findspeedinit( float K, float position, float tref) {
   double sqrtK = sqrt(K);
   double expp = exp(tref/sqrtK);
   return (2*position*expp)/(sqrtK*(expp*expp-1));
}

//Here we suppose that the size of the trajFeatures t, t.size, which depends on the parameters in def, has already been correctly set.
//We also assume that t.incrTime has already been set.
//This function is used to create the zmp and com trajectories along the x or y axis, in the case of an UPWARD half-step.
void construction_zmp_com_UPWARD(bool trueX_falseY, trajFeatures & t, const SE2 & supportconfig, const halfStepDefinition & def) {
    
    float incrTime = t.incrTime; 

    //First, we need to calculate the start value, and the end value.
    //Since it's an upward half-step, the start value is at the barycenter of the feet, and the end_value is at the center of the support foot.
    
    SE2 tmpconfig;
    composeSE2(tmpconfig, supportconfig, def.pos_and_orient);
    
    float start_value, end_value;
    if(trueX_falseY) {
	start_value = (tmpconfig.x + supportconfig.x) / 2.0;
	end_value = supportconfig.x;
    } else {
	start_value = (tmpconfig.y + supportconfig.y) / 2.0;
	end_value = supportconfig.y;
    }
    //the delta between these values is:
    float val = end_value - start_value;
    
    float K = def.constants.standard_height / def.constants.g;
    float r = r_PARAM;
    float T = def.constants.t_total - def.constants.t_start;
    
    //Now we need to calculate the value of 'position', which depends on the 
    //dimensions of the feet.
    float position;
    float partialradius = p_GAP * 0.5 * MIN(def.ft_dim.width,def.ft_dim.length);
    if(val < 0) {
	position = MAX(-partialradius, 0.3*val); //'position' should stay relatively small compare to 'val'
    }
    else position = MIN(partialradius, 0.3*val);
    
    
    //now we compute the intermediate speed, i.e. the speed of the com at tref = def.constants.t_start
    double speed = y_adjust(K, r, val, T, position);
    //we also compute the initial speed: 
    double speedinit = findspeedinit( K, position, def.constants.t_start);
    
    //Now with yinit and y we have our com trajectories.
    //Since it's an upward half-step, the zmp must shift as soon as possible so we use (CASE 1) (see the function y).

    if(trueX_falseY) {
	for(unsigned int i = 0 ; i < t.size; i++) {
	    float time = ((float) i) * incrTime;
	    if(time <= def.constants.t_start) {
		    t.traj[i].zmpX = start_value;
		    t.traj[i].comX = start_value + yinit( K, speedinit, time);	    
	    } else {
		    t.traj[i].zmpX = start_value + val*A(B(r,(time - def.constants.t_start)/T));
		    t.traj[i].comX = start_value + y( K, r, val, T, position, speed, (time - def.constants.t_start));
	    }
	}
    } else {
	for(unsigned int i = 0 ; i < t.size; i++) {
	    float time = ((float) i) * incrTime;
	    if(time <= def.constants.t_start) {
		    t.traj[i].zmpY = start_value;
		    t.traj[i].comY = start_value + yinit( K, speedinit, time);			    
	    } else {
		    t.traj[i].zmpY = start_value + val*A(B(r,(time - def.constants.t_start)/T));
		    t.traj[i].comY = start_value + y( K, r, val, T, position, speed, (time - def.constants.t_start));
	    }
	}
    }
}

//Here we suppose that the size of the trajFeatures t, t.size, which depends on the parameters in def, has already been correctly set.
//We also assume that t.incrTime has already been set.
//This function is used to create the zmp and com trajectories along the x or y axis, in the case of a DOWNWARD half-step.
void construction_zmp_com_DOWNWARD(bool trueX_falseY, trajFeatures & t, const SE2 & supportconfig, const halfStepDefinition & def) {
    
    float incrTime = t.incrTime; 

    //First, we need to calculate the start value, and the end value.
    //Since it's a downward half-step, the start value is at the center of the support foot, and the end_value is at the barycenter of the feet.
    
    SE2 tmpconfig;
    composeSE2(tmpconfig, supportconfig, def.pos_and_orient);
    
    float start_value, end_value;
    if(trueX_falseY) {
	start_value = supportconfig.x;
	end_value = (tmpconfig.x + supportconfig.x) / 2.0;
    } else {
	start_value = supportconfig.y;
	end_value = (tmpconfig.y + supportconfig.y) / 2.0;
    }
    //the delta between these values is:
    float val = end_value - start_value;
    
    float K = def.constants.standard_height / def.constants.g;
    float r = r_PARAM;
    float T = def.constants.t_total - def.constants.t_start;
    
    //Now we need to calculate the value of 'position', which depends on the 
    //dimensions of the feet.
    float position;
    float partialradius = p_GAP * 0.5 * MIN(def.ft_dim.width,def.ft_dim.length);
    if(val < 0) {
	position = MAX(-partialradius, 0.3*val); //'position' should stay relatively small compare to 'val'
    }
    else position = MIN(partialradius, 0.3*val);
    
    
    //now we compute the intermediate speed, i.e. the speed of the com at tref = def.constants.t_start 
    //(notice that here we are doing all the calculations just like for an upward half-step; later we will reverse everything)
    double speed = y_adjust(K, r, val, T, position);
    //we also compute the initial speed: 
    double speedinit = findspeedinit( K, position, def.constants.t_start);
    
    //Now with yinit and y we have our com trajectories.
    //Since it's a downward half-step, the zmp must shift as late as possible so we use (CASE 2) (see the function y).

    if(trueX_falseY) {
	for(unsigned int i = 0 ; i < t.size; i++) {
	    float time = ((float) i) * incrTime;
	    if(time < T) {
		    t.traj[i].zmpX = start_value + val - val*A(B(r,1-time/T));
		    t.traj[i].comX = start_value + val - y( K, r, val, T, position, speed, T-time);
	    } else {    
		    t.traj[i].zmpX = end_value;
		    t.traj[i].comX = end_value - yinit( K, speedinit, def.constants.t_start - (time - T));
	    }
	}
    } else {
	for(unsigned int i = 0 ; i < t.size; i++) {
	    float time = ((float) i) * incrTime;
	    if(time < T) {
		    t.traj[i].zmpY = start_value + val - val*A(B(r,1-time/T));
		    t.traj[i].comY = start_value + val - y( K, r, val, T, position, speed, T-time);
		
	    } else {  
		    t.traj[i].zmpY = end_value;
		    t.traj[i].comY = end_value - yinit( K, speedinit, def.constants.t_start - (time - T));
	    }
	}
    }
}

float U( float t, float x ) {    
    return x*x*( (3/(2*t-1))*x*x + (-(4*t+4)/(2*t-1))*x + (6*t/(2*t-1)) );                
}

//com height (upward half-step):
void construction_comHeight_UPWARD(trajFeatures & t, const SE2 & supportconfig, const halfStepDefinition & def) {

    float incrTime = t.incrTime; 
    float T = def.constants.t_total;       
    for(unsigned int i = 0 ; i < t.size; i++) {
	float time = ((float) i) * incrTime;
	t.traj[i].comHeight = def.constants.standard_height + U(0.30,B(1.015179,time/T)) * def.constants.standard_height * COEF_VARIATION_COM_HEIGHT; //parameters are to be tuned
    }    
}

//com height (downward half-step):
void construction_comHeight_DOWNWARD(trajFeatures & t, const SE2 & supportconfig, const halfStepDefinition & def) {

    float incrTime = t.incrTime; 
    float T = def.constants.t_total;       
    for(unsigned int i = 0 ; i < t.size; i++) {
	float time = ((float) i) * incrTime;
	t.traj[i].comHeight = def.constants.standard_height + U(0.30,B(1.015179,1-time/T)) * def.constants.standard_height * COEF_VARIATION_COM_HEIGHT; //parameters are to be tuned
    }    
    
}

//trajectories of the feet (upward half-step):            -WE ASSUME THAT THE ZMP TRAJECTORY HAS ALREADY BEEN COMPUTED-
void construction_feet_UPWARD(trajFeatures & t, const SE2 & supportconfig, const halfStepDefinition & def) {
    
    float incrTime = t.incrTime; 
    
    //First we need to know the start point for the swing foot (stored in tmpconfig).
    SE2 tmpconfig;
    composeSE2(tmpconfig, supportconfig, def.pos_and_orient);
    
    //Then, for the support foot, the trajectory is stationary:
    if(def.support_foot == LEFT) {
	for(unsigned int i = 0 ; i < t.size; i++) {
	    t.traj[i].leftfootX = supportconfig.x;
	    t.traj[i].leftfootY = supportconfig.y;
	    t.traj[i].leftfootHeight = 0.0;
	    t.traj[i].leftfootOrient = supportconfig.theta * 180.0/PI; //in degrees! 
	}
    } else {
	for(unsigned int i = 0 ; i < t.size; i++) {
	    t.traj[i].rightfootX = supportconfig.x;
	    t.traj[i].rightfootY = supportconfig.y;
	    t.traj[i].rightfootHeight = 0.0;
	    t.traj[i].rightfootOrient = supportconfig.theta * 180.0/PI; //in degrees! 
	}
    }
    
    //Since it's an upward half-step, the swing foot will go from the start point until the via-point configuration.
    //Let us calculate this via-point config.
    SE2 relativeviaconfig;
    SE2 viaconfig;
    
    relativeviaconfig.theta = 0.0;
    relativeviaconfig.x = 0.0;
    if(def.support_foot == LEFT) relativeviaconfig.y = -abs(def.vp_config.hDistance);
    else relativeviaconfig.y = abs(def.vp_config.hDistance);
    
    composeSE2(viaconfig, supportconfig, relativeviaconfig);
    
    //But before continuing with the horizontal trajectories, the swing foot height (from 0 to maxHeight):
    //we use the same curve as the ZMP to raise the foot quickly, but with a different (fixed) r coefficient.  
    //The important thing is: the foot can only be raised when the ZMP is inside the support foot.
    //SO WE ASSUME THAT THE ZMP TRAJECTORY HAS ALREADY BEEN COMPUTED
    float radius = 0.5 * MIN(def.ft_dim.width,def.ft_dim.length);
    float T = def.constants.t_total;
    float val = def.vp_config.maxHeight;
    float r = 1.015179; //standard value
    float raiseTime = 0.0;
    if(def.support_foot == LEFT) {
	for(unsigned int i = 0 ; i < t.size; i++) {
	    if( abs(t.traj[i].zmpX - supportconfig.x) < radius && abs(t.traj[i].zmpY - supportconfig.y) < radius ) {    
		float time = ((float) i) * incrTime;
		t.traj[i].rightfootHeight = val*A(B(r,(time - raiseTime)/(T - raiseTime)));
	    } else {
		raiseTime = ((float) i) * incrTime;
		t.traj[i].rightfootHeight = 0.0;
	    }
	}    
    } else {
	for(unsigned int i = 0 ; i < t.size; i++) {
	    if( abs(t.traj[i].zmpX - supportconfig.x) < radius && abs(t.traj[i].zmpY - supportconfig.y) < radius ) {  
		float time = ((float) i) * incrTime;
		t.traj[i].leftfootHeight = val*A(B(r,(time - raiseTime)/(T - raiseTime)));
	    } else {
		raiseTime = ((float) i) * incrTime;
		t.traj[i].leftfootHeight = 0.0;
	    }
	}
    }
    //We also have to compute the orientation trajectory. 
    //We use a more classical polynomial, of third order, going from initial to final orientation with initial and final speed zero.
    //Of course, the orientation can only start to change when to foot is rising.
    float Tstart = raiseTime; 
    float delta3 = pow(T-Tstart,3);
    float delta2 = pow(T-Tstart,2);   
    val = -def.pos_and_orient.theta; //assuming that def.pos_and_orient.theta is well defined (e.g. not PI/4+2PI instead of PI/4)

    if(def.support_foot == LEFT) {
	for(unsigned int i = 0 ; i < t.size; i++) {    
	    float time = ((float) i) * incrTime;
	    if(time <= Tstart) {
		    t.traj[i].rightfootOrient = tmpconfig.theta * 180.0/PI;    //in degrees!
	    } else {
		    t.traj[i].rightfootOrient = (tmpconfig.theta + (-2/delta3 * pow(time-Tstart,3) + 3/delta2 * pow(time-Tstart,2)) * val) * 180.0/PI;
	    }
	}
    } else {
	for(unsigned int i = 0 ; i < t.size; i++) {    
	    float time = ((float) i) * incrTime;
	    if(time <= Tstart) {
		    t.traj[i].leftfootOrient = tmpconfig.theta * 180.0/PI;    
	    } else {
		    t.traj[i].leftfootOrient = (tmpconfig.theta + (-2/delta3 * pow(time-Tstart,3) + 3/delta2 * pow(time-Tstart,2)) * val) * 180.0/PI;
	    }
	}
    }
       
    //Now we can continue with the horizontal trajectories.
    //Again, we use the third order polynomial going from a point A to a point B with initial and final speed zero
    //the start point is tmpconfig, and the end point is viaconfig.
    
    //We start to move the foot horizontally only after a while after the foot starts to rise.
    Tstart = raiseTime + (T - raiseTime) * (1 - h_PER); //see a definition of h_PER above
    delta3 = pow(T-Tstart,3);
    delta2 = pow(T-Tstart,2);
    
    //Along the x-axis:
    val = viaconfig.x - tmpconfig.x;
    
    if(def.support_foot == LEFT) {
	for(unsigned int i = 0 ; i < t.size; i++) {    
	    float time = ((float) i) * incrTime;
	    if(time <= Tstart) {
		    t.traj[i].rightfootX = tmpconfig.x;    
	    } else {
		    t.traj[i].rightfootX = tmpconfig.x + (-2/delta3 * pow(time-Tstart,3) + 3/delta2 * pow(time-Tstart,2)) * val;
	    }
	}
    } else {
	for(unsigned int i = 0 ; i < t.size; i++) {    
	    float time = ((float) i) * incrTime;
	    if(time <= Tstart) {
		    t.traj[i].leftfootX = tmpconfig.x;    
	    } else {
		    t.traj[i].leftfootX = tmpconfig.x + (-2/delta3 * pow(time-Tstart,3) + 3/delta2 * pow(time-Tstart,2)) * val;
	    }
	}
    }
    
    //Along the y-axis:
    val = viaconfig.y - tmpconfig.y; 
    
    if(def.support_foot == LEFT) {
	for(unsigned int i = 0 ; i < t.size; i++) {    
	    float time = ((float) i) * incrTime;
	    if(time <= Tstart) {
		    t.traj[i].rightfootY = tmpconfig.y;    
	    } else {
		    t.traj[i].rightfootY = tmpconfig.y + (-2/delta3 * pow(time-Tstart,3) + 3/delta2 * pow(time-Tstart,2)) * val;
	    }
	}
    } else {
	for(unsigned int i = 0 ; i < t.size; i++) {    
	    float time = ((float) i) * incrTime;
	    if(time <= Tstart) {
		    t.traj[i].leftfootY = tmpconfig.y;    
	    } else {
		    t.traj[i].leftfootY = tmpconfig.y + (-2/delta3 * pow(time-Tstart,3) + 3/delta2 * pow(time-Tstart,2)) * val;
	    }
	}
    }    
}

//trajectories of the feet (downward half-step):     -WE ASSUME THAT THE ZMP TRAJECTORY HAS ALREADY BEEN COMPUTED-
void construction_feet_DOWNWARD(trajFeatures & t, const SE2 & supportconfig, const halfStepDefinition & def) {
    
    float incrTime = t.incrTime; 
    
    //First we need to know the end point for the swing foot (stored in tmpconfig).
    SE2 tmpconfig;
    composeSE2(tmpconfig, supportconfig, def.pos_and_orient);
    
    //Then, for the support foot, the trajectory is stationary:
    if(def.support_foot == LEFT) {
	for(unsigned int i = 0 ; i < t.size; i++) {
	    t.traj[i].leftfootX = supportconfig.x;
	    t.traj[i].leftfootY = supportconfig.y;
	    t.traj[i].leftfootHeight = 0.0;
	    t.traj[i].leftfootOrient = supportconfig.theta * 180.0/PI; //in degrees! 
	}
    } else {
	for(unsigned int i = 0 ; i < t.size; i++) {
	    t.traj[i].rightfootX = supportconfig.x;
	    t.traj[i].rightfootY = supportconfig.y;
	    t.traj[i].rightfootHeight = 0.0;
	    t.traj[i].rightfootOrient = supportconfig.theta * 180.0/PI; //in degrees! 
	}
    }
    
    //Since it's a downward half-step, the swing foot will go from the via-point configuration until the end point.
    //Let us calculate this via-point config.
    SE2 relativeviaconfig;
    SE2 viaconfig;
    
    relativeviaconfig.theta = 0.0;
    relativeviaconfig.x = 0.0;
    if(def.support_foot == LEFT) relativeviaconfig.y = -abs(def.vp_config.hDistance);
    else relativeviaconfig.y = abs(def.vp_config.hDistance);
    
    composeSE2(viaconfig, supportconfig, relativeviaconfig);
    
    //But before continuing with the horizontal trajectories, the swing foot height (from maxHeight to 0):
    //we use the same curve as the ZMP to put the foot down as late as possible (CASE 2), but with a different (fixed) r coefficient.  
    //The important thing is: the foot must touch the ground BEFORE the ZMP leaves the support foot.
    //SO WE ASSUME THAT THE ZMP TRAJECTORY HAS ALREADY BEEN COMPUTED, 
    //and first, we use a dichotomy to obtain the time when the ZMP leaves the support foot (time_zmp_out).    
    float radius = 0.5 * MIN(def.ft_dim.width,def.ft_dim.length);
    float time_zmp_out;
    float min_time = 0.0;
    float max_time = def.constants.t_total;
    float current_tzo = (min_time + max_time)/2.0;
    while(min_time - max_time > incrTime) {
	int index = (int) (current_tzo / incrTime);
	if( abs(t.traj[index].zmpX - supportconfig.x) < radius && abs(t.traj[index].zmpY - supportconfig.y) < radius ) {
	    min_time = current_tzo;
	    current_tzo = (min_time + max_time)/2.0;
	} else {
	    max_time = current_tzo;
	    current_tzo = (min_time + max_time)/2.0;    
	}
    }
    time_zmp_out = current_tzo;    
    
    //Just to try something:
    //it puts the moment when the foot lands on the ground a bit later.
    //Theoretically, it's not very good since the zmp might already be outside the 
    //polygon of support, but it corresponds to the moment when the humans 
    //start to rotate around the tip of the foot (using the toes) with the support foot.
    //Therefore, this modification can be used to obtain a more "human" walk,
    //and besides, singularities become less likely to occur, so the steps can 
    //be larger.
    time_zmp_out = MIN(current_tzo + 0.5, def.constants.t_total);
          
    float T = time_zmp_out;
    float val = -def.vp_config.maxHeight;
    float r = 1.015179; //standard value
    if(def.support_foot == LEFT) {
	for(unsigned int i = 0 ; i < t.size; i++) {
	    float time = ((float) i) * incrTime;
	    if( time < T ) {    
		t.traj[i].rightfootHeight = -val*A(B(r,1-time/T));
	    } else {
		t.traj[i].rightfootHeight = 0.0;
	    }
	}    
    } else {
	for(unsigned int i = 0 ; i < t.size; i++) {
	    float time = ((float) i) * incrTime;
	    if( time < T ) {
		t.traj[i].leftfootHeight = -val*A(B(r,1-time/T));
	    } else {
		t.traj[i].leftfootHeight = 0.0;
	    }
	}
    }
    //We also have to compute the orientation trajectory. 
    //We use a more classical polynomial, of third order, going from initial to final orientation with initial and final speed zero.
    //Of course, the orientation can only change when to foot is in the air.
    float delta3 = pow(T,3);
    float delta2 = pow(T,2);
    val = def.pos_and_orient.theta; //assuming that def.pos_and_orient.theta is well defined (e.g. not PI/4+2PI instead of PI/4)

    if(def.support_foot == LEFT) {
	for(unsigned int i = 0 ; i < t.size; i++) {    
	    float time = ((float) i) * incrTime;
	    if(time < T) {
		    t.traj[i].rightfootOrient = (viaconfig.theta + (-2/delta3 * pow(time,3) + 3/delta2 * pow(time,2)) * val) * 180.0/PI;     
	    } else {
		    t.traj[i].rightfootOrient = tmpconfig.theta * 180.0/PI;   
	    }
	}
    } else {
	for(unsigned int i = 0 ; i < t.size; i++) {    
	    float time = ((float) i) * incrTime;
	    if(time < T) {
		    t.traj[i].leftfootOrient = (viaconfig.theta + (-2/delta3 * pow(time,3) + 3/delta2 * pow(time,2)) * val) * 180.0/PI;
	    } else {
		    t.traj[i].leftfootOrient = tmpconfig.theta * 180.0/PI;
	    }
	}
    }
       
    //Now we can continue with the horizontal trajectories.
    //Again, we use the third order polynomial going from a point A to a point B with initial and final speed zero
    //the start point is viaconfig, and the end point is tmpconfig.
    
    //We stop to move the foot horizontally a while before the foot lands.
    float Tstop = T * h_PER; //see a definition of h_PER above
    delta3 = pow(Tstop,3);
    delta2 = pow(Tstop,2);
   
    //Along the x-axis:
    val = tmpconfig.x - viaconfig.x;
    
    if(def.support_foot == LEFT) {
	for(unsigned int i = 0 ; i < t.size; i++) {    
	    float time = ((float) i) * incrTime;
	    if(time < Tstop) {
		    t.traj[i].rightfootX = viaconfig.x + (-2/delta3 * pow(time,3) + 3/delta2 * pow(time,2)) * val;    
	    } else {    
		    t.traj[i].rightfootX = tmpconfig.x;
	    }
	}
    } else {
	for(unsigned int i = 0 ; i < t.size; i++) {    
	    float time = ((float) i) * incrTime;
	    if(time <= Tstop) {
		    t.traj[i].leftfootX = viaconfig.x + (-2/delta3 * pow(time,3) + 3/delta2 * pow(time,2)) * val;    
	    } else {
		    t.traj[i].leftfootX = tmpconfig.x;
	    }
	}
    }
    
    //Along the y-axis:
    val = tmpconfig.y - viaconfig.y; 
    
    if(def.support_foot == LEFT) {
	for(unsigned int i = 0 ; i < t.size; i++) {    
	    float time = ((float) i) * incrTime;
	    if(time < Tstop) {
		    t.traj[i].rightfootY = viaconfig.y + (-2/delta3 * pow(time,3) + 3/delta2 * pow(time,2)) * val;    
	    } else {    
		    t.traj[i].rightfootY = tmpconfig.y;
	    }
	}
    } else {
	for(unsigned int i = 0 ; i < t.size; i++) {    
	    float time = ((float) i) * incrTime;
	    if(time <= Tstop) {
		    t.traj[i].leftfootY = viaconfig.y + (-2/delta3 * pow(time,3) + 3/delta2 * pow(time,2)) * val;    
	    } else {
		    t.traj[i].leftfootY = tmpconfig.y;
	    }
	}
    }    
}

//waist orientation (upward half-step):     -WE ASSUME THAT THE FEET TRAJECTORIES HAVE ALREADY BEEN COMPUTED-
void construction_waistOrient_UPWARD(trajFeatures & t, const SE2 & supportconfig, const halfStepDefinition & def) {
    //we simply keep the waist orientation always equal to the average between the two feet orientations
    for(unsigned int i = 0 ; i < t.size; i++) {
	t.traj[i].waistOrient = (t.traj[i].leftfootOrient + t.traj[i].rightfootOrient)/2.0;
    }
}

//waist orientation (downward half-step):     -WE ASSUME THAT THE FEET TRAJECTORIES HAVE ALREADY BEEN COMPUTED-
void construction_waistOrient_DOWNWARD(trajFeatures & t, const SE2 & supportconfig, const halfStepDefinition & def) {
    //we simply keep the waist orientation always equal to the average between the two feet orientations
    for(unsigned int i = 0 ; i < t.size; i++) {
	t.traj[i].waistOrient = (t.traj[i].leftfootOrient + t.traj[i].rightfootOrient)/2.0;
    }
}

void generate_halfStepFeatures(trajFeatures & t, const SE2 & supportconfig, const halfStepDefinition & def) {
        
//     cout << supportconfig.x << " " << supportconfig.y << " " << supportconfig.theta*180.0/PI << " ||| " << def.pos_and_orient.x << " " << def.pos_and_orient.y << " " << def.pos_and_orient.theta*180.0/PI << endl;
    SE2 tmpc;
    composeSE2(tmpc, supportconfig, def.pos_and_orient);
//     cout << "__________ " << tmpc.x << " " << tmpc.y << " " << tmpc.theta*180.0/PI << endl;   
    
    //t.incrTime is assumed to be already defined
    t.traj.clear();
    t.size = (int) (def.constants.t_total / t.incrTime);
    t.traj.resize(t.size);
    
    if(def.half_step_type == UP) {
	
	construction_zmp_com_UPWARD(true, t, supportconfig, def); //along x
	construction_zmp_com_UPWARD(false, t, supportconfig, def); //along y
	construction_comHeight_UPWARD(t, supportconfig, def);
	construction_feet_UPWARD(t, supportconfig, def);
	construction_waistOrient_UPWARD(t, supportconfig, def);
	
    } else {

	construction_zmp_com_DOWNWARD(true, t, supportconfig, def); //along x
	construction_zmp_com_DOWNWARD(false, t, supportconfig, def); //along y
	construction_comHeight_DOWNWARD(t, supportconfig, def);
	construction_feet_DOWNWARD(t, supportconfig, def);
	construction_waistOrient_DOWNWARD(t, supportconfig, def);
	
    }    
}