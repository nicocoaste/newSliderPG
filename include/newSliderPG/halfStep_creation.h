/*
 *  Copyright AIST-CNRS Joint Robotics Laboratory
 *  Author: Nicolas Perrin
 *
 */

#ifndef HALFSTEP_CREATION_H
#define HALFSTEP_CREATION_H

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

#ifndef PI
# define PI 3.14159265359
#endif

#ifndef min
# define min(X, Y)  ((X) < (Y) ? (X) : (Y))
#endif

#ifndef max
# define max(X, Y)  ((X) < (Y) ? (Y) : (X))
#endif

using namespace std;

/*!
 * Defines the special Euclidean group SE(2),
 * with two reals and a rotation angle, theta. 
 */
class SE2 {
    public:
	float x;
	float y;
	float theta;   //in radians
    private:
	friend float dist(SE2 &c1, SE2 & c2);	
};

/*!
 * Contains some constants that are necessary for the creation of half-steps:
 * g (9.81..), t1 the time when the ZMP shift starts, t2, the time when the 
 * ZMP shift stops, t3, the total duration of an isolated half-step, and
 * finally standard_height, a reference height for the center of mass.
 */
struct hsConstants {
	float g;
	float t_start;
	float t_total;
	float standard_height;
};

/*!
 * The dimensions of the robot feet: width and length.
 */
struct feetDimensions {
	float width;
	float length;
};

/*!
 *  This structure contains the same data as trajFeatures, but taken at 
 *  a particular instant.
 */
struct instantFeatures {
	float comX;
	float zmpX;
	float comY;
	float zmpY;
	float comHeight;
	float leftfootX;
	float leftfootY;
	float leftfootHeight;
	float leftfootOrient; //in degrees
	float rightfootX;
	float rightfootY;
	float rightfootHeight;
	float rightfootOrient; //in degrees
	float waistOrient; //in degrees
};

/*!
 *  This structure contains all the trajectories of the end-effecotrs of the lower-body, i.e.
 *  the feet and the CoM (plus waist orientation). The C-space trajectory is then created through
 *  inverse kinematics. trajFeatures objects always describe absolute trajectories, not relative
 *  ones.
 */
struct trajFeatures {
	vector<instantFeatures> traj;
	/*!
	 * traj is a discretized trajectory and incrTime is the duration of one discretization step (in s.).
	 */
	float incrTime;
	unsigned int size;
};

/*!
 * The type for left or right foot.
 */
enum LoR { LEFT, RIGHT };

/*!
 * The type used to distinguish upward and downward half-steps. 
 */
enum UoD { UP, DOWN };

/*!
 * Structure used to define the via point configuration. It
 * should be symmetric for both cases (left or right support foot),
 * and thus the support foot is not precised.
 */
struct viaPointConfig {
	/*!
	* The height of the swing foot.
	*/
	float maxHeight;
	/*!
	 * The horizontal distance between the feet (we recall that they are parallel,
	 * and with no sagittal difference).
	 */
	float hDistance;
};

/*!
 * This structure contains the low dimension definition of a half-step.
 * Only three continuous parameters are needed: x, y, and theta (in pos_and_orient),
 * which define the relative position and orientation of the swing
 * foot when on the ground.
 * The via-point configuration should be fixed.
 * The reference is ALWAYS given by the support foot.
 */
struct halfStepDefinition {
    LoR support_foot;
    UoD half_step_type;
    viaPointConfig vp_config;
    SE2 pos_and_orient;
    feetDimensions ft_dim;
    hsConstants constants;
};

/*!
 * A function belonging to no class that takes in input the 
 * position and orientation of the CENTER OF THE support foot in the global
 * frame, and a halfStepDefinition, and generates the trajFeatures
 * corresponding to the half-step defined. 
 */
void generate_halfStepFeatures(trajFeatures & t, const SE2 & supportconfig, const halfStepDefinition & def);

#endif