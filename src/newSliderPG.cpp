//  Copyright AIST-CNRS Joint Robotics Laboratory
//  Author: Nicolas Perrin

#include "newSliderPG/newSliderPG.h"

#ifndef PI
# define PI 3.14159265359
#endif

#ifndef MIN
# define MIN(X, Y)  ((X) < (Y) ? (X) : (Y))
#endif

#ifndef MAX
# define MAX(X, Y)  ((X) < (Y) ? (Y) : (X))
#endif

using namespace std;

CnewSliderPG::CnewSliderPG()
{

}

CnewSliderPG::~CnewSliderPG()
{

}

void CnewSliderPG::produceTraj(trajFeatures & t, const vector<float> & vect_input) {

	slidingClass slider;
    
	SE2 config_curr;
	SE2 config_prev;
	config_curr.x = vect_input[0];
	config_curr.y = vect_input[1];
	config_curr.theta = 0.0;
	
	halfStepDefinition hsdef;
	
	hsdef.support_foot = first_support_foot;
	hsdef.half_step_type = UP;
	hsdef.vp_config = vp_config;
	hsdef.pos_and_orient.x = 2.0 * vect_input[3];
	hsdef.pos_and_orient.y = 2.0 * vect_input[4];
	hsdef.pos_and_orient.theta = vect_input[5] * PI / 180.0;
	hsdef.ft_dim = ft_dim;
	hsdef.constants = constants;
	
	t.traj.clear();
// 	t.contact_indexes.clear();
	t.size = 0;
	
	trajFeatures t_tmp;
	t_tmp.incrTime = t.incrTime;
	t_tmp.size = 0;
// 	t_tmp.contact_indexes.clear();
	
	for(int i=2; i < (int) (vect_input.size()-6)/5+2 ; i++)
	{       	  
		float coef_slide1 = vect_input[5*i-4];
		float coef_slide2 = vect_input[5*i-3];
	    
// 		cout << config_curr.x << " " << config_curr.y << " " << hsdef.pos_and_orient.x << " " << hsdef.pos_and_orient.y << endl;
		generate_halfStepFeatures(t_tmp, config_curr, hsdef);		
		slider.slideDownUpCOEF(t, coef_slide1, t_tmp);		
		
		hsdef.pos_and_orient.x = vect_input[5*i-2];
		hsdef.pos_and_orient.y = vect_input[5*i-1];
		hsdef.pos_and_orient.theta = vect_input[5*i]*PI/180.0;
		hsdef.half_step_type = DOWN;
		
// 		cout << config_curr.x << " " << config_curr.y << " " << hsdef.pos_and_orient.x << " " << hsdef.pos_and_orient.y << endl;
		generate_halfStepFeatures(t_tmp, config_curr, hsdef);
		slider.slideUpDownCOEF(t, coef_slide2, t_tmp);
		
		config_prev.x = config_curr.x; config_prev.y = config_curr.y; config_prev.theta = config_curr.theta;
		config_curr.x += cos(config_prev.theta)*vect_input[5*i-2]-sin(config_prev.theta)*vect_input[5*i-1];
		config_curr.y += sin(config_prev.theta)*vect_input[5*i-2]+cos(config_prev.theta)*vect_input[5*i-1];
		config_curr.theta += vect_input[5*i]*PI/180.0;
		
		hsdef.pos_and_orient.x = (config_prev.x - config_curr.x) * cos(-config_curr.theta) - (config_prev.y - config_curr.y) * sin(-config_curr.theta);
		hsdef.pos_and_orient.y = (config_prev.x - config_curr.x) * sin(-config_curr.theta) + (config_prev.y - config_curr.y) * cos(-config_curr.theta);
		hsdef.pos_and_orient.theta =  -vect_input[5*i]*PI/180.0;
		hsdef.half_step_type = UP;
		
		if(hsdef.support_foot == LEFT) hsdef.support_foot = RIGHT; else hsdef.support_foot = LEFT;
	}
	
}

void CnewSliderPG::drawTraj(ofstream & fb, const trajFeatures & t, const vector<float> & vect_input)
{

	float downBound, upBound, leftBound, rightBound;
	float centre_x, centre_y, abs_orientation;
	float coefFeet = 1.0;

	for(int i=0; i < (int) (vect_input.size()-6)/5+2 ; i++)
	{

		float abs_orientationRAD;

		if(i==0) {
			centre_x = vect_input[0];
			centre_y = vect_input[1];
			abs_orientation = vect_input[2] * PI/180;

			leftBound = -centre_y;
			rightBound = -centre_y;
			upBound = centre_x;
			downBound = centre_x;
		}
		else if(i==1) {
			centre_x = vect_input[3];
			centre_y = vect_input[4];
			abs_orientationRAD = vect_input[5] * PI/180;
		}
		else if(i==2) {
			centre_x = vect_input[0];
			centre_y = vect_input[1];
			abs_orientation = vect_input[2];
		} 
	
		if(i>=2){

		abs_orientationRAD = abs_orientation * PI/180;

		centre_x = centre_x + cos(abs_orientationRAD)*vect_input[5*i-2]-sin(abs_orientationRAD)*vect_input[5*i-1];
		centre_y = centre_y + sin(abs_orientationRAD)*vect_input[5*i-2]+cos(abs_orientationRAD)*vect_input[5*i-1];
		abs_orientation = abs_orientation + vect_input[5*i];	

		abs_orientationRAD = abs_orientation * PI/180;

		}

		leftBound = MIN(leftBound,-centre_y-0.24*coefFeet);
		rightBound = MAX(rightBound,-centre_y+0.24*coefFeet);
		downBound = MIN(downBound,centre_x-0.24*coefFeet);
		upBound = MAX(upBound,centre_x+0.24*coefFeet);

		vector<float> cosinuss (4, 0);
		vector<float> sinuss (4, 0);
		vector<float> x (4, 0);
		vector<float> y (4, 0);

		cosinuss[0] = cos(abs_orientationRAD)*0.115 - sin(abs_orientationRAD)*0.065;
		sinuss[0] = cos(abs_orientationRAD)*0.065 + sin(abs_orientationRAD)*0.115;

		cosinuss[1] = cos(abs_orientationRAD)*(-0.115) - sin(abs_orientationRAD)*0.065;
		sinuss[1] = cos(abs_orientationRAD)*0.065 + sin(abs_orientationRAD)*(-0.115);

		cosinuss[2] = cos(abs_orientationRAD)*(-0.115) - sin(abs_orientationRAD)*(-0.065);
		sinuss[2] = cos(abs_orientationRAD)*(-0.065) + sin(abs_orientationRAD)*(-0.115);

		cosinuss[3] = cos(abs_orientationRAD)*0.115 - sin(abs_orientationRAD)*(-0.065);
		sinuss[3] = cos(abs_orientationRAD)*(-0.065) + sin(abs_orientationRAD)*0.115;

		for(int j = 0; j<4; j++)
		{
			x[j] = centre_x + cosinuss[j]*coefFeet;
			y[j] = centre_y + sinuss[j]*coefFeet;
		}

		for(int j = 0; j<4; j++)
		{
			fb << -y[j]
				<< " " << x[j]
				<< " " << -y[(j+1) % 4] + y[j]
				<< " " << x[(j+1) % 4] - x[j]
				<< endl;
		}
		fb << -y[0]
			<< " " << +x[0]
			<< " " << -y[0]
			<< " " << +x[0]
			<< endl << endl;

	}

	float squareCenter_h = (rightBound + leftBound)/2;
	float squareCenter_v = (upBound + downBound)/2;
	float halfSide = MAX((rightBound - leftBound)/2,(upBound - downBound)/2);

	leftBound = squareCenter_h - halfSide;
	rightBound = squareCenter_h + halfSide;
	downBound = squareCenter_v - halfSide;
	upBound = squareCenter_v + halfSide;

	//we plot a big rectangle which contains the whole track: 
	fb << leftBound
		<< " " << downBound
		<< " " << leftBound
		<< " " << upBound
		<< endl;
	fb << leftBound
		<< " " << upBound
		<< " " << rightBound
		<< " " << upBound
		<< endl;
	fb << rightBound
		<< " " << upBound
		<< " " << rightBound
		<< " " << downBound
		<< endl;
	fb << rightBound
		<< " " << downBound
		<< " " << leftBound
		<< " " << downBound
		<< endl;
	fb << leftBound
		<< " " << downBound
		<< " " << leftBound
		<< " " << downBound
		<< endl << endl;

	fb << endl;
	//After 3 endl, new index in gnuplot.	

	for(unsigned int i=0; i < t.size-1 ; i++)
	{
		fb << -t.traj[i].rightfootY
			<< " " << t.traj[i+1].rightfootX
			<< " " << -t.traj[i+1].rightfootY
			<< " " << t.traj[i+1].rightfootX
			<< endl;
		fb << -t.traj[i+1].rightfootY
			<< " " << t.traj[i+1].rightfootX
			<< " " << -t.traj[i+1].rightfootY
			<< " " << t.traj[i+1].rightfootX
			<< endl << endl;
	}
	fb << endl;
	//After 3 endl, new index in gnuplot.	

	for(unsigned int i=0; i < t.size-1 ; i++)
	{
		fb << -t.traj[i].leftfootY
			<< " " << t.traj[i].leftfootX
			<< " " << -t.traj[i+1].leftfootY
			<< " " << t.traj[i+1].leftfootX
			<< endl;
		fb << -t.traj[i+1].leftfootY
			<< " " << t.traj[i+1].leftfootX
			<< " " << -t.traj[i+1].leftfootY
			<< " " << t.traj[i+1].leftfootX
			<< endl << endl;
	}
	fb << endl;
	//After 3 endl, new index in gnuplot.	

	for(unsigned int i=0; i < t.size-1 ; i++)
	{
		fb << -t.traj[i].comY 
			<< " " << t.traj[i].comX
			<< " " << -t.traj[i+1].comY
			<< " " << t.traj[i+1].comX
			<< endl;
		fb << -t.traj[i+1].comY 
			<< " " << t.traj[i+1].comX
			<< " " << -t.traj[i+1].comY
			<< " " << t.traj[i+1].comX
			<< endl << endl;
	}
	fb << endl;
	//After 3 endl, new index in gnuplot.	

	for(unsigned int i=0; i < t.size-1 ; i++)
	{
		fb << -t.traj[i].zmpY
			<< " " << t.traj[i].zmpX
			<< " " << -t.traj[i+1].zmpY
			<< " " << t.traj[i+1].zmpX
			<< endl;
		fb << -t.traj[i+1].zmpY
			<< " " << t.traj[i+1].zmpX
			<< " " << -t.traj[i+1].zmpY
			<< " " << t.traj[i+1].zmpX
			<< endl << endl;
	}
	fb << endl;
}

void  CnewSliderPG::plot_com_zmp_along_x(ofstream & fb, const trajFeatures & t) {
	
	for(unsigned int i=0; i < t.size-1 ; i++)
	{
		fb << (float) i*incrTime << "  " << t.traj[i].comX << endl; 
	}
	fb << endl << endl;
	//After 3 endl, new index in gnuplot.	

	for(unsigned int i=0; i < t.size-1 ; i++)
	{
		fb << (float) i*incrTime << "  " << t.traj[i].zmpX << endl; 
	}
	fb << endl << endl;
}

void CnewSliderPG::plot_com_zmp_along_y(ofstream & fb, const trajFeatures & t) {

	for(unsigned int i=0; i < t.size-1 ; i++)
	{
		fb << (float) i*incrTime << "  " << t.traj[i].comY << endl; 
	}
	fb << endl << endl;
	//After 3 endl, new index in gnuplot.	

	for(unsigned int i=0; i < t.size-1 ; i++)
	{
		fb << (float) i*incrTime << "  " << t.traj[i].zmpY << endl; 
	}
	fb << endl << endl;
}

void CnewSliderPG::plot_feet_height(ofstream & fb, const trajFeatures & t){
	
	for(unsigned int i=0; i < t.size-1 ; i++)
	{
		fb << (float) i*incrTime << "  " << t.traj[i].leftfootHeight << endl; 
	}
	fb << endl << endl;	
	//After 3 endl, new index in gnuplot.	
	
	for(unsigned int i=0; i < t.size-1 ; i++)
	{
		fb << (float) i*incrTime << "  " << t.traj[i].rightfootHeight << endl; 
	}
	fb << endl << endl;	
}