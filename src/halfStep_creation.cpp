//  Copyright AIST-CNRS Joint Robotics Laboratory
//  Author: Nicolas Perrin

#define DELAY_1 0.005
#define DELAY_2 0.2

#include "newSliderPG/halfStep_creation.h"

float w (float t, float g, float zc, float delta0, float deltaX, float t1, float t2, float V, float W)
{

	return(delta0+(V*cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0*deltaX*zc/pow(t2-t1,2.0)/g)*cosh(sqrt(g/zc)*(t-t1))+(V*sinh(sqrt(g/zc)*t1)*sqrt(g/zc
		)+W*cosh(sqrt(g/zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)/g)*sinh(
		sqrt(g/zc)*(t-t1))/sqrt(g/zc)-2.0*deltaX*pow(t-t1,3.0)/pow(t2-t1,3.0)+3.0*
		deltaX*pow(t-t1,2.0)/pow(t2-t1,2.0)-12.0*deltaX*zc*(t-t1)/pow(t2-t1,3.0
		)/g+6.0*deltaX*zc/pow(t2-t1,2.0)/g);

};

float w2 (float t, float g, float zc, float deltaX2, float t2, float t3, float t4, float K2, float V2, float W2)
{

	return(K2+(V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2))-6.0*
		deltaX2*zc/pow(t4-t3,2.0)/g)*cosh(sqrt(g/zc)*(t-t3))+(V2*sinh(sqrt(g/zc)*(
		t3-t2))*sqrt(g/zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2*zc/
		pow(t4-t3,3.0)/g)*sinh(sqrt(g/zc)*(t-t3))/sqrt(g/zc)-2.0*deltaX2*pow(t-t3,
		3.0)/pow(t4-t3,3.0)+3.0*deltaX2*pow(t-t3,2.0)/pow(t4-t3,2.0)-12.0*deltaX2
		*zc*(t-t3)/pow(t4-t3,3.0)/g+6.0*deltaX2*zc/pow(t4-t3,2.0)/g);

};

float u (float t, float g, float zc, float t2, float K2, float V2, float W2)
{

	return(V2*cosh(sqrt(g/zc)*(t-t2))+W2*sinh(sqrt(g/zc)*(t-t2))+K2);

};

float u2 (float t, float g, float zc, float t4, float K3, float V3, float W3)
{

	return(V3*cosh(sqrt(g/zc)*(t-t4))+W3*sinh(sqrt(g/zc)*(t-t4))+K3);

};

float hZMP (float t, float g, float zc, float delta0, float deltaX, float deltaX2, float t1, float t2, float t3, float t4, float V, float W, float K2, float V2, float W2, float K3, float V3, float W3)
{

	if(t <= t1)
	{
		return delta0;
	}
	else if(t <= t2)
	{
		return (delta0*t1*t1*t1-delta0*t2*t2*t2+2.0*deltaX*t*t*t+deltaX*t1*t1*t1-3.0*deltaX*t*t*t1-3.0*deltaX*t*t*t2+6.0*deltaX*t*t1*t2
			-3.0*delta0*t1*t1*t2+3.0*delta0*t1*t2*t2-3.0*deltaX*t1*t1*t2)/pow(t1-
			t2,3.0);
	}
	else if(t <= t3)
	{
		return K2;
	}
	else if(t <= t4)
	{
		return (-K2*t4*t4*t4+K2*t3*t3*t3+2.0*deltaX2*t*t*t+deltaX2*t3*t3*t3
			-3.0*deltaX2*t*t*t3-3.0*deltaX2*t*t*t4+6.0*deltaX2*t*t3*t4-3.0*
			deltaX2*t4*t3*t3+3.0*K2*t4*t4*t3-3.0*K2*t4*t3*t3)/pow(-t4+t3,3.0);
	}
	else
	{
		return K3;
	}

};

float h (float t, float g, float zc, float delta0, float deltaX, float deltaX2, float t1, float t2, float t3, float t4, float V, float W, float K2, float V2, float W2, float K3, float V3, float W3)
{

	if(t <= t1)
	{
		return V*cosh(sqrt(g/zc)*t)+W*sinh(sqrt(g/zc)*t)+delta0;
	}
	else if(t <= t2)
	{
		return w(t, g, zc, delta0, deltaX, t1, t2, V, W);
	}
	else if(t <= t3)
	{
		return u(t, g, zc, t2, K2, V2, W2);
	}
	else if(t <= t4)
	{
		return w2(t, g, zc, deltaX2, t2, t3, t4, K2, V2, W2);
	}
	else
	{
		return u2(t, g, zc, t4, K3, V3, W3);
	}

};

vector<float> hVinit (float t, float g, float zc, float delta0, float deltaX, float deltaX2, float t1, float t2, float t3, float t4, float t5, float pinit, float vinit)
{

	vector<float> PairToReturn;
	float V = pinit-delta0;
	float W = vinit/sqrt(g/zc);
	float K2 = delta0+(V*cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0
		*deltaX*zc/pow(t2-t1,2.0)/g)*cosh(sqrt(g/zc)*(t2-t1))+(V*sinh(sqrt(g/zc)*t1
		)*sqrt(g/zc)+W*cosh(sqrt(g/zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)
		/g)*sinh(sqrt(g/zc)*(t2-t1))/sqrt(g/zc)+deltaX-6.0*deltaX*zc/pow(t2-t1,2.0)/g-zc/g*((V*
		cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0*deltaX*zc/pow(t2-t1,2.0)/g)*
		cosh(sqrt(g/zc)*(t2-t1))*g/zc+(V*sinh(sqrt(g/zc)*t1)*sqrt(g/zc)+W*cosh(sqrt(g/
		zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)/g)*sinh(sqrt(g/zc)*(t2-t1)
		)*sqrt(g/zc)-6.0*deltaX/pow(t2-t1,2.0));
	float V2 = zc/g*((V*cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0*deltaX*zc
		/pow(t2-t1,2.0)/g)*cosh(sqrt(g/zc)*(t2-t1))*g/zc+(V*sinh(sqrt(g/zc)*t1)*sqrt(g/
		zc)+W*cosh(sqrt(g/zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)/g)*sinh(
		sqrt(g/zc)*(t2-t1))*sqrt(g/zc)-6.0*deltaX/pow(t2-t1,2.0));
	float W2 = ((V*cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0*deltaX*zc/pow(
		t2-t1,2.0)/g)*sinh(sqrt(g/zc)*(t2-t1))*sqrt(g/zc)+(V*sinh(sqrt(g/zc)*t1)*sqrt(g
		/zc)+W*cosh(sqrt(g/zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)/g)*cosh
		(sqrt(g/zc)*(t2-t1))-12.0*deltaX*zc/pow(t2-t1,3.0)/g)/sqrt(g/zc);
	float K3 = K2+(V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2)
		)-6.0*deltaX2*zc/pow(t4-t3,2.0)/g)*cosh(sqrt(g/zc)*(t4-t3))+(V2*sinh(sqrt(g
		/zc)*(t3-t2))*sqrt(g/zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2
		*zc/pow(t4-t3,3.0)/g)*sinh(sqrt(g/zc)*(t4-t3))/sqrt(g/zc)+deltaX2-6.0*deltaX2*zc/pow(t4-t3,2.0)/g-zc/g*((
		V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2))-6.0*deltaX2*zc/pow(
		t4-t3,2.0)/g)*cosh(sqrt(g/zc)*(t4-t3))*g/zc+(V2*sinh(sqrt(g/zc)*(t3-t2))*sqrt(g
		/zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2*zc/pow(t4-t3,3.0)/
		g)*sinh(sqrt(g/zc)*(t4-t3))*sqrt(g/zc)-6.0*deltaX2/pow(t4-t3,2.0));
	float V3 = zc/g*((V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2))-6.0*
		deltaX2*zc/pow(t4-t3,2.0)/g)*cosh(sqrt(g/zc)*(t4-t3))*g/zc+(V2*sinh(sqrt(g/
		zc)*(t3-t2))*sqrt(g/zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2
		*zc/pow(t4-t3,3.0)/g)*sinh(sqrt(g/zc)*(t4-t3))*sqrt(g/zc)-6.0*deltaX2/pow(
		t4-t3,2.0));
	float W3 = ((V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2))-6.0*deltaX2*
		zc/pow(t4-t3,2.0)/g)*sinh(sqrt(g/zc)*(t4-t3))*sqrt(g/zc)+(V2*sinh(sqrt(g/
		zc)*(t3-t2))*sqrt(g/zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2
		*zc/pow(t4-t3,3.0)/g)*cosh(sqrt(g/zc)*(t4-t3))-12.0*deltaX2*zc/pow(t4-t3,
		3.0)/g)/sqrt(g/zc);
	PairToReturn.push_back(h(t, g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, V, W, K2, V2, W2, K3, V3, W3));
	PairToReturn.push_back(hZMP(t, g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, V, W, K2, V2, W2, K3, V3, W3));
	return PairToReturn;
};

float hVinitCOMonly (float t, float g, float zc, float delta0, float deltaX, float deltaX2, float t1, float t2, float t3, float t4, float t5, float pinit, float vinit)
{

	float V = pinit-delta0;
	float W = vinit/sqrt(g/zc);
	float K2 = delta0+(V*cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0
		*deltaX*zc/pow(t2-t1,2.0)/g)*cosh(sqrt(g/zc)*(t2-t1))+(V*sinh(sqrt(g/zc)*t1
		)*sqrt(g/zc)+W*cosh(sqrt(g/zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)
		/g)*sinh(sqrt(g/zc)*(t2-t1))/sqrt(g/zc)+deltaX-6.0*deltaX*zc/pow(t2-t1,2.0)/g-zc/g*((V*
		cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0*deltaX*zc/pow(t2-t1,2.0)/g)*
		cosh(sqrt(g/zc)*(t2-t1))*g/zc+(V*sinh(sqrt(g/zc)*t1)*sqrt(g/zc)+W*cosh(sqrt(g/
		zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)/g)*sinh(sqrt(g/zc)*(t2-t1)
		)*sqrt(g/zc)-6.0*deltaX/pow(t2-t1,2.0));
	float V2 = zc/g*((V*cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0*deltaX*zc
		/pow(t2-t1,2.0)/g)*cosh(sqrt(g/zc)*(t2-t1))*g/zc+(V*sinh(sqrt(g/zc)*t1)*sqrt(g/
		zc)+W*cosh(sqrt(g/zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)/g)*sinh(
		sqrt(g/zc)*(t2-t1))*sqrt(g/zc)-6.0*deltaX/pow(t2-t1,2.0));
	float W2 = ((V*cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0*deltaX*zc/pow(
		t2-t1,2.0)/g)*sinh(sqrt(g/zc)*(t2-t1))*sqrt(g/zc)+(V*sinh(sqrt(g/zc)*t1)*sqrt(g
		/zc)+W*cosh(sqrt(g/zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)/g)*cosh
		(sqrt(g/zc)*(t2-t1))-12.0*deltaX*zc/pow(t2-t1,3.0)/g)/sqrt(g/zc);
	float K3 = K2+(V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2)
		)-6.0*deltaX2*zc/pow(t4-t3,2.0)/g)*cosh(sqrt(g/zc)*(t4-t3))+(V2*sinh(sqrt(g
		/zc)*(t3-t2))*sqrt(g/zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2
		*zc/pow(t4-t3,3.0)/g)*sinh(sqrt(g/zc)*(t4-t3))/sqrt(g/zc)+deltaX2-6.0*deltaX2*zc/pow(t4-t3,2.0)/g-zc/g*((
		V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2))-6.0*deltaX2*zc/pow(
		t4-t3,2.0)/g)*cosh(sqrt(g/zc)*(t4-t3))*g/zc+(V2*sinh(sqrt(g/zc)*(t3-t2))*sqrt(g
		/zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2*zc/pow(t4-t3,3.0)/
		g)*sinh(sqrt(g/zc)*(t4-t3))*sqrt(g/zc)-6.0*deltaX2/pow(t4-t3,2.0));
	float V3 = zc/g*((V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2))-6.0*
		deltaX2*zc/pow(t4-t3,2.0)/g)*cosh(sqrt(g/zc)*(t4-t3))*g/zc+(V2*sinh(sqrt(g/
		zc)*(t3-t2))*sqrt(g/zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2
		*zc/pow(t4-t3,3.0)/g)*sinh(sqrt(g/zc)*(t4-t3))*sqrt(g/zc)-6.0*deltaX2/pow(
		t4-t3,2.0));
	float W3 = ((V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2))-6.0*deltaX2*
		zc/pow(t4-t3,2.0)/g)*sinh(sqrt(g/zc)*(t4-t3))*sqrt(g/zc)+(V2*sinh(sqrt(g/
		zc)*(t3-t2))*sqrt(g/zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2
		*zc/pow(t4-t3,3.0)/g)*cosh(sqrt(g/zc)*(t4-t3))-12.0*deltaX2*zc/pow(t4-t3,
		3.0)/g)/sqrt(g/zc);
	return h(t, g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, V, W, K2, V2, W2, K3, V3, W3);
};

float searchVinit (float g, float zc, float delta0, float deltaX, float deltaX2, float t1, float t2, float t3, float t4, float t5, float pinit)
{

	float vinitBmin = -10.0;

	float vinitBmax = 10.0;

	if (
		hVinitCOMonly(t5, g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, t5, pinit, vinitBmin) >= delta0 + deltaX + deltaX2
		||
		hVinitCOMonly(t5, g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, t5, pinit, vinitBmax) <= delta0 + deltaX + deltaX2
		) return -999;

	while (vinitBmax - vinitBmin > 0.00000001)
	{

		if (hVinitCOMonly(t5, g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, t5, pinit, (vinitBmax + vinitBmin) / 2) > delta0 + deltaX + deltaX2)
		{
			vinitBmax = (vinitBmax + vinitBmin) / 2;
		}
		else
		{
			vinitBmin = (vinitBmax + vinitBmin) / 2;
		}

	}

	return (vinitBmax + vinitBmin) / 2;

}

void genCOMZMPtrajectory(vector<float> & outputCOM, vector<float> & outputZMP, float incrTime, float zc, float g, float delta0, float deltaX, float deltaX2, float t1, float t2, float t3, float t4, float t5)
{    
	float sensitivityLimit = 0.00001; //because of the instability of the formula.

	outputCOM.clear();
	outputZMP.clear();
	
	float vinit = searchVinit(g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, t5, 0);
	
	if(abs(deltaX) < sensitivityLimit && abs(deltaX2) < sensitivityLimit) {	    
	    for(float i = 0.0 ; i < t5 ; i += incrTime)
	    {		
		    outputCOM.push_back(delta0);		    
		    outputZMP.push_back(delta0);
	    }
	}
	else {    

		int count = 0;
		unsigned int countSav = 0;
		//float minVal = 99999999;
		float valPrev = 99999999;
		float valTmp;
		for(float i = 0.0 ; i < t5 ; i += incrTime)
		{
			
		//fb << i << " " << hVinit(i, g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, t5, vinit) << endl;				
		vector<float> ComZmp = hVinit(i, g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, t5, 0, vinit);

		outputCOM.push_back(ComZmp[0]);		
		outputZMP.push_back(ComZmp[1]);

		valTmp = abs(ComZmp[0] - delta0 - deltaX - deltaX2);
		if(valTmp < valPrev) {
			countSav = count;
			}
		valPrev = valTmp;
		count++;
		}
		
		//again, due to the instability of the formula:
		if(countSav != outputCOM.size()-1) {
		for(unsigned int i = countSav ; i < outputCOM.size() ; i++)
		{				
		outputCOM[i] = (outputCOM[countSav]*(outputCOM.size()-1-i) + (delta0 + deltaX + deltaX2)*(i-countSav))/(outputCOM.size()-1-countSav);		
		}	
		}
	}

}

void genFOOTposition(vector<float> & outputX, vector<float> & outputY, float incrTime, float xinit, float yinit, float xend, float yend, float delay, float t1, float t2, float t3, float t4, float t5, char du)
{

	if(du == '2') {

		outputX.clear();
		outputY.clear();
	
		for(float i = 0.0 ; i < t5 ; i += incrTime)
		{
	
			if(i < t2+delay)
			{
	
				outputX.push_back(xinit);
				outputY.push_back(yinit);
	
			}
			else if(i < t3-delay)
			{
	
				outputX.push_back(xinit + (-2/pow(t3-t2-2*delay,3.0)*pow(i-t2-delay,3.0)+3/pow(t3-t2-2*delay,2.0)*pow(i-t2-delay,2.0))*(xend-xinit));
				outputY.push_back(yinit + (-2/pow(t3-t2-2*delay,3.0)*pow(i-t2-delay,3.0)+3/pow(t3-t2-2*delay,2.0)*pow(i-t2-delay,2.0))*(yend-yinit));
	
			}
			else
			{
	
				outputX.push_back(xend);
				outputY.push_back(yend);
	
			}
	
		}

	}
	
	if(du == 'd') {

		outputX.clear();
		outputY.clear();
	
		for(float i = 0.0 ; i < t5 ; i += incrTime)
		{
	
			if(i < t2)
			{
	
				outputX.push_back(xinit);
				outputY.push_back(yinit);
	
			}
			else if(i < t3-delay)
			{
	
				outputX.push_back(xinit + (-2/pow(t3-t2-delay,3.0)*pow(i-t2,3.0)+3/pow(t3-t2-delay,2.0)*pow(i-t2,2.0))*(xend-xinit));
				outputY.push_back(yinit + (-2/pow(t3-t2-delay,3.0)*pow(i-t2,3.0)+3/pow(t3-t2-delay,2.0)*pow(i-t2,2.0))*(yend-yinit));
	
			}
			else
			{
	
				outputX.push_back(xend);
				outputY.push_back(yend);
	
			}
	
		}

	}

	if(du == 'u') {

		outputX.clear();
		outputY.clear();
	
		for(float i = 0.0 ; i < t5 ; i += incrTime)
		{
	
			if(i < t2+delay)
			{
	
				outputX.push_back(xinit);
				outputY.push_back(yinit);
	
			}
			else if(i < t3)
			{
	
				outputX.push_back(xinit + (-2/pow(t3-t2-delay,3.0)*pow(i-t2-delay,3.0)+3/pow(t3-t2-delay,2.0)*pow(i-t2-delay,2.0))*(xend-xinit));
				outputY.push_back(yinit + (-2/pow(t3-t2-delay,3.0)*pow(i-t2-delay,3.0)+3/pow(t3-t2-delay,2.0)*pow(i-t2-delay,2.0))*(yend-yinit));
	
			}
			else
			{
	
				outputX.push_back(xend);
				outputY.push_back(yend);
	
			}
	
		}

	}

}

void genFOOTheight(vector<float> & output, float incrTime, float heightMax, float delay, float t1, float t2, float t3, float t4, float t5)
{

	output.clear();

	for(float i = 0.0 ; i < t5 ; i += incrTime)
	{

		if(i < t2+delay)
		{

			output.push_back(0);

		}
		else if(i < t3-delay)
		{

			output.push_back( 16*heightMax/pow(t3-t2-2*delay,4.0)*pow(i-t2-delay,4.0)  -  32*heightMax/pow(t3-t2-2*delay,3.0)*pow(i-t2-delay,3.0)  +  16*heightMax/pow(t3-t2-2*delay,2.0)*pow(i-t2-delay,2.0));

		}
		else
		{

			output.push_back(0);

		}

	}

}

void genFOOTdownUPheight(vector<float> & output, float incrTime, float heightMax, float delay, float t1, float t2, float t3)
{

	output.clear();

	for(float i = 0.0 ; i < t3 ; i += incrTime)
	{

		if(i < t2+delay)
		{

			output.push_back(0);

		}
		else if(i < t3-delay)
		{

			output.push_back( -2*heightMax/pow(t3-t2-2*delay,3.0)*pow(i-t2-delay,3.0) + 3*heightMax/pow(t3-t2-2*delay,2.0)*pow(i-t2-delay,2.0));

		}
		else
		{

			output.push_back(heightMax);

		}

	}

}

void genFOOTupDOWNheight(vector<float> & output, float incrTime, float heightMax, float delay, float t1, float t2, float t3)
{

	output.clear();

	for(float i = 0.0 ; i < t3 ; i += incrTime)
	{
		if(i < delay) {
			output.push_back(heightMax);
		} else if(i < t1-delay)
		{

			output.push_back( -2*heightMax/pow(delay-t1+delay,3.0)*pow(i-t1+delay,3.0)  +  3*heightMax/pow(delay-t1+delay,2.0)*pow(i-t1+delay,2.0));

		}
		else
		{

			output.push_back(0);

		}

	}

}

void genFOOTorientation(vector<float> & output, float incrTime, float initOrient, float endOrient, float delay, float t1, float t2, float t3, float t4, float t5, char du)
{

	if(du == '2') {

		output.clear();
	
		for(float i = 0.0 ; i < t5 ; i += incrTime)
		{
	
			if(i < t2+delay)
			{
	
				output.push_back(initOrient);
	
			}
			else if(i < t3-delay)
			{
	
				output.push_back( initOrient + (-2/pow(t3-t2-2*delay,3.0)*pow(i-t2-delay,3.0) + 3/pow(t3-t2-2*delay,2.0)*pow(i-t2-delay,2.0))*(endOrient - initOrient) );
	
			}
			else
			{
	
				output.push_back(endOrient);
	
			}
	
		}

	}

	if(du == 'd') {

		output.clear();
	
		for(float i = 0.0 ; i < t5 ; i += incrTime)
		{
	
			if(i < t2)
			{
	
				output.push_back(initOrient);
	
			}
			else if(i < t3-delay)
			{
	
				output.push_back( initOrient + (-2/pow(t3-t2-delay,3.0)*pow(i-t2,3.0) + 3/pow(t3-t2-delay,2.0)*pow(i-t2,2.0))*(endOrient - initOrient) );
	
			}
			else
			{
	
				output.push_back(endOrient);
	
			}
	
		}

	}

	if(du == 'u') {

		output.clear();
	
		for(float i = 0.0 ; i < t5 ; i += incrTime)
		{
	
			if(i < t2+delay)
			{
	
				output.push_back(initOrient);
	
			}
			else if(i < t3)
			{
	
				output.push_back( initOrient + (-2/pow(t3-t2-delay,3.0)*pow(i-t2-delay,3.0) + 3/pow(t3-t2-delay,2.0)*pow(i-t2-delay,2.0))*(endOrient - initOrient) );
	
			}
			else
			{
	
				output.push_back(endOrient);
	
			}
	
		}

	}

}

void genWAISTorientation(vector<float> & output, float incrTime, float initOrient, float endOrient, float delay, float t1, float t2, float t3, float t4, float t5, char du)
{

	if(du == '2') {

		output.clear();
	
		for(float i = 0.0 ; i < t5 ; i += incrTime)
		{
	
			if(i < t2+delay)
			{
	
				output.push_back(initOrient);
	
			}
			else if(i < t3-delay)
			{
	
				output.push_back( initOrient + (-2/pow(t3-t2-2*delay,3.0)*pow(i-t2-delay,3.0) + 3/pow(t3-t2-2*delay,2.0)*pow(i-t2-delay,2.0))*(endOrient - initOrient) );
	
			}
			else
			{
	
				output.push_back(endOrient);
	
			}
	
		}

	}
	if(du == 'd') {

		output.clear();
	
		for(float i = 0.0 ; i < t5 ; i += incrTime)
		{
	
			if(i < t2)
			{
	
				output.push_back(initOrient);
	
			}
			else if(i < t3-delay)
			{
	
				output.push_back( initOrient + (-2/pow(t3-t2-delay,3.0)*pow(i-t2,3.0) + 3/pow(t3-t2-delay,2.0)*pow(i-t2,2.0))*(endOrient - initOrient) );
	
			}
			else
			{
	
				output.push_back(endOrient);
	
			}
	
		}

	}
	if(du == 'u') {

		output.clear();
	
		for(float i = 0.0 ; i < t5 ; i += incrTime)
		{
	
			if(i < t2+delay)
			{
	
				output.push_back(initOrient);
	
			}
			else if(i < t3)
			{
	
				output.push_back( initOrient + (-2/pow(t3-t2-delay,3.0)*pow(i-t2-delay,3.0) + 3/pow(t3-t2-delay,2.0)*pow(i-t2-delay,2.0))*(endOrient - initOrient) );
	
			}
			else
			{
	
				output.push_back(endOrient);
	
			}
	
		}

	}

}

//TODO: to save computation time, even though the different trajectories are generated separetely,
//we should generate them directly in the good vecotr of instantFeatures.
void produceOneUPHalfStepFeatures(trajFeatures & tr, float incrTime, float zc, float g, float t1, float t2, float t3, float lateral_distance, float max_height, vector<float> vectUPHalfStep_input, char leftOrRightFootStable)
{
	vector<float> comTrajX;
	vector<float> zmpTrajX;
	genCOMZMPtrajectory(comTrajX, zmpTrajX, incrTime, zc, g, 0, 0, -0.5*vectUPHalfStep_input[0], t1/2, t1*3/4, t1, t2, t3);
	for(unsigned int i = 0; i < comTrajX.size(); i++) {
	    comTrajX[i] += 0.5*vectUPHalfStep_input[0];
	    zmpTrajX[i] += 0.5*vectUPHalfStep_input[0];
	}
	
	vector<float> comTrajY;
	vector<float> zmpTrajY;
	genCOMZMPtrajectory(comTrajY, zmpTrajY, incrTime, zc, g, 0*0.5*vectUPHalfStep_input[1], 0, -0.5*vectUPHalfStep_input[1], t1/2, t1*3/4, t1, t2, t3);			
	for(unsigned int i = 0; i < comTrajX.size(); i++) {
	    comTrajY[i] += 0.5*vectUPHalfStep_input[1];
	    zmpTrajY[i] += 0.5*vectUPHalfStep_input[1];
	}
	
	vector<float> footXtraj;
	vector<float> footYtraj;
	int leftRightCoef = 0;
	if(leftOrRightFootStable == 'L') leftRightCoef = -1; else leftRightCoef = 1;		
	genFOOTposition(footXtraj, footYtraj, incrTime, vectUPHalfStep_input[0], vectUPHalfStep_input[1], 0, 0+leftRightCoef*lateral_distance, DELAY_2, t1, t2, t3, t3, t3, 'u');
	
	vector<float> footHeight;
	genFOOTdownUPheight(footHeight, incrTime, max_height, DELAY_1, t1, t2, t3);

	vector<float> footOrient;
	genFOOTorientation(footOrient, incrTime, vectUPHalfStep_input[2], 0, DELAY_2, t1, t2, t3, t3, t3, 'u');

	vector<float> waistOrient;
	genWAISTorientation(waistOrient, incrTime, 0, 0, DELAY_1, t1, t2, t3, t3, t3, 'u');	
	
	vector<float> stablefootXtraj(comTrajX.size());
	vector<float> stablefootYtraj(comTrajX.size());
	vector<float> stablefootHeight(comTrajX.size());
	vector<float> stablefootOrient(comTrajX.size());
	
// 	for(float i = 0.0 ; i < t3 ; i += incrTime)
// 	{
// 		stablefootXtraj.push_back(0);
// 		stablefootYtraj.push_back(0);
// 		stablefootHeight.push_back(0);
// 		stablefootOrient.push_back(0);
// 	}	
	
	tr.traj.resize(waistOrient.size());
	
	for(unsigned int i = 0; i < tr.traj.size(); i++) {
	    
	    tr.traj[i].comX = comTrajX[i];	    
	    
	    tr.traj[i].zmpX = zmpTrajX[i];
	    tr.traj[i].comY = comTrajY[i];
	    tr.traj[i].zmpY = zmpTrajY[i];
	    
	    if(leftOrRightFootStable == 'L') {
	    tr.traj[i].leftfootX = stablefootXtraj[i];
	    tr.traj[i].leftfootY = stablefootYtraj[i];
	    tr.traj[i].leftfootHeight = stablefootHeight[i];
	    tr.traj[i].leftfootOrient = stablefootOrient[i];
	    tr.traj[i].rightfootX = footXtraj[i];
	    tr.traj[i].rightfootY = footYtraj[i];
	    tr.traj[i].rightfootHeight = footHeight[i];
	    tr.traj[i].rightfootOrient = footOrient[i];
	    } else {
	    tr.traj[i].leftfootX = footXtraj[i];
	    tr.traj[i].leftfootY = footYtraj[i];
	    tr.traj[i].leftfootHeight = footHeight[i];
	    tr.traj[i].leftfootOrient = footOrient[i];
	    tr.traj[i].rightfootX = stablefootXtraj[i];
	    tr.traj[i].rightfootY = stablefootYtraj[i];
	    tr.traj[i].rightfootHeight = stablefootHeight[i];
	    tr.traj[i].rightfootOrient = stablefootOrient[i];
	    }
	    tr.traj[i].waistOrient =  waistOrient[i];
	    tr.traj[i].comHeight = zc;	    
	}
	
	tr.incrTime = incrTime;
	tr.size = waistOrient.size();
}

void produceOneDOWNHalfStepFeatures(trajFeatures & tr, float incrTime, float zc, float g, float t1, float t2, float t3, float lateral_distance, float max_height, vector<float> vectDOWNHalfStep_input, char leftOrRightFootStable)
{

	vector<float> comTrajX;
	vector<float> zmpTrajX;
	genCOMZMPtrajectory(comTrajX, zmpTrajX, incrTime, zc, g, 0, 0, vectDOWNHalfStep_input[0]/2, t1/2, t1*3/4, t1, t2, t3);

	vector<float> comTrajY;
	vector<float> zmpTrajY;
	genCOMZMPtrajectory(comTrajY, zmpTrajY, incrTime, zc, g, 0, 0, vectDOWNHalfStep_input[1]/2, t1/2, t1*3/4, t1, t2, t3);

	vector<float> footXtraj;
	vector<float> footYtraj;
	int leftRightCoef = 0;	
	if(leftOrRightFootStable == 'L') leftRightCoef = -1; else leftRightCoef = 1;
	genFOOTposition(footXtraj, footYtraj, incrTime, 0, leftRightCoef*lateral_distance, vectDOWNHalfStep_input[0], vectDOWNHalfStep_input[1], DELAY_2, 0, 0, t1, t2, t3, 'd');

	vector<float> footHeight;
	genFOOTupDOWNheight(footHeight, incrTime, max_height, DELAY_1, t1, t2, t3);

	vector<float> footOrient;
	genFOOTorientation(footOrient, incrTime, 0, vectDOWNHalfStep_input[2], DELAY_2, 0, 0, t1, t2, t3, 'd');

	vector<float> waistOrient;
	genWAISTorientation(waistOrient, incrTime, 0, vectDOWNHalfStep_input[2], DELAY_1, 0, 0, t1, t2, t3, 'd');

	vector<float> stablefootXtraj(comTrajX.size());
	vector<float> stablefootYtraj(comTrajX.size());
	vector<float> stablefootHeight(comTrajX.size());
	vector<float> stablefootOrient(comTrajX.size());

// 	for(float i = 0.0 ; i < t3 ; i += incrTime)
// 	{
// 		stablefootXtraj.push_back(0);
// 		stablefootYtraj.push_back(0);
// 		stablefootHeight.push_back(0);
// 		stablefootOrient.push_back(0);
// 	}
	
	tr.traj.resize(waistOrient.size());
	
	for(unsigned int i = 0; i<tr.traj.size(); i++) {
	    tr.traj[i].comX = comTrajX[i];
	    tr.traj[i].zmpX = zmpTrajX[i];
	    tr.traj[i].comY = comTrajY[i];
	    tr.traj[i].zmpY = zmpTrajY[i];

	    if(leftOrRightFootStable == 'L') {
	    tr.traj[i].leftfootX = stablefootXtraj[i];
	    tr.traj[i].leftfootY = stablefootYtraj[i];
	    tr.traj[i].leftfootHeight = stablefootHeight[i];
	    tr.traj[i].leftfootOrient = stablefootOrient[i];
	    tr.traj[i].rightfootX = footXtraj[i];
	    tr.traj[i].rightfootY = footYtraj[i];
	    tr.traj[i].rightfootHeight = footHeight[i];
	    tr.traj[i].rightfootOrient = footOrient[i];
	    } else {
	    tr.traj[i].leftfootX = footXtraj[i];
	    tr.traj[i].leftfootY = footYtraj[i];
	    tr.traj[i].leftfootHeight = footHeight[i];
	    tr.traj[i].leftfootOrient = footOrient[i];
	    tr.traj[i].rightfootX = stablefootXtraj[i];
	    tr.traj[i].rightfootY = stablefootYtraj[i];
	    tr.traj[i].rightfootHeight = stablefootHeight[i];
	    tr.traj[i].rightfootOrient = stablefootOrient[i];
	    }
	    tr.traj[i].waistOrient =  waistOrient[i];
	    tr.traj[i].comHeight = zc;
	}
	
	tr.incrTime = incrTime;
	tr.size = waistOrient.size();
}

void generate_halfStepFeatures(trajFeatures & t, const SE2 & supportconfig, const halfStepDefinition & def) {
    
    char leftOrRightFootStable;
    
    if(def.support_foot == LEFT) leftOrRightFootStable = 'L'; else leftOrRightFootStable = 'R';
     
    float incrTime = 0.005;
    float zc = def.constants.standard_height;
    float g = def.constants.g;
    float t1 = def.constants.t1;
    float t2 = def.constants.t2;
    float t3 = def.constants.t3;

    float lateral_distance = def.vp_config.hDistance;
    float max_height = def.vp_config.maxHeight;
    
    vector<float> input(3);
    input[0] = def.pos_and_orient.x;
    input[1] = def.pos_and_orient.y;
    input[2] = def.pos_and_orient.theta;
    
    if(def.half_step_type == UP) {
	    produceOneUPHalfStepFeatures(t, incrTime, zc, g, t1, t2, t3, lateral_distance, max_height, input, leftOrRightFootStable);
    } else {
	    produceOneDOWNHalfStepFeatures(t, incrTime, zc, g, t1, t2, t3, lateral_distance, max_height, input, leftOrRightFootStable);
    } 
    
    float supportX = supportconfig.x;
    float supportY = supportconfig.y;
    float supportOrient = supportconfig.theta*PI/180;

    for(unsigned int count = 0 ; count < t.size ; count++) {

		float newcomX = (t.traj[count].comX)*cos(supportOrient)
				-(t.traj[count].comY)*sin(supportOrient)+supportX;
		float newcomY = (t.traj[count].comX)*sin(supportOrient)
				+(t.traj[count].comY)*cos(supportOrient)+supportY;
		float newzmpX = (t.traj[count].zmpX)*cos(supportOrient)
				-(t.traj[count].zmpY)*sin(supportOrient)+supportX;
		float newzmpY = (t.traj[count].zmpX)*sin(supportOrient)
				+(t.traj[count].zmpY)*cos(supportOrient)+supportY;
		float newlfX = (t.traj[count].leftfootX)*cos(supportOrient)
				-(t.traj[count].leftfootY)*sin(supportOrient)+supportX;	
		float newlfY = (t.traj[count].leftfootX)*sin(supportOrient)
				+(t.traj[count].leftfootY)*cos(supportOrient)+supportY;	
		float newrfX = (t.traj[count].rightfootX)*cos(supportOrient)
				-(t.traj[count].rightfootY)*sin(supportOrient)+supportX;
		float newrfY = (t.traj[count].rightfootX)*sin(supportOrient)   
				+(t.traj[count].rightfootY)*cos(supportOrient)+supportY;    

		t.traj[count].comX = newcomX;
		t.traj[count].zmpX = newzmpX;

		t.traj[count].comY = newcomY;
		t.traj[count].zmpY = newzmpY;

		t.traj[count].leftfootX = newlfX;
		t.traj[count].leftfootY = newlfY;

		t.traj[count].leftfootOrient += supportconfig.theta;

		t.traj[count].rightfootX = newrfX;
		t.traj[count].rightfootY = newrfY;

		t.traj[count].rightfootOrient += supportconfig.theta;

		t.traj[count].waistOrient += supportconfig.theta;

    }
}