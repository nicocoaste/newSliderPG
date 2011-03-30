//  Copyright AIST-CNRS Joint Robotics Laboratory
//  Author: Nicolas Perrin

#include "newSliderPG/halfStep_creation.h"

double w (double t, double g, double zc, double delta0, double deltaX, double t1, double t2, double V, double W)
{

	return(delta0+(V*cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0*deltaX*zc/pow(t2-t1,2.0)/g)*cosh(sqrt(g/zc)*(t-t1))+(V*sinh(sqrt(g/zc)*t1)*sqrt(g/zc
		)+W*cosh(sqrt(g/zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)/g)*sinh(
		sqrt(g/zc)*(t-t1))/sqrt(g/zc)-2.0*deltaX*pow(t-t1,3.0)/pow(t2-t1,3.0)+3.0*
		deltaX*pow(t-t1,2.0)/pow(t2-t1,2.0)-12.0*deltaX*zc*(t-t1)/pow(t2-t1,3.0
		)/g+6.0*deltaX*zc/pow(t2-t1,2.0)/g);

};

double w2 (double t, double g, double zc, double deltaX2, double t2, double t3, double t4, double K2, double V2, double W2)
{

	return(K2+(V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2))-6.0*
		deltaX2*zc/pow(t4-t3,2.0)/g)*cosh(sqrt(g/zc)*(t-t3))+(V2*sinh(sqrt(g/zc)*(
		t3-t2))*sqrt(g/zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2*zc/
		pow(t4-t3,3.0)/g)*sinh(sqrt(g/zc)*(t-t3))/sqrt(g/zc)-2.0*deltaX2*pow(t-t3,
		3.0)/pow(t4-t3,3.0)+3.0*deltaX2*pow(t-t3,2.0)/pow(t4-t3,2.0)-12.0*deltaX2
		*zc*(t-t3)/pow(t4-t3,3.0)/g+6.0*deltaX2*zc/pow(t4-t3,2.0)/g);

};

double u (double t, double g, double zc, double t2, double K2, double V2, double W2)
{

	return(V2*cosh(sqrt(g/zc)*(t-t2))+W2*sinh(sqrt(g/zc)*(t-t2))+K2);

};

double u2 (double t, double g, double zc, double t4, double K3, double V3, double W3)
{

	return(V3*cosh(sqrt(g/zc)*(t-t4))+W3*sinh(sqrt(g/zc)*(t-t4))+K3);

};

double hZMP (double t, double g, double zc, double delta0, double deltaX, double deltaX2, double t1, double t2, double t3, double t4, double V, double W, double K2, double V2, double W2, double K3, double V3, double W3)
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

double h (double t, double g, double zc, double delta0, double deltaX, double deltaX2, double t1, double t2, double t3, double t4, double V, double W, double K2, double V2, double W2, double K3, double V3, double W3)
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

vector<double> hVinit (double t, double g, double zc, double delta0, double deltaX, double deltaX2, double t1, double t2, double t3, double t4, double t5, double pinit, double vinit)
{

	vector<double> PairToReturn;
	double V = pinit-delta0;
	double W = vinit/sqrt(g/zc);
	double K2 = delta0+(V*cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0
		*deltaX*zc/pow(t2-t1,2.0)/g)*cosh(sqrt(g/zc)*(t2-t1))+(V*sinh(sqrt(g/zc)*t1
		)*sqrt(g/zc)+W*cosh(sqrt(g/zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)
		/g)*sinh(sqrt(g/zc)*(t2-t1))/sqrt(g/zc)+deltaX-6.0*deltaX*zc/pow(t2-t1,2.0)/g-zc/g*((V*
		cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0*deltaX*zc/pow(t2-t1,2.0)/g)*
		cosh(sqrt(g/zc)*(t2-t1))*g/zc+(V*sinh(sqrt(g/zc)*t1)*sqrt(g/zc)+W*cosh(sqrt(g/
		zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)/g)*sinh(sqrt(g/zc)*(t2-t1)
		)*sqrt(g/zc)-6.0*deltaX/pow(t2-t1,2.0));
	double V2 = zc/g*((V*cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0*deltaX*zc
		/pow(t2-t1,2.0)/g)*cosh(sqrt(g/zc)*(t2-t1))*g/zc+(V*sinh(sqrt(g/zc)*t1)*sqrt(g/
		zc)+W*cosh(sqrt(g/zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)/g)*sinh(
		sqrt(g/zc)*(t2-t1))*sqrt(g/zc)-6.0*deltaX/pow(t2-t1,2.0));
	double W2 = ((V*cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0*deltaX*zc/pow(
		t2-t1,2.0)/g)*sinh(sqrt(g/zc)*(t2-t1))*sqrt(g/zc)+(V*sinh(sqrt(g/zc)*t1)*sqrt(g
		/zc)+W*cosh(sqrt(g/zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)/g)*cosh
		(sqrt(g/zc)*(t2-t1))-12.0*deltaX*zc/pow(t2-t1,3.0)/g)/sqrt(g/zc);
	double K3 = K2+(V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2)
		)-6.0*deltaX2*zc/pow(t4-t3,2.0)/g)*cosh(sqrt(g/zc)*(t4-t3))+(V2*sinh(sqrt(g
		/zc)*(t3-t2))*sqrt(g/zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2
		*zc/pow(t4-t3,3.0)/g)*sinh(sqrt(g/zc)*(t4-t3))/sqrt(g/zc)+deltaX2-6.0*deltaX2*zc/pow(t4-t3,2.0)/g-zc/g*((
		V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2))-6.0*deltaX2*zc/pow(
		t4-t3,2.0)/g)*cosh(sqrt(g/zc)*(t4-t3))*g/zc+(V2*sinh(sqrt(g/zc)*(t3-t2))*sqrt(g
		/zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2*zc/pow(t4-t3,3.0)/
		g)*sinh(sqrt(g/zc)*(t4-t3))*sqrt(g/zc)-6.0*deltaX2/pow(t4-t3,2.0));
	double V3 = zc/g*((V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2))-6.0*
		deltaX2*zc/pow(t4-t3,2.0)/g)*cosh(sqrt(g/zc)*(t4-t3))*g/zc+(V2*sinh(sqrt(g/
		zc)*(t3-t2))*sqrt(g/zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2
		*zc/pow(t4-t3,3.0)/g)*sinh(sqrt(g/zc)*(t4-t3))*sqrt(g/zc)-6.0*deltaX2/pow(
		t4-t3,2.0));
	double W3 = ((V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2))-6.0*deltaX2*
		zc/pow(t4-t3,2.0)/g)*sinh(sqrt(g/zc)*(t4-t3))*sqrt(g/zc)+(V2*sinh(sqrt(g/
		zc)*(t3-t2))*sqrt(g/zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2
		*zc/pow(t4-t3,3.0)/g)*cosh(sqrt(g/zc)*(t4-t3))-12.0*deltaX2*zc/pow(t4-t3,
		3.0)/g)/sqrt(g/zc);
	PairToReturn.push_back(h(t, g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, V, W, K2, V2, W2, K3, V3, W3));
	PairToReturn.push_back(hZMP(t, g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, V, W, K2, V2, W2, K3, V3, W3));
	return PairToReturn;
};

double hVinitCOMonly (double t, double g, double zc, double delta0, double deltaX, double deltaX2, double t1, double t2, double t3, double t4, double t5, double pinit, double vinit)
{

	double V = pinit-delta0;
	double W = vinit/sqrt(g/zc);
	double K2 = delta0+(V*cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0
		*deltaX*zc/pow(t2-t1,2.0)/g)*cosh(sqrt(g/zc)*(t2-t1))+(V*sinh(sqrt(g/zc)*t1
		)*sqrt(g/zc)+W*cosh(sqrt(g/zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)
		/g)*sinh(sqrt(g/zc)*(t2-t1))/sqrt(g/zc)+deltaX-6.0*deltaX*zc/pow(t2-t1,2.0)/g-zc/g*((V*
		cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0*deltaX*zc/pow(t2-t1,2.0)/g)*
		cosh(sqrt(g/zc)*(t2-t1))*g/zc+(V*sinh(sqrt(g/zc)*t1)*sqrt(g/zc)+W*cosh(sqrt(g/
		zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)/g)*sinh(sqrt(g/zc)*(t2-t1)
		)*sqrt(g/zc)-6.0*deltaX/pow(t2-t1,2.0));
	double V2 = zc/g*((V*cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0*deltaX*zc
		/pow(t2-t1,2.0)/g)*cosh(sqrt(g/zc)*(t2-t1))*g/zc+(V*sinh(sqrt(g/zc)*t1)*sqrt(g/
		zc)+W*cosh(sqrt(g/zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)/g)*sinh(
		sqrt(g/zc)*(t2-t1))*sqrt(g/zc)-6.0*deltaX/pow(t2-t1,2.0));
	double W2 = ((V*cosh(sqrt(g/zc)*t1)+W*sinh(sqrt(g/zc)*t1)-6.0*deltaX*zc/pow(
		t2-t1,2.0)/g)*sinh(sqrt(g/zc)*(t2-t1))*sqrt(g/zc)+(V*sinh(sqrt(g/zc)*t1)*sqrt(g
		/zc)+W*cosh(sqrt(g/zc)*t1)*sqrt(g/zc)+12.0*deltaX*zc/pow(t2-t1,3.0)/g)*cosh
		(sqrt(g/zc)*(t2-t1))-12.0*deltaX*zc/pow(t2-t1,3.0)/g)/sqrt(g/zc);
	double K3 = K2+(V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2)
		)-6.0*deltaX2*zc/pow(t4-t3,2.0)/g)*cosh(sqrt(g/zc)*(t4-t3))+(V2*sinh(sqrt(g
		/zc)*(t3-t2))*sqrt(g/zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2
		*zc/pow(t4-t3,3.0)/g)*sinh(sqrt(g/zc)*(t4-t3))/sqrt(g/zc)+deltaX2-6.0*deltaX2*zc/pow(t4-t3,2.0)/g-zc/g*((
		V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2))-6.0*deltaX2*zc/pow(
		t4-t3,2.0)/g)*cosh(sqrt(g/zc)*(t4-t3))*g/zc+(V2*sinh(sqrt(g/zc)*(t3-t2))*sqrt(g
		/zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2*zc/pow(t4-t3,3.0)/
		g)*sinh(sqrt(g/zc)*(t4-t3))*sqrt(g/zc)-6.0*deltaX2/pow(t4-t3,2.0));
	double V3 = zc/g*((V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2))-6.0*
		deltaX2*zc/pow(t4-t3,2.0)/g)*cosh(sqrt(g/zc)*(t4-t3))*g/zc+(V2*sinh(sqrt(g/
		zc)*(t3-t2))*sqrt(g/zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2
		*zc/pow(t4-t3,3.0)/g)*sinh(sqrt(g/zc)*(t4-t3))*sqrt(g/zc)-6.0*deltaX2/pow(
		t4-t3,2.0));
	double W3 = ((V2*cosh(sqrt(g/zc)*(t3-t2))+W2*sinh(sqrt(g/zc)*(t3-t2))-6.0*deltaX2*
		zc/pow(t4-t3,2.0)/g)*sinh(sqrt(g/zc)*(t4-t3))*sqrt(g/zc)+(V2*sinh(sqrt(g/
		zc)*(t3-t2))*sqrt(g/zc)+W2*cosh(sqrt(g/zc)*(t3-t2))*sqrt(g/zc)+12.0*deltaX2
		*zc/pow(t4-t3,3.0)/g)*cosh(sqrt(g/zc)*(t4-t3))-12.0*deltaX2*zc/pow(t4-t3,
		3.0)/g)/sqrt(g/zc);
	return h(t, g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, V, W, K2, V2, W2, K3, V3, W3);
};

double searchVinit (double g, double zc, double delta0, double deltaX, double deltaX2, double t1, double t2, double t3, double t4, double t5, double pinit)
{

	double vinitBmin = -10.0;

	double vinitBmax = 10.0;

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

void genCOMZMPtrajectory(vector<double> & outputCOM, vector<double> & outputZMP, double incrTime, double zc, double g, double delta0, double deltaX, double deltaX2, double t1, double t2, double t3, double t4, double t5)
{

	double sensitivityLimit = 0.00001; //because of the instability of the formula.

	outputCOM.clear();
	outputZMP.clear();

	double vinit = searchVinit(g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, t5, 0);

	if(abs(deltaX) < sensitivityLimit && abs(deltaX2) < sensitivityLimit) {
	for(double i = 0.0 ; i < t5 ; i += incrTime)
	{		
		outputCOM.push_back(delta0);
		outputZMP.push_back(delta0);
	}
	}
	else {

		int count = 0;
		int countSav = 0;
		//double minVal = 99999999;
		double valPrev = 99999999;
		double valTmp;
		for(double i = 0.0 ; i < t5 ; i += incrTime)
		{
			
		//fb << i << " " << hVinit(i, g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, t5, vinit) << endl;
		vector<double> ComZmp = hVinit(i, g, zc, delta0, deltaX, deltaX2, t1, t2, t3, t4, t5, 0, vinit);

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

void genFOOTposition(vector<double> & outputX, vector<double> & outputY, double incrTime, double xinit, double yinit, double xend, double yend, double delay, double t1, double t2, double t3, double t4, double t5, char du)
{

	if(du == '2') {

		outputX.clear();
		outputY.clear();
	
		for(double i = 0.0 ; i < t5 ; i += incrTime)
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
	
		for(double i = 0.0 ; i < t5 ; i += incrTime)
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
	
		for(double i = 0.0 ; i < t5 ; i += incrTime)
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

void genFOOTheight(vector<double> & output, double incrTime, double heightMax, double delay, double t1, double t2, double t3, double t4, double t5)
{

	output.clear();

	for(double i = 0.0 ; i < t5 ; i += incrTime)
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

void genFOOTdownUPheight(vector<double> & output, double incrTime, double heightMax, double delay, double t1, double t2, double t3)
{

	output.clear();

	for(double i = 0.0 ; i < t3 ; i += incrTime)
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

void genFOOTupDOWNheight(vector<double> & output, double incrTime, double heightMax, double delay, double t1, double t2, double t3)
{

	output.clear();

	for(double i = 0.0 ; i < t3 ; i += incrTime)
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

void genFOOTorientation(vector<double> & output, double incrTime, double initOrient, double endOrient, double delay, double t1, double t2, double t3, double t4, double t5, char du)
{

	if(du == '2') {

		output.clear();
	
		for(double i = 0.0 ; i < t5 ; i += incrTime)
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
	
		for(double i = 0.0 ; i < t5 ; i += incrTime)
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
	
		for(double i = 0.0 ; i < t5 ; i += incrTime)
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

void genWAISTorientation(vector<double> & output, double incrTime, double initOrient, double endOrient, double delay, double t1, double t2, double t3, double t4, double t5, char du)
{

	if(du == '2') {

		output.clear();
	
		for(double i = 0.0 ; i < t5 ; i += incrTime)
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
	
		for(double i = 0.0 ; i < t5 ; i += incrTime)
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
	
		for(double i = 0.0 ; i < t5 ; i += incrTime)
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
void produceOneUPHalfStepFeatures(const StepFeatures & stepF, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vectUPHalfStep_input, char leftOrRightFootStable)
{

	vector<double> comTrajX;
	vector<double> zmpTrajX;
	genCOMZMPtrajectory(comTrajX, zmpTrajX, incrTime, zc, g, 0, 0, vectUPHalfStep_input[0], t1/2, t1*3/4, t1, t2, t3);

	vector<double> comTrajY;
	vector<double> zmpTrajY;
	genCOMZMPtrajectory(comTrajY, zmpTrajY, incrTime, zc, g, 0, 0, vectUPHalfStep_input[1], t1/2, t1*3/4, t1, t2, t3);

	vector<double> footXtraj;
	vector<double> footYtraj;
	int leftRightCoef = 0;	
	if(leftOrRightFootStable == 'L') leftRightCoef = -1; else leftRightCoef = 1;
	genFOOTposition(footXtraj, footYtraj, incrTime, vectUPHalfStep_input[3], vectUPHalfStep_input[4], vectUPHalfStep_input[0], vectUPHalfStep_input[1]+leftRightCoef*vectUPHalfStep_input[6], DELAY_2, t1, t2, t3, t3, t3, 'u');

	vector<double> footHeight;
	genFOOTdownUPheight(footHeight, incrTime, vectUPHalfStep_input[7], DELAY_1, t1, t2, t3);

	vector<double> footOrient;
	genFOOTorientation(footOrient, incrTime, vectUPHalfStep_input[5], 0, DELAY_2, t1, t2, t3, t3, t3, 'u');

	vector<double> stablefootXtraj;
	vector<double> stablefootYtraj;
	vector<double> stablefootHeight;
	vector<double> stablefootOrient;

	int count = -1;

	for(double i = 0.0 ; i < t3 ; i += incrTime)
	{
		count++;
		stablefootXtraj.push_back(vectUPHalfStep_input[0]);
		stablefootYtraj.push_back(vectUPHalfStep_input[1]);
		stablefootHeight.push_back(0);
		stablefootOrient.push_back(0);
	}

	vector<double> waistOrient;
	genWAISTorientation(waistOrient, incrTime, 0, 0, DELAY_1, t1, t2, t3, t3, t3, 'u');


	stepF.comTrajX = comTrajX;
	stepF.zmpTrajX = zmpTrajX;
	stepF.comTrajY = comTrajY;
	stepF.zmpTrajY = zmpTrajY;

	if(leftOrRightFootStable == 'L') {
	stepF.leftfootXtraj = stablefootXtraj;
	stepF.leftfootYtraj = stablefootYtraj;
	stepF.leftfootHeight = stablefootHeight;
	stepF.leftfootOrient = stablefootOrient;
	stepF.rightfootXtraj = footXtraj;
	stepF.rightfootYtraj = footYtraj;
	stepF.rightfootHeight = footHeight;
	stepF.rightfootOrient = footOrient;
	} else {
	stepF.leftfootXtraj = footXtraj;
	stepF.leftfootYtraj = footYtraj;
	stepF.leftfootHeight = footHeight;
	stepF.leftfootOrient = footOrient;
	stepF.rightfootXtraj = stablefootXtraj;
	stepF.rightfootYtraj = stablefootYtraj;
	stepF.rightfootHeight = stablefootHeight;
	stepF.rightfootOrient = stablefootOrient;
	}

	stepF.waistOrient =  waistOrient;	
	stepF.incrTime = incrTime;
	stepF.zc = zc;
	stepF.size = waistOrient.size();

}

void produceOneDOWNHalfStepFeatures(const StepFeatures & stepF, double incrTime, double zc, double g, double t1, double t2, double t3, vector<double> vectDOWNHalfStep_input, char leftOrRightFootStable)
{

	vector<double> comTrajX;
	vector<double> zmpTrajX;
	genCOMZMPtrajectory(comTrajX, zmpTrajX, incrTime, zc, g, 0, 0, vectDOWNHalfStep_input[2]/2, t1/2, t1*3/4, t1, t2, t3);

	vector<double> comTrajY;
	vector<double> zmpTrajY;
	genCOMZMPtrajectory(comTrajY, zmpTrajY, incrTime, zc, g, 0, 0, vectDOWNHalfStep_input[3]/2, t1/2, t1*3/4, t1, t2, t3);

	vector<double> footXtraj;
	vector<double> footYtraj;
	int leftRightCoef = 0;	
	if(leftOrRightFootStable == 'L') leftRightCoef = -1; else leftRightCoef = 1;
	genFOOTposition(footXtraj, footYtraj, incrTime, 0, leftRightCoef*vectDOWNHalfStep_input[0], vectDOWNHalfStep_input[2], vectDOWNHalfStep_input[3], DELAY_2, 0, 0, t1, t2, t3, 'd');

	vector<double> footHeight;
	genFOOTupDOWNheight(footHeight, incrTime, vectDOWNHalfStep_input[1], DELAY_1, t1, t2, t3);

	vector<double> footOrient;
	genFOOTorientation(footOrient, incrTime, 0, vectDOWNHalfStep_input[4], DELAY_2, 0, 0, t1, t2, t3, 'd');

	vector<double> waistOrient;
	genWAISTorientation(waistOrient, incrTime, 0, vectDOWNHalfStep_input[4], DELAY_1, 0, 0, t1, t2, t3, 'd');

	vector<double> stablefootXtraj;
	vector<double> stablefootYtraj;
	vector<double> stablefootHeight;
	vector<double> stablefootOrient;

	for(double i = 0.0 ; i < t3 ; i += incrTime)
	{
		stablefootXtraj.push_back(0);
		stablefootYtraj.push_back(0);
		stablefootHeight.push_back(0);
		stablefootOrient.push_back(0);
	}	

	stepF.comTrajX = comTrajX;
	stepF.zmpTrajX = zmpTrajX;
	stepF.comTrajY = comTrajY;
	stepF.zmpTrajY = zmpTrajY;

	if(leftOrRightFootStable == 'L') {
	stepF.leftfootXtraj = stablefootXtraj;
	stepF.leftfootYtraj = stablefootYtraj;
	stepF.leftfootHeight = stablefootHeight;
	stepF.leftfootOrient = stablefootOrient;
	stepF.rightfootXtraj = footXtraj;
	stepF.rightfootYtraj = footYtraj;
	stepF.rightfootHeight = footHeight;
	stepF.rightfootOrient = footOrient;
	} else {
	stepF.leftfootXtraj = footXtraj;
	stepF.leftfootYtraj = footYtraj;
	stepF.leftfootHeight = footHeight;
	stepF.leftfootOrient = footOrient;
	stepF.rightfootXtraj = stablefootXtraj;
	stepF.rightfootYtraj = stablefootYtraj;
	stepF.rightfootHeight = stablefootHeight;
	stepF.rightfootOrient = stablefootOrient;
	}

	stepF.waistOrient =  waistOrient;	
	stepF.incrTime = incrTime;
	stepF.zc = zc;
	stepF.size = waistOrient.size();

}

void generate_halfStepFeatures(trajFeatures & t, const SE2 & supportconfig, const halfStepDefinition & def) {
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
}