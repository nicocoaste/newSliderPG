

void CnewPGstepStudy::addStepFeaturesWithSlide(
	StepFeatures & stepF1,
	StepFeatures & stepF2,	
	double negativeSlideTime
	) 
{
	
	//PHASE 1: modify stepF2 according to the change of origin
	double lastwaistX = stepF1.comTrajX[stepF1.size - 1];
	double lastwaistY = stepF1.comTrajY[stepF1.size - 1]; 
	double radlastwaistOrient = stepF1.waistOrient[stepF1.size - 1]*PI/180;

	for(unsigned int count = 0 ; count < stepF2.size ; count++) {

/*	double newcomX = (stepF2.comTrajX[count]+lastwaistX)*cos(radlastwaistOrient)
			-(stepF2.comTrajY[count]+lastwaistY)*sin(radlastwaistOrient);
	double newcomY = (stepF2.comTrajX[count]+lastwaistX)*sin(radlastwaistOrient)
			+(stepF2.comTrajY[count]+lastwaistY)*cos(radlastwaistOrient);
	double newzmpX = (stepF2.zmpTrajX[count]+lastwaistX)*cos(radlastwaistOrient)
			-(stepF2.zmpTrajY[count]+lastwaistY)*sin(radlastwaistOrient);
	double newzmpY = (stepF2.zmpTrajX[count]+lastwaistX)*sin(radlastwaistOrient)
			+(stepF2.zmpTrajY[count]+lastwaistY)*cos(radlastwaistOrient);
	double newlfX = (stepF2.leftfootXtraj[count]+lastwaistX)*cos(radlastwaistOrient)
			-(stepF2.leftfootYtraj[count]+lastwaistY)*sin(radlastwaistOrient);	
	double newlfY = (stepF2.leftfootXtraj[count]+lastwaistX)*sin(radlastwaistOrient)
			+(stepF2.leftfootYtraj[count]+lastwaistY)*cos(radlastwaistOrient);	
	double newrfX = (stepF2.rightfootXtraj[count]+lastwaistX)*cos(radlastwaistOrient)
			-(stepF2.rightfootYtraj[count]+lastwaistY)*sin(radlastwaistOrient);
	double newrfY = (stepF2.rightfootXtraj[count]+lastwaistX)*sin(radlastwaistOrient)   
			+(stepF2.rightfootYtraj[count]+lastwaistY)*cos(radlastwaistOrient);*/

	double newcomX = (stepF2.comTrajX[count])*cos(radlastwaistOrient)
			-(stepF2.comTrajY[count])*sin(radlastwaistOrient)+lastwaistX;
	double newcomY = (stepF2.comTrajX[count])*sin(radlastwaistOrient)
			+(stepF2.comTrajY[count])*cos(radlastwaistOrient)+lastwaistY;
	double newzmpX = (stepF2.zmpTrajX[count])*cos(radlastwaistOrient)
			-(stepF2.zmpTrajY[count])*sin(radlastwaistOrient)+lastwaistX;
	double newzmpY = (stepF2.zmpTrajX[count])*sin(radlastwaistOrient)
			+(stepF2.zmpTrajY[count])*cos(radlastwaistOrient)+lastwaistY;
	double newlfX = (stepF2.leftfootXtraj[count])*cos(radlastwaistOrient)
			-(stepF2.leftfootYtraj[count])*sin(radlastwaistOrient)+lastwaistX;	
	double newlfY = (stepF2.leftfootXtraj[count])*sin(radlastwaistOrient)
			+(stepF2.leftfootYtraj[count])*cos(radlastwaistOrient)+lastwaistY;	
	double newrfX = (stepF2.rightfootXtraj[count])*cos(radlastwaistOrient)
			-(stepF2.rightfootYtraj[count])*sin(radlastwaistOrient)+lastwaistX;
	double newrfY = (stepF2.rightfootXtraj[count])*sin(radlastwaistOrient)   
			+(stepF2.rightfootYtraj[count])*cos(radlastwaistOrient)+lastwaistY;	

	stepF2.comTrajX[count] = newcomX;
	stepF2.zmpTrajX[count] = newzmpX;

	stepF2.comTrajY[count] = newcomY;
	stepF2.zmpTrajY[count] = newzmpY;

	stepF2.leftfootXtraj[count] = newlfX;
	stepF2.leftfootYtraj[count] = newlfY;

	stepF2.leftfootOrient[count] += stepF1.waistOrient[stepF1.size - 1];

	stepF2.rightfootXtraj[count] = newrfX;
	stepF2.rightfootYtraj[count] = newrfY;

	stepF2.rightfootOrient[count] += stepF1.waistOrient[stepF1.size - 1];

	stepF2.waistOrient[count] += stepF1.waistOrient[stepF1.size - 1];	

	}

	int delayInt = (int) (abs(negativeSlideTime)/stepF2.incrTime);

	//PHASE 2: add the new stepF2 to stepF1
	for(unsigned int count = 0 ; count < stepF2.size ; count++) {

	if(count < delayInt) {

	stepF1.comTrajX[stepF1.size - delayInt + count] =
		(stepF1.comTrajX[stepF1.size - delayInt + count] + stepF2.comTrajX[count])
		-stepF1.comTrajX[stepF1.size - 1];
	stepF1.zmpTrajX[stepF1.size - delayInt + count] =
		(stepF1.zmpTrajX[stepF1.size - delayInt + count] + stepF2.zmpTrajX[count])
		-stepF1.zmpTrajX[stepF1.size - 1];

	stepF1.comTrajY[stepF1.size - delayInt + count] =
		( stepF1.comTrajY[stepF1.size - delayInt + count] + stepF2.comTrajY[count])
		-stepF1.comTrajY[stepF1.size - 1];
	stepF1.zmpTrajY[stepF1.size - delayInt + count] =
		( stepF1.zmpTrajY[stepF1.size - delayInt + count] + stepF2.zmpTrajY[count])
		-stepF1.zmpTrajY[stepF1.size - 1];

	stepF1.leftfootXtraj[stepF1.size - delayInt + count] =
		( stepF1.leftfootXtraj[stepF1.size - delayInt + count] + stepF2.leftfootXtraj[count])
		-stepF1.leftfootXtraj[stepF1.size - 1];
	stepF1.leftfootYtraj[stepF1.size - delayInt + count] =
		( stepF1.leftfootYtraj[stepF1.size - delayInt + count] + stepF2.leftfootYtraj[count])
		-stepF1.leftfootYtraj[stepF1.size - 1];

	stepF1.leftfootOrient[stepF1.size - delayInt + count] =
		( stepF1.leftfootOrient[stepF1.size - delayInt + count] + stepF2.leftfootOrient[count])
		-stepF1.leftfootOrient[stepF1.size - 1];
	stepF1.leftfootHeight[stepF1.size - delayInt + count] =
		(  stepF1.leftfootHeight[stepF1.size - delayInt + count] + stepF2.leftfootHeight[count])
		-stepF1.leftfootHeight[stepF1.size - 1];

	stepF1.rightfootXtraj[stepF1.size - delayInt + count] =
		( stepF1.rightfootXtraj[stepF1.size - delayInt + count] + stepF2.rightfootXtraj[count])
		-stepF1.rightfootXtraj[stepF1.size - 1];
	stepF1.rightfootYtraj[stepF1.size - delayInt + count] =
		( stepF1.rightfootYtraj[stepF1.size - delayInt + count] + stepF2.rightfootYtraj[count])
		-stepF1.rightfootYtraj[stepF1.size - 1];

	stepF1.rightfootOrient[stepF1.size - delayInt + count] =
		( stepF1.rightfootOrient[stepF1.size - delayInt + count] + stepF2.rightfootOrient[count])
		-stepF1.rightfootOrient[stepF1.size - 1];
	stepF1.rightfootHeight[stepF1.size - delayInt + count] =
		( stepF1.rightfootHeight[stepF1.size - delayInt + count] + stepF2.rightfootHeight[count])
		-stepF1.rightfootHeight[stepF1.size - 1];

	stepF1.waistOrient[stepF1.size - delayInt + count] =
		( stepF1.waistOrient[stepF1.size - delayInt + count] + stepF2.waistOrient[count])
		-stepF1.waistOrient[stepF1.size - 1];

	} else {

	stepF1.comTrajX.push_back(stepF2.comTrajX[count]);
	stepF1.zmpTrajX.push_back(stepF2.zmpTrajX[count]);

	stepF1.comTrajY.push_back(stepF2.comTrajY[count]);
	stepF1.zmpTrajY.push_back(stepF2.zmpTrajY[count]);

	stepF1.leftfootXtraj.push_back(stepF2.leftfootXtraj[count]);
	stepF1.leftfootYtraj.push_back(stepF2.leftfootYtraj[count]);

	stepF1.leftfootOrient.push_back(stepF2.leftfootOrient[count]);
	stepF1.leftfootHeight.push_back(stepF2.leftfootHeight[count]);

	stepF1.rightfootXtraj.push_back(stepF2.rightfootXtraj[count]);
	stepF1.rightfootYtraj.push_back(stepF2.rightfootYtraj[count]);

	stepF1.rightfootOrient.push_back(stepF2.rightfootOrient[count]);
	stepF1.rightfootHeight.push_back(stepF2.rightfootHeight[count]);

	stepF1.waistOrient.push_back(stepF2.waistOrient[count]);	

	}

	}

	stepF1.size = stepF1.size + stepF2.size - delayInt;

}