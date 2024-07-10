double funGetDistance()
{/*ALCODESTART::1712319208038*/
//Distance is updated in Km per half a second
varDistance = varDistance + ((varUpdateInterval*varVelocity)/(60*60));

/*ALCODEEND*/}

DataSet funGetLatestDS()
{/*ALCODESTART::1712730311259*/
double time = time();
double distance = varDistance;
dsTrain.add(time, distance);
parModel.chartTrainDistances.update();
/*ALCODEEND*/}

double funInitialseTrainData()
{/*ALCODESTART::1717674832166*/
int red = getDefaultRandomGenerator().nextInt(256);
int green = getDefaultRandomGenerator().nextInt(256);
int blue = getDefaultRandomGenerator().nextInt(256);
if (parDisplayTrainJourney == true){
	double time = time();
	double distance = varDistance;
	dsTrain.add(time, distance);
	parModel.chartTrainDistances.addDataSet(dsTrain, "Train"+parDirection+" "+parIndex, new Color(red, green, blue), true, false, TimePlot.INTERPOLATION_LINEAR, 1.3, TimePlot.POINT_NONE);
}
/*ALCODEEND*/}

