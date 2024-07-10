void eventUpdateLoopsTable()
{/*ALCODESTART::1712730454038*/
if (parDisplayGraphs == true){
	for (int i = 0; i < colLoopData.size(); i++){
		DataSet ds = colLoopData.get(i);
		if (varInt0 < 1 ){
			double time = time();
			double distance = colNodeDistance.get(i);
			ds.add(time, distance);
			chartTrainDistances.addDataSet(ds, "Loop"+i, Color.lightGray, true, false, TimePlot.INTERPOLATION_LINEAR, 0.8, TimePlot.POINT_NONE);	
		}else {
			double time = time();
			double distance = colNodeDistance.get(i);
			ds.add(time, distance);
			chartTrainDistances.update();
		}
	}
	varInt0 ++;

	for (int i = 0; i < pop_Trains0.size(); i++) {
		Train train = pop_Trains0.get(i);
		train.funGetLatestDS();
	}
	for (int i = 0; i < pop_TrainsX.size(); i++) {
		Train train = pop_TrainsX.get(i);
		train.funGetLatestDS();
	}
}
/*ALCODEEND*/}

void eventInject0()
{/*ALCODESTART::1714999550364*/
int numTrainsFrom0 = pop_Trains0.size();
int numTrainsFromX = pop_TrainsX.size();
if (numTrainsFrom0 + numTrainsFromX < parSystemCapacity	){
	funInject0();
}
/*ALCODEEND*/}

void eventInjectX()
{/*ALCODESTART::1714999899214*/
int numTrainsFrom0 = pop_Trains0.size();
int numTrainsFromX = pop_TrainsX.size();
if (numTrainsFrom0 + numTrainsFromX < parSystemCapacity){
	funInjectX();
}
/*ALCODEEND*/}

