double funInitialiseNodes()
{/*ALCODESTART::1714125431771*/
for (int i = 0; i < colNodeNumber.size(); i++){
	Node_Loop node = add_pop_Nodes(i, colNodeDistance.get(i), 
	colNodeLength.get(i));
}
/*ALCODEEND*/}

double funInject0()
{/*ALCODESTART::1714127012937*/
//Instantiate train
double trainLength = uniform_discr(3, 4);
//int trainPriority = getDefaultRandomGenerator().nextInt(2);
Train train = add_pop_Trains0(varTrain0Index, parTrainFrom0MaxVelocity, trainLength, this, parTrainFromDistance0, parTrainFrom0DefaultPriority, parDisplayGraphs);
traceln(" ");
traceln("__Train "+parTrainFromDistance0+" added");
traceln("The current time is "+time(HOUR));
//Give train its colleciton elements
for (int i = 0; i < pop_Nodes.size(); i++){
	train.colNodes.add(pop_Nodes.get(i));
}
for (int i = 0; i < pop_Segments.size(); i++){
	train.colSegments.add(pop_Segments.get(i));
}
//Give train its route
for (int i = 0; i < train.colSegments.size(); i++){
	if (train.parLength < train.colNodes.get(i).parLenght){
		train.colRoute.add(train.colNodes.get(i));
	}
	train.colRoute.add(train.colSegments.get(i));
}
train.colRoute.add(train.colNodes.get(train.colNodes.size()-1));

//Populate train variables (Starting)
Node_Loop firstNode = (Node_Loop)train.colRoute.get(0);
train.varDistance = firstNode.parDistance;
train.varVelocity = 0.0;
//train.varPriority = getDefaultRandomGenerator().nextInt(2);
//train.varPriority = 0;

//Setting trains initial time bookings
double endTimePrevious = time(HOUR);
for (int i = 0; i < train.colRoute.size(); i++){
	if (train.colRoute.get(i) instanceof Node_Loop){
		Node_Loop node = (Node_Loop)train.colRoute.get(i);
		TimeBooking TBN = add_pop_TBN("Node", node, train);
		TBN.varVelocity = 0.0;
		TBN.varTimeStart = endTimePrevious;
		TBN.varTimeEnd = TBN.varTimeStart;
		TBN.varPriority = train.parDefaultPriority;
		train.colTBN.add(TBN);
		train.colTB.add(TBN);
		node.colTBN.add(TBN);
		endTimePrevious = TBN.varTimeEnd;
	} else if (train.colRoute.get(i) instanceof Segment){
		Segment segment = (Segment)train.colRoute.get(i);
		TimeBooking TBS = add_pop_TBS("Segment", segment, train);
		TBS.varTimeStart = endTimePrevious;
		double travelDuration = /*triangular(0.75, 1.25, 1)**/segment.parTravelTimeEmpty;
		TBS.varVelocity = segment.parLength/travelDuration;
		//double travelDuration = segment.parLength/TBS.varVelocity;
		TBS.varTimeEnd = TBS.varTimeStart + travelDuration;
		TBS.varPriority = train.parDefaultPriority;
		train.colTBS.add(TBS);
		train.colTB.add(TBS);
		segment.colTBS.add(TBS);
		endTimePrevious = TBS.varTimeEnd;
	}
}
traceln("train "+parTrainFromDistance0+" initialised");

//Execute Collision Avoidance
traceln("running pre injection collision avoidance");
funConflictAvoidance1();

//pauseSimulation();
enter2.take(train);
varTrain0Index++;
/*ALCODEEND*/}

double funInjectX()
{/*ALCODESTART::1714127022412*/
//Instantiate train
double trainLength = uniform_discr(3, 4);
//int trainPriority = getDefaultRandomGenerator().nextInt(2);
Train train = add_pop_TrainsX(varTrainXIndex, -parTrainFromXMaxVelocity, trainLength, this, parTrainFromDistanceX, parTrainFromXDefaultPriority, parDisplayGraphs);
traceln(" ");
traceln("__Train "+parTrainFromDistanceX+" added");
traceln("The current time is "+time(HOUR));
//Give train its colleciton elements
for (int i = pop_Nodes.size()-1; i > -1; i--){
	train.colNodes.add(pop_Nodes.get(i));
}
for (int i = pop_Segments.size()-1; i > -1; i--){
	train.colSegments.add(pop_Segments.get(i));
}
//Give train its route
for (int i = 0; i < train.colSegments.size(); i++){
	if (train.parLength < train.colNodes.get(i).parLenght){
		train.colRoute.add(train.colNodes.get(i));
	}
	train.colRoute.add(train.colSegments.get(i));
}
train.colRoute.add(train.colNodes.get(train.colNodes.size()-1));

//Populate train variables (Starting)
Node_Loop firstNode = (Node_Loop)train.colRoute.get(0);
train.varDistance = firstNode.parDistance;
train.varVelocity = 0.0;
//train.varPriority = getDefaultRandomGenerator().nextInt(2);
//train.varPriority = 1;

//Setting trains initial time bookings
double endTimePrevious = time(HOUR);
for (int i = 0; i < train.colRoute.size(); i++){
	if (train.colRoute.get(i) instanceof Node_Loop){
		Node_Loop node = (Node_Loop)train.colRoute.get(i);
		TimeBooking TBN = add_pop_TBN("Node", node, train);
		TBN.varVelocity = 0.0;
		TBN.varTimeStart = endTimePrevious;
		TBN.varTimeEnd = TBN.varTimeStart;
		TBN.varPriority = train.parDefaultPriority;
		train.colTBN.add(TBN);
		train.colTB.add(TBN);
		node.colTBN.add(TBN);
		endTimePrevious = TBN.varTimeEnd;
	} else if (train.colRoute.get(i) instanceof Segment){
		Segment segment = (Segment)train.colRoute.get(i);
		TimeBooking TBS = add_pop_TBS("Segment", segment, train);
		TBS.varTimeStart = endTimePrevious;
		double travelDuration = /*triangular(0.75, 1.25, 1)**/segment.parTravelTimeLoaded;
		TBS.varVelocity = -segment.parLength/travelDuration;
		//double travelDuration = -segment.parLength/TBS.varVelocity;
		TBS.varTimeEnd = TBS.varTimeStart + travelDuration;
		TBS.varPriority = train.parDefaultPriority;
		train.colTBS.add(TBS);
		train.colTB.add(TBS);
		segment.colTBS.add(TBS);
		endTimePrevious = TBS.varTimeEnd;
	}
}
traceln("train "+parTrainFromDistanceX+" initialised");

//Execute Collision Avoidance
traceln("running pre injection collision avoidance");
funConflictAvoidance1();

//pauseSimulation();
enter3.take(train);
varTrainXIndex++;
/*ALCODEEND*/}

double funInitialiseSegments()
{/*ALCODESTART::1714128834808*/
for (int i = 1; i < colSegmentLength.size(); i++){
	double travelTimeEmpties = triangular(0.75, 1.25, 1)*colSegmentTravelTimeEmpties.get(i);
	double travelTimeLoaded = triangular(0.75, 1.25, 1)*colSegmentTravelTimeLoaded.get(i);
	Segment segment = add_pop_Segments(i-1, pop_Nodes.get(i-1), pop_Nodes.get(i), 
	colSegmentLength.get(i), 70.0, travelTimeEmpties, travelTimeLoaded);
}
/*ALCODEEND*/}

ArrayList<TimeBooking> funTrainPrioritisation2(TimeBooking booking1,TimeBooking booking2)
{/*ALCODESTART::1714722572445*/
//Determing which train will be prioritised

	//Creating references
		Train train1 = booking1.parTrain;
		Train train2 = booking2.parTrain;
		TimeBooking booking0 = new TimeBooking();
		TimeBooking bookingX = new TimeBooking();
		if (!train1.parDirection.equals(train2.parDirection)){
			if (booking1.parTrain.parDirection == parTrainFromDistance0){
				booking0 = booking1;
				bookingX = booking2;
			}else if (booking1.parTrain.parDirection == parTrainFromDistanceX){
				booking0 = booking2;
				bookingX = booking1;
			}
		}
		TimeBooking priorityBooking = new TimeBooking();
		TimeBooking nonPriorityBooking = new TimeBooking();
		//traceln("  comparing train priorities");
	
	//Determine which train will be prioritied
		if (((Segment)booking1.parResource) == pop_Segments.get(0) &&
			((Segment)booking2.parResource) == pop_Segments.get(0) &&
			!train1.parDirection.equals(train2.parDirection)){
			priorityBooking = bookingX;
			nonPriorityBooking = booking0;
			traceln("   Special condition: first segment - nonPriority booking is: "+nonPriorityBooking);
		}else if (((Segment)booking1.parResource) == pop_Segments.get(pop_Segments.size()-1) &&
			((Segment)booking2.parResource) == pop_Segments.get(pop_Segments.size()-1) &&
			!train1.parDirection.equals(train2.parDirection)){
			priorityBooking = booking0;
			nonPriorityBooking = bookingX;
			traceln("   Special condition: last segment - nonPriority booking is: "+nonPriorityBooking);
		}else if (booking1.varPriority > booking2.varPriority){
			priorityBooking = booking1;
			nonPriorityBooking = booking2;
			traceln("   nonPriority booking is: "+nonPriorityBooking);
			traceln("   nonPriority train is: "+nonPriorityBooking.parTrain+" with index "
				+nonPriorityBooking.parTrain.parIndex);
		} else if (booking1.varPriority < booking2.varPriority){
			priorityBooking = booking2;
			nonPriorityBooking = booking1;
			traceln("   nonPriority booking is: "+nonPriorityBooking);
			traceln("   nonPriority train is: "+nonPriorityBooking.parTrain+" with index "
				+nonPriorityBooking.parTrain.parIndex);
		} else if (booking1.varPriority == booking2.varPriority){
			if (booking1.varTimeStart <= booking2.varTimeStart){
				priorityBooking = booking1;
				nonPriorityBooking = booking2;
				traceln("   nonPriority booking is: "+nonPriorityBooking);
				traceln("   nonPriority train is: "+nonPriorityBooking.parTrain+" with index "
					+nonPriorityBooking.parTrain.parIndex);
			} else if (booking1.varTimeStart > booking2.varTimeStart){
				priorityBooking = booking2;
				nonPriorityBooking = booking1;
				traceln("   nonPriority booking is: "+nonPriorityBooking);
				traceln("   nonPriority train is: "+nonPriorityBooking.parTrain+" with index "
					+nonPriorityBooking.parTrain.parIndex);
			}
		}
	
	//Create a list of the determined priority and nonpriority bookings
		ArrayList<TimeBooking> trainPriorities = new ArrayList();
		trainPriorities.add(priorityBooking);
		trainPriorities.add(nonPriorityBooking);
	
	return trainPriorities;
/*ALCODEEND*/}

Train funSubsequentTBAdjustments(TimeBooking nodeBookingAdjustment,TimeBooking prioritySegmentBooking)
{/*ALCODESTART::1714726213451*/
//Update all subsequent bookings

	//The nonpriority train	
		//Update nonpriority booking times and priorities
			Train train = nodeBookingAdjustment.parTrain;//the non priority train
			int tbIndex = train.colTB.indexOf(nodeBookingAdjustment);//the index of the non priority booking in the trains TB list
			double endTimePrevious = nodeBookingAdjustment.varTimeEnd;
			
			for (int i = tbIndex+1; i < train.colTB.size(); i++){
				TimeBooking tb = train.colTB.get(i);
				//For the Node bookings
					if (tb.parResource instanceof Node_Loop){
						tb.varTimeStart = endTimePrevious;
						tb.varTimeEnd = tb.varTimeStart;
						//Update varPriority if passing trains
						if (!nodeBookingAdjustment.parTrain.parDirection.equals(prioritySegmentBooking.parTrain.parDirection)) {
							tb.varPriority = nodeBookingAdjustment.varPriority;
						}
						//traceln("Booking Node new priority is "+tb.varPriority);
						endTimePrevious = tb.varTimeEnd;
					}
				//For Segment bookings
					else if (tb.parResource instanceof Segment){
						Segment segment = (Segment)tb.parResource;
						tb.varTimeStart = endTimePrevious;
						double travelDuration = 0.0;
						if (train.parDirection == parTrainFromDistance0){	
							travelDuration = segment.parLength/tb.varVelocity;
						} else if (train.parDirection == parTrainFromDistanceX){
							travelDuration = -segment.parLength/tb.varVelocity;
						}
						tb.varTimeEnd = tb.varTimeStart + travelDuration;
						//Update varPriority of next segment booking only, if trains are passing
						if (!nodeBookingAdjustment.parTrain.parDirection.equals(prioritySegmentBooking.parTrain.parDirection)){
							if (i == tbIndex + 1) {
								//tb.varPriority ++;
								tb.varPriority = nodeBookingAdjustment.parTrain.parDefaultPriority + 1;
							}else if (i > tbIndex + 1){
								//tb.varPriority ++;
								tb.varPriority = nodeBookingAdjustment.parTrain.parDefaultPriority;
							}
						}
						//traceln("Booking Segment new priority is "+tb.varPriority);
						endTimePrevious = tb.varTimeEnd;
					}
			}		
/*ALCODEEND*/}

Train funNodeBookingAdjust3(TimeBooking priorityBooking,TimeBooking nonPriorityBooking)
{/*ALCODESTART::1715089002805*/
//Adjust the non priority train's node booking based on the end time of the priority booking

	//Find the node booking to adjust for the non priority train (if possible) based on node availability.
	//If not possible find the node booking to adjust for the priority train.
		Train nonPriorityTrain = nonPriorityBooking.parTrain;
		Train priorityTrain = priorityBooking.parTrain;
		//traceln(" runing non priority train node finding func.");
		ArrayList<TimeBooking> adjustmentTBs = new ArrayList();
		TimeBooking tbNodeAdjust = new TimeBooking();
		TimeBooking otherTrainSegmentBooking = new TimeBooking();
		
		if (!nonPriorityTrain.parDirection.equals(priorityTrain.parDirection)){
			traceln("  Passing Trains");
			adjustmentTBs = funFindTBNtoAdjustPassingTrains(priorityBooking, nonPriorityBooking);
			if (!adjustmentTBs.isEmpty()){
				//traceln("  Adjusting nonPriority passing train: "+adjustmentTBs.get(0).parTrain);
				tbNodeAdjust = adjustmentTBs.get(0);
				otherTrainSegmentBooking = adjustmentTBs.get(1);
			}else if (adjustmentTBs.isEmpty()){ 
			//If the non priority train cannot be adjusted, find node for priority train
				traceln(" runing priority passing train node finding func.");
				adjustmentTBs = funFindTBNtoAdjustPassingTrains(nonPriorityBooking, priorityBooking);
				if (adjustmentTBs.isEmpty()){
					error("System Failure: Girdlock");
				}
				tbNodeAdjust = adjustmentTBs.get(0);
				otherTrainSegmentBooking = adjustmentTBs.get(1);
			}
		}else {
			adjustmentTBs = funFindTBNtoAdjustFollowingTrains(priorityBooking, nonPriorityBooking);
			if (adjustmentTBs.isEmpty()){
				error("System Failure: Girdlock");
			}
			tbNodeAdjust = adjustmentTBs.get(0);
			otherTrainSegmentBooking = adjustmentTBs.get(1);
		}
	
	//Adjust the end time of the node time booking for the nonpriority train
		traceln("   TBN to adjust is "+tbNodeAdjust+" with a start time of "+tbNodeAdjust.varTimeStart+" and end time of "
			+tbNodeAdjust.varTimeEnd+" for train "+tbNodeAdjust.parTrain);
		traceln("   other train segment booking is "+otherTrainSegmentBooking+" with segment "
			+otherTrainSegmentBooking.parResource+" and start time of "+otherTrainSegmentBooking.varTimeStart+
			" and end time of "+otherTrainSegmentBooking.varTimeEnd+" for train "+otherTrainSegmentBooking.parTrain);
		
		if (!nonPriorityTrain.parDirection.equals(priorityTrain.parDirection)){
			tbNodeAdjust.varTimeEnd = otherTrainSegmentBooking.varTimeEnd + 0.0333;
			traceln("   node time booking adjusted for "+tbNodeAdjust.parTrain+" to a new end time of "+tbNodeAdjust.varTimeEnd);
		} else if (nonPriorityTrain.parDirection.equals(priorityTrain.parDirection)){
			tbNodeAdjust.varTimeEnd = otherTrainSegmentBooking.varTimeEnd;
			traceln("   node time booking adjusted for "+tbNodeAdjust.parTrain+" to a new end time of "+tbNodeAdjust.varTimeEnd);
		}
		
	//Adjust the subsequent priority and non priority train bookings
		funSubsequentTBAdjustments(tbNodeAdjust, otherTrainSegmentBooking);
/*ALCODEEND*/}

ArrayList<TimeBooking> funFindTBNtoAdjustPassingTrains(TimeBooking priorityBooking,TimeBooking nonPriorityBooking)
{/*ALCODESTART::1715154143025*/
//Finding the node time booking to adjust for passing trains

	//Finding the node booking to adjust (going backwards in the non priority trains list of TBs and selecting the first nodeTB)
		Train priorityTrain = priorityBooking.parTrain;
		Train nonPriorityTrain = nonPriorityBooking.parTrain;
		int nonPriorityTBIndex = nonPriorityTrain.colTB.indexOf(nonPriorityBooking);
		
		ArrayList<TimeBooking> adjustmentTBs = new ArrayList();//the arraylist of timebookings to be returned and used for adjustmnets
		TimeBooking nonPriorityTBNodeAdjust = new TimeBooking();//the node booking to adjust
		TimeBooking prioritySegmentBooking = new TimeBooking();//the priority train's relevant segment booking for the node TB to adjsut
		
		outerLoop:
		for (int i = nonPriorityTBIndex-1; i > -1; i--){
			TimeBooking booking = nonPriorityTrain.colTB.get(i);
			
			if (booking.parResource instanceof Node_Loop){
				//Get the relevant segment time booking of the priority train to calculate the theoretical end time if "booking" were to be adjusted
				Segment referenceSegment = (Segment)nonPriorityTrain.colTB.get(i+1).parResource;
				//traceln("  Priority Train segment Index is : "+priorityTrainSegmentIndex);
				for (TimeBooking referenceSegmentBooking : referenceSegment.colTBS){
					if (referenceSegmentBooking.parTrain.equals(priorityTrain)){
						prioritySegmentBooking = referenceSegmentBooking;
					}
				}
				double bookingEndTheoretical = prioritySegmentBooking.varTimeEnd + 0.0333;
				//check availability of the node
				Node_Loop nodeToCheck = (Node_Loop)booking.parResource;
				//Create a list of time bookings where a train uses this node (i.e the difference in start and end time is > 0)
				ArrayList<TimeBooking> relevantBookings = new ArrayList();
				ArrayList<TimeBooking> sortedRelevantBookings = new ArrayList();
				for (int j = 0; j < nodeToCheck.colTBN.size(); j++){
					TimeBooking possibleRelevantBooking = nodeToCheck.colTBN.get(j);
					if (possibleRelevantBooking.varTimeEnd - possibleRelevantBooking.varTimeStart != 0.0
					&& booking.varTimeStart < possibleRelevantBooking.varTimeEnd
					&& booking != possibleRelevantBooking){
						relevantBookings.add(possibleRelevantBooking);
					}
				}
				//Sort the list into ascending order by start time
				sortedRelevantBookings = (ArrayList<TimeBooking>)sortAscending(relevantBookings, b -> b.varTimeStart);
				
				//Remove bookings that will not cause conflict if non priority train theoretically used the loop
				for (int k = 0; k < sortedRelevantBookings.size(); k++){
					if (bookingEndTheoretical < sortedRelevantBookings.get(k).varTimeStart){
						sortedRelevantBookings.remove(k);
					}
				}
				if (nodeToCheck == pop_Nodes.get(0) || nodeToCheck == pop_Nodes.get(pop_Nodes.size()-1)){
					nonPriorityTBNodeAdjust = booking;
					adjustmentTBs.add(nonPriorityTBNodeAdjust);
					adjustmentTBs.add(prioritySegmentBooking);
					traceln("   TBN to addjsut is "+nonPriorityTBNodeAdjust+" with node index "+((Node_Loop)nonPriorityTBNodeAdjust.parResource).parNodeNumber);
					break outerLoop;
				}
				
				//If the list is empty it means that the node is available at that time. If not another node needs to be considered
				else if (sortedRelevantBookings.isEmpty()){
					nonPriorityTBNodeAdjust = booking;
					adjustmentTBs.add(nonPriorityTBNodeAdjust);
					adjustmentTBs.add(prioritySegmentBooking);
					traceln("   TBN to addjsut is "+nonPriorityTBNodeAdjust+" with node index "+((Node_Loop)nonPriorityTBNodeAdjust.parResource).parNodeNumber);
					break outerLoop;
				}
				traceln("   Sorted relevant bookings: "+sortedRelevantBookings);		
			}	
		}

return adjustmentTBs;
/*ALCODEEND*/}

double funConflictAvoidance1()
{/*ALCODESTART::1715610031897*/
//This function detects relevant time bookings and resolves conflicts based on time bookings

	//get a chronological list of segment time bookings in the future
		traceln("STARTING CONFLICT AVOIDANCE");
		ArrayList<TimeBooking> segmentBookings = funFilterTimeBookings1();
		//traceln("bookings filtered");
	
	//Detect first colnflict
		ArrayList<TimeBooking> conflictBookings = funConflictDetection1(segmentBookings);
		//traceln("conflict detection ran");
	
	//Resolve all conflicts untill there are none left
		for ( ; conflictBookings.size() > 0; ){
				//traceln("Resolving first conflict");
			//Get first conflict time bookings
				TimeBooking booking1 = conflictBookings.get(0);
				//traceln("Booking1 is: "+booking1);
				TimeBooking booking2 = conflictBookings.get(1);
				//traceln("Booking2 is: "+booking2);
			//Resolve the conflict
				funBookingAdjustment1(booking1, booking2);
				traceln("__adjustments made");
			//Get new and updated list of segment time bookings and check for conflicts
				segmentBookings = funFilterTimeBookings1();
				conflictBookings = funConflictDetection1(segmentBookings);
				//traceln("bookings filteres and next conflict determined");
		}
		traceln("No more conflicts detected");
		traceln(" ");
/*ALCODEEND*/}

ArrayList<TimeBooking> funConflictDetection1(ArrayList<TimeBooking> sortedTimeBookings)
{/*ALCODESTART::1715610031899*/
//Detect first conflict and pass to the conflict avoidance function 

	//Compare each segment time booking in given list to each segment time booking proceeding it
		ArrayList<TimeBooking> conflictBookings = new ArrayList<TimeBooking>();
		conflictSearch:
		for (int i = 0; i < sortedTimeBookings.size()-1; i++){
			TimeBooking booking1 = sortedTimeBookings.get(i);
			
			for (int j = i+1; j < sortedTimeBookings.size(); j++){
				TimeBooking booking2 = sortedTimeBookings.get(j);
			
			//If they share the same resource and have overlapping times then there is a conflict	
				if (booking1.parResource == booking2.parResource){
				//Resource Conflict
					if (booking1.varTimeEnd > booking2.varTimeStart){
						conflictBookings.add(booking1);
						conflictBookings.add(booking2);
						traceln("  There is a conflict between "+booking1+" and "+booking2+
							" on Segment "+((Segment)booking1.parResource).parSegmentNumber);
						traceln("  "+booking1+" has a start time of "+booking1.varTimeStart+
							" and an end time of "+booking1.varTimeEnd);
						traceln("   Booking1 is for train: "+booking1.parTrain+" with index "
							+booking1.parTrain.parIndex+" currently at "+booking1.parTrain.colRoute.get(0));
						traceln("  "+booking2+" has a start time of "+booking2.varTimeStart+
							" and an end time of "+booking2.varTimeEnd);
						traceln("   Booking2 is for train: "+booking2.parTrain+" with index "
							+booking2.parTrain.parIndex+" currently at "+booking2.parTrain.colRoute.get(0));
						break conflictSearch;
					}
				//Overtaking Conflict
					else if ((booking1.parTrain.parDirection.equals(booking2.parTrain.parDirection)) &&
					(booking1.parTrain.parIndex > booking2.parTrain.parIndex) &&
					(booking1.varTimeStart < booking2.varTimeStart)){
						conflictBookings.add(booking1);
						conflictBookings.add(booking2);
						traceln("  Overtaking Conflict");
						traceln("  There is a conflict between "+booking1+" and "+booking2+
							" on Segment "+((Segment)booking1.parResource).parSegmentNumber);
						traceln("  "+booking1+" has a start time of "+booking1.varTimeStart+
							" and an end time of "+booking1.varTimeEnd);
						traceln("   Booking1 is for train: "+booking1.parTrain+" with index "
							+booking1.parTrain.parIndex+" currently at "+booking1.parTrain.colRoute.get(0));
						traceln("  "+booking2+" has a start time of "+booking2.varTimeStart+
							" and an end time of "+booking2.varTimeEnd);
						traceln("   Booking2 is for train: "+booking2.parTrain+" with index "
							+booking2.parTrain.parIndex+" currently at "+booking2.parTrain.colRoute.get(0));
						break conflictSearch;
					}
					
				}
			}
		}
	
	return conflictBookings;
/*ALCODEEND*/}

ArrayList<TimeBooking> funFilterTimeBookings1()
{/*ALCODESTART::1715610031901*/
//Create a new list of TimeBookings that have a start time less than 2 hours in the future
	
	//Create a list of future segment time bookings
		double currentTime = time(HOUR);
		ArrayList<TimeBooking> bookings = new ArrayList();
		
		for (int i = 0; i < pop_TBS.size(); i++){
			TimeBooking booking = pop_TBS.get(i);
			if (booking.varTimeEnd >= currentTime){
		//	&& booking.varTimeStart <= currentTime+2.0){
				bookings.add(booking);
			}
		}
	//Sort the list into ascending order by start time
		ArrayList<TimeBooking> sortedTimeBookings = 
			(ArrayList<TimeBooking>)sortAscending(bookings, b -> b.varTimeStart);
		
	return sortedTimeBookings;	
/*ALCODEEND*/}

Train funBookingAdjustment1(TimeBooking booking1,TimeBooking booking2)
{/*ALCODESTART::1715610031903*/
//Adjust Bookings
	//Determing the non priority train (i.e the non priority booking) to be adjusted
		//traceln("Determining train priorities");
		ArrayList<TimeBooking> trainPriorities = funTrainPrioritisation2(booking1, booking2);
		TimeBooking priorityBooking = trainPriorities.get(0);
		TimeBooking nonPriorityBooking = trainPriorities.get(1);
		
	//Adjsut the non priority train bookings
		//traceln("Adjusting the non priority train bookings");
		funNodeBookingAdjust3(priorityBooking, nonPriorityBooking);
/*ALCODEEND*/}

ArrayList<TimeBooking> funFindTBNtoAdjustFollowingTrains(TimeBooking priorityBooking,TimeBooking nonPriorityBooking)
{/*ALCODESTART::1715764802743*/
//Finding the node to adjust for following trains

	//Finding the node booking to adjust (going backwards in the non priority trains list of TBs and selecting the first nodeTB)
		
		ArrayList<TimeBooking> adjustmentTBs = new ArrayList();//the arraylist of timebookings to be returned and used for adjustmnets
		TimeBooking nonPriorityTBNodeAdjust = new TimeBooking();//the node booking to adjust
		TimeBooking prioritySegmentBooking = new TimeBooking();//the priority train's relevant segment booking for the node TB to adjsut
		TimeBooking nonPrioritisedBooking = new TimeBooking();//the booking that will not be prioritised
		TimeBooking prioritisedBooking = new TimeBooking();//the booking that will be prioritised
		
		if (nonPriorityBooking.parTrain.parIndex > priorityBooking.parTrain.parIndex){
			nonPrioritisedBooking = nonPriorityBooking;
			prioritisedBooking = priorityBooking;
			traceln("   prioritised train is "+prioritisedBooking.parTrain);
			traceln("   nonPrioritised train is "+nonPrioritisedBooking.parTrain);
		}else if (nonPriorityBooking.parTrain.parIndex < priorityBooking.parTrain.parIndex) {
			nonPrioritisedBooking = priorityBooking;
			prioritisedBooking = nonPriorityBooking;
			traceln("   prioritised train is "+prioritisedBooking.parTrain);
			traceln("   nonPrioritised train is "+nonPrioritisedBooking.parTrain);
		}
		int nonPrioritisedTBIndex = nonPrioritisedBooking.parTrain.colTB.indexOf(nonPrioritisedBooking);
		int prioritisedTBIndex = prioritisedBooking.parTrain.colTB.indexOf(prioritisedBooking);
		
		outerLoop:
		for (int i = nonPrioritisedTBIndex-1; i > -1; i--){
			TimeBooking booking = nonPrioritisedBooking.parTrain.colTB.get(i);
			
			if (booking.parResource instanceof Node_Loop){
				//Get the relevant thoeretical waiting time to calculate the theoretical end time if "booking" were to be adjusted
					double theoreticalWaitingDuration = 0.0;
					if (prioritisedBooking.varTimeStart > nonPrioritisedBooking.varTimeStart){
						traceln(" Overtaking trains");
						theoreticalWaitingDuration = prioritisedBooking.varTimeStart-nonPrioritisedBooking.varTimeStart;
						traceln("   booking start time is "+booking.varTimeStart);
					}
					else {
						traceln(" Following trains");
						theoreticalWaitingDuration = prioritisedBooking.varTimeEnd-nonPrioritisedBooking.varTimeStart;
					}
					double bookingEndTheoretical = booking.varTimeEnd + theoreticalWaitingDuration + 0.0333;
					traceln("  theoretical end time is "+bookingEndTheoretical);
				//check availability of the node
					Node_Loop nodeToCheck = (Node_Loop)booking.parResource;
				//Create a list of time bookings where a train uses this node (i.e the difference in start and end time is > 0)
					ArrayList<TimeBooking> relevantBookings = new ArrayList();
					ArrayList<TimeBooking> sortedRelevantBookings = new ArrayList();
					for (int j = 0; j < nodeToCheck.colTBN.size(); j++){
						TimeBooking possibleRelevantBooking = nodeToCheck.colTBN.get(j);
						if (possibleRelevantBooking.varTimeEnd - possibleRelevantBooking.varTimeStart != 0.0
						&& booking.varTimeStart < possibleRelevantBooking.varTimeEnd
						&& booking != possibleRelevantBooking){
							relevantBookings.add(possibleRelevantBooking);
						}
					}
				//Sort the list into ascending order by start time
					sortedRelevantBookings = (ArrayList<TimeBooking>)sortAscending(relevantBookings, b -> b.varTimeStart);
					traceln("  sorted relevant bookings are: "+sortedRelevantBookings);
				//Identify and remove bookings that will not cause conflict if non priority train theoretically used the loop
				//Identify
					ArrayList<TimeBooking> irrelevantBookings = new ArrayList();
					for (int k = 0; k < sortedRelevantBookings.size(); k++){
						traceln("  k is: "+sortedRelevantBookings.get(k));
						if (bookingEndTheoretical < sortedRelevantBookings.get(k).varTimeStart){
							traceln("   adding to irellevant bookings: "+sortedRelevantBookings.get(k));
							irrelevantBookings.add(sortedRelevantBookings.get(k));
						}
					}
				//Remove irrelevant bookings
					for (int k = 0; k < irrelevantBookings.size(); k++){
						traceln("   removing irellevant booking: "+irrelevantBookings.get(k));
						sortedRelevantBookings.remove(irrelevantBookings.get(k));
					}
				//If the node being considered is the first or last node then that one will need to be adjusted	
					if (nodeToCheck == pop_Nodes.get(0) || nodeToCheck == pop_Nodes.get(pop_Nodes.size()-1)){
						nonPriorityTBNodeAdjust = booking;
						adjustmentTBs.add(nonPriorityTBNodeAdjust);
						TimeBooking referenceBooking = new TimeBooking();
						referenceBooking.varTimeEnd = bookingEndTheoretical;
						referenceBooking.parTrain = prioritisedBooking.parTrain;
						adjustmentTBs.add(referenceBooking);
						traceln("   TBN to addjsut is "+nonPriorityTBNodeAdjust+" with node index "
							+((Node_Loop)nonPriorityTBNodeAdjust.parResource).parNodeNumber);
						break outerLoop;
					}
				
				//If the list is empty it means that the node is available at that time. If not another node needs to be considered
					else if (sortedRelevantBookings.isEmpty()){
						nonPriorityTBNodeAdjust = booking;
						adjustmentTBs.add(nonPriorityTBNodeAdjust);
						TimeBooking referenceBooking = new TimeBooking();
						referenceBooking.varTimeEnd = bookingEndTheoretical;
						referenceBooking.parTrain = prioritisedBooking.parTrain;
						adjustmentTBs.add(referenceBooking);
						traceln("   TBN to addjsut is "+nonPriorityTBNodeAdjust+" with node index "
							+((Node_Loop)nonPriorityTBNodeAdjust.parResource).parNodeNumber);
						break outerLoop;
					}
					traceln("   Sorted relevant bookings: "+sortedRelevantBookings);
					traceln(" ");		
			}	
		}
		
	return adjustmentTBs;
/*ALCODEEND*/}

double funUpdateLoopWaitingTimes(Train train)
{/*ALCODESTART::1717662459894*/
for (int i = 0; i < train.colTBN.size(); i++){
	//traceln("getting waiting time of train "+train.parDirection+train.parIndex+" for loop"+i);
	TimeBooking tbn = train.colTBN.get(i);
	int j = ((Node_Loop)tbn.parResource).parNodeNumber;
	Double waitingTime =  (tbn.varTimeEnd-tbn.varTimeStart);
	//traceln(" current loop waiting time "+waitingTime);
	DataItem waitingTimeTotal = colLoopWaitingData.get(j);
	Double waitingTimeTotalValue = waitingTimeTotal.getValue();
	//traceln(" current loop total waiting time is "+colLoopWaitingData.get(j));
	waitingTimeTotal.setValue(waitingTimeTotalValue + waitingTime);
	//traceln(" new loop total waiting time is "+colLoopWaitingData.get(j));
}
/*ALCODEEND*/}

double funInitialiseWaitingData()
{/*ALCODESTART::1717663228991*/
for (int i = 0; i < pop_Nodes.size(); i++){
	DataItem loopWaitingData = new DataItem();
	loopWaitingData.setValue(0.0);
	colLoopWaitingData.add(loopWaitingData);
}
for(int i = 0; i < colLoopWaitingData.size(); i++){
	int red = getDefaultRandomGenerator().nextInt(256);
	int green = getDefaultRandomGenerator().nextInt(256);
	int blue = getDefaultRandomGenerator().nextInt(256);
	if(parDisplayGraphs == true){
		chartLoopWaitingTime.addDataItem(colLoopWaitingData.get(i), "Loop"+i, new Color(red, green, blue));
	}
}
/*ALCODEEND*/}

