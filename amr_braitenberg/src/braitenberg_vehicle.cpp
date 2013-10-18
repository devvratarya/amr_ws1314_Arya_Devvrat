//Description : BraitenbergVehicle file for vehicle object creation and speed calculations
//Modifier : Devvrat Arya
//Change Descriptio : Added code for speed calculations
//Modification Date : October 18, 2013
#include "braitenberg_vehicle.h"
#include <iostream>

using namespace std;

BraitenbergVehicle::BraitenbergVehicle()
: type_(TYPE_A)
, factor1_(1.0)
, factor2_(0.0)
{
}

BraitenbergVehicle::BraitenbergVehicle(Type type, float factor1, float factor2)
: type_(type)
, factor1_(factor1)
, factor2_(factor2)
{
}

void BraitenbergVehicle::computeWheelSpeeds(float left_in, float right_in, float& left_out, float& right_out)
{
  //==================== YOUR CODE HERE ====================
  // Instructions: based on the input from the left and
  //               right sonars compute the speeds of the
  //               wheels. Use the parameters stored in the
  //               private fields type_, factor1_, and
  //               factor2_ (if applicable).
	//calculating vehicle speed according the vehicle Type(A,B or C) ad range recieved from sonar
	Type Veh_type = type_;
	switch(Veh_type){
		case TYPE_A:
				left_out = left_in*factor1_;
				right_out = right_in*factor1_;
				break;
		case TYPE_B:
				left_out = right_in*factor1_;
				right_out = left_in*factor1_;
				break;
		case TYPE_C:
				left_out = right_in*factor1_+ left_in*factor2_;
				right_out = left_in*factor1_ + right_in*factor2_;
				break;
		default:
				std::cout<<"No such braitenberg vehicle found.\n";
				break;
	}

  // =======================================================
}

