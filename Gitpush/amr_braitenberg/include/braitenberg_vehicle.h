#ifndef BRAITENBERG_VEHICLE_H
#define BRAITENBERG_VEHICLE_H

#include <memory>

class BraitenbergVehicle
{

public:

  typedef std::unique_ptr<BraitenbergVehicle> UPtr;

  /** Braitenberg vehicle type. */
  enum Type
  {
    TYPE_A, ///< direct connections
    TYPE_B, ///< cross connections
    TYPE_C, ///< direct and cross connections
  };

  /** Default constructor creates a vehicle of type A with the connection
    * factor equal to 1. */
  BraitenbergVehicle();

  /** Construct a braitenberg vehicle of the desired type.
    *
    * @a factor2 has an effect only for vehicles of type C, therefore this
    * parameter may be omitted when constructing vehicles of other types. */
  BraitenbergVehicle(Type type, float factor1, float factor2 = 0.0);

  ~BraitenbergVehicle() { };

  /** Compute wheel speeds of the vehicle depending on the input from sonars. */
  void computeWheelSpeeds(float left_in, float right_in, float& left_out, float& right_out);

private:

  Type type_;
  float factor1_;
  float factor2_;

};

#endif /* BRAITENBERG_VEHICLE_H */

