#include "omni_velocity_controller.h"

//========================= YOUR CODE HERE =========================
// Instructions: implement all the functions that you have declared
//               in the header file.

// <summary>
// Creates a new 'OmniVelocityController' using default values for the class fields.
// </summary>
OmniVelocityController::OmniVelocityController()
{
    this->linearMaxVelocity = 0.0;
    this->linearDistanceErrorTolerance = 0.0;
    this->angularMaxVelocity = 0.0;
    this->angularDistanceErrorTolerance = 0.0;
    this->linearMotionComplete = false;
    this->angularMotionComplete = false;
}

// <summary>
// Creates a new 'OmniVelocityController' using the given parameters.
// </summary>
// <param name='linearMaxVelocity'>Maximum allowed linear velocity.</param>
// <param name='angularMaxVelocity'>Maximum allowed angular velocity.</param>
// <param name='maxLinearAcceleration'>Maximum allowed linear acceleration.</param>
// <param name='linearTolerance'>Distance error tolerance between an actual pose and a desired pose.</param>
// <param name='angularTolerance'>Angular error tolerance between an actual pose and a desired pose.</param>
OmniVelocityController::OmniVelocityController(double linearMaxVelocity, double angularMaxVelocity, double maxLinearAcceleration, double linearTolerance, double angularTolerance)
{
    this->linearMaxVelocity = linearMaxVelocity;
    this->linearDistanceErrorTolerance = linearTolerance;
    this->linearMaxAcceleration = maxLinearAcceleration;
    this->angularMaxVelocity = angularMaxVelocity;
    this->angularDistanceErrorTolerance = angularTolerance;

    //vmax + 0
    //-------- tmin = l
    //   2
    //where tmin is the time that the robot needs in order to slow down from maximum speed to 0,
    //and l is the distance at which it needs to start decelerating
    //
    //   vmax * vmax          v^2max
    //=> ----------- = l  => ------ = l
    //      2amax             2amax
    this->decelerationPoint = this->linearMaxVelocity * linearMaxVelocity / (2 * this->linearMaxAcceleration);

    this->actualMaxVelocity = this->linearMaxVelocity;
    this->accelerationRate = 10.0;
    this->linearMotionComplete = false;
    this->angularMotionComplete = false;
}

// <summary>
// Sets the desired pose using the input parameter.
// </summary>
// <param name='pose'>A desired pose.</param>
void OmniVelocityController::setTargetPose(const Pose& pose)
{
    this->desiredPose = pose;
    this->linearMotionComplete = false;
    this->angularMotionComplete = false;
}

// <summary>
// Checks if the motion of the robot is over.
// </summary>
// <returns>
// True if both the linear and angular motions are over and false otherwise.
// </returns>
bool OmniVelocityController::isTargetReached() const
{
    return this->linearMotionComplete && this->angularMotionComplete;
}

// <summary>
// Computes velocities that the robot should use while moving.
// </summary>
// <param name='actual_pose'>The current pose of the robot.</param>
// <returns>
// A 'Velocity' object with the desired motion parameters.
// </returns>
Velocity OmniVelocityController::computeVelocity(const Pose& actual_pose)
{
    //we find the magnitude of the current linear and angular displacement
    double linearDistance = this->getDistance(this->desiredPose, actual_pose);
    double angularDistance = this->getShortestAngle(this->desiredPose.theta, actual_pose.theta);

    //if the robot has reached the desired pose, we mark the motion as complete and return a zero velocity vector
    if(linearDistance < this->linearDistanceErrorTolerance && angularDistance < this->angularDistanceErrorTolerance)
    {
        this->linearMotionComplete = true;
        this->angularMotionComplete = true;
        this->actualMaxVelocity = this->linearMaxVelocity;
        return Velocity(0.0, 0.0, 0.0);
    }

    //we calculate the current linear displacement of the robot
    double dx = this->desiredPose.x - actual_pose.x;
    double dy = this->desiredPose.y - actual_pose.y;

    //decrease max velocity
    if(this->decelerationPoint > linearDistance)
        this->actualMaxVelocity -= this->linearMaxAcceleration / this->accelerationRate;

    //the angle of the velocity vector is equal to the displacement angle
    //minus the current pose of the robot
    double angularDifference = atan2(dy, dx) - actual_pose.theta;

    //we calculate the x and y components of the linear velocity
    double desiredXVelocity = this->actualMaxVelocity * cos(angularDifference);
    double desiredYVelocity = this->actualMaxVelocity * sin(angularDifference);

    double velocityMagnitude = sqrt(desiredXVelocity * desiredXVelocity + desiredYVelocity * desiredYVelocity);

    //v * t = d (linear speed * time = linear distance)
    //omega * t = theta (angular speed * time = angular distance)
    //     d              d                     theta * v
    //t = --- => omega * --- = theta => omega = ---------
    //     v              v                         d
    double desiredAngularVelocity = angularDistance * velocityMagnitude / linearDistance;

    return Velocity(desiredXVelocity, desiredYVelocity, desiredAngularVelocity);
}

//==================================================================
