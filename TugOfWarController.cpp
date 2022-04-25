#include <OpenSim/OpenSim.h>
#include "TugOfWarController.h"


#define AMP 0.0014
#define MAX_Z 0.0142
#define MIN_Z  -0.002
#define RISE_T 0.12 


using namespace OpenSim;
using namespace SimTK;

TugOfWarController::TugOfWarController(double aKp) : Controller(), kp(aKp)
{
}

double desiredModelZPosition(double t) {
    // z(t) = 0.15 sin( pi * t )
    return 0.15 * sin(Pi * t);
}
//////////////////////////////////////////////////////////////////////
// 1) Add a function to compute the desired velocity of the model   //
//    in the z direction.                                           //
//////////////////////////////////////////////////////////////////////
//______________________________________________________________________________
/**
 * The controller will try to make the model follow this acceleration
 * in the z direction.
 */
double desiredModelZAcceleration(double t) {
    // z''(t) = -(0.15*pi^2) sin( pi * t )
    return -0.15 * Pi * Pi * sin(Pi * t);
}

void TugOfWarController::computeControls(const SimTK::State& s, SimTK::Vector& controls) const
{
    // Get the current time in the simulation.
    double t = s.getTime();

    // Read the mass of the block.
    double blockMass = getModel().getBodySet().get("block").getMass();

    // Get pointers to each of the muscles in the model.
    auto leftMuscle = dynamic_cast<const Muscle*>  (&getActuatorSet().get(0));
    auto rightMuscle = dynamic_cast<const Muscle*> (&getActuatorSet().get(1));

    // Compute the desired position of the block in the tug-of-war
    // model.
    double zdes = desiredModelZPosition(t);

    //////////////////////////////////////////////////////////////
    // 3) Compute the desired velocity of the block in the tug- //
    //    of-war model.  Create a new variable zdesv to hold    //
    //    this value.                                           //
    //////////////////////////////////////////////////////////////

    // Compute the desired acceleration of the block in the tug-
    // of-war model.
    double zdesa = desiredModelZAcceleration(t);

    // Get the z translation coordinate in the model.
    const Coordinate& zCoord = _model->getCoordinateSet().
        get("blockToGround_zTranslation");

    // Get the current position of the block in the tug-of-war
    // model.
    double z = zCoord.getValue(s);

    //////////////////////////////////////////////////////////////
    // 4) Get the current velocity of the block in the tug-of-  //
    //    war model.  Create a new variable zv to hold this     //
    //    value.                                                //
    //////////////////////////////////////////////////////////////

    // Compute the correction to the desired acceleration arising
    // from the deviation of the block's current position from its
    // desired position (this deviation is the "position error").
    double pErrTerm = kp * (zdes - z);

    //////////////////////////////////////////////////////////////
    // 5) Compute the correction to the desired acceleration    //
    //     arising from the deviation of the block's current    //
    //     velocity from its desired velocity (this deviation   //
    //     is the "velocity error").  Create a new variable     //
    //     vErrTerm to hold this value.                         //
    //////////////////////////////////////////////////////////////

    //////////////////////////////////////////////////////////////
    // 6) In the computation of desAcc below, add the velocity  //
    //    error term you created in item #5 above.  Please      //
    //    update the comment for desAcc below to reflect this   //
    //    change.                                               //
    //////////////////////////////////////////////////////////////

    // Compute the total desired acceleration based on the initial
    // desired acceleration plus the position error term we
    // computed above.
    double desAcc = zdesa + pErrTerm;

    // Compute the desired force on the block as the mass of the
    // block times the total desired acceleration of the block.
    double desFrc = desAcc * blockMass;

    // Get the maximum isometric force for the left muscle.
    double FoptL = leftMuscle->getMaxIsometricForce();

    // Get the maximum isometric force for the right muscle.
    double FoptR = rightMuscle->getMaxIsometricForce();

    // If desired force is in direction of one muscle's pull
    // direction, then set that muscle's control based on desired
    // force.  Otherwise, set the muscle's control to zero.

    double leftControl = leftMuscle->getMinControl(),
        rightControl = rightMuscle->getMinControl();
    if (desFrc < 0) {
        leftControl = std::abs(desFrc) / FoptL;
    }
    else if (desFrc > 0) {
        rightControl = std::abs(desFrc) / FoptR;
    }
    // Don't allow any control value to be greater than one.
    if (leftControl > leftMuscle->getMaxControl())
        leftControl = leftMuscle->getMaxControl();
    if (rightControl > rightMuscle->getMaxControl())
        rightControl = rightMuscle->getMaxControl();

    // Thelen muscle has only one control
    Vector muscleControl(1, leftControl);
    // Add in the controls computed for this muscle to the set of all model controls
    leftMuscle->addInControls(muscleControl, controls);
    // Specify control for other actuator (muscle) controlled by this controller
    muscleControl[0] = rightControl;
    rightMuscle->addInControls(muscleControl, controls);
}