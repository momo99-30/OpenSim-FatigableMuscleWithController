/*
LinGolgiTendonOrgan
@author Morgane Garaudet
*/

#pragma once
#include <OpenSim/Simulation/Control/Controller.h>
using namespace SimTK;

namespace OpenSim {


    class TugOfWarController : public Controller {
        OpenSim_DECLARE_CONCRETE_OBJECT(TugOfWarController, Controller);

        // This section contains methods that can be called in this controller class.
    public:
        /**
         * Constructor
         *
         * @param aModel Model to be controlled
         * @param aKp Position gain by which the position error will be multiplied
         */
         /////////////////////////////////////////////////////////////
         // 2) Add a parameter aKv for velocity gain to the         //
         //    argument list for this function.  Also add this      //
         //    parameter to the initializer list below so that a    //
         //    new member variable kv is initialized to the value   //
         //    of aKv.  Remember to add a line describing aKv in    //
         //    the comment above (below the line describing aKp).   //
         /////////////////////////////////////////////////////////////
        TugOfWarController(double aKp);

        /**
         * This function is called at every time step for every actuator.
         *
         * @param s Current state of the system
         * @param controls Controls being calculated
         */
        void computeControls(const SimTK::State& s, SimTK::Vector& controls) const;

        // This section contains the member variables of this controller class.
    private:

        /** Position gain for this controller */
        double kp;

        //////////////////////////////////////////////////////////////
        // 7) Add a member variable kv that is the velocity gain    //
        //    for this controller.                                  //
        //////////////////////////////////////////////////////////////

    }
    ;
}