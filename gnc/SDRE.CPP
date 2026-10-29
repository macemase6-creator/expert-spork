/*
 * Simulation unity-build wrapper for the shared M4 SDRE implementation.
 *
 * Keep this file deliberately free of simulation-only aerodynamic or mass
 * calculations.  The firmware project builds the same four implementation
 * files through thin wrappers in Core/Src; including them here gives MEX and
 * sim602_cli exactly the same operating-point builder and controller law.
 */
#include "../common/gnc/sdre/SDRE_M4_Aero.cpp"
#include "../common/gnc/sdre/SDRE_M4_Tables.cpp"
#include "../common/gnc/sdre/SDRE_M4_Model.cpp"
#include "../common/gnc/sdre/SDRE.cpp"
