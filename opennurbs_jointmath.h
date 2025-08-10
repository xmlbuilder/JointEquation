#pragma once

class ON_JointMatrix
{
  static bool NormalizeQuaternion(
    double Q[4]);


  static ON_Xform CalcJointFree(
    double Q[4], // Euler parameters
    double T[3]  // Translation along x, y, z-axis
  );


  static ON_Xform CalcJointFreeBryant(
    double Q[3], // Rotation angle around x,y,z-axis
    double T[3]  // Translation along x,y,z-axis
  );


  static ON_Xform CalcJointFreeEuler(
    double Q[3], // Rotation angle around x, y, z-axis
    double T[3]  // Translation along x,y,z-axis
  );


  static ON_Xform CalcJointFreeRotDisp(
    double Q[4], // Euler parameters
    double T[3]  // Translation along x,y,z-axis
  );


  static ON_Xform CalcJointRevo(
    double Q // Rotation angle around x-axis
  );
 

  static ON_Xform CalcJointSphere(
    double Q[4] // Euler parameters
  );


  static ON_Xform CalcJointSphereEuler(
    double Q[3] // Rotation angle around x, y, z-axis
    );


  //ZYX Euler
  static ON_Xform CalcJointSphereBryant(
    double Q[3] // Rotation angle around x,y,z axis
  );


  static ON_Xform CalcJointUniveral(
    double Q[2] // rotation around the x-axis followed by rotation around the new y-axis
  );


  static ON_Xform CalcJointTrans(
    double Q // Translation along x-axis
  );


  static ON_Xform CalcJointCylinder(
    double Q, // Rotation angle around x-axis
    double T  // Translation along x-axis
  );


  static ON_Xform CalcJointPlanar(
    double Q,   // Rotation angle around x-axis
    double T[2] // Translation along y,z-axis
  );


  static ON_Xform CalcJointTransUniveral(
    double Q[2], // Rotation angle around y,z-axis
    double T     // Translation distance
  );


  static ON_Xform CalcJointUniveralTrans(
    double Q[2], // Rotation angle around y,z-axis
    double T     // Translation along x-axis
  );


  static ON_Xform CalcJointTransRevo(
    double Q, // Rotation angle around y-axis
    double T  //  Translation along x-axis
  );


  static ON_Xform CalcJointRevoTrans(
    double Q, // Rotation angle around y-axis
    double T  //  Translation along x-axis
  );

};

