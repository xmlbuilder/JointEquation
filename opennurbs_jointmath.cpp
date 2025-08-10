#include "opennurbs.h"

bool ON_JointMatrix::NormalizeQuaternion(double Q[4])
{
  try
  {
    double len = Q[0] * Q[0] + Q[1] * Q[1] + Q[2] * Q[2] + Q[3] * Q[3];
    if (len < 1.0E-6)
    {
      Q[0] = 1.0;
      Q[1] = 0.0;
      Q[2] = 0.0;
      Q[3] = 0.0;
      return false;
    }

    double Identity = sqrt(2.0 / len);
    Q[0] = Identity * Q[0];
    Q[1] = Identity * Q[1];
    Q[2] = Identity * Q[2];
    Q[3] = Identity * Q[3];
    return true;
  }
  catch (...)
  {
    Q[0] = 1.0;
    Q[1] = 0.0;
    Q[2] = 0.0;
    Q[3] = 0.0;
  }
  return false;
}

ON_Xform ON_JointMatrix::CalcJointFree(double Q[4], double T[3])
{
  ON_Xform form = ON_Xform::IdentityTransformation;
  form.m_xform[0][0] = Q[0] * Q[0] + Q[1] * Q[1] - 1.0;
  form.m_xform[0][1] = Q[1] * Q[2] - Q[0] * Q[3];
  form.m_xform[0][2] = Q[1] * Q[3] + Q[0] * Q[2];
  form.m_xform[1][0] = Q[1] * Q[2] + Q[0] * Q[3];
  form.m_xform[1][1] = Q[0] * Q[0] + Q[2] * Q[2] - 1.0;
  form.m_xform[1][2] = Q[2] * Q[3] - Q[0] * Q[1];
  form.m_xform[2][0] = Q[1] * Q[3] - Q[0] * Q[2];
  form.m_xform[2][1] = Q[2] * Q[3] + Q[0] * Q[1];
  form.m_xform[2][2] = Q[0] * Q[0] + Q[3] * Q[3] - 1.0;
  form.m_xform[0][3] = T[0];
  form.m_xform[1][3] = T[1];
  form.m_xform[2][3] = T[2];
  return form;
}


ON_Xform ON_JointMatrix::CalcJointFreeBryant(double Q[3], double T[3])
{
  ON_Xform form = ON_Xform::IdentityTransformation;
  form.m_xform[0][0] = cos(Q[1]) * cos(Q[2]);
  form.m_xform[0][1] = -cos(Q[1]) * sin(Q[2]);
  form.m_xform[0][2] = sin(Q[1]);
  form.m_xform[1][0] = cos(Q[0]) * sin(Q[2]) + sin(Q[0]) * sin(Q[1]) * cos(Q[2]);
  form.m_xform[1][1] = cos(Q[0]) * cos(Q[2]) - sin(Q[0]) * sin(Q[1]) * sin(Q[2]);
  form.m_xform[1][2] = -sin(Q[0]) * cos(Q[1]);
  form.m_xform[2][0] = sin(Q[0]) * sin(Q[2]) - cos(Q[0]) * sin(Q[1]) * cos(Q[2]);
  form.m_xform[2][1] = sin(Q[0]) * cos(Q[2]) + cos(Q[0]) * sin(Q[1]) * sin(Q[2]);
  form.m_xform[2][2] = cos(Q[0]) * cos(Q[1]);
  form.m_xform[0][3] = T[0];
  form.m_xform[1][3] = T[1];
  form.m_xform[2][3] = T[2];
  return form;
}


ON_Xform ON_JointMatrix::CalcJointFreeEuler(double Q[3], double T[3])
{

  ON_Xform form = ON_Xform::IdentityTransformation;
  form.m_xform[0][0] = cos(Q[1]);
  form.m_xform[0][1] = sin(Q[1]) * sin(Q[2]);
  form.m_xform[0][2] = sin(Q[1]) * cos(Q[2]);
  form.m_xform[1][0] = sin(Q[0]) * sin(Q[1]);
  form.m_xform[1][1] = cos(Q[0]) * cos(Q[2]) - sin(Q[0]) * cos(Q[1]) * sin(Q[2]);
  form.m_xform[1][2] = -cos(Q[0]) * sin(Q[2]) - sin(Q[0]) * cos(Q[1]) * cos(Q[2]);
  form.m_xform[2][0] = -cos(Q[0]) * sin(Q[1]);
  form.m_xform[2][1] = sin(Q[0]) * cos(Q[2]) + cos(Q[0]) * cos(Q[1]) * sin(Q[2]);
  form.m_xform[2][2] = -sin(Q[0]) * sin(Q[2]) + cos(Q[0]) * cos(Q[1]) * cos(Q[2]);
  form.m_xform[0][3] = T[0];
  form.m_xform[1][3] = T[1];
  form.m_xform[2][3] = T[2];

  return form;
}

ON_Xform ON_JointMatrix::CalcJointFreeRotDisp(double Q[4], double T[3])
{
  ON_Xform form = ON_Xform::IdentityTransformation;
  NormalizeQuaternion(Q);
  form.m_xform[0][0] = Q[0] * Q[0] + Q[1] * Q[1] - 1.0;
  form.m_xform[0][1] = Q[1] * Q[2] - Q[0] * Q[3];
  form.m_xform[0][2] = Q[1] * Q[3] + Q[0] * Q[2];
  form.m_xform[1][0] = Q[1] * Q[2] + Q[0] * Q[3];
  form.m_xform[1][1] = Q[0] * Q[0] + Q[2] * Q[2] - 1.0;
  form.m_xform[1][2] = Q[2] * Q[3] - Q[0] * Q[1];
  form.m_xform[2][0] = Q[1] * Q[3] - Q[0] * Q[2];
  form.m_xform[2][1] = Q[2] * Q[3] + Q[0] * Q[1];
  form.m_xform[2][2] = Q[0] * Q[0] + Q[3] * Q[3] - 1.0;
  form.m_xform[0][3] = form.m_xform[0][0] * T[0] + form.m_xform[0][1] * T[1] + form.m_xform[0][2] * T[2];
  form.m_xform[1][3] = form.m_xform[1][0] * T[0] + form.m_xform[1][1] * T[1] + form.m_xform[1][2] * T[2];
  form.m_xform[2][3] = form.m_xform[2][0] * T[0] + form.m_xform[2][1] * T[1] + form.m_xform[2][2] * T[2];

  return form;
}

ON_Xform ON_JointMatrix::CalcJointRevo(double Q)
{
  ON_Xform form = ON_Xform::IdentityTransformation;
  form.m_xform[1][1] = cos(Q);
  form.m_xform[1][2] = -sin(Q);
  form.m_xform[2][1] = sin(Q);
  form.m_xform[2][2] = cos(Q);

  return form;
}

ON_Xform ON_JointMatrix::CalcJointSphere(double Q[4])
{
  ON_Xform form = ON_Xform::IdentityTransformation;
  form.m_xform[0][0] = Q[0] * Q[0] + Q[1] * Q[1] - 1.0;
  form.m_xform[0][1] = Q[1] * Q[2] - Q[0] * Q[3];
  form.m_xform[0][2] = Q[1] * Q[3] + Q[0] * Q[2];
  form.m_xform[1][0] = Q[1] * Q[2] + Q[0] * Q[3];
  form.m_xform[1][1] = Q[0] * Q[0] + Q[2] * Q[2] - 1.0;
  form.m_xform[1][2] = Q[2] * Q[3] - Q[0] * Q[1];
  form.m_xform[2][0] = Q[1] * Q[3] - Q[0] * Q[2];
  form.m_xform[2][1] = Q[2] * Q[3] + Q[0] * Q[1];
  form.m_xform[2][2] = Q[0] * Q[0] + Q[3] * Q[3] - 1.0;
  return form;
}

ON_Xform ON_JointMatrix::CalcJointSphereEuler(double Q[3])
{
  ON_Xform form = ON_Xform::IdentityTransformation;
  form.m_xform[0][0] = cos(Q[1]);
  form.m_xform[0][1] = sin(Q[1]) * sin(Q[2]);
  form.m_xform[0][2] = sin(Q[1]) * cos(Q[2]);
  form.m_xform[1][0] = sin(Q[0]) * sin(Q[1]);
  form.m_xform[1][1] = cos(Q[0]) * cos(Q[2]) - sin(Q[0]) * cos(Q[1]) * sin(Q[2]);
  form.m_xform[1][2] = -cos(Q[0]) * sin(Q[2]) - sin(Q[0]) * cos(Q[1]) * cos(Q[2]);
  form.m_xform[2][0] = -cos(Q[0]) * sin(Q[1]);
  form.m_xform[2][1] = sin(Q[0]) * cos(Q[2]) + cos(Q[0]) * cos(Q[1]) * sin(Q[2]);
  form.m_xform[2][2] = -sin(Q[0]) * sin(Q[2]) + cos(Q[0]) * cos(Q[1]) * cos(Q[2]);
  return form;
}

ON_Xform ON_JointMatrix::CalcJointSphereBryant(double Q[3])
{
  ON_Xform form = ON_Xform::IdentityTransformation;
  form.m_xform[0][0] = cos(Q[1]) * cos(Q[2]);
  form.m_xform[0][1] = -cos(Q[1]) * sin(Q[2]);
  form.m_xform[0][2] = sin(Q[1]);
  form.m_xform[1][0] = cos(Q[0]) * sin(Q[2]) + sin(Q[0]) * sin(Q[1]) * cos(Q[2]);
  form.m_xform[1][1] = cos(Q[0]) * cos(Q[2]) - sin(Q[0]) * sin(Q[1]) * sin(Q[2]);
  form.m_xform[1][2] = -sin(Q[0]) * cos(Q[1]);
  form.m_xform[2][0] = sin(Q[0]) * sin(Q[2]) - cos(Q[0]) * sin(Q[1]) * cos(Q[2]);
  form.m_xform[2][1] = sin(Q[0]) * cos(Q[2]) + cos(Q[0]) * sin(Q[1]) * sin(Q[2]);
  form.m_xform[2][2] = cos(Q[0]) * cos(Q[1]);
  return form;
}

ON_Xform ON_JointMatrix::CalcJointUniveral(double Q[2])
{

  ON_Xform form = ON_Xform::IdentityTransformation;
  form.m_xform[0][0] = cos(Q[1]);
  form.m_xform[0][1] = 0.0;
  form.m_xform[0][2] = sin(Q[1]);
  form.m_xform[1][0] = sin(Q[0]) * sin(Q[1]);
  form.m_xform[1][1] = cos(Q[0]);
  form.m_xform[1][2] = -sin(Q[0]) * cos(Q[1]);
  form.m_xform[2][0] = -cos(Q[0]) * sin(Q[1]);
  form.m_xform[2][1] = sin(Q[0]);
  form.m_xform[2][2] = cos(Q[0]) * cos(Q[1]);
  return form;
}


ON_Xform ON_JointMatrix::CalcJointTrans(double Q)
{
  ON_Xform form = ON_Xform::IdentityTransformation;
  form.m_xform[0][3] = Q;
  return form;
}

ON_Xform ON_JointMatrix::CalcJointCylinder(double Q, double T)
{
  ON_Xform form = ON_Xform::IdentityTransformation;
  form.m_xform[1][1] = cos(Q);
  form.m_xform[1][2] = -sin(Q);
  form.m_xform[2][1] = sin(Q);
  form.m_xform[2][2] = cos(Q);
  form.m_xform[0][3] = T;
  return form;
}


ON_Xform ON_JointMatrix::CalcJointPlanar(double Q, double T[2])
{
  ON_Xform form = ON_Xform::IdentityTransformation;
  form.m_xform[1][1] = cos(Q);
  form.m_xform[1][2] = -sin(Q);
  form.m_xform[2][1] = sin(Q);
  form.m_xform[2][2] = cos(Q);
  form.m_xform[1][3] = T[0];
  form.m_xform[2][3] = T[1];
  return form;
}

ON_Xform ON_JointMatrix::CalcJointTransUniveral(double Q[2], double T)
{
  ON_Xform form = ON_Xform::IdentityTransformation;
  form.m_xform[0][0] = cos(Q[0]) * cos(Q[1]);
  form.m_xform[0][1] = -cos(Q[0]) * sin(Q[1]);
  form.m_xform[0][2] = sin(Q[0]);
  form.m_xform[1][0] = sin(Q[1]);
  form.m_xform[1][1] = cos(Q[1]);
  form.m_xform[1][2] = 0.0;
  form.m_xform[2][0] = -sin(Q[0]) * cos(Q[1]);
  form.m_xform[2][1] = sin(Q[0]) * sin(Q[1]);
  form.m_xform[2][2] = cos(Q[0]);
  form.m_xform[0][3] = T;
  return form;
}

ON_Xform ON_JointMatrix::CalcJointUniveralTrans(double Q[2], double T)
{
  ON_Xform form = ON_Xform::IdentityTransformation;
  form.m_xform[0][0] = cos(Q[0]) * cos(Q[1]);
  form.m_xform[0][1] = -sin(Q[1]);
  form.m_xform[0][2] = sin(Q[0]) * cos(Q[1]);
  form.m_xform[1][0] = cos(Q[0]) * sin(Q[1]);
  form.m_xform[1][1] = cos(Q[1]);
  form.m_xform[1][2] = sin(Q[0]) * sin(Q[1]);
  form.m_xform[2][0] = -sin(Q[0]);
  form.m_xform[2][1] = 0.0;
  form.m_xform[2][2] = cos(Q[0]);
  form.m_xform[0][3] = T * cos(Q[0]) * cos(Q[1]);
  form.m_xform[1][3] = T * cos(Q[0]) * sin(Q[1]);
  form.m_xform[2][3] = -T * sin(Q[0]);
  return form;
}

ON_Xform ON_JointMatrix::CalcJointTransRevo(double Q, double T)
{
  ON_Xform form = ON_Xform::IdentityTransformation;
  form.m_xform[0][0] = cos(Q);
  form.m_xform[0][2] = sin(Q);
  form.m_xform[2][0] = -sin(Q);
  form.m_xform[2][2] = cos(Q);
  form.m_xform[0][3] = T;
  return form;
}

ON_Xform ON_JointMatrix::CalcJointRevoTrans(double Q, double T)
{
  ON_Xform form = ON_Xform::IdentityTransformation;
  form.m_xform[0][0] = cos(Q);
  form.m_xform[0][2] = sin(Q);
  form.m_xform[2][0] = -sin(Q);
  form.m_xform[2][2] = cos(Q);
  form.m_xform[0][3] = T * cos(Q);
  form.m_xform[2][3] = -T * sin(Q);
  return form;
}
