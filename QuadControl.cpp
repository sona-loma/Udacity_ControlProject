#include "Common.h"
#include "QuadControl.h"

#include "Utility/SimpleConfig.h"

#include "Utility/StringUtils.h"
#include "Trajectory.h"
#include "BaseController.h"
#include "Math/Mat3x3F.h"

#ifdef __PX4_NUTTX
#include <systemlib/param/param.h>
#endif

void QuadControl::Init()
{
  BaseController::Init();

  // variables needed for integral control
  integratedAltitudeError = 0;
    
#ifndef __PX4_NUTTX
  // Load params from simulator parameter system
  ParamsHandle config = SimpleConfig::GetInstance();
   
  // Load parameters (default to 0)
  kpPosXY = config->Get(_config+".kpPosXY", 0);
  kpPosZ = config->Get(_config + ".kpPosZ", 0);
  KiPosZ = config->Get(_config + ".KiPosZ", 0);
     
  kpVelXY = config->Get(_config + ".kpVelXY", 0);
  kpVelZ = config->Get(_config + ".kpVelZ", 0);

  kpBank = config->Get(_config + ".kpBank", 0);
  kpYaw = config->Get(_config + ".kpYaw", 0);

  kpPQR = config->Get(_config + ".kpPQR", V3F());

  maxDescentRate = config->Get(_config + ".maxDescentRate", 100);
  maxAscentRate = config->Get(_config + ".maxAscentRate", 100);
  maxSpeedXY = config->Get(_config + ".maxSpeedXY", 100);
  maxAccelXY = config->Get(_config + ".maxHorizAccel", 100);

  maxTiltAngle = config->Get(_config + ".maxTiltAngle", 100);

  minMotorThrust = config->Get(_config + ".minMotorThrust", 0);
  maxMotorThrust = config->Get(_config + ".maxMotorThrust", 100);
#else
  // load params from PX4 parameter system
  //TODO
  param_get(param_find("MC_PITCH_P"), &Kp_bank);
  param_get(param_find("MC_YAW_P"), &Kp_yaw);
#endif
}

VehicleCommand QuadControl::GenerateMotorCommands(float collThrustCmd, V3F momentCmd)
{
  // Convert a desired 3-axis moment and collective thrust command to 
  //   individual motor thrust commands
  // INPUTS: 
  //   collThrustCmd: desired collective thrust [N]
  //   momentCmd: desired rotation moment about each axis [N m]
  // OUTPUT:
  //   set class member variable cmd (class variable for graphing) where
  //   cmd.desiredThrustsN[0..3]: motor commands, in [N]

  // HINTS: 
  // - you can access parts of momentCmd via e.g. momentCmd.x
  // You'll need the arm length parameter L, and the drag/thrust ratio kappa

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  


  //First, define the Forces orientation of the rotors by: F = M/L 
    float Thrust = collThrustCmd;               // This is the "up" movement  so it is a pure force
    float Roll = momentCmd.x / L;               // note: moment about x-axis = Roll movement   
    float Pitch = momentCmd.y / L;              // note: moment about y-axis = Pitch moevment 
    float Yaw = momentCmd.z / kappa;            // note: moment about z-axis = Yaw movement

    cmd.desiredThrustsN[0] = (Thrust + Roll + Pitch + Yaw) / 4.f; // dynamic moment equation of the "front left" rotor 
    cmd.desiredThrustsN[1] = (Thrust - Roll + Pitch - Yaw) / 4.f; // dynamic moment equation of the "front right" rotor
    cmd.desiredThrustsN[2] = (Thrust + Roll - Pitch + Yaw) / 4.f; // dynamic moment equation of the "rear lift" rotor 
    cmd.desiredThrustsN[3] = (Thrust - Roll - Pitch - Yaw) / 4.f; // dynamic moment equation of the "rear right" rotor
  



  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return cmd;
}

V3F QuadControl::BodyRateControl(V3F pqrCmd, V3F pqr)
{
  // Calculate a desired 3-axis moment given a desired and current body rate
  // INPUTS: 
  //   pqrCmd: desired body rates [rad/s]
  //   pqr: current or estimated body rates [rad/s]
  // OUTPUT:
  //   return a V3F containing the desired moments for each of the 3 axes

  // HINTS: 
  //  - you can use V3Fs just like scalars: V3F a(1,1,1), b(2,3,4), c; c=a-b;
  //  - you'll need parameters for moments of inertia Ixx, Iyy, Izz
  //  - you'll also need the gain parameter kpPQR (it's a V3F)

  V3F momentCmd;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  


  //https://knowledge.udacity.com/questions/216747
  // Here, we will use the equations in pic """Con_equations20""" to get: u2 / u3 / u4

  //First: get the errors of the p / q / r:
  float p_err = (pqrCmd.x - pqr.x);
  float q_err = (pqrCmd.y - pqr.y);
  float r_err = (pqrCmd.z - pqr.z);

  //Second: Calculate the controller Acceleration by implementing 
  //        the rule of z_dot_dot = K*e_error
  float x_dot_dot = kpPQR.x * p_err;
  float y_dot_dot = kpPQR.y * q_err;       // remember: BodyRateControl is a P-Controller
  float z_dot_dot = kpPQR.z * r_err;


  //Finaly: Get the moments by implemting the rule: M = I * Acc
  momentCmd.x = Ixx * x_dot_dot;
  momentCmd.y = Iyy * y_dot_dot;
  momentCmd.z = Izz * z_dot_dot;           // Congrates!! you got the u2 / u3 / u4 commands 
  



  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return momentCmd;
}

// returns a desired roll and pitch rate 
V3F QuadControl::RollPitchControl(V3F accelCmd, Quaternion<float> attitude, float collThrustCmd)
{
  // Calculate a desired pitch and roll angle rates based on a desired global
  //   lateral acceleration, the current attitude of the quad, and desired
  //   collective thrust command
  // INPUTS: 
  //   accelCmd: desired acceleration in global XY coordinates [m/s2]
  //   attitude: current or estimated attitude of the vehicle
  //   collThrustCmd: desired collective thrust of the quad [N]
  // OUTPUT:
  //   return a V3F containing the desired pitch and roll rates. The Z
  //     element of the V3F should be left at its default value (0)

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the roll/pitch gain kpBank
  //  - collThrustCmd is a force in Newtons! You'll likely want to convert it to acceleration first

  V3F pqrCmd;
  Mat3x3F R = attitude.RotationMatrix_IwrtB();

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  //https://knowledge.udacity.com/questions/204405
  // Here, we will use the equations in pic """Con_equations19""" to get: p_command and q_command
  {
      V3F pqrCmd;
      float c_d = -(collThrustCmd / mass);

      float b_x_c = accelCmd.x / c_d;     // Define b_x_c
      float b_y_c = accelCmd.y / c_d;     // Define b_y_c

      float b_x_a = R(0, 2);              // Define b_x_a
      float b_y_a = R(1, 2);              // Define b_y_a
      
      //Noting that, there is a matrix notation difference between that "pic" and the C++  
      // which means: R13 = R(0,2)  (as you subtract -1 because of the index)
      //              R23 = R(1,2)
      //              R22 = R(1,1)
      //              R33 = R(2,2) and so on..

      float b_x_c_dot = (b_x_c - b_x_a);    // Get b_x_c_dot
      float b_y_c_dot = (b_y_c - b_y_a);    // Get b_y_c_dot

      float p_cmd = (1 / R(2, 2)) * (R(1, 0) * (kpBank * b_x_c_dot) - R(0, 0) * (kpBank * b_y_c_dot));
      float q_cmd = (1 / R(2, 2)) * (R(1, 1) * (kpBank * b_x_c_dot) - R(0, 1) * (kpBank * b_y_c_dot));

      pqrCmd.x = p_cmd;
      pqrCmd.y = q_cmd;
      pqrCmd.z = 0.f;
  }

  /////////////////////////////// EN+D STUDENT CODE ////////////////////////////

  return pqrCmd;
}

float QuadControl::AltitudeControl(float posZCmd, float velZCmd, float posZ, float velZ, Quaternion<float> attitude, float accelZCmd, float dt)
{
  // Calculate desired quad thrust based on altitude setpoint, actual altitude,
  //   vertical velocity setpoint, actual vertical velocity, and a vertical 
  //   acceleration feed-forward command
  // INPUTS: 
  //   posZCmd, velZCmd: desired vertical position and velocity in NED [m]
  //   posZ, velZ: current vertical position and velocity in NED [m]
  //   accelZCmd: feed-forward vertical acceleration in NED [m/s2]
  //   dt: the time step of the measurements [seconds]
  // OUTPUT:
  //   return a collective thrust command in [N]

  // HINTS: 
  //  - we already provide rotation matrix R: to get element R[1,2] (python) use R(1,2) (C++)
  //  - you'll need the gain parameters kpPosZ and kpVelZ
  //  - maxAscentRate and maxDescentRate are maximum vertical speeds. Note they're both >=0!
  //  - make sure to return a force, not an acceleration
  //  - remember that for an upright quad in NED, thrust should be HIGHER if the desired Z acceleration is LOWER

  Mat3x3F R = attitude.RotationMatrix_IwrtB();
  float thrust = 0;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  //https://knowledge.udacity.com/questions/270051



  //velZCmd = CONSTRAIN(velZCmd, -maxAscentRate, maxDescentRate);

  float z_err = posZCmd - posZ;
  float z_dot_err = velZCmd - velZ;
  integratedAltitudeError += (posZCmd - posZ) * dt;

  float accel = (kpPosZ * z_err) + (kpVelZ * z_dot_err) + (KiPosZ * integratedAltitudeError) + accelZCmd;

  //float accel_Cmd = mass * (9.81f - accel) / R(2, 2);
  //thrust = CONSTRAIN(accel_Cmd, -maxAscentRate / dt, maxAscentRate / dt);

  thrust = -CONSTRAIN((accel - CONST_GRAVITY) / R(2, 2), -maxDescentRate / dt, maxAscentRate / dt) * mass;
  

  /////////////////////////////// END STUDENT CODE ////////////////////////////
  
  return thrust;
}




// returns a desired acceleration in global frame
V3F QuadControl::LateralPositionControl(V3F posCmd, V3F velCmd, V3F pos, V3F vel, V3F accelCmdFF)
{
  // Calculate a desired horizontal acceleration based on 
  //  desired lateral position/velocity/acceleration and current pose
  // INPUTS: 
  //   posCmd: desired position, in NED [m]
  //   velCmd: desired velocity, in NED [m/s]
  //   pos: current position, NED [m]
  //   vel: current velocity, NED [m/s]
  //   accelCmdFF: feed-forward acceleration, NED [m/s2]
  // OUTPUT:
  //   return a V3F with desired horizontal accelerations. 
  //     the Z component should be 0
  // HINTS: 
  //  - use the gain parameters kpPosXY and kpVelXY
  //  - make sure you limit the maximum horizontal velocity and acceleration
  //    to maxSpeedXY and maxAccelXY

  // make sure we don't have any incoming z-component
  accelCmdFF.z = 0;
  velCmd.z = 0;
  posCmd.z = pos.z;

  // we initialize the returned desired acceleration to the feed-forward value.
  // Make sure to _add_, not simply replace, the result of your controller
  // to this variable
  V3F accelCmd = accelCmdFF;

  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  //https://knowledge.udacity.com/questions/270051



  velCmd.x = CONSTRAIN(velCmd.x, -maxSpeedXY, maxSpeedXY);
  velCmd.y = CONSTRAIN(velCmd.y, -maxSpeedXY, maxSpeedXY);
  accelCmd.x = CONSTRAIN(accelCmd.x, -maxAccelXY, maxAccelXY);
  accelCmd.y = CONSTRAIN(accelCmd.y, -maxAccelXY, maxAccelXY);

  accelCmd = kpPosXY * (posCmd - pos) + kpVelXY * (velCmd - vel) + accelCmd;



  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return accelCmd;
}

// returns desired yaw rate
float QuadControl::YawControl(float yawCmd, float yaw)
{
  // Calculate a desired yaw rate to control yaw to yawCmd
  // INPUTS: 
  //   yawCmd: commanded yaw [rad]
  //   yaw: current yaw [rad]
  // OUTPUT:
  //   return a desired yaw rate [rad/s]
  // HINTS: 
  //  - use fmodf(foo,b) to unwrap a radian angle measure float foo to range [0,b]. 
  //  - use the yaw control gain parameter kpYaw

  float yawRateCmd=0;
  ////////////////////////////// BEGIN STUDENT CODE ///////////////////////////
  //https:/knowledge.udacity.com/questions/311735


  float yaw_sub = fmodf(yaw, 2 * F_PI);
  float yawCmd_sub = fmodf(yawCmd, 2 * F_PI);
  float yaw_err = yawCmd_sub - yaw_sub;

  if (yaw_err > F_PI) {
      yaw_err -= 2 * F_PI;
  }
  if (yaw_err < -F_PI) {
      yaw_err += 2 * F_PI;
  }

  yawRateCmd = kpYaw * yaw_err;


  /////////////////////////////// END STUDENT CODE ////////////////////////////

  return yawRateCmd;

}

VehicleCommand QuadControl::RunControl(float dt, float simTime)
{
  curTrajPoint = GetNextTrajectoryPoint(simTime);

  float collThrustCmd = AltitudeControl(curTrajPoint.position.z, curTrajPoint.velocity.z, estPos.z, estVel.z, estAtt, curTrajPoint.accel.z, dt);

  // reserve some thrust margin for angle control
  float thrustMargin = .1f*(maxMotorThrust - minMotorThrust);
  collThrustCmd = CONSTRAIN(collThrustCmd, (minMotorThrust+ thrustMargin)*4.f, (maxMotorThrust-thrustMargin)*4.f);
  
  V3F desAcc = LateralPositionControl(curTrajPoint.position, curTrajPoint.velocity, estPos, estVel, curTrajPoint.accel);
  
  V3F desOmega = RollPitchControl(desAcc, estAtt, collThrustCmd);
  desOmega.z = YawControl(curTrajPoint.attitude.Yaw(), estAtt.Yaw());

  V3F desMoment = BodyRateControl(desOmega, estOmega);

  return GenerateMotorCommands(collThrustCmd, desMoment);
}
