// **************************************************************************
// FILENAME: Leg.cpp
// AUTHOR:   Kenji Suzuki
// DATE:     15 June 1993
// **************************************************************************

#include "Leg.hpp"


// **************************************************************************
// Constructor
// **************************************************************************
Leg::Leg()
{
  leg_no = 1;
  int ln = leg_no - 1;
  link0 = LegLink(0, LINK0,   ln*60.00*DR, J0ANGLE_MIN, J0ANGLE_MAX);
  link1 = LegLink(1, LINK1, J1ANGLE_START, J1ANGLE_MIN, J1ANGLE_MAX);
  link2 = LegLink(2, LINK2, J2ANGLE_START, J2ANGLE_MIN, J2ANGLE_MAX);
  link3 = LegLink(3, LINK3, J3ANGLE_START, J3ANGLE_MIN, J3ANGLE_MAX);
  link4 = LegLink(4, LINK4, J4ANGLE_START, J4ANGLE_MIN, J4ANGLE_MAX);
  W_foot_pos = W_cwv_pos = W_foothold =
    Vector(STANCE*cos(ln*60.0*DR), STANCE*sin(ln*60.0*DR), 0.0);
  B_foot_pos = B_cwv_pos = B_foothold =
    Vector(STANCE*cos(ln*60.0*DR), STANCE*sin(ln*60.0*DR),-BODY_HEIGHT);
  W_cwv_vel = Vector(0.0, 0.0, 0.0);
  contact_flag = 1;
  duty_factor  = 1.0;
  cwv_height = FOOT_HEIGHT;
  cwv_radius = R_CWV;
  relative_phase = ln*(1.0 - duty_factor);
  leg_phase_variable = 0.0;
  transfer_phase_variable = 0.0;
  support_phase_variable = 0.0;
  temporal_kinematic_margin = LONG_TIME;
  foot_lift_height = -FOOT_HEIGHT;
  ground_level = 5.0; 
}



// **************************************************************************
// Constructor
// **************************************************************************
Leg::Leg(int l_no)
{
  leg_no = l_no;
  int ln = leg_no - 1;
  link0 = LegLink(0, LINK0,   ln*60.00*DR, J0ANGLE_MIN, J0ANGLE_MAX);
  link1 = LegLink(1, LINK1, J1ANGLE_START, J1ANGLE_MIN, J1ANGLE_MAX);
  link2 = LegLink(2, LINK2, J2ANGLE_START, J2ANGLE_MIN, J2ANGLE_MAX);
  link3 = LegLink(3, LINK3, J3ANGLE_START, J3ANGLE_MIN, J3ANGLE_MAX);
  link4 = LegLink(4, LINK4, J4ANGLE_START, J4ANGLE_MIN, J4ANGLE_MAX);
  W_foot_pos = W_cwv_pos = W_foothold =
    Vector(STANCE*cos(ln*60.0*DR), STANCE*sin(ln*60.0*DR), 0.0);
  B_foot_pos = B_cwv_pos = B_foothold =
    Vector(STANCE*cos(ln*60.0*DR), STANCE*sin(ln*60.0*DR),-BODY_HEIGHT);
  W_cwv_vel = Vector(0.0, 0.0, 0.0);
  contact_flag = 1;
  duty_factor  = 1.0;
  cwv_height = FOOT_HEIGHT;
  cwv_radius = R_CWV;
  relative_phase = ln*(1.0 - duty_factor);
  leg_phase_variable = 0.0;
  transfer_phase_variable = 0.0;
  support_phase_variable = 0.0;
  temporal_kinematic_margin = LONG_TIME;
  foot_lift_height = -FOOT_HEIGHT;
  ground_level = 5.0; 
}



// **************************************************************************
// Operator function
// **************************************************************************
ostream &operator<<(ostream &strm, Leg &l)
{
  return (strm << "LEG" << l.leg_no
          << " FOOT position("
          << l.W_foot_pos.x << ", "
          << l.W_foot_pos.y << ", "
          << l.W_foot_pos.z << ") "
          << "\nJoint angle("
          << l.link0.joint_angle_pos/(DR) << ", "
          << l.link1.joint_angle_pos/(DR) << ", "
          << l.link2.joint_angle_pos/(DR) << ", "
          << l.link3.joint_angle_pos/(DR) << ")");

}



// **************************************************************************
// Member Function: Kinematics()
// **************************************************************************
void Leg::Kinematics()
{
  double c0, s0, c2, s2;
  double c01, s01, c23, s23;

  c0  = cos(link0.joint_angle_pos);  s0  = sin(link0.joint_angle_pos);
  //c1  = cos(link1.joint_angle_pos);  s1  = sin(link1.joint_angle_pos);
  c2  = cos(link2.joint_angle_pos);  s2  = sin(link2.joint_angle_pos);
  //c3  = cos(link3.joint_angle_pos);  s3  = sin(link3.joint_angle_pos);
  c01 = cos(link0.joint_angle_pos + link1.joint_angle_pos);
  s01 = sin(link0.joint_angle_pos + link1.joint_angle_pos);
  c23 = cos(link2.joint_angle_pos + link3.joint_angle_pos);
  s23 = sin(link2.joint_angle_pos + link3.joint_angle_pos);

  // HIP Joint Position
  link1.B_joint_pos.x = LINK0*c0;
  link1.B_joint_pos.y = LINK0*s0;
  link1.B_joint_pos.z = 0.0;

  // KNEE1 Joint Position
  link2.B_joint_pos.x = link1.B_joint_pos.x + LINK1*c01;
  link2.B_joint_pos.y = link1.B_joint_pos.y + LINK1*s01;
  link2.B_joint_pos.z = 0.0;
 
  // KNEE2 Joint Position
  link3.B_joint_pos.x = link2.B_joint_pos.x + LINK2*c01*c2;
  link3.B_joint_pos.y = link2.B_joint_pos.y + LINK2*s01*c2;
  link3.B_joint_pos.z =                   - LINK2    *s2;

  // FOOT Position
  B_foot_pos.x = link4.B_joint_pos.x = link3.B_joint_pos.x + LINK3*c01*c23;
  B_foot_pos.y = link4.B_joint_pos.y = link3.B_joint_pos.y + LINK3*s01*c23;
  B_foot_pos.z = link4.B_joint_pos.z = link3.B_joint_pos.z - LINK3    *s23; 

} // End of Kinematics()



// **************************************************************************
// Member Function: Inv_Kinematics()
// **************************************************************************
void Leg::Inv_Kinematics()
{
  double px, py, pz;         // foot position(BODY coordiates)
  double px2, py2, pz2;      // px^2, py^2, pz^2
  double x0, y0; //, z0;         // CB coordinates origin
  double c0, s0, c1, s1;     // sin(theta_i),         cos(theta_j)
  double c3, s3;             // sin(theta_i),         cos(theta_j)
  //double c01, s01;           // sin(theta_i+theta_j), cos(theta_i+theta_j)
  double b1, b2, b3;         // length bi
  double b12, b22, b32;      // bi^2
  double beta;               // angle beta
  double cpsi, spsi;         // cos(psi), sin(psi)
  double theta[4];           // joint angle, [0]:CB to [3]:KNEE2
  double theta2_p, theta2_n; // 
  double theta3_p, theta3_n; // 
  int no_solution_flag = 0;  // 1:No solution


  // set temp data
  theta[0] = link0.joint_angle_pos; // CB
  theta[1] = link1.joint_angle_pos; // HIP
  theta[2] = link2.joint_angle_pos; // KNEE1
  theta[3] = link3.joint_angle_pos; // KNEE2

  // set temp data
  px = B_foot_pos.x; px2 = px*px; 
  py = B_foot_pos.y; py2 = py*py; 
  pz = B_foot_pos.z; pz2 = pz*pz;
  
  c0  = cos(link0.joint_angle_pos); 
  s0  = sin(link0.joint_angle_pos); 
  b12 = (px - LINK0*c0)*(px - LINK0*c0) + (py - LINK0*s0)*(py - LINK0*s0);
  if (b12 < 0.0) {
    printf("Warning: Leg%d b12 FOOT is out of WorkSpace\n", leg_no);
    no_solution_flag = 1;
  }
  b1  = sqrt(b12);
  b2  = b1 - LINK1;
  b22 = b2*b2;
  b32 = b22 + pz2;
  if (b32 < 0.0) {
    printf("Warning: Leg%d b32 FOOT is out of WorkSpace\n", leg_no);
    no_solution_flag = 1;
  }
  b3  = sqrt(b32);


  // Calculation for Joint1(HIP) Angle
  c1 = ( px2 + py2 - LINK02 - b12 )/( 2.0*LINK0*b1 );
  c1 = aqmin(1.0, c1);
  if ( fabs(c1) > 1.0 ) {
    printf("Warning: Leg%d c1 FOOT is out of WorkSpace\n", leg_no);
    no_solution_flag = 1;
    printf("c1=%f\n", c1);
  }
  s1 = sqrt(1.0 - c1*c1);

  // FOOT Position Transformation from CB to J0 Coordinates
  x0 = px*c0 + py*s0;
  y0 =-px*s0 + py*c0;
  //z0 = pz;
  
  if ( x0 <= 0.0 ) {
    printf("Warning: Leg%d XB range error", leg_no);
    no_solution_flag = 1;
    printf("FOOT is out of WorkSpace, FOOT is under the BODY\n");
  }
  if ( y0 >= 0.0 ) {
    link1.joint_angle_pos = atan2( s1, c1);
  }
  else {
    link1.joint_angle_pos = atan2(-s1, c1);
  }
  if ( (link1.joint_angle_pos < link1.min_joint_angle) ||
       (link1.joint_angle_pos > link1.max_joint_angle) ) {
    printf("Warning: Leg%d JOINT1, out of joint limit\n", leg_no);
    no_solution_flag = 1;
  }
  

  // Calculation for JOINT2(KNEE1) Angle
  if ( no_solution_flag == 0 )
  {
    // set c01, s01
    //c01 = cos(link0.joint_angle_pos + link1.joint_angle_pos);
    //s01 = sin(link0.joint_angle_pos + link1.joint_angle_pos);

    beta = atan2( pz, b2 );
    cpsi = ( LINK22 + b32 - LINK32 )/( 2.0*LINK2*b3 );
    cpsi = aqmin(1.0, cpsi);
    if (fabs(cpsi) > 1.0) {
      printf("Warning: Leg%d cpsi FOOT is out of WorkSpace\n", leg_no);
      no_solution_flag = 1;
      printf("cpsi=%f\n", cpsi);
    }
    spsi = sqrt(1.0 - cpsi*cpsi);
    theta2_p = atan2( spsi, cpsi) - beta;
    theta2_n = atan2(-spsi, cpsi) - beta;
    
    // Select closest solution(JOINT2)
    if ( fabs(theta2_p - theta[2]) < fabs(theta2_n - theta[2]) ) {
      link2.joint_angle_pos = theta2_p;
    }
    else {
      link2.joint_angle_pos = theta2_n;
    }

    // Check joint limit(JOINT2)
    if ( (link2.joint_angle_pos < link2.min_joint_angle) ||
         (link2.joint_angle_pos > link2.max_joint_angle) ) {
      printf("Warning: Leg%d JOINT2, out of joint limit\n", leg_no);
      no_solution_flag = 1;
    }
  }

  // Calculation for JOINT3(KNEE2) Angle
  if ( no_solution_flag == 0 ) {

    c3 = ( b32 - LINK22 - LINK32 )/( 2.0*LINK2*LINK3 );
    c3 = aqmin(1.0, c3);
    if (fabs(c3) > 1.0) {
      printf("Warning:Leg%d c3 FOOT is out of WorkSpace\n", leg_no);
      no_solution_flag = 1;
      printf("c3=%f\n", c3);
    }
    s3 = sqrt(1.0 - c3*c3);
    theta3_p = atan2( s3, c3);
    theta3_n = atan2(-s3, c3);
    
    // Select closest solution(JOINT3)
    if ( fabs(theta3_p - theta[3]) < fabs(theta3_n - theta[3]) ) {
      link3.joint_angle_pos = theta3_p;
    }
    else {
      link3.joint_angle_pos = theta3_n;
    }

    // Check joint limit(JOINT3)
    if ( (link3.joint_angle_pos < link3.min_joint_angle) ||
         (link3.joint_angle_pos > link3.max_joint_angle) ) {
      printf("Warning: Leg%d JOINT3, out of joint limit\n", leg_no);   
      no_solution_flag = 1;
    }
  }

  if ( no_solution_flag == 0 ) {
    // Test Kinematics
    Kinematics();

    // If a result of Kinematics is correspond to original position, 
    // then determines it be a real solution.
    if ( (fabs(px - B_foot_pos.x) > 1.0) ||
         (fabs(py - B_foot_pos.y) > 1.0) ||
         (fabs(pz - B_foot_pos.z) > 1.0) ) { 
      printf("Warning: Leg%d NOT Correspond to Desired Foot Position\n", 
             leg_no);
      no_solution_flag = 1;
    }
  }

  if ( no_solution_flag == 1 ) {
    printf("Warning: Leg%d NO SOLUTION, Joint angle is reset previous value\n",
           leg_no);
    
    // reset joint angle to previous value
    link0.joint_angle_pos = theta[0];
    link1.joint_angle_pos = theta[1];
    link2.joint_angle_pos = theta[2];
    link3.joint_angle_pos = theta[3];
  }

} // End of Inv_Kinematics()


// **************************************************************************
// Member Function: Jacobian()
// **************************************************************************
void Leg::Update_Joint_Position(MatrixMy &H_matrix_B2W)
{
  link1.W_joint_pos = H_matrix_B2W*link1.B_joint_pos;
  link2.W_joint_pos = H_matrix_B2W*link2.B_joint_pos;
  link3.W_joint_pos = H_matrix_B2W*link3.B_joint_pos;
  link4.W_joint_pos = W_foot_pos;
//  link4.W_joint_pos = H_matrix_B2W*link4.B_joint_pos;
}







// **************************************************************************
// Member Function: Jacobian()
// **************************************************************************
void Leg::Jacobian()
{
  double c2, s2; //, c3, s3;
  double c01, s01, c23, s23;

  c2 = cos(link2.joint_angle_pos);  s2 = sin(link2.joint_angle_pos);
  //c3 = cos(link3.joint_angle_pos);  s3 = sin(link3.joint_angle_pos);
  c01 = cos(link0.joint_angle_pos + link1.joint_angle_pos);
  s01 = sin(link0.joint_angle_pos + link1.joint_angle_pos);
  c23 = cos(link2.joint_angle_pos + link3.joint_angle_pos);
  s23 = sin(link2.joint_angle_pos + link3.joint_angle_pos);

  B_foot_vel.x =
    - s01*(LINK1 + LINK2*c2 + LINK3*c23)*link1.joint_angle_vel
    - c01*(        LINK2*s2 + LINK3*s23)*link2.joint_angle_vel
    - c01*(                   LINK3*s23)*link3.joint_angle_vel;

  B_foot_vel.y = 
    + c01*(LINK1 + LINK2*c2 + LINK3*c23)*link1.joint_angle_vel
    - s01*(        LINK2*s2 + LINK3*s23)*link2.joint_angle_vel
    - s01*(                   LINK3*s23)*link3.joint_angle_vel;

  B_foot_vel.z =
    -     (        LINK2*c2 + LINK3*c23)*link2.joint_angle_vel
    -     (                   LINK3*c23)*link3.joint_angle_vel;

} // End of Jacobian()



// **************************************************************************
// Member Function: Inv_Jacobian()
// **************************************************************************
void Leg::Inv_Jacobian()
{
  double c2, s2, s3; //, c3;
  double c01, s01, c23, s23;
  double detJ;

  c2 = cos(link2.joint_angle_pos);  s2 = sin(link2.joint_angle_pos);
  //c3 = cos(link3.joint_angle_pos);  
  s3 = sin(link3.joint_angle_pos);
  c01 = cos(link0.joint_angle_pos + link1.joint_angle_pos);
  s01 = sin(link0.joint_angle_pos + link1.joint_angle_pos);
  c23 = cos(link2.joint_angle_pos + link3.joint_angle_pos);
  s23 = sin(link2.joint_angle_pos + link3.joint_angle_pos);

  detJ = LINK2*LINK3*s3*(LINK1 + LINK2*c2 + LINK3*c23);
//  detJ = link2.link_length*link3.link_length*s3*
//        (link1.link_length + link2.link_length*c2 + link3*c23);

  if (detJ == 0.0) {
    printf("Warning: Leg%d INVERSE JACOBIAN NOT EXIST.\n", leg_no);
  }
  else {
    link1.joint_angle_vel =
      (- LINK2*LINK3*s01*s3*B_foot_vel.x
       + LINK2*LINK3*c01*s3*B_foot_vel.y)/detJ;
//  link1.joint_angle_vel =
//    (- link2.link_length*link3.link_length*s01*s3*B_foot_vel.x
//     + link2.link_length*link3.link_length*c01*s3*B_foot_vel.y)/detJ;
    
    link2.joint_angle_vel =
      (+ LINK3*c01*c23*(LINK1 + LINK2*c2 + LINK3*c23)*B_foot_vel.x
       + LINK3*s01*c23*(LINK1 + LINK2*c2 + LINK3*c23)*B_foot_vel.y
       - LINK3    *s23*(LINK1 + LINK2*c2 + LINK3*c23)*B_foot_vel.z)/detJ;
// 	link2.joint_angle_vel =
// 	  (+ link3.link_length*c01*c23*
// 	   (link1.link_length + link2.link_length*c2 + link3.link_length*c23)*
// 	   B_foot_vel.x
// 	   + link3.link_length*s01*c23*
// 	   (link1.link_length + link2.link_length*c2 + link3.link_length*c23)*
// 	   B_foot_vel.y
// 	   - link3.link_length*s23*
// 	   (link1.link_length + link2.link_length*c2 + link3.link_length*c23)*
// 	   B_foot_vel.z)/detJ; 
    
    link3.joint_angle_vel =
      (- c01*(LINK1 + LINK2*c2 + LINK3*c23)*(LINK2*c2 + LINK3*c23)
       *B_foot_vel.x
       - s01*(LINK1 + LINK2*c2 + LINK3*c23)*(LINK2*c2 + LINK3*c23)
       *B_foot_vel.y
       +     (LINK1 + LINK2*c2 + LINK3*c23)*(LINK2*s2 + LINK3*s23)
       *B_foot_vel.z)/detJ;
//     link3.joint_angle_vel =
//       (- c01*(link1.link_length + 
// 			  link2.link_length*c2 + 
// 			  link3.link_length*c23)*
// 	   (link2.link_length*c2 + link3.link_length*c23)*B_foot_vel.x
//        - s01*(link1.link_length + 
// 			  link2.link_length*c2 + 
// 			  link3.link_length*c23)*
// 	   (link2.link_length*c2 + link3.link_length*c23)*B_foot_vel.y
//        +     (link1.link_length + 
// 			  link2.link_length*c2 + 
// 			  link3.link_length*c23)*
// 	   (LINK2*s2 + LINK3*s23)*B_foot_vel.z)/detJ;
  }

} // End of Inv_Jacobian()



// **************************************************************************
// Member Function: Find_Max_Instantaneous_Support_Period_Sequence()
// CALLED BY: Robot::Support_Phase_Block()
// **************************************************************************
void Leg::Update_Joint_Angle(MatrixMy &H_matrix_B2W, double sampling_time)
{
  // compute joint angle
  link1.joint_angle_pos += link1.joint_angle_vel*sampling_time;
  if (link1.joint_angle_pos >= link1.min_joint_angle) {
	link1.joint_angle_pos = link1.min_joint_angle;
  }

  link2.joint_angle_pos += link2.joint_angle_vel*sampling_time;
  if (link2.joint_angle_pos >= link2.min_joint_angle) {
	link2.joint_angle_pos = link2.min_joint_angle;
  }

  link3.joint_angle_pos += link3.joint_angle_vel*sampling_time;
  if (link3.joint_angle_pos >= link3.min_joint_angle) {
	link3.joint_angle_pos = link3.min_joint_angle;
  }

  link4.joint_angle_pos += link4.joint_angle_vel*sampling_time;
  if (link4.joint_angle_pos >= link4.min_joint_angle) {
	link4.joint_angle_pos = link4.min_joint_angle;
  }


  // compute joint position
  Kinematics();


  // Coordinates transformation
  link1.W_joint_pos = H_matrix_B2W*link1.B_joint_pos;
  link2.W_joint_pos = H_matrix_B2W*link2.B_joint_pos;
  link3.W_joint_pos = H_matrix_B2W*link3.B_joint_pos;
  link4.W_joint_pos = W_foot_pos;
//  link4.W_joint_pos = H_matrix_B2W*link4.B_joint_pos;

} // End of Update_Joint_Angle(double sampling_time)




// **************************************************************************
// Member Function: Find_Max_Instantaneous_Support_Period_Sequence()
// CALLED BY: Robot::Support_Phase_Block()
// **************************************************************************
void Leg::Find_Max_Instantaneous_Support_Period_Sequence(Posture &B_body_vel)
{
  Vector B_body_trans_rate(B_body_vel.x,
                           B_body_vel.y,
                           B_body_vel.z);
  Vector B_body_euler_rate(B_body_vel.roll,
                           B_body_vel.elevation,
                           B_body_vel.azimuth);


  // if foot is contact state, do support phase process.
  if (contact_flag == 1) { 

    // Find Foot Velocity(body coordinates) in Support Phase
    B_foot_vel = - (B_body_trans_rate +
                    B_body_euler_rate.Outerproduct(B_foot_pos));

    // Find Temporal Kinematic Margin
    Find_Temporal_Kinematic_Margin();

    // Find Maximun Instantaneous Support Period
    Find_Max_Instantaneous_Support_Period();
  }
  else {
    max_instantaneous_support_period = LONG_TIME;
  }

} // End of Find_Max_Instantaneous_Support_Period_Sequence()



// **************************************************************************
// Member Function: Find_Temporal_Kinematic_Margin()
// CALLED BY: Find_Max_Instantaneous_Support_Period_Sequence()
// **************************************************************************
void Leg::Find_Temporal_Kinematic_Margin()
{
  double remaining_distance;

  Vector cwv_intersection;       // Body coordinates.(liftoff_point)
  cwv_intersection = B_foothold; // initialize previous position.//?


  if ((B_foot_vel.x == 0.0) && (B_foot_vel.y == 0.0)) {
    temporal_kinematic_margin = LONG_TIME;
  }
  else {

    // If cwv intersection is found, retrun value is 1(TRUE).
    if ( Find_CWV_Intersection(cwv_intersection) ) {
      remaining_distance = sqrt( sqr(cwv_intersection.x - B_foot_pos.x) + 
                                 sqr(cwv_intersection.y - B_foot_pos.y) );

      // compute temporal kinematic margin
      temporal_kinematic_margin 
        = remaining_distance/sqrt( sqr(B_foot_vel.x) + sqr(B_foot_vel.y) );
    }
    else {
      temporal_kinematic_margin = LONG_TIME; //???
    }

    // Here, foothold represents predicted liftoff point. //??
    B_foothold = cwv_intersection;        // Body coordinates.
  }

} // End of Find_Temporal_Kinematic_Margin()



// **************************************************************************
// Member Function: Find_CWV_Intersection()
// CALLED BY: Find_Temporal_Kinematic_Margin()
// COMMENT:   return value: No Intersection:0, Intersection was found:1
// See CS4313 class note for detail.
// **************************************************************************
int Leg::Find_CWV_Intersection(Vector &intersect) // BODY coordinates
                                                  // liftoff point
{
  double alpha; // direction of the foot_vel vector from XB axis.
  double d;     // distance between cwv_pos and foot_vel vector.
  double l;     // distance between intersection and image of cwv_pos 
                // to foot_vel vector.

  
  if ( (sqr(B_cwv_pos.x - B_foot_pos.x) + sqr(B_cwv_pos.y - B_foot_pos.y)) > 
       (sqr(cwv_radius)) ) {

    printf("Warning: Leg%d FOOT POSITION OUT OF CWV\n", leg_no);
    return (0); // return value: No Intersection:0
  }
  else {
    if ((B_foot_vel.x == 0.0) && (B_foot_vel.y == 0.0)) {
      alpha = 0.0;
    }
    else {
      alpha = atan2(B_foot_vel.y, B_foot_vel.x);
    }
    d = (B_cwv_pos.y - B_foot_pos.y)*cos(alpha) 
      - (B_cwv_pos.x - B_foot_pos.x)*sin(alpha);
    l = sqrt( sqr(cwv_radius) - sqr(d) );
    
    intersect.x = B_cwv_pos.x + d*sin(alpha) + l*cos(alpha);
    intersect.y = B_cwv_pos.y - d*cos(alpha) + l*sin(alpha);
    intersect.z = B_cwv_pos.z; //???

    return (1);
  }

} // End of Find_CWV_Intersection()



// **************************************************************************
// Member Function: Find_Max_Instantaneous_Support_Period()
// CALLED BY: Find_Max_Instantaneous_Support_Period_Sequence()
// **************************************************************************
void Leg::Find_Max_Instantaneous_Support_Period()
{
  double misp_old; // previous max_inst_sup_period
  misp_old = max_instantaneous_support_period;


  if (support_phase_variable < 1.0) {
    max_instantaneous_support_period
      = temporal_kinematic_margin/(1.0 - support_phase_variable);
  }
  else {
    max_instantaneous_support_period = misp_old;
//  max_instantaneous_support_period = LONG_TIME;
  }

} // End of Find_Max_Instantaneous_Support_Period()



// **************************************************************************
// Member Function: Find_Leg_Phase_Var()
// **************************************************************************
void Leg::Find_Leg_Phase_Var(double kinematic_phase_variable)
{
  leg_phase_variable = mod1(kinematic_phase_variable - relative_phase);

} // End of Find_Leg_Phase_Var()



// **************************************************************************
// Member Function: Find_Support_Phase_Var()
// CALLED BY: Robot::Support_Phase_Block()
// **************************************************************************
void Leg::Find_Support_Phase_Var(double kinematic_period)
{
  if (leg_phase_variable < duty_factor) {

    if (kinematic_period >= 0.0) {  
      support_phase_variable = leg_phase_variable/duty_factor;
    }
    else {
      support_phase_variable = 
               (duty_factor - leg_phase_variable)/duty_factor;
    }
  }
  else {
    support_phase_variable = 0.0;
  }

} // End of Find_Support_Phase_Var()



// **************************************************************************
// Member Function: Transfer_Phase_Sequence()
// CALLED BY: Robot::Transfer_Phase_Block()
// **************************************************************************
void Leg::Transfer_Phase_Sequence(Posture &B_body_posrate,
                                  MatrixMy &R_matrix_B2W,
                                  MatrixMy &R_matrix_W2B,
                                  double kinematic_period)
{
  Vector B_trans_rate(B_body_posrate.x,
                      B_body_posrate.y,
                      B_body_posrate.z);
  Vector B_euler_rate(B_body_posrate.roll,
                      B_body_posrate.elevation,
                      B_body_posrate.azimuth);

  // ?????? 
  if ((contact_flag != 1) || (leg_phase_status_flag != 1)) {

    if (leg_phase_variable >= duty_factor) {

      double transfer_period;

      // Find transfer period
      transfer_period = (1.0 - duty_factor)*kinematic_period;
      
      // Find transfer phase variable    
      Find_Transfer_Phase_Var(kinematic_period);
      
      // Compute Foot Velocity
      // Find CWV Center Velocity
      Find_CWV_Velocity(B_body_posrate, R_matrix_B2W);
      
      // Find Predicted Foothold Position
      Find_Predicted_Foothold(transfer_period);//(WORLD coordinates)
      
      // Find Transfer Phase Foot Velocity
      Find_Transfer_Foot_Velocity(4, transfer_period);//(WORLD coordinates)
      // 1:cycloid, 2:ellipse, 3:triangle, 4:arbitrary
      
      // Transform to BODY coordinates.
      B_foot_vel = R_matrix_W2B*W_foot_vel
                 - B_trans_rate 
                 - B_euler_rate.Outerproduct(B_foot_pos);

      leg_phase_status_flag = 1; // Transfer Phase
    }
  }

  // Set or Reset leg_phase_status_flag
  if ( leg_phase_variable <= duty_factor ) {
    leg_phase_status_flag = 0; // Support Phase
  }
  else {
    leg_phase_status_flag = 1; // Transfer Phase
  }
  
  if (contact_flag == 1) {
    W_foot_vel = Vector(0.0, 0.0, W_foot_vel.z);
  }


} // End of Transfer_Phase_Sequence()



// **************************************************************************
// Member Function: Find_Transfer_Phase_Var()
// CALLED BY:  Transfer_Phase_Sequence()
// **************************************************************************
void Leg::Find_Transfer_Phase_Var(double kinematic_period)
{
  if (kinematic_period >= 0.0) {
    transfer_phase_variable 
      = (leg_phase_variable - duty_factor)/(1.0 - duty_factor);
  }
  else {
    transfer_phase_variable 
      = (1.0 - leg_phase_variable)/(1.0 - duty_factor);
  } 

} // End of Find_Transfer_Phase_Var()



// **************************************************************************
// Member Function: Find_CWV_Velocity()
// CALLED BY:  Transfer_Phase_Sequence()
// OUTPUT: W_cwv_velocity.
// **************************************************************************
void Leg::Find_CWV_Velocity(Posture &B_body_vel, // BODY coordinates
                            MatrixMy &R_matrix_B2W)
{
  Vector B_trans_vel(B_body_vel.x, 
                     B_body_vel.y, 
                     B_body_vel.z);
  Vector B_rotate_rate(B_body_vel.roll, 
                       B_body_vel.elevation, 
                       B_body_vel.azimuth);

  Vector W_trans_vel, W_rotate_rate;


  W_trans_vel   = R_matrix_B2W*B_trans_vel;
  W_rotate_rate = R_matrix_B2W*(B_rotate_rate.Outerproduct(B_cwv_pos));

  W_cwv_vel = W_trans_vel + W_rotate_rate;

}// End of Find_CWV_Velocity()



// **************************************************************************
// Member Function: Find_Predicted_Foothold()
// CALLED BY:  Transfer_Phase_Sequence()
// OUTPUT: predicted foothold.
// **************************************************************************
void Leg::Find_Predicted_Foothold(double transfer_period)
{
  Vector unit_cwv_vel;
  unit_cwv_vel = W_cwv_vel.Normalize();


  W_foothold = 
    W_cwv_pos +
    ( (1.0 - transfer_phase_variable)*transfer_period*W_cwv_vel.Norm() +
      (0.75*cwv_radius) )*  // cwv radius is made smaller to keep foot inside.
      unit_cwv_vel;  

}// End of Find_Predicted_Foothold()



// **************************************************************************
// Member Function: Find_Transfer_Foot_Velocity()
// CALLED BY:  Transfer_Phase_Sequence()
// OUTPUT: W_foot_vel(Foot Velocity in WORLD coordinate system)
// **************************************************************************
void Leg::Find_Transfer_Foot_Velocity(int menu, double transfer_period)
{
  // NOTE: Zw-axis is down.

  double remainning_phase_h = 0.0;    // Horizontal Remainning distance
  double remainning_phase_v = 0.0;    // Vertical Remainning distance
  double normalized_velocity_h = 0.0; //
  double normalized_velocity_v = 0.0; //

  
  switch (menu) {

  case 1: // cycloid
    normalized_velocity_h = 1.0 - cos(DPI*transfer_phase_variable);
    normalized_velocity_v = sin(DPI*transfer_phase_variable);

    remainning_phase_h = 1.0 - transfer_phase_variable +
                        (1.0/DPI)*sin(DPI*transfer_phase_variable);

    if (transfer_phase_variable < 0.5) {
      remainning_phase_v = (1.0/DPI)*(1.0 + cos(DPI*transfer_phase_variable));
      W_foothold.z =-20.0; // foot lift height
    }
    else {
      remainning_phase_v =-(1.0/DPI)*(1.0 - cos(DPI*transfer_phase_variable));
      W_foothold.z = 5.0;  // ground level - 5[cm]
    }
    break;


  case 2: // ellipse
    normalized_velocity_h = sin(PI*transfer_phase_variable);
    normalized_velocity_v = cos(PI*transfer_phase_variable);

    remainning_phase_h = (1.0/PI)*(1.0 + cos(PI*transfer_phase_variable));
    if (transfer_phase_variable < 0.5) {
      remainning_phase_v = (1.0/PI)*(1.0 - sin(PI*transfer_phase_variable));
      W_foothold.z =-20.0;// foot lift height
    }
    else {
      remainning_phase_v = (1.0/PI)*(-sin(PI*transfer_phase_variable));
      W_foothold.z = 0.0; // ground level - 5[cm]
    }
    break;


  case 3: // triangle
    remainning_phase_h = 1.0 - transfer_phase_variable;

    if (transfer_phase_variable < 0.5) {
      remainning_phase_v = 0.5 - transfer_phase_variable;
      W_foothold.z =-20.0;// foot lift height
    }
    else {
      remainning_phase_v = 1.0 - transfer_phase_variable;
      W_foothold.z = 0.0; // ground level - 5[cm]
    }
    normalized_velocity_h = 1.0;
    normalized_velocity_v = 1.0;
    break;


  case 4: // rectangular
  {
    double ph1 = 0.15, ph2 = 0.5, ph3 = 0.85;
    double pv1 = 0.20, pv2 = 0.47, pv3 = 0.53, pv4 = 0.8;

    // Horizontal component
    if (transfer_phase_variable < ph1) {
      normalized_velocity_h = 0.0;
      remainning_phase_h = (ph3 - ph1)/2.0;
    }
    else if (transfer_phase_variable < ph2) {
      normalized_velocity_h = (transfer_phase_variable - ph1)/(ph2 - ph1);
      remainning_phase_h = ((transfer_phase_variable - ph1)/(ph2 - ph1) + 1.0)*
                           (ph2 - transfer_phase_variable)/2.0 +
                           (ph3 - ph2)/2.0;

    }
    else if (transfer_phase_variable < ph3) {
      normalized_velocity_h =-(transfer_phase_variable - ph3)/(ph3 - ph2);
      remainning_phase_h = sqr(transfer_phase_variable - ph3)/
                           (2.0*(ph3 - ph2));
    }
    else {
      normalized_velocity_h = 0.0;
      remainning_phase_h = 0.0; // modification needed
    }


    // Vertical component
    if (transfer_phase_variable < pv1) {
      normalized_velocity_v = (transfer_phase_variable)/pv1;
      remainning_phase_v = (sqr(pv1) - sqr(transfer_phase_variable))/
                           (2.0*pv1) +
                           (pv2 - pv1)/2.0;
      W_foothold.z =-20.0;// foot lift height
    }
    else if (transfer_phase_variable < pv2) {
      normalized_velocity_v = -(transfer_phase_variable - pv2)/(pv2 - pv1);
      remainning_phase_v = sqr(transfer_phase_variable - pv2)/
                           (2.0*(pv2 - pv1));
      W_foothold.z =-20.0;// foot lift height
    }
    else if (transfer_phase_variable < pv3) {
      normalized_velocity_v = 0.0;
      remainning_phase_v = (1.0 - pv3)/2.0;
      W_foothold.z = 5.0; // ground level - 5[cm]
    }
    else if (transfer_phase_variable < pv4) {
      normalized_velocity_v =  (transfer_phase_variable - pv3)/(pv4 - pv3);
      remainning_phase_v = ( sqr(pv4 - pv3) - 
                             sqr(transfer_phase_variable - pv3) )/
                           (2.0*(pv4 - pv3)) +
                           (1.0 - pv4)/2.0;
      W_foothold.z = 5.0; // ground level - 5[cm]
    }
    else {
      normalized_velocity_v = (transfer_phase_variable - 1.0)/
                              (1.0 - pv4);
      remainning_phase_v = -sqr(transfer_phase_variable - 1.0)/
                            (2.0*(1.0 - pv4)); 
      W_foothold.z = 5.0; // ground level - 5[cm]
    }
  }
    break;


  default:
    break;
  }


  // Compute Foot velocity
  if (remainning_phase_h == 0.0) {
    W_foot_vel.x = 0.0;
    W_foot_vel.y = 0.0;
  }
  else {
    W_foot_vel.x = ( (W_foothold.x - W_foot_pos.x)/
                     (transfer_period*remainning_phase_h) )*
                     normalized_velocity_h;

    W_foot_vel.y = ( (W_foothold.y - W_foot_pos.y)/
                     (transfer_period*remainning_phase_h) )*
                     normalized_velocity_h;
  }

  if (remainning_phase_v == 0.0) {
    W_foot_vel.z = 0.0;
  }
  else {
    W_foot_vel.z = ( (W_foothold.z - W_foot_pos.z)/
                     (transfer_period*remainning_phase_v) )*
                     normalized_velocity_v;

//  W_foot_vel.z = (-DPI*FOOT_HEIGHT*normalized_velocity_v);
  }

}// End of Find_Transfer_Foot_Velocity()



// **************************************************************************
// FUNCTION: Update_Foot_Position()
// CALLED BY:  
// PURPOSE:  Compute New Foot Position both BODY and WORLD coordinates.
// OUTPUT: Foot position(WORLD & BODY coordinates).
// **************************************************************************
void Leg::Update_Foot_Position(MatrixMy &H_matrix_W2B, double sampling_time)
{
  // Update foot position in World coordinate system
  W_foot_pos = W_foot_pos + W_foot_vel*sampling_time;

  // Set or Reset contact_flag
  if (W_foot_pos.z >= 0.0) { // if foot contacts to ground
    W_foot_pos.z = 0.0; // Flat Terrain
    contact_flag = 1;
  }
  else {
    contact_flag = 0;
  }

  // Update foot position in Body coordinate system
  B_foot_pos = H_matrix_W2B*W_foot_pos;


} // End of Update_Foot_Position()



// **************************************************************************
// Member Function: Body_to_World()
// **************************************************************************
void Leg::Body_to_World(Posture &body)
{
  double c1, c2, c3, s1, s2, s3;
  double x, y, z;

  c1 = cos(body.azimuth);   s1 = sin(body.azimuth); 
  c2 = cos(body.elevation); s2 = sin(body.elevation); 
  c3 = cos(body.roll);      s3 = sin(body.roll);

  x = B_foot_pos.x; 
  y = B_foot_pos.y;
  z = B_foot_pos.z;

  W_foot_pos.x =  c1*c2*x + (c1*s2*s3-s1*c3)*y + (c1*s2*c3+s1*s3)*z + body.x;
  W_foot_pos.y =  s1*c2*x + (s1*s2*s3+c1*c3)*y + (s1*s2*c3-c1*s3)*z + body.y;
  W_foot_pos.z =    -s2*x +            c2*s3*y +            c2*c3*z + body.z;

} // End of Body_to_World()


// **************************************************************************
// Member Function: World_to_Body()
// **************************************************************************
void Leg::World_to_Body(Posture &body)
{
  double c1, c2, c3, s1, s2, s3; // cos(), sin()
  double x, y, z;                // WORLD coordinates

  c1 = cos(body.azimuth);   s1 = sin(body.azimuth); 
  c2 = cos(body.elevation); s2 = sin(body.elevation); 
  c3 = cos(body.roll);      s3 = sin(body.roll);
  
  x = W_foot_pos.x; 
  y = W_foot_pos.y;
  z = W_foot_pos.z;

  B_foot_pos.x = (c1*c2           )*(x - body.x) 
               + (s1*c2           )*(y - body.y) - s2   *(z - body.z);
  B_foot_pos.y = (c1*s2*s3 - s1*c3)*(x - body.x) 
               + (s1*s2*s3 + c1*c3)*(y - body.y) + c2*s3*(z - body.z);
  B_foot_pos.z = (c1*s2*c3 + s1*s3)*(x - body.x) 
               + (s1*s2*c3 - c1*s3)*(y - body.y) + c2*c3*(z - body.z);

} // End of _World_to_Body()



// **************************************************************************
// Member Function: Velocity_Body_to_World()
// **************************************************************************
void Leg::Velocity_Body_to_World(Body &body)
{
  MatrixMy WFootPos(3, 1);
  MatrixMy BFootVel(3, 1);
  MatrixMy WFootVel(3, 1);
  MatrixMy WBodyVel(3, 1);
  MatrixMy WBodyOmg(3, 1);
  MatrixMy RW2B(3, 3);
  MatrixMy RB2W(3, 3);

  WFootPos.Set_Vector3x1(W_foot_pos.x, W_foot_pos.y, W_foot_pos.z);
  BFootVel.Set_Vector3x1(B_foot_vel.x, B_foot_vel.y, B_foot_vel.z);
  WBodyVel.Set_Vector3x1(body.W_body_vel.x, 
                         body.W_body_vel.y, 
                         body.W_body_vel.z);
  WBodyOmg.Set_Vector3x1(body.W_body_vel.azimuth,
                         body.W_body_vel.elevation, 
                         body.W_body_vel.roll);
  RW2B.Set_Hr_Matrix(body.W_body_pos.azimuth,
                     body.W_body_pos.elevation, 
                     body.W_body_pos.roll);  

//  WFootVel = WBodyVel + RW2B*BFootVel + WBodyOmg.Outerproduct(WFootPos);
  WFootVel = WBodyVel + RB2W*BFootVel + WBodyOmg.Outerproduct(WFootPos);

  W_foot_vel.x = WFootVel.Get_Value(0, 0);
  W_foot_vel.y = WFootVel.Get_Value(1, 0);
  W_foot_vel.z = WFootVel.Get_Value(2, 0);

} // End of Velocity_Body_to_World()



// **************************************************************************
// Member Function: Velocity_World_to_Body()
// **************************************************************************
void Leg::Velocity_World_to_Body(Body &body)
{
  MatrixMy WFootPos(3, 1);
  MatrixMy BFootVel(3, 1);
  MatrixMy WFootVel(3, 1);
  MatrixMy WBodyVel(3, 1);
  MatrixMy WBodyOmg(3, 1);
  MatrixMy RW2B(3, 3);
  MatrixMy RB2W(3, 3);

  WFootPos.Set_Vector3x1(W_foot_pos.x, W_foot_pos.y, W_foot_pos.z);
  WFootVel.Set_Vector3x1(W_foot_vel.x, W_foot_vel.y, W_foot_vel.z);
  WBodyVel.Set_Vector3x1(body.W_body_vel.x, 
                         body.W_body_vel.y, 
                         body.W_body_vel.z);
  WBodyOmg.Set_Vector3x1(body.W_body_vel.azimuth,
                         body.W_body_vel.elevation, 
                         body.W_body_vel.roll);
  RW2B.Set_Hr_Matrix(body.W_body_pos.azimuth,
                     body.W_body_pos.elevation, 
                     body.W_body_pos.roll);  
  RB2W = RW2B.Transpose();

//  BFootVel = RB2W*(WFootVel - WBodyVel - WBodyOmg.Outerproduct(WFootPos));
  BFootVel = RW2B*(WFootVel - WBodyVel - WBodyOmg.Outerproduct(WFootPos));

  B_foot_vel.x = BFootVel.Get_Value(0, 0);
  B_foot_vel.y = BFootVel.Get_Value(1, 0);
  B_foot_vel.z = BFootVel.Get_Value(2, 0);

} // End of Velocity_World_to_Body()



// **************************************************************************
// Member Function: Body_to_World()
// **************************************************************************
GAIT_DLL_API Vector Body_to_World(Posture &body, Vector &bp)
{
  Vector wp; // world coordinates
  double c1, c2, c3, s1, s2, s3;

  c1 = cos(body.azimuth);   s1 = sin(body.azimuth); 
  c2 = cos(body.elevation); s2 = sin(body.elevation); 
  c3 = cos(body.roll);      s3 = sin(body.roll);

  wp.x = c1*c2*bp.x + (c1*s2*s3-s1*c3)*bp.y + (c1*s2*c3+s1*s3)*bp.z + body.x;
  wp.y = s1*c2*bp.x + (s1*s2*s3+c1*c3)*bp.y + (s1*s2*c3-c1*s3)*bp.z + body.y;
  wp.z =   -s2*bp.x +            c2*s3*bp.y +            c2*c3*bp.z + body.z;

  return (wp);

} // End of Body_to_World()



// **************************************************************************
// Member Function: World_to_Body()
// **************************************************************************
GAIT_DLL_API Vector World_to_Body(Posture &body, // Body Position
                                  Vector &wp)    //
{
  Vector bp; // Body coordinates
  double c1, c2, c3, s1, s2, s3; // cos(), sin()

  c1 = cos(body.azimuth);   s1 = sin(body.azimuth); 
  c2 = cos(body.elevation); s2 = sin(body.elevation); 
  c3 = cos(body.roll);      s3 = sin(body.roll);
  
  bp.x = (c1*c2           )*(wp.x - body.x) 
       + (s1*c2           )*(wp.y - body.y) - s2   *(wp.z - body.z);
  bp.y = (c1*s2*s3 - s1*c3)*(wp.x - body.x)
       + (s1*s2*s3 + c1*c3)*(wp.y - body.y) + c2*s3*(wp.z - body.z);
  bp.z = (c1*s2*c3 + s1*s3)*(wp.x - body.x) 
       + (s1*s2*c3 - c1*s3)*(wp.y - body.y) + c2*c3*(wp.z - body.z);

  return (bp);

} // End of World_to_Body()


// EOF
