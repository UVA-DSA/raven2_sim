//
//  Kinematics.cpp
//  LearnOMPL_2
//
//  Created by ming on 11/15/17.
//  Copyright © 2017 ming. All rights reserved.
//
#include "tf/LinearMath/Transform.h"
#include <iostream>


#include "Kinematics.hpp"

const static double d2r = M_PI/180;
const static double r2d = 180/M_PI;
const static double eps = 1.0e-5;

// Pointers are set to DH table for left or right arm
double const* dh_alpha;
double const* dh_a;
double *dh_theta;
double *dh_d;

// DH Parameters.
// Variable entries are marked "V" for clarity.
// Entered in table form, so alphas[0][0] is alpha_0 and thetas[0][0] is theta_1.
const double alphas[2][6] = {{0,    La12, M_PI- La23, 0,   M_PI/2, M_PI/2},
    {M_PI, La12, La23,       0,   M_PI/2, M_PI/2}};  // Left / Right
const double aas[2][6]    = {{0,    0,    0,          La3, 0,      Lw},
    {0,    0,    0,          La3, 0,      Lw}};
double ds[2][6]           = {{0,    0,    V,          d4,  0,      0},
    {0,    0,    V,          d4,  0,      0}};
double robot_thetas[2][6] = {{V,    V,    M_PI/2,     V,   V,      V},
    {V,    V,    -M_PI/2,    V,   V,      V}};





//--------------------------------------------------------------------------------
//  Calculate a transform between two links
//--------------------------------------------------------------------------------

/**\fn tf::Transform getFKTransform (int a, int b)
 * \brief Retrieve the forward kinematics transform from a to b, i.e., ^a_bT
 * \param a - an integer value, starting link frame id
 * \param b - an integer value, ending link frame id
 * \return a tf::Transform object transforms link a to link b
 *  \ingroup Kinematics
 */
tf::Transform getFKTransform(int a, int b)
{
    tf::Transform xf;
    if ( (b <= a) || b==0 )
    {
        std::cout << "Invalid start/end indices." << std::endl;
    }
    
    double xx = cos(dh_theta[a]),                  xy = -sin(dh_theta[a]),                   xz =  0;
    double yx = sin(dh_theta[a])*cos(dh_alpha[a]), yy =  cos(dh_theta[a])*cos(dh_alpha[a]),  yz = -sin(dh_alpha[a]);
    double zx = sin(dh_theta[a])*sin(dh_alpha[a]), zy =  cos(dh_theta[a])*sin(dh_alpha[a]),  zz =  cos(dh_alpha[a]);
    
    double px =  dh_a[a];
    double py = -sin(dh_alpha[a])*dh_d[a];
    double pz =  cos(dh_alpha[a])*dh_d[a];
    
    xf.setBasis(tf::Matrix3x3(xx, xy, xz, yx, yy, yz, zx, zy, zz ));
    xf.setOrigin(tf::Vector3(px, py, pz));
    
    // recursively find transforms for following links
    if (b > a+1)
        xf *= getFKTransform(a+1, b);
    
    return xf;
}

/**\fn int fwd_kin (double in_j[6], l_r in_arm, tf::Transform &out_xform )
 * \brief Runs the Raven II forward kinematics to determine end effector position of one arm
 * \param in_j[6] - 6 element array of joint angles ( float j[] = {shoulder, elbow, ins, roll, wrist, grasp} )
 * \param in_arm - Arm type, left / right ( kin.armtype arm = left/right)
 * \param out_xform - a reference of tf::Transform object represents the forward kinematic transfrom from zero frame to endeffector frame of one arm
 * \return: 0 on success, -1 on failure
 *  \ingroup Kinematics
 */
int fwd_kin (double in_j[6], l_r in_arm, tf::Transform &out_xform)
{
    dh_alpha = alphas[in_arm];
    dh_theta = robot_thetas[in_arm];
    dh_a     = aas[in_arm];
    dh_d     = ds[in_arm];
    
    for (int i=0;i<6;i++)
    {
        if (i==2)
            dh_d[i] = in_j[i];
        else
            dh_theta[i] = in_j[i];// *M_PI/180;
    }
    
    out_xform = getFKTransform(0,6);
    
    // rotate to match "tilted" base
    /*
     const static tf::Transform zrot_l( tf::Matrix3x3 (cos(25*d2r),-sin(25*d2r),0,  sin(25*d2r),cos(25*d2r),0,  0,0,1), tf::Vector3 (0,0,0) );
     const static tf::Transform zrot_r( tf::Matrix3x3 (cos(-25*d2r),-sin(-25*d2r),0,  sin(-25*d2r),cos(-25*d2r),0,  0,0,1), tf::Vector3 (0,0,0) );
     
     
     if (in_arm == dh_left)
     {
     out_xform = zrot_l * out_xform;
     }
     else
     {
     out_xform = zrot_r * out_xform;
     }
     */
    return 0;
}

/**\fn  inv_kin(tf::Transform in_T06, l_r in_arm, ik_solution iksol[8])
 * \brief Runs the Raven II INVERSE kinematics to determine end effector position.
 *
 * See Hawkeye King, Sina Nia Kosari, Blake Hannaford, Ji Ma, 'Kinematic Analysis of the Raven-II(tm) Research Surgical Robot Platform,' University of Washington Electrical Engineering Department Technical Report ,Number 2012-0006, June 29, 2012. (Revised March 2014)
 *
 * \param in_T06 - a btTransfrom obejct, transforms the end effector frame to zero frame
 * \param in_arm - Arm type, left / right ( kin.armtype arm = left/right)
 * \param ik_solution iksol[8] - The 8 solutions:  8 element array of joint angles ( float j[] = {shoulder, elbow, vacant joint, ins,roll, wrist, grasp1, grasp2} )
 * \return 0 - success, -1 - bad arm, -2 - too close to RCM.
 * \question  why __attribute__ optimize?
 * \ingroup Kinematics
 */

int  __attribute__ ((optimize("0"))) inv_kin(tf::Transform in_T06, l_r in_arm, ik_solution iksol[8])
{
    dh_theta = robot_thetas[in_arm];
    dh_d     = ds[in_arm];
    dh_alpha = alphas[in_arm];
    dh_a     = aas[in_arm];
    for (int i=0;i<8;i++)
        iksol[i] = ik_zerosol;
    
    if  ( in_arm  >= dh_l_r_last)
    {
        std::cout << "BAD ARM IN IK!!!" << std::endl;
        return -1;
    }
    
    for (int i=0;i<8;i++)    iksol[i].arm = in_arm;
    
    
    //  Step 1, Compute P5
    tf::Transform  T60 = in_T06.inverse();
    tf::Vector3    p6rcm = T60.getOrigin();
    tf::Vector3    p05[8];
    
    p6rcm[2]=0;    // take projection onto x-y plane
    for (int i= 0; i<2; i++)
    {
        tf::Vector3 p65 = (-1+2*i) * Lw * p6rcm.normalize();
        p05[4*i] = p05[4*i+1] = p05[4*i+2] = p05[4*i+3] = in_T06 * p65;
    }
    
    
    //  Step 2, compute displacement of prismatic joint d3
    for (int i=0;i<2;i++)
    {
        double insertion = 0;
        insertion += p05[4*i].length();  // Two step process avoids compiler optimization problem. (Yeah, right. It was the compiler's problem...)
        
        if (insertion <= Lw)
        {
            std::cerr << "WARNING: mechanism at RCM singularity(Lw:"<< Lw <<"ins:" << insertion << ").  IK failing.\n";
            iksol[4*i + 0].invalid = iksol[4*i + 1].invalid = ik_invalid;
            iksol[4*i + 2].invalid = iksol[4*i + 3].invalid = ik_invalid;
            return -2;
        }
        iksol[4*i + 0].d3 = iksol[4*i + 1].d3 = -d4 - insertion;
        iksol[4*i + 2].d3 = iksol[4*i + 3].d3 = -d4 + insertion;
    }
    
    
    
    
    //  Step 3, calculate theta 2
    for (int i=0; i<8; i+=2) // p05 solutions
    {
        double z0p5 = p05[i][2];
        
        double d = iksol[i].d3 + d4;
        double cth2=0;
        
        if (in_arm  == dh_left)
            cth2 = 1 / (GM1*GM3) * ((-z0p5 / d) - GM2*GM4);
        else
            cth2 = 1 / (GM1*GM3) * ((z0p5 / d) + GM2*GM4);
        
        // Smooth roundoff errors at +/- 1.
        if      (cth2 > 1  && cth2 <  1+eps) cth2 =  1;
        else if (cth2 < -1 && cth2 > -1-eps) cth2 = -1;
        
        if (cth2>1 || cth2 < -1) {
            iksol[i].invalid = iksol[i+1].invalid = ik_invalid;
        }
        else
        {
            iksol[ i ].th2 =  acos( cth2 );
            iksol[i+1].th2 = -acos( cth2 );
        }
    }
    
    
    //  Step 4: Compute theta 1
    for (int i=0;i<8;i++)
    {
        if (iksol[i].invalid == ik_invalid)
            continue;
        
        double cth2 = cos(iksol[i].th2);
        double sth2 = sin(iksol[i].th2);
        double d    = iksol[i].d3 + d4;
        double BB1 = sth2*GM3;
        double BB2=0;
        tf::Matrix3x3 Bmx;     // using 3 vector and matrix bullet types for convenience.
        tf::Vector3   xyp05(p05[i]);
        xyp05[2]=0;
        
        if (in_arm == dh_left)
        {
            BB2 = cth2*GM2*GM3 - GM1*GM4;
            Bmx.setValue(BB1,  BB2,0,   -BB2, BB1,0,   0,    0,  1 );
        }
        else
        {
            BB2 = cth2*GM2*GM3 + GM1*GM4;
            Bmx.setValue( BB1, BB2,0,   BB2,-BB1,0,    0,   0,  1 );
        }
        
        tf::Vector3 scth1 = Bmx.inverse() * xyp05 * (1/d);
        iksol[i].th1 = atan2(scth1[1],scth1[0]);
    }
    
    
    //  Step 5: get theta 4, 5, 6
    for (int i=0; i<8;i++)
    {
        if (iksol[i].invalid == ik_invalid)
            continue;
        
        // compute T03:
        dh_theta[0] = iksol[i].th1;
        dh_theta[1] = iksol[i].th2;
        dh_d[2]     = iksol[i].d3;
        tf::Transform T03 = getFKTransform(0, 3);
        tf::Transform T36 = T03.inverse() * in_T06;
        
        double c5 = -T36.getBasis()[2][2];
        double s5 = (T36.getOrigin()[2]-d4)/Lw;
        
        // Compute theta 4:
        double c4, s4;
        if (fabs(c5) > eps)
        {
            c4 =T36.getOrigin()[0] / (Lw * c5);
            s4 =T36.getOrigin()[1] / (Lw * c5);
        }
        else
        {
            c4 = T36.getBasis()[0][2] / s5;
            s4 = T36.getBasis()[1][2] / s5;
        }
        iksol[i].th4 = atan2(s4,c4);
        
        // Compute theta 5:
        iksol[i].th5 = atan2(s5, c5);
        
        
        // Compute theta 6:
        double s6, c6;
        if (fabs(s5) > eps)
        {
            c6 =  T36.getBasis()[2][0] / s5;
            s6 = -T36.getBasis()[2][1] / s5;
        }
        else
        {
            dh_theta[3] = iksol[i].th4;
            dh_theta[4] = iksol[i].th5;
            tf::Transform T05 = T03 * getFKTransform(3, 5);
            tf::Transform T56 = T05.inverse() * in_T06;
            c6 =T56.getBasis()[0][0];
            s6 =T56.getBasis()[2][0];
        }
        iksol[i].th6 = atan2(s6, c6);
        
        //		if (gTime%1000 == 0 && in_arm == dh_left )
        //		{
        //			log_msg("dh_iksols: [%d]\t( %3f,\t %3f,\t %3f,\t %3f,\t %3f,\t %3f)",0,
        //					iksol[i].th1 * r2d,
        //					iksol[i].th2 * r2d,
        //					iksol[i].d3,
        //					iksol[i].th4 * r2d,
        //					iksol[i].th5 * r2d,
        //					iksol[i].th6 * r2d
        //					);
        //		}
        
        
    }
    return 0;
}
