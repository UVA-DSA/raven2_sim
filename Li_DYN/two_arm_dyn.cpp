#include "two_arm_dyn.h"
#include <ctime>

extern double DACs[3];

void sys_states(const state_type &x)
{
	cout << x[0] << '\t' << x[1]  << '\t' << x[2] << '\t' << x[3]  << '\t' << x[4]<< '\t' << x[5]
	<< '\t' << x[6]<< '\t' << x[7] << x[8]<< '\t' << x[9] << x[10]<< '\t' << x[11] <<endl;
}

int sign_func(double input)
{
	if (input > 0)
		return 1;
	else if (input < 0)
		return -1;
	else
		return 0;
}

void sys_dyn_gold(const state_type &x, state_type &dxdt, double t)
{
	double t1  = x[0];
	double t2  = x[1];
	double t3  = x[2];

	double t1_d = x[3];
	double t2_d = x[4];
	double t3_d = x[5];

	double tm_1 = x[6];
	double tm_2 = x[7];
	double tm_3 = x[8];


    dxdt[0] = x[3];
    dxdt[1] = x[4];
	dxdt[2] = x[5];


    if (x[9]>790)
        dxdt[6] = 790;
    else
    {
		if (x[9]<-790)
        	dxdt[6] = -790;
        else
        	dxdt[6] = x[9];
    }

    if (x[10]>790)
        dxdt[7] = 790;
    else
    {
    	if (x[10]<-790)
    	    dxdt[7] = -790;
        else
        	dxdt[7] = x[10];
    }

    if (x[11]>790)
        dxdt[8] = 790;
    else
    {
    	if (x[11]<-790)
    	    dxdt[8] = -790;
        else
        	dxdt[8] = x[11];
    }

	double s1 = sin(t1);
	double s2 = sin(t2);
	double c1 = cos(t1);
	double c2 = cos(t2);

	double r1_1 = 0.01692977392*pow(c2,4)*pow(t3,2) - 0.0005122649939*pow(c2,4)*t3 + 0.001497158703*pow(c2,4) - 0.09872759015*pow(c2,3)*pow(t3,2) - 0.0008191700436*pow(c2,3)*t3 - 0.01423283509*pow(c2,3) + 0.03385954784*pow(c2,2)*pow(s2,2)*pow(t3,2) - 0.001024529988*pow(c2,2)*pow(s2,2)*t3 + 0.002994317406*pow(c2,2)*pow(s2,2) + 0.00001885909284*pow(c2,2)*s2*t3 - 0.000339919713*pow(c2,2)*s2 + 0.1439348378*pow(c2,2)*pow(t3,2) + 0.006743747525*pow(c2,2)*t3 + 0.07826487932*pow(c2,2) - 0.09872759015*c2*pow(s2,2)*pow(t3,2) - 0.0008191700436*c2*pow(s2,2)*t3 - 0.01423283509*c2*pow(s2,2) + 0.00009008625051*c2*s2*t3 + 0.00035266128*c2*s2 + 0.01897574261*c2 + 0.01692977392*pow(s2,4)*pow(t3,2) - 0.0005122649939*pow(s2,4)*t3 + 0.001497158703*pow(s2,4) + 0.00001885909284*pow(s2,3)*t3 - 0.000339919713*pow(s2,3) + 0.3797361697*pow(s2,2)*pow(t3,2) - 0.0003911834176*pow(s2,2)*t3 + 0.1315274116*pow(s2,2) + 0.0003692169262*s2 + 0.02026589191;
	double r1_2 = 0.06541162346*pow(c2,2)*pow(t3,2) - 0.00197923995*pow(c2,2)*t3 + 0.005784577028*pow(c2,2) - 0.1907270582*c2*pow(t3,2) - 0.001582515003*c2*t3 + 0.009162593725*c2 + 0.06541162346*pow(s2,2)*pow(t3,2) - 0.00197923995*pow(s2,2)*t3 + 0.005784577028*pow(s2,2) + 0.00003643296967*s2*t3 + 0.00005659786982*s2 + 0.01052412488;
	double r1_3 = -0.00001249502675*pow(c2,2) + 0.00003643296967*c2 - 0.00001249502675*pow(s2,2) - 0.007353578204*s2;

	double e1_left = 0.3165424025*c1*c2 - 0.03541124607*s1 - 2.144340615*c1 + 0.0076895156*c1*s2 + 0.02971000684*c2*s1 - 2.374374248*c1*t3 - 1.223025927*s1*s2 + 0.00005659786982*c2*pow(t2_d,2) - 0.009162593725*s2*pow(t2_d,2) + 0.00003643296967*c2*t3*pow(t2_d,2) + 0.00035266128*pow(c2,2)*t1_d*t2_d + 0.006743747525*pow(c2,2)*t1_d*t3_d - 0.000339919713*pow(c2,3)*t1_d*t2_d - 0.00197923995*pow(c2,2)*t2_d*t3_d - 0.0008191700436*pow(c2,3)*t1_d*t3_d - 0.0005122649939*pow(c2,4)*t1_d*t3_d + 0.001582515003*s2*t3*pow(t2_d,2) - 0.00035266128*pow(s2,2)*t1_d*t2_d - 0.0003911834176*pow(s2,2)*t1_d*t3_d + 0.01423283509*pow(s2,3)*t1_d*t2_d - 0.00197923995*pow(s2,2)*t2_d*t3_d + 0.00001885909284*pow(s2,3)*t1_d*t3_d - 0.0005122649939*pow(s2,4)*t1_d*t3_d + 0.1907270582*s2*pow(t3,2)*pow(t2_d,2) + 0.8143137936*c1*c2*t3 + 0.0003692169262*c2*t1_d*t2_d - 0.008936093207*c2*t2_d*t3_d - 3.146266896*s1*s2*t3 - 0.01897574261*s2*t1_d*t2_d + 0.1065250645*c2*s2*t1_d*t2_d + 0.00009008625051*c2*s2*t1_d*t3_d - 0.3814541164*c2*t3*t2_d*t3_d - 0.000339919713*c2*pow(s2,2)*t1_d*t2_d + 0.01423283509*pow(c2,2)*s2*t1_d*t2_d - 0.0008191700436*c2*pow(s2,2)*t1_d*t3_d + 0.00001885909284*pow(c2,2)*s2*t1_d*t3_d + 0.00009008625051*pow(c2,2)*t3*t1_d*t2_d + 0.2878696757*pow(c2,2)*t3*t1_d*t3_d + 0.00001885909284*pow(c2,3)*t3*t1_d*t2_d + 0.1308232469*pow(c2,2)*t3*t2_d*t3_d - 0.1974551803*pow(c2,3)*t3*t1_d*t3_d + 0.03385954784*pow(c2,4)*t3*t1_d*t3_d - 0.00009008625051*pow(s2,2)*t3*t1_d*t2_d + 0.7594723393*pow(s2,2)*t3*t1_d*t3_d + 0.0008191700436*pow(s2,3)*t3*t1_d*t2_d + 0.1308232469*pow(s2,2)*t3*t2_d*t3_d + 0.03385954784*pow(s2,4)*t3*t1_d*t3_d - 0.001024529988*pow(c2,2)*pow(s2,2)*t1_d*t3_d + 0.09872759015*pow(s2,3)*pow(t3,2)*t1_d*t2_d + 0.09872759015*pow(c2,2)*s2*pow(t3,2)*t1_d*t2_d + 0.06771909569*pow(c2,2)*pow(s2,2)*t3*t1_d*t3_d - 0.01426986189*c2*s2*t3*t1_d*t2_d + 0.4716026637*c2*s2*pow(t3,2)*t1_d*t2_d + 0.00001885909284*c2*pow(s2,2)*t3*t1_d*t2_d + 0.0008191700436*pow(c2,2)*s2*t3*t1_d*t2_d - 0.1974551803*c2*pow(s2,2)*t3*t1_d*t3_d;


	double r2_1 = 0.06541162346*pow(c2,2)*pow(t3,2) - 0.00197923995*pow(c2,2)*t3 + 0.005784577028*pow(c2,2) - 0.1907270582*c2*pow(t3,2) - 0.001582515003*c2*t3 + 0.009162593725*c2 + 0.06541162346*pow(s2,2)*pow(t3,2) - 0.00197923995*pow(s2,2)*t3 + 0.005784577028*pow(s2,2) + 0.00003643296967*s2*t3 + 0.00005659786982*s2 + 0.01052412488;
	double r2_2 = 0.2527311058*pow(t3,2) - 0.007647195937*t3 + 0.06301198548;
	double r2_3 = -0.00004827707615;

	double e2_left =0.00017633064*pow(s2,2)*pow(t1_d,2) - 0.007116417545*pow(s2,3)*pow(t1_d,2) + 1.223025927*c1*c2 + 0.02971000684*c1*s2 + 0.0076895156*c2*s1 - 0.3165424025*s1*s2 - 0.007647195937*t2_d*t3_d - 0.0001846084631*c2*pow(t1_d,2) + 0.009487871303*s2*pow(t1_d,2) - 0.00017633064*pow(c2,2)*pow(t1_d,2) + 0.0001699598565*pow(c2,3)*pow(t1_d,2) - 0.05326253227*c2*s2*pow(t1_d,2) - 0.00197923995*pow(c2,2)*t1_d*t3_d - 0.00197923995*pow(s2,2)*t1_d*t3_d + 0.0001699598565*c2*pow(s2,2)*pow(t1_d,2) - 0.007116417545*pow(c2,2)*s2*pow(t1_d,2) - 0.00004504312525*pow(c2,2)*t3*pow(t1_d,2) - 0.000009429546421*pow(c2,3)*t3*pow(t1_d,2) + 0.00004504312525*pow(s2,2)*t3*pow(t1_d,2) - 0.0004095850218*pow(s2,3)*t3*pow(t1_d,2) + 3.146266896*c1*c2*t3 + 0.005771063202*c2*t1_d*t3_d - 0.8143137936*s1*s2*t3 + 0.00007286593934*s2*t1_d*t3_d + 0.5054622115*t3*t2_d*t3_d - 0.04936379507*pow(s2,3)*pow(t3,2)*pow(t1_d,2) - 0.3814541164*c2*t3*t1_d*t3_d - 0.04936379507*pow(c2,2)*s2*pow(t3,2)*pow(t1_d,2) + 0.007134930943*c2*s2*t3*pow(t1_d,2) + 0.1308232469*pow(c2,2)*t3*t1_d*t3_d + 0.1308232469*pow(s2,2)*t3*t1_d*t3_d - 0.2358013318*c2*s2*pow(t3,2)*pow(t1_d,2) - 0.000009429546421*c2*pow(s2,2)*t3*pow(t1_d,2) - 0.0004095850218*pow(c2,2)*s2*t3*pow(t1_d,2);


	double r3_1 = -0.00001249502675*pow(c2,2) + 0.00003643296967*c2 - 0.00001249502675*pow(s2,2) - 0.007353578204*s2;
	double r3_2 = -0.00004827707615;
	double r3_3 = 0.407;

	double e3_left = 0.0001955917088*pow(s2,2)*pow(t1_d,2) - 2.374374248*s1 - 0.000009429546421*pow(s2,3)*pow(t1_d,2) + 0.000256132497*pow(s2,4)*pow(t1_d,2) + 3.146266896*c1*s2 + 0.8143137936*c2*s1 - 0.2527311058*t3*pow(t2_d,2) + 0.003823597968*pow(t2_d,2) - 0.003371873763*pow(c2,2)*pow(t1_d,2) + 0.0004095850218*pow(c2,3)*pow(t1_d,2) + 0.000256132497*pow(c2,4)*pow(t1_d,2) - 0.00004504312525*c2*s2*pow(t1_d,2) + 0.00197923995*pow(c2,2)*t1_d*t2_d + 0.00197923995*pow(s2,2)*t1_d*t2_d + 0.0004095850218*c2*pow(s2,2)*pow(t1_d,2) - 0.000009429546421*pow(c2,2)*s2*pow(t1_d,2) - 0.1439348378*pow(c2,2)*t3*pow(t1_d,2) + 0.09872759015*pow(c2,3)*t3*pow(t1_d,2) - 0.01692977392*pow(c2,4)*t3*pow(t1_d,2) - 0.3797361697*pow(s2,2)*t3*pow(t1_d,2) - 0.01692977392*pow(s2,4)*t3*pow(t1_d,2) - 0.005771063202*c2*t1_d*t2_d - 0.00007286593934*s2*t1_d*t2_d + 0.0005122649939*pow(c2,2)*pow(s2,2)*pow(t1_d,2) + 0.3814541164*c2*t3*t1_d*t2_d - 0.03385954784*pow(c2,2)*pow(s2,2)*t3*pow(t1_d,2) - 0.1308232469*pow(c2,2)*t3*t1_d*t2_d - 0.1308232469*pow(s2,2)*t3*t1_d*t2_d + 0.09872759015*c2*pow(s2,2)*t3*pow(t1_d,2);

	Array2D<double> M(3,3);
	M[0][0]=r1_1;
	M[0][1]=r1_2;
	M[0][2]=r1_3;

	M[1][0]=r2_1;
	M[1][1]=r2_2;
	M[1][2]=r2_3;

	M[2][0]=r3_1;
	M[2][1]=r3_2;
	M[2][2]=r3_3;


	// here comes the first three motor dynamics models
	double N = 12.25;
	double Im = 2.847e-4;

	/*double ke1 = 720e3;
	double be1 = 4000*1;
	double tau_cm1 = 8.11e-2*0.9;
	double tau_vm1 = 1.9e-4*2;
	 double r_mc1 = 5.675e-3;
	 double r_l1 = 63.095e-3;
	 double F_cl1 = 0.505*1.0;
	 double F_vl1 = 0.00505*1.0;

	 double ke2 = 1*88e3;
	 double be2 = 1.2*1380;
	 double tau_cm2 = 4.11e-2*1.2;
	 double tau_vm2 = 1.8e-4*12;//16
	 double r_mc2 = 5.675e-3;
	 double r_l2 = 56.298e-3;
	 double F_cl2 = 0.49*1;
	 double F_vl2 = 0.0049*1;

	 double ke3 = 1*9.25e3;
	 double be3 = 1.0*400;
	 double tau_cm3 = 5.11e-2*2;
	 double tau_vm3 = 7.1e-4*2;
	 double r_mc3 = 5.675e-3;
	 double r_l3 = 1;
	 double F_cl3 = 0.53*10;
	 double F_vl3 = 0.0053*10;*/


	/* double ke1 = 720e3;
	 double be1 = 4000*1;
	 double tau_cm1 = 8.11e-2*1.0;
	 double tau_vm1 = 1.9e-4*1.0;
	 double r_mc1 = 5.675e-3;
	 double r_l1 = 63.095e-3;
	 double F_cl1 = 0.505*1.0;
	 double F_vl1 = 0.00505*1.0;

	 double ke2 = 1*88e3;
	 double be2 = 1*1380;
	 double tau_cm2 = 4.11e-2*1.0;
	 double tau_vm2 = 1.8e-4*1;//16
	 double r_mc2 = 5.675e-3;
	 double r_l2 = 56.298e-3;
	 double F_cl2 = 0.49*1;
	 double F_vl2 = 0.0049*1;

	 double ke3 = 1*9.25e3;
	 double be3 = 1.0*400;
	 double tau_cm3 = 5.11e-2;
	 double tau_vm3 = 7.1e-4*1;
	 double r_mc3 = 5.675e-3;
	 double r_l3 = 1;
	 double F_cl3 = 0.53*1;
	 double F_vl3 = 0.0053*1;*/

/*
 	 double ke1 = 720e3;
	 double be1 = 4000*1;
	 double tau_cm1 = 8.11e-2*0.75;
	 double tau_vm1 = 1.9e-4*1;
	 double r_mc1 = 5.675e-3;
	 double r_l1 = 63.095e-3;
	 double F_cl1 = 0.505*1.0;
	 double F_vl1 = 0.00505*1.0;

	 double ke2 = 1*88e3;
	 double be2 = 1*1380;
	 double tau_cm2 = 4.11e-2*2.0;
	 double tau_vm2 = 1.8e-4*15.0;//16
	 double r_mc2 = 5.675e-3;
	 double r_l2 = 56.298e-3;
	 double F_cl2 = 0.49*1;
	 double F_vl2 = 0.0049*0.5;

	 double ke3 = 1.0*9.25e3;
	 double be3 = 1.0*400;
	 double tau_cm3 = 5.11e-2*0.8;
	 double tau_vm3 = 7.1e-4*5;
	 double r_mc3 = 5.675e-3*1.0;
	 double r_l3 = 1.0;
	 double F_cl3 = 0.53*0.2;
	 double F_vl3 = 0.0053*0.1;*/ //NOV 24 	tuned parameters


	 /*double ke1 = 720e3;
	 double be1 = 4000*1;
	 double tau_cm1 = 8.11e-2*0.45;
	 double tau_vm1 = 1.9e-4*14.0;
	 double r_mc1 = 5.675e-3;
	 double r_l1 = 63.095e-3;
	 double F_cl1 = 0.505*1.0;
	 double F_vl1 = 0.00505*1.0;

	 double ke2 = 1*88e3;
	 double be2 = 1*1380;
	 double tau_cm2 = 4.11e-2*1.75;
	 double tau_vm2 = 1.8e-4*14.5;//16
	 double r_mc2 = 5.675e-3;
	 double r_l2 = 56.298e-3;
	 double F_cl2 = 0.49*1;
	 double F_vl2 = 0.0049*0.5;

	 double ke3 = 1.0*9.25e3;
	 double be3 = 1.0*400;
	 double tau_cm3 = 5.11e-2*1.0;
	 double tau_vm3 = 7.1e-4*5.3;
	 double r_mc3 = 5.675e-3*1.0;
	 double r_l3 = 1.0;
	 double F_cl3 = 0.53*1.0;
	 double F_vl3 = 0.0053*1.0; //Nov 24, afternoon tuned*/

 	 /*double ke1 = 720e3;
	 double be1 = 4000*1;
	 double tau_cm1 = 8.11e-2*0.45;
	 double tau_vm1 = 1.9e-4*12.5;//14
	 double r_mc1 = 5.675e-3;
	 double r_l1 = 63.095e-3;
	 double F_cl1 = 0.505*1.0;
	 double F_vl1 = 0.00505*1.0;

	 double ke2 = 1*88e3;
	 double be2 = 1*1380;
	 double tau_cm2 = 4.11e-2*1.75;
	 double tau_vm2 = 1.8e-4*11.5;//14.5
	 double r_mc2 = 5.675e-3;
	 double r_l2 = 56.298e-3;
	 double F_cl2 = 0.49*1;
	 double F_vl2 = 0.0049*0.5;

	 double ke3 = 1.0*9.25e3;
	 double be3 = 1.0*400;
	 double tau_cm3 = 5.11e-2*1.0;
	 double tau_vm3 = 7.1e-4*5.0;  //5.3
	 double r_mc3 = 5.675e-3*1.0;
	 double r_l3 = 1.0;
	 double F_cl3 = 0.53*1.0;
	 double F_vl3 = 0.0053*1.0; //Nov 27, afternoon tuned*/

	// tune for traj 3
	 double ke1 = 720e3;
	 double be1 = 4000*1;
	 double tau_cm1 = 8.11e-2*0.5*1;
	 double tau_vm1 = 1.9e-4*13;//14
	 double r_mc1 = 5.675e-3;
	 double r_l1 = 63.095e-3;
	 double F_cl1 = 0.505*1.0;
	 double F_vl1 = 0.00505*1.0;

	 double ke2 = 2*88e3;
	 double be2 = 1*1380;
	 double tau_cm2 = 4.11e-2*1.75;
	 double tau_vm2 = 1.8e-4*13.5;//14.5
	 double r_mc2 = 5.675e-3;
	 double r_l2 = 56.298e-3;
	 double F_cl2 = 0.49*1;
	 double F_vl2 = 0.0049*0.5;

	 double ke3 = 1.0*9.25e3;
	 double be3 = 1.0*400;
	 double tau_cm3 = 5.11e-2*1.0;
	 double tau_vm3 = 7.1e-4*4.0;  //5.3
	 double r_mc3 = 5.675e-3*1.0;
	 double r_l3 = 1.0;
	 double F_cl3 = 0.53*1.0;
	 double F_vl3 = 0.0053*1.0;



	double tau_m1 = tau_cm1*sign_func(x[9])+tau_vm1*x[9];
	double q_mc1 = x[6]/N;
	double d_q_mc1 = x[9]/N;
	double gamma1 = ke1*(exp(q_mc1*r_mc1-(x[0]+PI)*r_l1)-exp((x[0]+PI)*r_l1-q_mc1*r_mc1))+2*be1*(d_q_mc1*r_mc1-x[3]*r_l1);
	double tau_rn1 = r_mc1*gamma1/N;

	double tau_m2 = tau_cm2*sign_func(x[10])+tau_vm2*x[10];
	double q_mc2 = x[7]/N;
	double d_q_mc2 = x[10]/N;
	double gamma2 = ke2*(exp(q_mc2*r_mc2-(x[1]+PI)*r_l2)-exp((x[1]+PI)*r_l2-q_mc2*r_mc2))+2*be2*(d_q_mc2*r_mc2-x[4]*r_l2);
	double tau_rn2 = r_mc2*gamma2/N;

	double tau_m3 = tau_cm3*sign_func(x[11])+tau_vm3*x[11];
	double q_mc3 = x[8]/N;
	double d_q_mc3 = x[11]/N;
	double gamma3 = ke3*(exp(q_mc3*r_mc3-(x[2])*r_l3)-exp((x[2])*r_l3-q_mc3*r_mc3))+2*be3*(d_q_mc3*r_mc3-x[5]*r_l3);
	double tau_rn3 = r_mc3*gamma1/N;

	//double input1 = (2.579399807733391e-04*DACs[iter][0]+1.1*0.0378)*0.8;
	//double input2 = (2.579399807733391e-04*DACs[iter][1]+0.94*0.0672)*0.8;
	//double input3 = (2.579399807733391e-04*DACs[iter][2]-0.91*0.0398)*1;

	//double input1 = (2.737629663234212e-04*DACs[0]+1.0*0.041613800000000)*1.0;
  //  double input2 = (2.737629663234212e-04*DACs[1]+1.0*0.0697023000)*0.85;
    //double input3 = (2.737629663234212e-04*DACs[2]-1.0*0.039108200000000)*1;

	double input1 = (2.737629663234212e-04*(DACs[0])-1.153*0.041613800000000)*0.9; //1.16 1.0 new 1.183
    double input2 = (2.737629663234212e-04*(DACs[1])-0.9*0.0697023000)*1.0;  //0.9 1.0
    double input3 = (2.737629663234212e-04*(DACs[2])+0.4*0.039108200000000)*0.9; //0.4 1.0


	dxdt[9] = 1/Im*(-input1-tau_m1-tau_rn1);
	dxdt[10] = 1/Im*(-input2-tau_m2-tau_rn2);
	dxdt[11] = 1/Im*(-input3-tau_m3-tau_rn3);

	//cout<<"1: "<<input1<<", "<<tau_m1<<", "<<tau_rn1<<endl;
	//cout<<"2: "<<input2<<", "<<tau_m2<<", "<<tau_rn2<<endl;
	//cout<<"3: "<<input3<<", "<<tau_m3<<", "<<tau_rn3<<endl;

	double tau1 = gamma1*r_l1;
	double tau2 = gamma2*r_l2;
	double tau3 = gamma3*r_l3;

	Array2D<double> right_side(3,1),dd(3,1);

	right_side[0][0] = tau1-x[3]*F_vl1-sign_func(x[3])*F_cl1-e1_left;
	right_side[1][0] = tau2-x[4]*F_vl2-sign_func(x[4])*F_cl2-e2_left;
	right_side[2][0] = tau3-x[5]*F_vl3-sign_func(x[5])*F_cl3-e3_left;

	dd= matmult(inverse(M),right_side);

	dxdt[3] = dd[0][0];
	dxdt[4] = dd[1][0];
	dxdt[5] = dd[2][0];
}


void sys_dyn_gold_euler(state_type &x, double dt)
{

state_type dxdt = {0,0,0,0,0,0,0,0,0,0,0,0};
// dt's unit is in milisecond
int total = 1/dt;

for(int i = 0;i<total;i++){

	double t1  = x[0];
	double t2  = x[1];
	double t3  = x[2];

	double t1_d = x[3];
	double t2_d = x[4];
	double t3_d = x[5];

	double tm_1 = x[6];
	double tm_2 = x[7];
	double tm_3 = x[8];

	dxdt[0] = x[3];
	dxdt[1] = x[4];
	dxdt[2] = x[5];

	dxdt[6] = x[9];
	dxdt[7] = x[10];
	dxdt[8] = x[11];

	double s1 = sin(t1);
	double s2 = sin(t2);
	double c1 = cos(t1);
	double c2 = cos(t2);

	double r1_1 = 0.01692977392*pow(c2,4)*pow(t3,2) - 0.0005122649939*pow(c2,4)*t3 + 0.001497158703*pow(c2,4) - 0.09872759015*pow(c2,3)*pow(t3,2) - 0.0008191700436*pow(c2,3)*t3 - 0.01423283509*pow(c2,3) + 0.03385954784*pow(c2,2)*pow(s2,2)*pow(t3,2) - 0.001024529988*pow(c2,2)*pow(s2,2)*t3 + 0.002994317406*pow(c2,2)*pow(s2,2) + 0.00001885909284*pow(c2,2)*s2*t3 - 0.000339919713*pow(c2,2)*s2 + 0.1439348378*pow(c2,2)*pow(t3,2) + 0.006743747525*pow(c2,2)*t3 + 0.07826487932*pow(c2,2) - 0.09872759015*c2*pow(s2,2)*pow(t3,2) - 0.0008191700436*c2*pow(s2,2)*t3 - 0.01423283509*c2*pow(s2,2) + 0.00009008625051*c2*s2*t3 + 0.00035266128*c2*s2 + 0.01897574261*c2 + 0.01692977392*pow(s2,4)*pow(t3,2) - 0.0005122649939*pow(s2,4)*t3 + 0.001497158703*pow(s2,4) + 0.00001885909284*pow(s2,3)*t3 - 0.000339919713*pow(s2,3) + 0.3797361697*pow(s2,2)*pow(t3,2) - 0.0003911834176*pow(s2,2)*t3 + 0.1315274116*pow(s2,2) + 0.0003692169262*s2 + 0.02026589191;
	double r1_2 = 0.06541162346*pow(c2,2)*pow(t3,2) - 0.00197923995*pow(c2,2)*t3 + 0.005784577028*pow(c2,2) - 0.1907270582*c2*pow(t3,2) - 0.001582515003*c2*t3 + 0.009162593725*c2 + 0.06541162346*pow(s2,2)*pow(t3,2) - 0.00197923995*pow(s2,2)*t3 + 0.005784577028*pow(s2,2) + 0.00003643296967*s2*t3 + 0.00005659786982*s2 + 0.01052412488;
	double r1_3 = -0.00001249502675*pow(c2,2) + 0.00003643296967*c2 - 0.00001249502675*pow(s2,2) - 0.007353578204*s2;

	double e1_left = 0.3165424025*c1*c2 - 0.03541124607*s1 - 2.144340615*c1 + 0.0076895156*c1*s2 + 0.02971000684*c2*s1 - 2.374374248*c1*t3 - 1.223025927*s1*s2 + 0.00005659786982*c2*pow(t2_d,2) - 0.009162593725*s2*pow(t2_d,2) + 0.00003643296967*c2*t3*pow(t2_d,2) + 0.00035266128*pow(c2,2)*t1_d*t2_d + 0.006743747525*pow(c2,2)*t1_d*t3_d - 0.000339919713*pow(c2,3)*t1_d*t2_d - 0.00197923995*pow(c2,2)*t2_d*t3_d - 0.0008191700436*pow(c2,3)*t1_d*t3_d - 0.0005122649939*pow(c2,4)*t1_d*t3_d + 0.001582515003*s2*t3*pow(t2_d,2) - 0.00035266128*pow(s2,2)*t1_d*t2_d - 0.0003911834176*pow(s2,2)*t1_d*t3_d + 0.01423283509*pow(s2,3)*t1_d*t2_d - 0.00197923995*pow(s2,2)*t2_d*t3_d + 0.00001885909284*pow(s2,3)*t1_d*t3_d - 0.0005122649939*pow(s2,4)*t1_d*t3_d + 0.1907270582*s2*pow(t3,2)*pow(t2_d,2) + 0.8143137936*c1*c2*t3 + 0.0003692169262*c2*t1_d*t2_d - 0.008936093207*c2*t2_d*t3_d - 3.146266896*s1*s2*t3 - 0.01897574261*s2*t1_d*t2_d + 0.1065250645*c2*s2*t1_d*t2_d + 0.00009008625051*c2*s2*t1_d*t3_d - 0.3814541164*c2*t3*t2_d*t3_d - 0.000339919713*c2*pow(s2,2)*t1_d*t2_d + 0.01423283509*pow(c2,2)*s2*t1_d*t2_d - 0.0008191700436*c2*pow(s2,2)*t1_d*t3_d + 0.00001885909284*pow(c2,2)*s2*t1_d*t3_d + 0.00009008625051*pow(c2,2)*t3*t1_d*t2_d + 0.2878696757*pow(c2,2)*t3*t1_d*t3_d + 0.00001885909284*pow(c2,3)*t3*t1_d*t2_d + 0.1308232469*pow(c2,2)*t3*t2_d*t3_d - 0.1974551803*pow(c2,3)*t3*t1_d*t3_d + 0.03385954784*pow(c2,4)*t3*t1_d*t3_d - 0.00009008625051*pow(s2,2)*t3*t1_d*t2_d + 0.7594723393*pow(s2,2)*t3*t1_d*t3_d + 0.0008191700436*pow(s2,3)*t3*t1_d*t2_d + 0.1308232469*pow(s2,2)*t3*t2_d*t3_d + 0.03385954784*pow(s2,4)*t3*t1_d*t3_d - 0.001024529988*pow(c2,2)*pow(s2,2)*t1_d*t3_d + 0.09872759015*pow(s2,3)*pow(t3,2)*t1_d*t2_d + 0.09872759015*pow(c2,2)*s2*pow(t3,2)*t1_d*t2_d + 0.06771909569*pow(c2,2)*pow(s2,2)*t3*t1_d*t3_d - 0.01426986189*c2*s2*t3*t1_d*t2_d + 0.4716026637*c2*s2*pow(t3,2)*t1_d*t2_d + 0.00001885909284*c2*pow(s2,2)*t3*t1_d*t2_d + 0.0008191700436*pow(c2,2)*s2*t3*t1_d*t2_d - 0.1974551803*c2*pow(s2,2)*t3*t1_d*t3_d;


	double r2_1 = 0.06541162346*pow(c2,2)*pow(t3,2) - 0.00197923995*pow(c2,2)*t3 + 0.005784577028*pow(c2,2) - 0.1907270582*c2*pow(t3,2) - 0.001582515003*c2*t3 + 0.009162593725*c2 + 0.06541162346*pow(s2,2)*pow(t3,2) - 0.00197923995*pow(s2,2)*t3 + 0.005784577028*pow(s2,2) + 0.00003643296967*s2*t3 + 0.00005659786982*s2 + 0.01052412488;
	double r2_2 = 0.2527311058*pow(t3,2) - 0.007647195937*t3 + 0.06301198548;
	double r2_3 = -0.00004827707615;

	double e2_left =0.00017633064*pow(s2,2)*pow(t1_d,2) - 0.007116417545*pow(s2,3)*pow(t1_d,2) + 1.223025927*c1*c2 + 0.02971000684*c1*s2 + 0.0076895156*c2*s1 - 0.3165424025*s1*s2 - 0.007647195937*t2_d*t3_d - 0.0001846084631*c2*pow(t1_d,2) + 0.009487871303*s2*pow(t1_d,2) - 0.00017633064*pow(c2,2)*pow(t1_d,2) + 0.0001699598565*pow(c2,3)*pow(t1_d,2) - 0.05326253227*c2*s2*pow(t1_d,2) - 0.00197923995*pow(c2,2)*t1_d*t3_d - 0.00197923995*pow(s2,2)*t1_d*t3_d + 0.0001699598565*c2*pow(s2,2)*pow(t1_d,2) - 0.007116417545*pow(c2,2)*s2*pow(t1_d,2) - 0.00004504312525*pow(c2,2)*t3*pow(t1_d,2) - 0.000009429546421*pow(c2,3)*t3*pow(t1_d,2) + 0.00004504312525*pow(s2,2)*t3*pow(t1_d,2) - 0.0004095850218*pow(s2,3)*t3*pow(t1_d,2) + 3.146266896*c1*c2*t3 + 0.005771063202*c2*t1_d*t3_d - 0.8143137936*s1*s2*t3 + 0.00007286593934*s2*t1_d*t3_d + 0.5054622115*t3*t2_d*t3_d - 0.04936379507*pow(s2,3)*pow(t3,2)*pow(t1_d,2) - 0.3814541164*c2*t3*t1_d*t3_d - 0.04936379507*pow(c2,2)*s2*pow(t3,2)*pow(t1_d,2) + 0.007134930943*c2*s2*t3*pow(t1_d,2) + 0.1308232469*pow(c2,2)*t3*t1_d*t3_d + 0.1308232469*pow(s2,2)*t3*t1_d*t3_d - 0.2358013318*c2*s2*pow(t3,2)*pow(t1_d,2) - 0.000009429546421*c2*pow(s2,2)*t3*pow(t1_d,2) - 0.0004095850218*pow(c2,2)*s2*t3*pow(t1_d,2);


	double r3_1 = -0.00001249502675*pow(c2,2) + 0.00003643296967*c2 - 0.00001249502675*pow(s2,2) - 0.007353578204*s2;
	double r3_2 = -0.00004827707615;
	double r3_3 = 0.407;

	double e3_left = 0.0001955917088*pow(s2,2)*pow(t1_d,2) - 2.374374248*s1 - 0.000009429546421*pow(s2,3)*pow(t1_d,2) + 0.000256132497*pow(s2,4)*pow(t1_d,2) + 3.146266896*c1*s2 + 0.8143137936*c2*s1 - 0.2527311058*t3*pow(t2_d,2) + 0.003823597968*pow(t2_d,2) - 0.003371873763*pow(c2,2)*pow(t1_d,2) + 0.0004095850218*pow(c2,3)*pow(t1_d,2) + 0.000256132497*pow(c2,4)*pow(t1_d,2) - 0.00004504312525*c2*s2*pow(t1_d,2) + 0.00197923995*pow(c2,2)*t1_d*t2_d + 0.00197923995*pow(s2,2)*t1_d*t2_d + 0.0004095850218*c2*pow(s2,2)*pow(t1_d,2) - 0.000009429546421*pow(c2,2)*s2*pow(t1_d,2) - 0.1439348378*pow(c2,2)*t3*pow(t1_d,2) + 0.09872759015*pow(c2,3)*t3*pow(t1_d,2) - 0.01692977392*pow(c2,4)*t3*pow(t1_d,2) - 0.3797361697*pow(s2,2)*t3*pow(t1_d,2) - 0.01692977392*pow(s2,4)*t3*pow(t1_d,2) - 0.005771063202*c2*t1_d*t2_d - 0.00007286593934*s2*t1_d*t2_d + 0.0005122649939*pow(c2,2)*pow(s2,2)*pow(t1_d,2) + 0.3814541164*c2*t3*t1_d*t2_d - 0.03385954784*pow(c2,2)*pow(s2,2)*t3*pow(t1_d,2) - 0.1308232469*pow(c2,2)*t3*t1_d*t2_d - 0.1308232469*pow(s2,2)*t3*t1_d*t2_d + 0.09872759015*c2*pow(s2,2)*t3*pow(t1_d,2);

	Array2D<double> M(3,3);
	M[0][0]=r1_1;
	M[0][1]=r1_2;
	M[0][2]=r1_3;

	M[1][0]=r2_1;
	M[1][1]=r2_2;
	M[1][2]=r2_3;

	M[2][0]=r3_1;
	M[2][1]=r3_2;
	M[2][2]=r3_3;


	// here comes the first three motor dynamics models
	double N = 12.25;
	double Im = 2.847e-4;

	/*double ke1 = 720e3;

	double be1 = 4000*1;
	double tau_cm1 = 8.11e-2*0.9;
	double tau_vm1 = 1.9e-4*2;
	 double r_mc1 = 5.675e-3;
	 double r_l1 = 63.095e-3;
	 double F_cl1 = 0.505*1.0;

	 double F_vl1 = 0.00505*1.0;

	 double ke2 = 1*88e3;
	 double be2 = 1.2*1380;
	 double tau_cm2 = 4.11e-2*1.2;

	 double tau_vm2 = 1.8e-4*12;//16
	 double r_mc2 = 5.675e-3;
	 double r_l2 = 56.298e-3;
	 double F_cl2 = 0.49*1;
	 double F_vl2 = 0.0049*1;


	 double ke3 = 1*9.25e3;
	 double be3 = 1.0*400;
	 double tau_cm3 = 5.11e-2*2;
	 double tau_vm3 = 7.1e-4*2;
	 double r_mc3 = 5.675e-3;

	 double r_l3 = 1;
	 double F_cl3 = 0.53*10;
	 double F_vl3 = 0.0053*10;*/


	/* double ke1 = 720e3;
	 double be1 = 4000*1;
	 double tau_cm1 = 8.11e-2*1.0;
	 double tau_vm1 = 1.9e-4*1.0;
	 double r_mc1 = 5.675e-3;

	 double r_l1 = 63.095e-3;
	 double F_cl1 = 0.505*1.0;
	 double F_vl1 = 0.00505*1.0;

	 double ke2 = 1*88e3;

	 double be2 = 1*1380;
	 double tau_cm2 = 4.11e-2*1.0;
	 double tau_vm2 = 1.8e-4*1;//16
	 double r_mc2 = 5.675e-3;
	 double r_l2 = 56.298e-3;
	 double F_cl2 = 0.49*1;

	 double F_vl2 = 0.0049*1;

	 double ke3 = 1*9.25e3;
	 double be3 = 1.0*400;
	 double tau_cm3 = 5.11e-2;

	 double tau_vm3 = 7.1e-4*1;
	 double r_mc3 = 5.675e-3;
	 double r_l3 = 1;
	 double F_cl3 = 0.53*1;
	 double F_vl3 = 0.0053*1;*/

/*
 	 double ke1 = 720e3;
	 double be1 = 4000*1;
	 double tau_cm1 = 8.11e-2*0.75;

	 double tau_vm1 = 1.9e-4*1;
	 double r_mc1 = 5.675e-3;
	 double r_l1 = 63.095e-3;
	 double F_cl1 = 0.505*1.0;
	 double F_vl1 = 0.00505*1.0;


	 double ke2 = 1*88e3;
	 double be2 = 1*1380;
	 double tau_cm2 = 4.11e-2*2.0;
	 double tau_vm2 = 1.8e-4*15.0;//16
	 double r_mc2 = 5.675e-3;

	 double r_l2 = 56.298e-3;
	 double F_cl2 = 0.49*1;
	 double F_vl2 = 0.0049*0.5;

	 double ke3 = 1.0*9.25e3;

	 double be3 = 1.0*400;
	 double tau_cm3 = 5.11e-2*0.8;
	 double tau_vm3 = 7.1e-4*5;
	 double r_mc3 = 5.675e-3*1.0;
	 double r_l3 = 1.0;

	 double F_cl3 = 0.53*0.2;
	 double F_vl3 = 0.0053*0.1;*/ //NOV 24 	tuned parameters


	 /*double ke1 = 720e3;
	 double be1 = 4000*1;

	 double tau_cm1 = 8.11e-2*0.45;
	 double tau_vm1 = 1.9e-4*14.0;
	 double r_mc1 = 5.675e-3;
	 double r_l1 = 63.095e-3;
	 double F_cl1 = 0.505*1.0;

	 double F_vl1 = 0.00505*1.0;

	 double ke2 = 1*88e3;
	 double be2 = 1*1380;
	 double tau_cm2 = 4.11e-2*1.75;

	 double tau_vm2 = 1.8e-4*14.5;//16
	 double r_mc2 = 5.675e-3;
	 double r_l2 = 56.298e-3;
	 double F_cl2 = 0.49*1;
	 double F_vl2 = 0.0049*0.5;


	 double ke3 = 1.0*9.25e3;
	 double be3 = 1.0*400;
	 double tau_cm3 = 5.11e-2*1.0;
	 double tau_vm3 = 7.1e-4*5.3;
	 double r_mc3 = 5.675e-3*1.0;

	 double r_l3 = 1.0;
	 double F_cl3 = 0.53*1.0;
	 double F_vl3 = 0.0053*1.0; //Nov 24, afternoon tuned*/

 	 /*double ke1 = 720e3;

	 double be1 = 4000*1;
	 double tau_cm1 = 8.11e-2*0.45;
	 double tau_vm1 = 1.9e-4*12.5;//14
	 double r_mc1 = 5.675e-3;
	 double r_l1 = 63.095e-3;

	 double F_cl1 = 0.505*1.0;
	 double F_vl1 = 0.00505*1.0;

	 double ke2 = 1*88e3;
	 double be2 = 1*1380;

	 double tau_cm2 = 4.11e-2*1.75;
	 double tau_vm2 = 1.8e-4*11.5;//14.5
	 double r_mc2 = 5.675e-3;
	 double r_l2 = 56.298e-3;
	 double F_cl2 = 0.49*1;
	 double F_vl2 = 0.0049*0.5;



	 double ke3 = 1.0*9.25e3;
	 double be3 = 1.0*400;
	 double tau_cm3 = 5.11e-2*1.0;
	 double tau_vm3 = 7.1e-4*5.0;  //5.3
	 double r_mc3 = 5.675e-3*1.0;
	 double r_l3 = 1.0;
	 double F_cl3 = 0.53*1.0;
	 double F_vl3 = 0.0053*1.0; //Nov 27, afternoon tuned*/

	// tune for traj 3
	 double ke1 = 720e3;
	 double be1 = 4000*1;
	 double tau_cm1 = 8.11e-2*0.5*1;
	 double tau_vm1 = 1.9e-4*13;//14
	 double r_mc1 = 5.675e-3;
	 double r_l1 = 63.095e-3;
	 double F_cl1 = 0.505*1.0;
	 double F_vl1 = 0.00505*1.0;

	 double ke2 = 2*88e3;
	 double be2 = 1*1380;
	 double tau_cm2 = 4.11e-2*1.75;
	 double tau_vm2 = 1.8e-4*13.5;//14.5
	 double r_mc2 = 5.675e-3;
	 double r_l2 = 56.298e-3;
	 double F_cl2 = 0.49*1;
	 double F_vl2 = 0.0049*0.5;

	 double ke3 = 1.0*9.25e3;
	 double be3 = 1.0*400;
	 double tau_cm3 = 5.11e-2*1.0;
	 double tau_vm3 = 7.1e-4*4.0;  //5.3
	 double r_mc3 = 5.675e-3*1.0;
	 double r_l3 = 1.0;
	 double F_cl3 = 0.53*1.0;
	 double F_vl3 = 0.0053*1.0;



	double tau_m1 = tau_cm1*sign_func(x[9])+tau_vm1*x[9];
	double q_mc1 = x[6]/N;
	double d_q_mc1 = x[9]/N;
	double gamma1 = ke1*(exp(q_mc1*r_mc1-(x[0]+PI)*r_l1)-exp((x[0]+PI)*r_l1-q_mc1*r_mc1))+2*be1*(d_q_mc1*r_mc1-x[3]*r_l1);
	double tau_rn1 = r_mc1*gamma1/N;

	double tau_m2 = tau_cm2*sign_func(x[10])+tau_vm2*x[10];
	double q_mc2 = x[7]/N;
	double d_q_mc2 = x[10]/N;
	double gamma2 = ke2*(exp(q_mc2*r_mc2-(x[1]+PI)*r_l2)-exp((x[1]+PI)*r_l2-q_mc2*r_mc2))+2*be2*(d_q_mc2*r_mc2-x[4]*r_l2);
	double tau_rn2 = r_mc2*gamma2/N;

	double tau_m3 = tau_cm3*sign_func(x[11])+tau_vm3*x[11];
	double q_mc3 = x[8]/N;
	double d_q_mc3 = x[11]/N;
	double gamma3 = ke3*(exp(q_mc3*r_mc3-(x[2])*r_l3)-exp((x[2])*r_l3-q_mc3*r_mc3))+2*be3*(d_q_mc3*r_mc3-x[5]*r_l3);
	double tau_rn3 = r_mc3*gamma1/N;

	//double input1 = (2.579399807733391e-04*DACs[iter][0]+1.1*0.0378)*0.8;
	//double input2 = (2.579399807733391e-04*DACs[iter][1]+0.94*0.0672)*0.8;
	//double input3 = (2.579399807733391e-04*DACs[iter][2]-0.91*0.0398)*1;

	//double input1 = (2.737629663234212e-04*DACs[0]+1.0*0.041613800000000)*1.0;
  //  double input2 = (2.737629663234212e-04*DACs[1]+1.0*0.0697023000)*0.85;
    //double input3 = (2.737629663234212e-04*DACs[2]-1.0*0.039108200000000)*1;

	double input1 = (2.737629663234212e-04*(DACs[0])-1.153*0.041613800000000)*0.9; //1.16 1.0 new 1.183
    double input2 = (2.737629663234212e-04*(DACs[1])-0.9*0.0697023000)*1.0;  //0.9 1.0
    double input3 = (2.737629663234212e-04*(DACs[2])+0.4*0.039108200000000)*0.9; //0.4 1.0


	dxdt[9] = 1/Im*(-input1-tau_m1-tau_rn1);
	dxdt[10] = 1/Im*(-input2-tau_m2-tau_rn2);
	dxdt[11] = 1/Im*(-input3-tau_m3-tau_rn3);

	//cout<<"1: "<<input1<<", "<<tau_m1<<", "<<tau_rn1<<endl;
	//cout<<"2: "<<input2<<", "<<tau_m2<<", "<<tau_rn2<<endl;
	//cout<<"3: "<<input3<<", "<<tau_m3<<", "<<tau_rn3<<endl;

	double tau1 = gamma1*r_l1;
	double tau2 = gamma2*r_l2;
	double tau3 = gamma3*r_l3;

	Array2D<double> right_side(3,1),dd(3,1);

	right_side[0][0] = tau1-x[3]*F_vl1-sign_func(x[3])*F_cl1-e1_left;
	right_side[1][0] = tau2-x[4]*F_vl2-sign_func(x[4])*F_cl2-e2_left;
	right_side[2][0] = tau3-x[5]*F_vl3-sign_func(x[5])*F_cl3-e3_left;

	dd= matmult(inverse(M),right_side);

	dxdt[3] = dd[0][0];
	dxdt[4] = dd[1][0];
	dxdt[5] = dd[2][0];

	x[0] = x[0]+dxdt[0]*dt/1000.0;
	x[1] = x[1]+dxdt[1]*dt/1000.0;
	x[2] = x[2]+dxdt[2]*dt/1000.0;
	x[3] = x[3]+dxdt[3]*dt/1000.0;
	x[4] = x[4]+dxdt[4]*dt/1000.0;
	x[5] = x[5]+dxdt[5]*dt/1000.0;
	x[6] = x[6]+dxdt[6]*dt/1000.0;
	x[7] = x[7]+dxdt[7]*dt/1000.0;
	x[8] = x[8]+dxdt[8]*dt/1000.0;
	x[9] = x[9]+dxdt[9]*dt/1000.0;
	x[10] = x[10]+dxdt[10]*dt/1000.0;
	x[11] = x[11]+dxdt[11]*dt/1000.0;
}
}


int sys_dyn_gold_euler_detect(state_type &x, double dt,int iter_time)
{

state_type dxdt = {0,0,0,0,0,0,0,0,0,0,0,0};
// dt's unit is in milisecond
int total = 1/dt;

for(int i = 0;i<total;i++){

	double t1  = x[0];
	double t2  = x[1];
	double t3  = x[2];

	double t1_d = x[3];
	double t2_d = x[4];
	double t3_d = x[5];

	double tm_1 = x[6];
	double tm_2 = x[7];
	double tm_3 = x[8];

	dxdt[0] = x[3];
	dxdt[1] = x[4];
	dxdt[2] = x[5];

	dxdt[6] = x[9];
	dxdt[7] = x[10];
	dxdt[8] = x[11];

	double s1 = sin(t1);
	double s2 = sin(t2);
	double c1 = cos(t1);
	double c2 = cos(t2);

	double r1_1 = 0.01692977392*pow(c2,4)*pow(t3,2) - 0.0005122649939*pow(c2,4)*t3 + 0.001497158703*pow(c2,4) - 0.09872759015*pow(c2,3)*pow(t3,2) - 0.0008191700436*pow(c2,3)*t3 - 0.01423283509*pow(c2,3) + 0.03385954784*pow(c2,2)*pow(s2,2)*pow(t3,2) - 0.001024529988*pow(c2,2)*pow(s2,2)*t3 + 0.002994317406*pow(c2,2)*pow(s2,2) + 0.00001885909284*pow(c2,2)*s2*t3 - 0.000339919713*pow(c2,2)*s2 + 0.1439348378*pow(c2,2)*pow(t3,2) + 0.006743747525*pow(c2,2)*t3 + 0.07826487932*pow(c2,2) - 0.09872759015*c2*pow(s2,2)*pow(t3,2) - 0.0008191700436*c2*pow(s2,2)*t3 - 0.01423283509*c2*pow(s2,2) + 0.00009008625051*c2*s2*t3 + 0.00035266128*c2*s2 + 0.01897574261*c2 + 0.01692977392*pow(s2,4)*pow(t3,2) - 0.0005122649939*pow(s2,4)*t3 + 0.001497158703*pow(s2,4) + 0.00001885909284*pow(s2,3)*t3 - 0.000339919713*pow(s2,3) + 0.3797361697*pow(s2,2)*pow(t3,2) - 0.0003911834176*pow(s2,2)*t3 + 0.1315274116*pow(s2,2) + 0.0003692169262*s2 + 0.02026589191;
	double r1_2 = 0.06541162346*pow(c2,2)*pow(t3,2) - 0.00197923995*pow(c2,2)*t3 + 0.005784577028*pow(c2,2) - 0.1907270582*c2*pow(t3,2) - 0.001582515003*c2*t3 + 0.009162593725*c2 + 0.06541162346*pow(s2,2)*pow(t3,2) - 0.00197923995*pow(s2,2)*t3 + 0.005784577028*pow(s2,2) + 0.00003643296967*s2*t3 + 0.00005659786982*s2 + 0.01052412488;
	double r1_3 = -0.00001249502675*pow(c2,2) + 0.00003643296967*c2 - 0.00001249502675*pow(s2,2) - 0.007353578204*s2;

	double e1_left = 0.3165424025*c1*c2 - 0.03541124607*s1 - 2.144340615*c1 + 0.0076895156*c1*s2 + 0.02971000684*c2*s1 - 2.374374248*c1*t3 - 1.223025927*s1*s2 + 0.00005659786982*c2*pow(t2_d,2) - 0.009162593725*s2*pow(t2_d,2) + 0.00003643296967*c2*t3*pow(t2_d,2) + 0.00035266128*pow(c2,2)*t1_d*t2_d + 0.006743747525*pow(c2,2)*t1_d*t3_d - 0.000339919713*pow(c2,3)*t1_d*t2_d - 0.00197923995*pow(c2,2)*t2_d*t3_d - 0.0008191700436*pow(c2,3)*t1_d*t3_d - 0.0005122649939*pow(c2,4)*t1_d*t3_d + 0.001582515003*s2*t3*pow(t2_d,2) - 0.00035266128*pow(s2,2)*t1_d*t2_d - 0.0003911834176*pow(s2,2)*t1_d*t3_d + 0.01423283509*pow(s2,3)*t1_d*t2_d - 0.00197923995*pow(s2,2)*t2_d*t3_d + 0.00001885909284*pow(s2,3)*t1_d*t3_d - 0.0005122649939*pow(s2,4)*t1_d*t3_d + 0.1907270582*s2*pow(t3,2)*pow(t2_d,2) + 0.8143137936*c1*c2*t3 + 0.0003692169262*c2*t1_d*t2_d - 0.008936093207*c2*t2_d*t3_d - 3.146266896*s1*s2*t3 - 0.01897574261*s2*t1_d*t2_d + 0.1065250645*c2*s2*t1_d*t2_d + 0.00009008625051*c2*s2*t1_d*t3_d - 0.3814541164*c2*t3*t2_d*t3_d - 0.000339919713*c2*pow(s2,2)*t1_d*t2_d + 0.01423283509*pow(c2,2)*s2*t1_d*t2_d - 0.0008191700436*c2*pow(s2,2)*t1_d*t3_d + 0.00001885909284*pow(c2,2)*s2*t1_d*t3_d + 0.00009008625051*pow(c2,2)*t3*t1_d*t2_d + 0.2878696757*pow(c2,2)*t3*t1_d*t3_d + 0.00001885909284*pow(c2,3)*t3*t1_d*t2_d + 0.1308232469*pow(c2,2)*t3*t2_d*t3_d - 0.1974551803*pow(c2,3)*t3*t1_d*t3_d + 0.03385954784*pow(c2,4)*t3*t1_d*t3_d - 0.00009008625051*pow(s2,2)*t3*t1_d*t2_d + 0.7594723393*pow(s2,2)*t3*t1_d*t3_d + 0.0008191700436*pow(s2,3)*t3*t1_d*t2_d + 0.1308232469*pow(s2,2)*t3*t2_d*t3_d + 0.03385954784*pow(s2,4)*t3*t1_d*t3_d - 0.001024529988*pow(c2,2)*pow(s2,2)*t1_d*t3_d + 0.09872759015*pow(s2,3)*pow(t3,2)*t1_d*t2_d + 0.09872759015*pow(c2,2)*s2*pow(t3,2)*t1_d*t2_d + 0.06771909569*pow(c2,2)*pow(s2,2)*t3*t1_d*t3_d - 0.01426986189*c2*s2*t3*t1_d*t2_d + 0.4716026637*c2*s2*pow(t3,2)*t1_d*t2_d + 0.00001885909284*c2*pow(s2,2)*t3*t1_d*t2_d + 0.0008191700436*pow(c2,2)*s2*t3*t1_d*t2_d - 0.1974551803*c2*pow(s2,2)*t3*t1_d*t3_d;


	double r2_1 = 0.06541162346*pow(c2,2)*pow(t3,2) - 0.00197923995*pow(c2,2)*t3 + 0.005784577028*pow(c2,2) - 0.1907270582*c2*pow(t3,2) - 0.001582515003*c2*t3 + 0.009162593725*c2 + 0.06541162346*pow(s2,2)*pow(t3,2) - 0.00197923995*pow(s2,2)*t3 + 0.005784577028*pow(s2,2) + 0.00003643296967*s2*t3 + 0.00005659786982*s2 + 0.01052412488;
	double r2_2 = 0.2527311058*pow(t3,2) - 0.007647195937*t3 + 0.06301198548;
	double r2_3 = -0.00004827707615;

	double e2_left =0.00017633064*pow(s2,2)*pow(t1_d,2) - 0.007116417545*pow(s2,3)*pow(t1_d,2) + 1.223025927*c1*c2 + 0.02971000684*c1*s2 + 0.0076895156*c2*s1 - 0.3165424025*s1*s2 - 0.007647195937*t2_d*t3_d - 0.0001846084631*c2*pow(t1_d,2) + 0.009487871303*s2*pow(t1_d,2) - 0.00017633064*pow(c2,2)*pow(t1_d,2) + 0.0001699598565*pow(c2,3)*pow(t1_d,2) - 0.05326253227*c2*s2*pow(t1_d,2) - 0.00197923995*pow(c2,2)*t1_d*t3_d - 0.00197923995*pow(s2,2)*t1_d*t3_d + 0.0001699598565*c2*pow(s2,2)*pow(t1_d,2) - 0.007116417545*pow(c2,2)*s2*pow(t1_d,2) - 0.00004504312525*pow(c2,2)*t3*pow(t1_d,2) - 0.000009429546421*pow(c2,3)*t3*pow(t1_d,2) + 0.00004504312525*pow(s2,2)*t3*pow(t1_d,2) - 0.0004095850218*pow(s2,3)*t3*pow(t1_d,2) + 3.146266896*c1*c2*t3 + 0.005771063202*c2*t1_d*t3_d - 0.8143137936*s1*s2*t3 + 0.00007286593934*s2*t1_d*t3_d + 0.5054622115*t3*t2_d*t3_d - 0.04936379507*pow(s2,3)*pow(t3,2)*pow(t1_d,2) - 0.3814541164*c2*t3*t1_d*t3_d - 0.04936379507*pow(c2,2)*s2*pow(t3,2)*pow(t1_d,2) + 0.007134930943*c2*s2*t3*pow(t1_d,2) + 0.1308232469*pow(c2,2)*t3*t1_d*t3_d + 0.1308232469*pow(s2,2)*t3*t1_d*t3_d - 0.2358013318*c2*s2*pow(t3,2)*pow(t1_d,2) - 0.000009429546421*c2*pow(s2,2)*t3*pow(t1_d,2) - 0.0004095850218*pow(c2,2)*s2*t3*pow(t1_d,2);


	double r3_1 = -0.00001249502675*pow(c2,2) + 0.00003643296967*c2 - 0.00001249502675*pow(s2,2) - 0.007353578204*s2;
	double r3_2 = -0.00004827707615;
	double r3_3 = 0.407;

	double e3_left = 0.0001955917088*pow(s2,2)*pow(t1_d,2) - 2.374374248*s1 - 0.000009429546421*pow(s2,3)*pow(t1_d,2) + 0.000256132497*pow(s2,4)*pow(t1_d,2) + 3.146266896*c1*s2 + 0.8143137936*c2*s1 - 0.2527311058*t3*pow(t2_d,2) + 0.003823597968*pow(t2_d,2) - 0.003371873763*pow(c2,2)*pow(t1_d,2) + 0.0004095850218*pow(c2,3)*pow(t1_d,2) + 0.000256132497*pow(c2,4)*pow(t1_d,2) - 0.00004504312525*c2*s2*pow(t1_d,2) + 0.00197923995*pow(c2,2)*t1_d*t2_d + 0.00197923995*pow(s2,2)*t1_d*t2_d + 0.0004095850218*c2*pow(s2,2)*pow(t1_d,2) - 0.000009429546421*pow(c2,2)*s2*pow(t1_d,2) - 0.1439348378*pow(c2,2)*t3*pow(t1_d,2) + 0.09872759015*pow(c2,3)*t3*pow(t1_d,2) - 0.01692977392*pow(c2,4)*t3*pow(t1_d,2) - 0.3797361697*pow(s2,2)*t3*pow(t1_d,2) - 0.01692977392*pow(s2,4)*t3*pow(t1_d,2) - 0.005771063202*c2*t1_d*t2_d - 0.00007286593934*s2*t1_d*t2_d + 0.0005122649939*pow(c2,2)*pow(s2,2)*pow(t1_d,2) + 0.3814541164*c2*t3*t1_d*t2_d - 0.03385954784*pow(c2,2)*pow(s2,2)*t3*pow(t1_d,2) - 0.1308232469*pow(c2,2)*t3*t1_d*t2_d - 0.1308232469*pow(s2,2)*t3*t1_d*t2_d + 0.09872759015*c2*pow(s2,2)*t3*pow(t1_d,2);

	Array2D<double> M(3,3);
	M[0][0]=r1_1;
	M[0][1]=r1_2;
	M[0][2]=r1_3;

	M[1][0]=r2_1;
	M[1][1]=r2_2;
	M[1][2]=r2_3;

	M[2][0]=r3_1;
	M[2][1]=r3_2;
	M[2][2]=r3_3;


	// here comes the first three motor dynamics models
	double N = 12.25;
	double Im = 2.847e-4;

	/*double ke1 = 720e3;
	double be1 = 4000*1;
	double tau_cm1 = 8.11e-2*0.9;
	double tau_vm1 = 1.9e-4*2;
	 double r_mc1 = 5.675e-3;
	 double r_l1 = 63.095e-3;
	 double F_cl1 = 0.505*1.0;
	 double F_vl1 = 0.00505*1.0;

	 double ke2 = 1*88e3;
	 double be2 = 1.2*1380;
	 double tau_cm2 = 4.11e-2*1.2;
	 double tau_vm2 = 1.8e-4*12;//16
	 double r_mc2 = 5.675e-3;
	 double r_l2 = 56.298e-3;
	 double F_cl2 = 0.49*1;
	 double F_vl2 = 0.0049*1;


	 double ke3 = 1*9.25e3;
	 double be3 = 1.0*400;
	 double tau_cm3 = 5.11e-2*2;
	 double tau_vm3 = 7.1e-4*2;
	 double r_mc3 = 5.675e-3;
	 double r_l3 = 1;
	 double F_cl3 = 0.53*10;
	 double F_vl3 = 0.0053*10;*/


	/* double ke1 = 720e3;
	 double be1 = 4000*1;
	 double tau_cm1 = 8.11e-2*1.0;
	 double tau_vm1 = 1.9e-4*1.0;
	 double r_mc1 = 5.675e-3;
	 double r_l1 = 63.095e-3;
	 double F_cl1 = 0.505*1.0;
	 double F_vl1 = 0.00505*1.0;

	 double ke2 = 1*88e3;
	 double be2 = 1*1380;
	 double tau_cm2 = 4.11e-2*1.0;
	 double tau_vm2 = 1.8e-4*1;//16
	 double r_mc2 = 5.675e-3;
	 double r_l2 = 56.298e-3;
	 double F_cl2 = 0.49*1;
	 double F_vl2 = 0.0049*1;

	 double ke3 = 1*9.25e3;
	 double be3 = 1.0*400;
	 double tau_cm3 = 5.11e-2;
	 double tau_vm3 = 7.1e-4*1;
	 double r_mc3 = 5.675e-3;
	 double r_l3 = 1;
	 double F_cl3 = 0.53*1;
	 double F_vl3 = 0.0053*1;*/

/*
 	 double ke1 = 720e3;
	 double be1 = 4000*1;
	 double tau_cm1 = 8.11e-2*0.75;
	 double tau_vm1 = 1.9e-4*1;
	 double r_mc1 = 5.675e-3;
	 double r_l1 = 63.095e-3;
	 double F_cl1 = 0.505*1.0;
	 double F_vl1 = 0.00505*1.0;

	 double ke2 = 1*88e3;
	 double be2 = 1*1380;
	 double tau_cm2 = 4.11e-2*2.0;
	 double tau_vm2 = 1.8e-4*15.0;//16
	 double r_mc2 = 5.675e-3;
	 double r_l2 = 56.298e-3;
	 double F_cl2 = 0.49*1;
	 double F_vl2 = 0.0049*0.5;

	 double ke3 = 1.0*9.25e3;
	 double be3 = 1.0*400;
	 double tau_cm3 = 5.11e-2*0.8;
	 double tau_vm3 = 7.1e-4*5;
	 double r_mc3 = 5.675e-3*1.0;
	 double r_l3 = 1.0;
	 double F_cl3 = 0.53*0.2;
	 double F_vl3 = 0.0053*0.1;*/ //NOV 24 	tuned parameters


	 /*double ke1 = 720e3;
	 double be1 = 4000*1;
	 double tau_cm1 = 8.11e-2*0.45;
	 double tau_vm1 = 1.9e-4*14.0;
	 double r_mc1 = 5.675e-3;
	 double r_l1 = 63.095e-3;
	 double F_cl1 = 0.505*1.0;
	 double F_vl1 = 0.00505*1.0;

	 double ke2 = 1*88e3;
	 double be2 = 1*1380;
	 double tau_cm2 = 4.11e-2*1.75;
	 double tau_vm2 = 1.8e-4*14.5;//16
	 double r_mc2 = 5.675e-3;
	 double r_l2 = 56.298e-3;
	 double F_cl2 = 0.49*1;
	 double F_vl2 = 0.0049*0.5;

	 double ke3 = 1.0*9.25e3;
	 double be3 = 1.0*400;
	 double tau_cm3 = 5.11e-2*1.0;
	 double tau_vm3 = 7.1e-4*5.3;
	 double r_mc3 = 5.675e-3*1.0;
	 double r_l3 = 1.0;
	 double F_cl3 = 0.53*1.0;
	 double F_vl3 = 0.0053*1.0; //Nov 24, afternoon tuned*/

 	 /*double ke1 = 720e3;
	 double be1 = 4000*1;
	 double tau_cm1 = 8.11e-2*0.45;
	 double tau_vm1 = 1.9e-4*12.5;//14
	 double r_mc1 = 5.675e-3;
	 double r_l1 = 63.095e-3;
	 double F_cl1 = 0.505*1.0;
	 double F_vl1 = 0.00505*1.0;

	 double ke2 = 1*88e3;
	 double be2 = 1*1380;
	 double tau_cm2 = 4.11e-2*1.75;
	 double tau_vm2 = 1.8e-4*11.5;//14.5
	 double r_mc2 = 5.675e-3;
	 double r_l2 = 56.298e-3;
	 double F_cl2 = 0.49*1;
	 double F_vl2 = 0.0049*0.5;

	 double ke3 = 1.0*9.25e3;
	 double be3 = 1.0*400;
	 double tau_cm3 = 5.11e-2*1.0;
	 double tau_vm3 = 7.1e-4*5.0;  //5.3
	 double r_mc3 = 5.675e-3*1.0;
	 double r_l3 = 1.0;
	 double F_cl3 = 0.53*1.0;
	 double F_vl3 = 0.0053*1.0; //Nov 27, afternoon tuned*/

	// tune for traj 3
	 double ke1 = 720e3;
	 double be1 = 4000*1;
	 double tau_cm1 = 8.11e-2*0.5*1;
	 double tau_vm1 = 1.9e-4*13;//14
	 double r_mc1 = 5.675e-3;
	 double r_l1 = 63.095e-3;
	 double F_cl1 = 0.505*1.0;
	 double F_vl1 = 0.00505*1.0;

	 double ke2 = 2*88e3;
	 double be2 = 1*1380;
	 double tau_cm2 = 4.11e-2*1.75;
	 double tau_vm2 = 1.8e-4*13.5;//14.5
	 double r_mc2 = 5.675e-3;
	 double r_l2 = 56.298e-3;
	 double F_cl2 = 0.49*1;
	 double F_vl2 = 0.0049*0.5;

	 double ke3 = 1.0*9.25e3;
	 double be3 = 1.0*400;
	 double tau_cm3 = 5.11e-2*1.0;
	 double tau_vm3 = 7.1e-4*4.0;  //5.3
	 double r_mc3 = 5.675e-3*1.0;
	 double r_l3 = 1.0;
	 double F_cl3 = 0.53*1.0;
	 double F_vl3 = 0.0053*1.0;



	double tau_m1 = tau_cm1*sign_func(x[9])+tau_vm1*x[9];
	double q_mc1 = x[6]/N;
	double d_q_mc1 = x[9]/N;
	double gamma1 = ke1*(exp(q_mc1*r_mc1-(x[0]+PI)*r_l1)-exp((x[0]+PI)*r_l1-q_mc1*r_mc1))+2*be1*(d_q_mc1*r_mc1-x[3]*r_l1);
	double tau_rn1 = r_mc1*gamma1/N;

	double tau_m2 = tau_cm2*sign_func(x[10])+tau_vm2*x[10];
	double q_mc2 = x[7]/N;
	double d_q_mc2 = x[10]/N;
	double gamma2 = ke2*(exp(q_mc2*r_mc2-(x[1]+PI)*r_l2)-exp((x[1]+PI)*r_l2-q_mc2*r_mc2))+2*be2*(d_q_mc2*r_mc2-x[4]*r_l2);
	double tau_rn2 = r_mc2*gamma2/N;

	double tau_m3 = tau_cm3*sign_func(x[11])+tau_vm3*x[11];
	double q_mc3 = x[8]/N;
	double d_q_mc3 = x[11]/N;
	double gamma3 = ke3*(exp(q_mc3*r_mc3-(x[2])*r_l3)-exp((x[2])*r_l3-q_mc3*r_mc3))+2*be3*(d_q_mc3*r_mc3-x[5]*r_l3);
	double tau_rn3 = r_mc3*gamma1/N;

	//double input1 = (2.579399807733391e-04*DACs[iter][0]+1.1*0.0378)*0.8;
	//double input2 = (2.579399807733391e-04*DACs[iter][1]+0.94*0.0672)*0.8;
	//double input3 = (2.579399807733391e-04*DACs[iter][2]-0.91*0.0398)*1;

	//double input1 = (2.737629663234212e-04*DACs[0]+1.0*0.041613800000000)*1.0;
  //  double input2 = (2.737629663234212e-04*DACs[1]+1.0*0.0697023000)*0.85;
    //double input3 = (2.737629663234212e-04*DACs[2]-1.0*0.039108200000000)*1;

	double input1 = (2.737629663234212e-04*(DACs[0])-1.153*0.041613800000000)*0.9; //1.16 1.0 new 1.183
    double input2 = (2.737629663234212e-04*(DACs[1])-0.9*0.0697023000)*1.0;  //0.9 1.0
    double input3 = (2.737629663234212e-04*(DACs[2])+0.4*0.039108200000000)*0.9; //0.4 1.0


	dxdt[9] = 1/Im*(-input1-tau_m1-tau_rn1);
	dxdt[10] = 1/Im*(-input2-tau_m2-tau_rn2);
	dxdt[11] = 1/Im*(-input3-tau_m3-tau_rn3);

	//cout<<"1: "<<input1<<", "<<tau_m1<<", "<<tau_rn1<<endl;
	//cout<<"2: "<<input2<<", "<<tau_m2<<", "<<tau_rn2<<endl;
	//cout<<"3: "<<input3<<", "<<tau_m3<<", "<<tau_rn3<<endl;

	double tau1 = gamma1*r_l1;
	double tau2 = gamma2*r_l2;
	double tau3 = gamma3*r_l3;

	Array2D<double> right_side(3,1),dd(3,1);

	right_side[0][0] = tau1-x[3]*F_vl1-sign_func(x[3])*F_cl1-e1_left;
	right_side[1][0] = tau2-x[4]*F_vl2-sign_func(x[4])*F_cl2-e2_left;
	right_side[2][0] = tau3-x[5]*F_vl3-sign_func(x[5])*F_cl3-e3_left;

	dd= matmult(inverse(M),right_side);

	dxdt[3] = dd[0][0];
	dxdt[4] = dd[1][0];
	dxdt[5] = dd[2][0];

	x[0] = x[0]+dxdt[0]*dt/1000.0;
	x[1] = x[1]+dxdt[1]*dt/1000.0;
	x[2] = x[2]+dxdt[2]*dt/1000.0;
	x[3] = x[3]+dxdt[3]*dt/1000.0;
	x[4] = x[4]+dxdt[4]*dt/1000.0;
	x[5] = x[5]+dxdt[5]*dt/1000.0;
	x[6] = x[6]+dxdt[6]*dt/1000.0;
	x[7] = x[7]+dxdt[7]*dt/1000.0;
	x[8] = x[8]+dxdt[8]*dt/1000.0;
	x[9] = x[9]+dxdt[9]*dt/1000.0;
	x[10] = x[10]+dxdt[10]*dt/1000.0;
	x[11] = x[11]+dxdt[11]*dt/1000.0;
}

double delta = 55;
if (abs(dxdt[0])+abs(dxdt[1])+abs(dxdt[2])+abs(dxdt[3])+abs(dxdt[4])+abs(dxdt[5])+0.001*(abs(dxdt[6])+abs(dxdt[7])+abs(dxdt[8])+abs(dxdt[9])+abs(dxdt[10])+abs(dxdt[11]))>delta)
{
	return iter_time;
}
else
{
	return 0;
}
}
