//#include "stdafx.h"

#include<math.h>
#include"kinematic.h"

#define PO 42.5
#define AJ_X_INIT 107
#define AJ_Y 4
#define PB_X 18
#define THIRD_INIT 63.5

#define PF 270.0
#define FH 585.0
#define PD 135.0
#define PJ 75.5
#define DJ 60.5
#define DE 45
#define BE 150.0
#define AP1 115
#define AP2 79
#define PI 3.14159265358979

const double PM1[16] = 
{
	1,     0,     0,     0,
	0,     1,     0,    243,
	0,     0,     1,     0,
	0,     0,     0,     1
};
const double PM2[16] =
{
                       0.5,        -0.866025403784439,                         0,         -210.4442,
         0.866025403784439,                       0.5,                         0,                       121.5,
                         0,                         0,                         1,                         0,
                         0,                         0,                         0,                         1
};
const double PM3[16] =
{
                      -0.5,        -0.866025403784439,                         0,         -210.4442,
         0.866025403784439,                      -0.5,                         0,                      -121.5,
                         0,                         0,                         1,                         0,
                         0,                         0,                         0,                         1
};
const double PM4[16] =
{

                        -1,     -1.22464679914735e-16,                         0,                         0,
      1.22464679914735e-16,                        -1,                         0,                      -243,
                         0,                         0,                         1,                         0,
                         0,                         0,                         0,                         1
};
const double PM5[16] =
{
                      -0.5,         0.866025403784438,                         0,          210.4442,
        -0.866025403784438,                      -0.5,                         0,                      -121.5,
                         0,                         0,                         1,                         0,
                         0,                         0,                         0,                         1
};
const double PM6[16] =
{
                       0.5,         0.866025403784439,                         0,          210.4442,
        -0.866025403784439,                       0.5,                         0,                       121.5,
                         0,                         0,                         1,                         0,
                         0,                         0,                         0,                         1
};


void s_inv_pm(const double *pm_in, double *pm_out)
{
	//转置
	pm_out[0] = pm_in[0];
	pm_out[1] = pm_in[4];
	pm_out[2] = pm_in[8];
	pm_out[4] = pm_in[1];
	pm_out[5] = pm_in[5];
	pm_out[6] = pm_in[9];
	pm_out[8] = pm_in[2];
	pm_out[9] = pm_in[6];
	pm_out[10] = pm_in[10];

	//位置
	pm_out[3] = -pm_out[0] * pm_in[3] - pm_out[1] * pm_in[7] - pm_out[2] * pm_in[11];
	pm_out[7] = -pm_out[4] * pm_in[3] - pm_out[5] * pm_in[7] - pm_out[6] * pm_in[11];
	pm_out[11] = -pm_out[8] * pm_in[3] - pm_out[9] * pm_in[7] - pm_out[10] * pm_in[11];

	//其他
	pm_out[12] = 0;
	pm_out[13] = 0;
	pm_out[14] = 0;
	pm_out[15] = 1;
}
double* s_pm_dot_pm(const double *pm1, const double *pm2, double *pm_out)
{
	pm_out[0] = pm1[0] * pm2[0] + pm1[1] * pm2[4] + pm1[2] * pm2[8];
	pm_out[1] = pm1[0] * pm2[1] + pm1[1] * pm2[5] + pm1[2] * pm2[9];
	pm_out[2] = pm1[0] * pm2[2] + pm1[1] * pm2[6] + pm1[2] * pm2[10];
	pm_out[3] = pm1[0] * pm2[3] + pm1[1] * pm2[7] + pm1[2] * pm2[11] + pm1[3];

	pm_out[4] = pm1[4] * pm2[0] + pm1[5] * pm2[4] + pm1[6] * pm2[8];
	pm_out[5] = pm1[4] * pm2[1] + pm1[5] * pm2[5] + pm1[6] * pm2[9];
	pm_out[6] = pm1[4] * pm2[2] + pm1[5] * pm2[6] + pm1[6] * pm2[10];
	pm_out[7] = pm1[4] * pm2[3] + pm1[5] * pm2[7] + pm1[6] * pm2[11] + pm1[7];

	pm_out[8] = pm1[8] * pm2[0] + pm1[9] * pm2[4] + pm1[10] * pm2[8];
	pm_out[9] = pm1[8] * pm2[1] + pm1[9] * pm2[5] + pm1[10] * pm2[9];
	pm_out[10] = pm1[8] * pm2[2] + pm1[9] * pm2[6] + pm1[10] * pm2[10];
	pm_out[11] = pm1[8] * pm2[3] + pm1[9] * pm2[7] + pm1[10] * pm2[11] + pm1[11];

	pm_out[12] = 0;
	pm_out[13] = 0;
	pm_out[14] = 0;
	pm_out[15] = 1;

	return pm_out;
}
double* s_inv_pm_dot_pm(const double *inv_pm, const double *pm, double *pm_out)
{
	pm_out[0] = inv_pm[0] * pm[0] + inv_pm[4] * pm[4] + inv_pm[8] * pm[8];
	pm_out[1] = inv_pm[0] * pm[1] + inv_pm[4] * pm[5] + inv_pm[8] * pm[9];
	pm_out[2] = inv_pm[0] * pm[2] + inv_pm[4] * pm[6] + inv_pm[8] * pm[10];
	pm_out[3] = inv_pm[0] * (pm[3] - inv_pm[3]) + inv_pm[4] * (pm[7] - inv_pm[7]) + inv_pm[8] * (pm[11] - inv_pm[11]);

	pm_out[4] = inv_pm[1] * pm[0] + inv_pm[5] * pm[4] + inv_pm[9] * pm[8];
	pm_out[5] = inv_pm[1] * pm[1] + inv_pm[5] * pm[5] + inv_pm[9] * pm[9];
	pm_out[6] = inv_pm[1] * pm[2] + inv_pm[5] * pm[6] + inv_pm[9] * pm[10];
	pm_out[7] = inv_pm[1] * (pm[3] - inv_pm[3]) + inv_pm[5] * (pm[7] - inv_pm[7]) + inv_pm[9] * (pm[11] - inv_pm[11]);

	pm_out[8] = inv_pm[2] * pm[0] + inv_pm[6] * pm[4] + inv_pm[10] * pm[8];
	pm_out[9] = inv_pm[2] * pm[1] + inv_pm[6] * pm[5] + inv_pm[10] * pm[9];
	pm_out[10] = inv_pm[2] * pm[2] + inv_pm[6] * pm[6] + inv_pm[10] * pm[10];
	pm_out[11] = inv_pm[2] * (pm[3] - inv_pm[3]) + inv_pm[6] * (pm[7] - inv_pm[7]) + inv_pm[10] * (pm[11] - inv_pm[11]);

	pm_out[12] = 0;
	pm_out[13] = 0;
	pm_out[14] = 0;
	pm_out[15] = 1;

	return pm_out;
}
double* s_pm_dot_inv_pm(const double *pm, const double *inv_pm, double *pm_out)
{
	pm_out[0] = pm[0] * inv_pm[0] + pm[1] * inv_pm[1] + pm[2] * inv_pm[2];
	pm_out[1] = pm[0] * inv_pm[4] + pm[1] * inv_pm[5] + pm[2] * inv_pm[6];
	pm_out[2] = pm[0] * inv_pm[8] + pm[1] * inv_pm[9] + pm[2] * inv_pm[10];
	pm_out[3] = -pm_out[0] * inv_pm[3] - pm_out[1] * inv_pm[7] - pm_out[2] * inv_pm[11] + pm[3];

	pm_out[4] = pm[4] * inv_pm[0] + pm[5] * inv_pm[1] + pm[6] * inv_pm[2];
	pm_out[5] = pm[4] * inv_pm[4] + pm[5] * inv_pm[5] + pm[6] * inv_pm[6];
	pm_out[6] = pm[4] * inv_pm[8] + pm[5] * inv_pm[9] + pm[6] * inv_pm[10];
	pm_out[7] = -pm_out[4] * inv_pm[3] - pm_out[5] * inv_pm[7] - pm_out[6] * inv_pm[11] + pm[7];

	pm_out[8] = pm[8] * inv_pm[0] + pm[9] * inv_pm[1] + pm[10] * inv_pm[2];
	pm_out[9] = pm[8] * inv_pm[4] + pm[9] * inv_pm[5] + pm[10] * inv_pm[6];
	pm_out[10] = pm[8] * inv_pm[8] + pm[9] * inv_pm[9] + pm[10] * inv_pm[10];
	pm_out[11] = -pm_out[8] * inv_pm[3] - pm_out[9] * inv_pm[7] - pm_out[10] * inv_pm[11] + pm[11];

	pm_out[12] = 0;
	pm_out[13] = 0;
	pm_out[14] = 0;
	pm_out[15] = 1;

	return pm_out;
}
double* s_pm_dot_v3(const double *pm, const double *v3, double *v3_out)
{
	v3_out[0] = pm[0] * v3[0] + pm[1] * v3[1] + pm[2] * v3[2];
	v3_out[1] = pm[4] * v3[0] + pm[5] * v3[1] + pm[6] * v3[2];
	v3_out[2] = pm[8] * v3[0] + pm[9] * v3[1] + pm[10] * v3[2];

	return v3_out;
}
double* s_inv_pm_dot_v3(const double *inv_pm, const double *v3, double *v3_out)
{
	v3_out[0] = inv_pm[0] * v3[0] + inv_pm[4] * v3[1] + inv_pm[8] * v3[2];
	v3_out[1] = inv_pm[1] * v3[0] + inv_pm[5] * v3[1] + inv_pm[9] * v3[2];
	v3_out[2] = inv_pm[2] * v3[0] + inv_pm[6] * v3[1] + inv_pm[10] * v3[2];

	return v3_out;
}
double* s_pp2pp(const double *relative_pm, const double *from_pp, double *to_pp)
{
	to_pp[0] = relative_pm[0] * from_pp[0] + relative_pm[1] * from_pp[1] + relative_pm[2] * from_pp[2] + relative_pm[3];
	to_pp[1] = relative_pm[4] * from_pp[0] + relative_pm[5] * from_pp[1] + relative_pm[6] * from_pp[2] + relative_pm[7];
	to_pp[2] = relative_pm[8] * from_pp[0] + relative_pm[9] * from_pp[1] + relative_pm[10] * from_pp[2] + relative_pm[11];

	return to_pp;
}
double* s_inv_pp2pp(const double *inv_relative_pm, const double *from_pp, double *to_pp)
{
	double tem[3] = { from_pp[0] - inv_relative_pm[3] ,from_pp[1] - inv_relative_pm[7] ,from_pp[2] - inv_relative_pm[11] };

	to_pp[0] = inv_relative_pm[0] * tem[0] + inv_relative_pm[4] * tem[1] + inv_relative_pm[8] * tem[2];
	to_pp[1] = inv_relative_pm[1] * tem[0] + inv_relative_pm[5] * tem[1] + inv_relative_pm[9] * tem[2];
	to_pp[2] = inv_relative_pm[2] * tem[0] + inv_relative_pm[6] * tem[1] + inv_relative_pm[10] * tem[2];

	return to_pp;
}


int inverse(double *body_pm_wrt_ground, double *ee_xyz_wrt_ground, double *mot_pos_18)
{
	double real_pm1[16], real_pm2[16], real_pm3[16], real_pm4[16], real_pm5[16], real_pm6[16];
	s_pm_dot_pm(body_pm_wrt_ground, PM1, real_pm1);
	s_pm_dot_pm(body_pm_wrt_ground, PM2, real_pm2);
	s_pm_dot_pm(body_pm_wrt_ground, PM3, real_pm3);
	s_pm_dot_pm(body_pm_wrt_ground, PM4, real_pm4);
	s_pm_dot_pm(body_pm_wrt_ground, PM5, real_pm5);
	s_pm_dot_pm(body_pm_wrt_ground, PM6, real_pm6);

	double xyz_in_leg[18];
	s_inv_pp2pp(real_pm1, ee_xyz_wrt_ground + 0 * 3, xyz_in_leg + 0 * 3);
	s_inv_pp2pp(real_pm2, ee_xyz_wrt_ground + 1 * 3, xyz_in_leg + 1 * 3);
	s_inv_pp2pp(real_pm3, ee_xyz_wrt_ground + 2 * 3, xyz_in_leg + 2 * 3);
	s_inv_pp2pp(real_pm4, ee_xyz_wrt_ground + 3 * 3, xyz_in_leg + 3 * 3);
	s_inv_pp2pp(real_pm5, ee_xyz_wrt_ground + 4 * 3, xyz_in_leg + 4 * 3);
	s_inv_pp2pp(real_pm6, ee_xyz_wrt_ground + 5 * 3, xyz_in_leg + 5 * 3);

	leg_inverse(xyz_in_leg + 0 * 3, mot_pos_18 + 0 * 3);
	leg_inverse(xyz_in_leg + 1 * 3, mot_pos_18 + 1 * 3);
	leg_inverse(xyz_in_leg + 2 * 3, mot_pos_18 + 2 * 3);
	leg_inverse(xyz_in_leg + 3 * 3, mot_pos_18 + 3 * 3);
	leg_inverse(xyz_in_leg + 4 * 3, mot_pos_18 + 4 * 3);
	leg_inverse(xyz_in_leg + 5 * 3, mot_pos_18 + 5 * 3);

	return 0;
}
int leg_inverse(double *ee_xyz_wrt_leg, double *mot_pos_3)
{
	mot_pos_3[0] = -atan2(ee_xyz_wrt_leg[0], ee_xyz_wrt_leg[1]);// 竖直转动轴

	double x = sqrt(ee_xyz_wrt_leg[0] * ee_xyz_wrt_leg[0] + ee_xyz_wrt_leg[1] * ee_xyz_wrt_leg[1]) - PO;
	double y = ee_xyz_wrt_leg[2];

	double cosF = (PF * PF + FH * FH - x * x - y * y) / 2 / PF / FH;
	double HFP = acos((PF * PF + FH * FH - x * x - y * y) / 2 / PF / FH);
	double P3 = -atan(x/y);
	double P1 = acos((PF * PF + x * x + y * y - FH * FH) / 2 / PF / sqrt(x * x + y * y));
	double P2 = PI - HFP - P1;
	double P4 = P3 - P2;
	double pd1 = sin(P4)*PD;
	double pd2 = -cos(P4)*PD;
	double DPJ = acos((PJ * PJ + PD * PD - DJ * DJ) / 2 / PJ / PD);
	double P5 = P4 - DPJ;
	double pj1 = sin(P5)*PJ;
	double pj2 = - cos(P5)*PJ;
	double AJ1 = AP1 + pj1;
	double AJ2 = AP2 + pj2;
	mot_pos_3[1] = sqrt(AJ1*AJ1 + AJ2 * AJ2 - AJ_Y * AJ_Y) - AJ_X_INIT;// 推杆1
	double fh1 = (FH / PD)*pd1;
	double fh2 = (FH / PD)*pd2;
	double PF1 = x - fh1;
	double PF2 = y - fh2;
	double de1 = (DE / PF)*PF1;
	double de2 = (DE / PF)*PF2;
	double PE1 = pd1 + de1;
	double PE2 = pd2 + de2;
	double ke = PE1 - PB_X;
	double bk = sqrt(BE * BE - ke * ke);
	mot_pos_3[2] = -PE2 - bk + THIRD_INIT; // 推杆2

	return 0;
}