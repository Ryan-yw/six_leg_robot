//#include "stdafx.h"

#include <math.h>
#include <string.h>
#include "plan.h"
#include "kinematic.h"

#define PI 3.14159265358979

double acc_up(int n, int i) { return (-1.0 / 2 / n / n / n * i*i*i + 3.0 / 2.0 / n / n * i*i); }
double acc_down(int n, int i) { return (-1.0*i*i*i / 2.0 / n / n / n + 3.0 * i*i / 2.0 / n / n); }
double dec_up(int n, int i) { return 1.0 - (-1.0 / 2.0 / n / n / n * (n - i)*(n - i)*(n - i) + 3.0 / 2.0 / n / n * (n - i)*(n - i)); }
double dec_down(int n, int i) { return 1.0 - (-1.0*(n - i)*(n - i)*(n - i) / 2.0 / n / n / n + 3.0 * (n - i)*(n - i) / 2.0 / n / n); }

double acc_even(int n, int i) { return 1.0 / n / n * i * i; }
double dec_even(int n, int i) { return 1.0 - 1.0 / n / n * (n - i)*(n - i); }
double even(int n, int i) { return 1.0 / n * i; }

double s_p2p(int n, int i, double begin_pos, double end_pos)
{
	double a = 4.0 * (end_pos - begin_pos) / n / n;
	return i <= n / 2.0 ? 0.5*a*i*i + begin_pos : end_pos - 0.5*a*(n - i)*(n - i);
}
double s_v2v(int n, int i, double begin_vel, double end_vel)
{
	double s = (double)(i) / n;
	double m = 1.0 - s;

	return (s*s*s - s * s)*end_vel*n + (m*m - m * m*m)*begin_vel*n;
}
double s_interp(int n, int i, double begin_pos, double end_pos, double begin_vel, double end_vel)
{
	double s = (double)(i) / n;

	double a, b, c, d;

	c = begin_vel * n;
	d = begin_pos;
	a = end_vel * n - 2.0 * end_pos + c + 2.0 * d;
	b = end_pos - c - d - a;

	return a * s*s*s + b * s*s + c * s + d;
}


double* s_rq2rm(const double *rq_in, double *rm_out, int rm_ld)
{
	// 正式开始计算 //
	rm_out[0 * rm_ld + 0] = 1 - 2 * rq_in[1] * rq_in[1] - 2 * rq_in[2] * rq_in[2];
	rm_out[0 * rm_ld + 1] = 2 * rq_in[0] * rq_in[1] - 2 * rq_in[3] * rq_in[2];
	rm_out[0 * rm_ld + 2] = 2 * rq_in[0] * rq_in[2] + 2 * rq_in[3] * rq_in[1];

	rm_out[1 * rm_ld + 0] = 2 * rq_in[0] * rq_in[1] + 2 * rq_in[3] * rq_in[2];
	rm_out[1 * rm_ld + 1] = 1 - 2 * rq_in[0] * rq_in[0] - 2 * rq_in[2] * rq_in[2];
	rm_out[1 * rm_ld + 2] = 2 * rq_in[1] * rq_in[2] - 2 * rq_in[3] * rq_in[0];

	rm_out[2 * rm_ld + 0] = 2 * rq_in[0] * rq_in[2] - 2 * rq_in[3] * rq_in[1];
	rm_out[2 * rm_ld + 1] = 2 * rq_in[1] * rq_in[2] + 2 * rq_in[3] * rq_in[0];
	rm_out[2 * rm_ld + 2] = 1 - 2 * rq_in[0] * rq_in[0] - 2 * rq_in[1] * rq_in[1];

	return rm_out;
}
double* s_rq2pm(const double *rq_in, double *pm_out){ return s_rq2rm(rq_in, pm_out, 4); };
double* s_pp2pm(const double *pp_in, double *pm_out)
{
	// 正式开始计算 //
	pm_out[3] = pp_in[0];
	pm_out[7] = pp_in[1];
	pm_out[11] = pp_in[2];

	return pm_out;
}
double* s_pq2pm(const double *pq_in, double *pm_out)
{
	// 正式开始计算 //
	s_pp2pm(pq_in, pm_out);
	s_rq2pm(pq_in + 3, pm_out);

	pm_out[12] = 0;
	pm_out[13] = 0;
	pm_out[14] = 0;
	pm_out[15] = 1;

	return pm_out;
}

double* s_pm_dot_v3(const double *pm, const double *v3, double *v3_out);
double* s_inv_pp2pp(const double *inv_relative_pm, const double *from_pp, double *to_pp);
double* s_pm_dot_pm(const double *pm1, const double *pm2, double *pm_out);

void s_va(int n, double alpha, const double* x, double* y) { for (int i = 0; i < n; ++i)y[i] += alpha * x[i]; }

int walk_plan(int count, struct WalkParam *param)
{
	const double *begin_pm_wrt_ground = param->begin_pm_wrt_ground;
	const double *begin_pee_wrt_ground = param->begin_pee_wrt_ground;
	const double d = param->d;
	const double h = param->h;
	const double a = param->alpha;
	const double b = param->beta;
	const int total_count = param->total_count;
	const int n = param->n;
	double* mot_pos = param->mot_pos;

	//const double front[3]={ -sin(a),0,-cos(a) };
	//const double left[3]={ -cos(a),0,sin(a) };
	//const double up[3]={ 0,1,0 };

	const double front[3] = { -sin(a),cos(a),0 };
	const double left[3] = { -cos(a),-sin(a),0 };
	const double up[3] = { 0,0,1 };

	static double begin_body_pm[16], body_pm[16], begin_pee[18], pee[18];
	if (count == 0)
	{
		memcpy(begin_body_pm, begin_pm_wrt_ground, sizeof(double) * 16);
		memcpy(body_pm, begin_pm_wrt_ground, sizeof(double) * 16);
		memcpy(begin_pee, begin_pee_wrt_ground, sizeof(double) * 18);
		memcpy(pee, begin_pee_wrt_ground, sizeof(double) * 18);
	}


	
	if (count % total_count == 0)
	{
		double tem[18];
		for (int i = 0; i < 6; ++i)
		{
			s_inv_pp2pp(body_pm, pee + i * 3, begin_pee + i * 3);
		}
		
		
		s_pm_dot_pm(begin_body_pm, body_pm, tem);
		memcpy(begin_body_pm, tem, sizeof(double) * 16);
		
		//s_vc(16, tem, begin_body_pm);
		
		
		
		//beginMak.setPrtPm(*robot.body().pm());
		//beginMak.update();
		//robot.GetPee(beginPee, beginMak);
	}





	int period_count = count % total_count;
	const double s = -(PI / 2)*cos(PI * (period_count + 1) / total_count) + PI / 2;//s 从0到PI. 

	double pq_body[7] = {0,0,0,0,0,0,0};
	memcpy(pee, begin_pee, sizeof(double) * 18);

	double pq_b[7] = { 0,0,0,sin(b / 2)*up[0],sin(b / 2)*up[1],sin(b / 2)*up[2],cos(b / 2) };
	double pq_b_half[7] = { 0,0,0,sin(b / 4)*up[0],sin(b / 4)*up[1],sin(b / 4)*up[2],cos(b / 4) };
	double pq_b_quad[7] = { 0,0,0,sin(b / 8)*up[0],sin(b / 8)*up[1],sin(b / 8)*up[2],cos(b / 8) };
	double pq_b_eighth[7] = { 0,0,0,sin(b / 16)*up[0],sin(b / 16)*up[1],sin(b / 16)*up[2],cos(b / 16) };
	double pm_b[16], pm_b_half[16], pm_b_quad[16], pm_b_eighth[16];

	s_pq2pm(pq_b, pm_b);
	s_pq2pm(pq_b_half, pm_b_half);
	s_pq2pm(pq_b_quad, pm_b_quad);
	s_pq2pm(pq_b_eighth, pm_b_eighth);

	const int leg_begin_id = (count / total_count) % 2 == 1 ? 3 : 0;

	if ((count / total_count) == 0)//加速段
	{
		//规划腿
		for (int i = leg_begin_id; i < 18; i += 6)
		{
			//单腿运动需要分解成延圆周的直线运动，还有延自身的转动
			double leg_forward_dir[3], forward_d[3];
			s_pm_dot_v3(pm_b_quad, front, leg_forward_dir);

			s_pm_dot_v3(pm_b_half, begin_pee + i, forward_d);
			s_va(3, -1.0, begin_pee + i, forward_d);
			s_va(3, d / 2, leg_forward_dir, forward_d);

			for (int j = 0; j < 3; ++j)
			{
				pee[i + j] = begin_pee[i + j] + (1 - cos(s)) / 2 * forward_d[j] + h * up[j] * sin(s);
			}
		}

		//规划身体位置
		double body_forward_dir[3], body_left_dir[3];
		s_pm_dot_v3(pm_b_eighth, front, body_forward_dir);
		s_pm_dot_v3(pm_b_eighth, left, body_left_dir);

		for (int i = 0; i < 3; ++i)
		{
			pq_body[i] = left[i] * s_interp(total_count, period_count + 1, 0, d*tan(b / 8) / 4 / cos(b / 8), 0, d / 2 / total_count / cos(b / 2)*sin(b / 4))
				+ front[i] * s_interp(total_count, period_count + 1, 0, d / 4 / cos(b / 4), 0, d / 2 / total_count / cos(b / 2)*cos(b / 4));
		}

		//规划身体姿态
		double s_acc = acc_even(total_count, period_count + 1);
		double pq[7] = { 0,0,0,sin(s_acc*b / 8)*up[0],sin(s_acc*b / 8)*up[1] ,sin(s_acc*b / 8)*up[2],cos(s_acc*b / 8) };
		memcpy(pq_body + 3, pq + 3, sizeof(double) * 4);
	}
	else if ((count / total_count) == (n * 2 - 1))//减速段
	{
		//规划腿
		for (int i = leg_begin_id; i < 18; i += 6)
		{
			//单腿运动需要分解成延圆周的直线运动，还有延自身的转动
			double leg_forward_dir[3], forward_d[3];
			s_pm_dot_v3(pm_b_quad, front, leg_forward_dir);

			s_pm_dot_v3(pm_b_half, begin_pee + i, forward_d);
			s_va(3, -1.0, begin_pee + i, forward_d);
			s_va(3, d / 2, leg_forward_dir, forward_d);

			for (int j = 0; j < 3; ++j)
			{
				pee[i + j] = begin_pee[i + j] + (1 - cos(s)) / 2 * forward_d[j] + h * up[j] * sin(s);
			}
		}

		//规划身体位置
		double body_forward_dir[3], body_left_dir[3];
		s_pm_dot_v3(pm_b_eighth, front, body_forward_dir);
		s_pm_dot_v3(pm_b_eighth, left, body_left_dir);

		for (int i = 0; i < 3; ++i)
		{
			pq_body[i] = left[i] * s_interp(total_count, period_count + 1, 0, d*tan(b / 8) / 4 / cos(b / 8), 0, 0)
				+ front[i] * s_interp(total_count, period_count + 1, 0, d / 4 / cos(b / 4), d / 2 / total_count / cos(b / 2), 0);
		}

		//规划身体姿态
		double s_dec = dec_even(total_count, period_count + 1);
		double pq[7] = { 0,0,0,sin(s_dec*b / 8)*up[0],sin(s_dec*b / 8)*up[1] ,sin(s_dec*b / 8)*up[2],cos(s_dec*b / 8) };
		memcpy(pq_body + 3, pq + 3, sizeof(double) * 4);
	}
	else//匀速段
	{
		//规划腿
		for (int i = leg_begin_id; i < 18; i += 6)
		{
			//单腿运动需要分解成延圆周的直线运动，还有延自身的转动
			double leg_forward_dir[3], forward_d[3];
			s_pm_dot_v3(pm_b_half, front, leg_forward_dir);

			s_pm_dot_v3(pm_b, begin_pee + i, forward_d);
			s_va(3, -1.0, begin_pee + i, forward_d);
			s_va(3, d, leg_forward_dir, forward_d);

			for (int j = 0; j < 3; ++j)
			{
				pee[i + j] = begin_pee[i + j] + (1 - cos(s)) / 2 * forward_d[j] + h * up[j] * sin(s);
			}
		}

		//规划身体位置
		double d2 = d / 2 / cos(b / 4);
		for (int i = 0; i < 3; ++i)
		{
			pq_body[i] = left[i] * s_interp(total_count, period_count + 1, 0, d2*sin(b / 4), 0, d / 2 / total_count / cos(b / 2)*sin(b / 2))
				+ front[i] * s_interp(total_count, period_count + 1, 0, d / 2, d / 2 / total_count / cos(b / 2), d / 2 / total_count / cos(b / 2)*cos(b / 2));
		}

		//规划身体姿态
		double s_even = even(total_count, period_count + 1);
		double pq[7] = { 0,0,0,sin(s_even*b / 4)*up[0],sin(s_even*b / 4)*up[1] ,sin(s_even*b / 4)*up[2],cos(s_even*b / 4) };
		memcpy(pq_body + 3, pq + 3, sizeof(double) * 4);
	}

	s_pq2pm(pq_body, body_pm);

	inverse(body_pm, pee, mot_pos);


	//printf("body: %f  %f  %f\n", body_pm[3], body_pm[7], body_pm[11]);
	//printf("leg : %f  %f  %f  %f  %f  %f\n", pee[0], pee[1], pee[2], pee[3], pee[4], pee[5]);
	return 2 * n * total_count - count - 1;
}