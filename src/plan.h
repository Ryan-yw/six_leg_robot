#pragma once

struct WalkParam
{
	const double *begin_pm_wrt_ground;         // input, 起始身体坐标系的位姿矩阵
	const double *begin_pee_wrt_ground;        // input, 起始足端的位置
	double d;                                  // input, 步长
	double h;                                  // input, 步高
	double alpha;                              // input, 前进方向
	double beta;                               // input, 单步的转动角度
	int total_count;                           // input, 单步时间长度
	int n;                                     // input, 步数
	double *mot_pos;                           // output, 电机位置
};
int walk_plan(int count, struct WalkParam *param);
