#pragma once

struct WalkParam
{
	const double *begin_pm_wrt_ground;         // input, ��ʼ��������ϵ��λ�˾���
	const double *begin_pee_wrt_ground;        // input, ��ʼ��˵�λ��
	double d;                                  // input, ����
	double h;                                  // input, ����
	double alpha;                              // input, ǰ������
	double beta;                               // input, ������ת���Ƕ�
	int total_count;                           // input, ����ʱ�䳤��
	int n;                                     // input, ����
	double *mot_pos;                           // output, ���λ��
};
int walk_plan(int count, struct WalkParam *param);
