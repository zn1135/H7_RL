/*
 * File: LQR_K_WBR.h
 *
 * WBR LQR 最优反馈增益矩阵 —— 由 MATLAB Symbolic Math Toolbox 生成。
 *
 * 来源: Leg2_v1/Code/Matlab/LQR_K_WBR.{c,h} 原样移植 (上交轮腿, 机械构造与本机一致)。
 * 内容: 240 个 poly22 系数, K = c + a*lL + b*lR + d*lL^2 + e*lR^2 + f*lL*lR,
 *       拟合域 lL, lR ∈ 0.13~0.23 m。
 * 用法: LQR_K_WBR(h_l, h_r, K_sym) → K_sym[状态*4 + 输出]。
 * 注意: 本文件为生成物, 不要手改; 换用自研脚本增益时整表替换即可。
 */

#ifndef LQR_K_WBR_H
#define LQR_K_WBR_H

#include <stddef.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Function Declarations */
extern void LQR_K_WBR(float lL, float lR, float K_sym[40]);

#ifdef __cplusplus
}
#endif

#endif
/*
 * File trailer for LQR_K_WBR.h
 *
 * [EOF]
 */
