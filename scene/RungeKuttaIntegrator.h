//
// Created by LiuQiang on 2025/1/7.
//

#ifndef SCENE_RUNGEKUTTAINTEGRATOR_H
#define SCENE_RUNGEKUTTAINTEGRATOR_H

class RungeKuttaIntegrator
{
public:

};

//def runge_kutta_4th_order(f, x0, y0, h, n):
//"""�Ľ�����-��������
//f: ΢�ַ��̺���
//x0, y0: ��ʼ����
//h: ����
//n: ��������
//"""
//x = x0
//y = y0
//for i in range(n):
//k1 = h * f(x, y)
//k2 = h * f(x + 0.5 * h, y + 0.5 * k1)
//k3 = h * f(x + 0.5 * h, y + 0.5 * k2)
//k4 = h * f(x + h, y + k3)
//y += (k1 + 2 * k2 + 2 * k3 + k4) / 6
//x += h
//return y
#endif // SCENE_RUNGEKUTTAINTEGRATOR_H
