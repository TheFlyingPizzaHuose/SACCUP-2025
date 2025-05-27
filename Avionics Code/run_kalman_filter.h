#ifndef run_kalman_filter_h
#define run_kalman_filter_h
#include <iostream>
#include <vector>
#include <cmath>
#include "kalman_filter.h"
using namespace std;

double constant_alpha(double velocity){
    // These constants are good
    double pi = 3.1415926535897932384626433832795028841971693993751058209749;
    double C_r = 0.381;
    double C_t = 0.0762;
    double h = 0.1778;
    double Diam = 0.1524;
    double D_le = 59.5 * pi/180;
    double D_4 = atan((tan(D_le)*h + (C_t / 4) - (C_r / 4))/h);
    double S = (C_r + C_t) * h;
    double AR = pow((h*2 + Diam),2)/S;
    double K_p = 2.95;
    double E = 4.61 * (1 - 0.045 * pow(AR, 0.68)) * pow(cos(D_le), 0.15) - 3.1;
    double K_i = 1 / (pi * E * AR);
    double K_v = (K_p - K_i * pow(K_p, 2)) / cos(D_4);

    // ----------------------------------------------------------------------

    double rho = 1.225; //Replace with equation
    double M = velocity / 343;
    double lamb = C_t / C_r;
    double mgc = (2.0/3.0) * C_r * ((pow(lamb, 2) + lamb + 1) / (lamb + 1)); 
    double u = 1.789e-5; // Replace with equation
    double Rn = rho * velocity * mgc/u;
    double R_wf = 1.05;
    double R_ls = 0.9;
    double C_f = 0.455/(pow(log10(Rn),2.58) * pow((1 + 0.144 * pow(M,2)), 0.58)); 
    double L_p = 1.2;
    double tcr = (0.25 * 0.0254) / mgc;
    double C_do = R_wf * R_ls * C_f * (1+L_p * tcr + 100 * pow(tcr, 4)) * 2;




    double alpha = 5 * pi /180.0;
    double w = 0;
    double m = 60.0/2.2;
    double L = 144.0 * 0.0254;
    double r = L/2;
    double I = m * pow(L, 2) / 12.0;
    double q = pow(velocity,2) * rho / 2.0;
    double t = 0;
    double dt = 0.001;
    
    double rads = w * r;
    double temp = pi / 2 - alpha;
    double a_n = atan(rads * sin(temp) / (velocity - rads * cos(temp)));
    double a_eff = alpha - a_n;
    double CL = K_p * sin(abs(a_eff)) * pow(cos(a_eff),2) + K_v * cos(a_eff) * pow(sin(a_eff),2);

    if (a_eff < 0){
        L = -1.0 * CL * q * S; //SIGN NEG or POS            
    } else if (a_eff > 0){
        L = CL * q * S; 
    }

    double D = (C_do + K_i * pow(CL, 2)) * q * S;
    double Ly = L*cos(a_n);
    double Lx = L*sin(a_n);
    double Dy = -D*sin(a_n);
    double Dx = D*cos(a_n);
    double ry = sin(alpha) * r;
    double rx = cos(alpha) * r;
    M = (Lx + Dx) * ry + (Ly + Dy) * rx;
    w = w + (M/I) * dt;
    alpha = alpha - w * dt;
    alpha = alpha * 180.0/pi;
    t = t + dt;
    

    return alpha;
    

}

vector<vector<double>> state_transition(double alpha, double alpha_prev, double theta, double phi, double gamma){
    double alpha = cos(theta/2) * cos(phi/2) * cos(gamma / 2) + sin(theta / 2) * sin (phi / 2) * sin(gamma / 2);
    double beta = sin(theta/2) * cos(phi / 2) * cos(gamma / 2) - cos(theta / 2) * sin(phi/2) * sin(gamma/2);
    double gamma = cos(theta/2)*sin(phi/2) * cos(gamma / 2) + sin(theta / 2) * cos(phi / 2) * sin(gamma / 2);
    double delta = cos(theta / 2) * cos(phi / 2) * sin(gamma / 2) - sin(theta/2) * sin(phi/2) * cos(gamma/2);
    double dt = 0.001;

    
    vector<vector<double>> A = {{1, 0, 0, dt, 0, 0, (1/2) * pow(dt,2), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0},
                                {0, 1, 0, 0, dt, 0, 0, (1/2) * pow(dt,2), 0, 0, 0, 0, 0, 0, 0, 0, 0,  0},
                                {0, 0, 1, 0, 0, dt, 0, 0, (1/2) * pow(dt,2), 0, 0, 0, 0, 0, 0, 0, 0,  0},
                                {0, 0, 0, 1, 0, 0, dt, 0, 0,                 0, 0, 0, 0, 0, 0, 0, 0,  0},
                                {0, 0, 0, 0, 1, 0, 0, dt,    0,              0, 0, 0, 0, 0, 0, 0, 0,  0},
                                {0, 0, 0, 0, 0, 1, 0, 0, dt,    0,              0, 0, 0, 0, 0, 0, 0,  0},
                                {0, 0, 0, 0, 0, 0, 1, 0,  0,    0,              0, 0, 0, 0, 0, 0, 0,  0},
                                {0, 0, 0, 0, 0, 0, 0, 1,  0, 0,    0,              0, 0, 0, 0, 0, 0,  0},
                                {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,                  0, 0, 0, 0, 0, 0,  0},
                                {0, 0, 0, 0, 0, 0, 0, 0, 0, alpha,              0, 0, 0, 0, 0, 0, 0,  0},         
                                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, beta,               0, 0, 0, 0, 0, 0,  0},
                                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, gamma,              0, 0, 0, 0, 0,  0},
                                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, delta,              0, 0, 0, 0,  0},
                                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0,                1, 0, 0, 0,  0},
                                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0,               0, 1, 0, dt,  0},
                                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0,               0, 0, 1, 0,  dt},
                                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0,  alpha/alpha_prev, 0},
                                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, alpha/ alpha_prev}};

    

    return A;
}


#endif /* run_kalman_filter_h */
