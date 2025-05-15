#ifndef run_kalman_filter_h
#define run_kalman_filter_h
#include <iostream>
#include <vector>
#include <cmath>
#include "kalman_filter.h"
using namespace std;

vector<vector<double>> state_transition(float dt, float alpha, float theta, float phi, float gamma){
    alpha = cos(theta/2) * cos(phi/2) * cos(gamma / 2) + sin(theta / 2) * sin (phi / 2) * sin(gamma / 2);
    beta = sin(theta/2) * cos(phi / 2) * cos(gamma / 2) - cos(theta/2) * sin(phi/2) * sin(gamma/2);
    gamma = cos(theta/2)*sin(phi/2) * cos(gamma / 2) + sin(theta / 2) * cos(phi / 2) * sin(gamma / 2);
    delta = cos(theta / 2) * cos(phi / 2) * sin(gamma / 2) - sin(theta/2) * sin(phi/2) * cos(gamma/2);
    
    vector<vector<double>> A = {{1, 0, 0, dt, 0, 0, (1/2) * pow(dt,2), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0},
                                {0, 1, 0, 0, dt, 0, 0, (1/2) * pow(dt,2), 0, 0, 0, 0, 0, 0, 0, 0, 0,  0},
                                {0, 0, 1, 0, 0, dt, 0, 0, (1/2) * pow(dt,2), 0, 0, 0, 0, 0, 0, 0, 0,  0},
                                {0, 0, 0, 1, 0, 0, dt, 0, 0,                 0, 0, 0, 0, 0, 0, 0, 0,  0},
                                {0, 0, 0, 0, 1, 0, 0, dt,    0,              0, 0, 0, 0, 0, 0, 0, 0,  0},
                                {0, 0, 0, 0, 0, 1, 0, 0, dt,    0,              0, 0, 0, 0, 0, 0, 0,  0},
                                {0, 0, 0, 0, 0, 0, 1, 0,  0,    0,              0, 0, 0, 0, 0, 0, 0,  0},
                                {0, 0, 0, 0, 0, 0, 0, 1,  0, 0,    0,              0, 0, 0, 0, 0, 0,  0},
                                {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,                  0, 0, 0, 0, 0, 0,  0}
                                {0, 0, 0, 0, 0, 0, 0, 0, 0, alpha,              0, 0, 0, 0, 0, 0, 0,  0},         
                                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, beta,               0, 0, 0, 0, 0, 0,  0},
                                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, gamma,              0, 0, 0, 0, 0,  0},
                                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, delta,              0, 0, 0, 0,  0},
                                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0,                1, 0, 0, 0,  0},
                                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0,               0, 1, 0, dt,  0},
                                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0,               0, 0, 1, 0,  dt},
                                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  0, 0, 0, 0,  alpha_i+1/alpha_i, 0},
                                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, alpha_i+1 / alpha_i}};

    

    return A;
}

float constant_alpha(float velocity){
    C_r = 0.381;
    C_t = 0.0762;
    h = 0.1778;
    Diam = 0.1524;
    D_le = 59.5 * pi/180;
    D_4 = atan((tan(D_le)*h + (C_t / 4) - (C_r / ))/4);
    S = (C_r + C_c) * h;
    AR = pow((h*2 + Diam),2)/S;
    K_p = 2.95;
    E = 4.61 * (1 - 0.045 * pow(AR, 0.68)) * pow(cos(D_le), 0.15) - 3.1;
    K_i = 1 / (pi * E * AR);
    K_v = (K_p - K_i * pow(K_p, 2)) / cos(D_4);

    rho = 1.225 //Replace with equation
    M = velocity / 343;
    lamb = C_t / C_r;
    mgc = (2/3) * C_r * ((pow(lamb, 2) + lamb + 1) / (lamb + 1));
    u = 1.789e-5; // Replace with equation
    Rn = rho * V * mgc/u;
    R_wf = 1.05;
    R_ls = 0.9;
    C_f = 0.455/(pow(log10(Rn),2.58) * pow((1 + 0.144 * pow(M,2)), 0.58));
    L_p = 1.2;
    tcr = (0.25 * 0.0254) / mgc;
    C_do = R_wf * R_ls * C_f * (1+L_p * tcr + 100 * pow(tcr, 4)) * 2;

    alpha = 5 * pi /180;
    w = 0;
    m = 60/2.2;
    L = 144 * 0.0254;
    r = L/2;
    I = m * pow(L, 2) / 12;
    q = pow(V,2) * rho / 2;
    t = 0;
    dt = 0.001;

    while (t < 10){
        rads = w * r;
        temp = pi / 2 - alpha;
        a_n = atan(rads * sin(temp) / (V - rads * cos(temp)));
        a_eff = alpha - a_n;
        CL = K_p * sin(abs(a_eff)) * pow(cos(a_eff),2) + K_v * cos(a_eff) * pow(sin(a_eff, 2));
        if (a_eff < 0){
            L = -1 * CL * q * S; //SIGN NEG or POS            
        } elif (a_eff > 0){
            L = CL * q * S; 
        }

        D = (C_do + K_i * pow(CL, 2)) * q * S;
        Ly = L*cos(a_n);
        Lx = L*sin(a_n);
        Dy = -D*sin(a_n);
        Dx = D*cos(a_n);
        ry = sin(alpha) * r;
        rx = cos(alpha) * r;
        M = (Lx + Dx) * ry + (Ly + Dy) * rx;
        w = w + (M/I) * dt;
        alpha = alpha - w * dt;
        t = t + dt;
    }

    return alpha;
    
}

vector<vector<double>> run_kalman_filter(float dt, float alpha, float theta, float phi, float gamma, vector<vector<double>> measurement){
    vector<vector<double>> P = {{0}}; //Define covariance matrix
    vector<vector<double>> state = {{0}}; //define
    vector<vector<double>> A = state_transition(dt, alpha, float theta, float phi, float gamma);
    kalman_filter myObj(A, H, Q, R);
    estimate = myObj.run_kalman_filter_estimate(P, state, measurement);

    return estimate;
    
}

#endif /* run_kalman_filter_h */
