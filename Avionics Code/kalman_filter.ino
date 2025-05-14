#include <iostream>
#include <vector>
#include <cmath>
#include "kalman_filter.h"
using namespace std;

vector<vector<double>> state_transition(float dt,){
    vector<vector<double>> A = {{1, 0, 0, dt, 0, 0, (1/2) * pow(dt,2), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                {0, 1, 0, 0, dt, 0, 0, (1/2) * pow(dt,2), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                {0, 0, 1, 0, 0, dt, 0, 0, (1/2) * pow(dt,2), 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                {0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                {0, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                {0, 0, 0, 0, 0, 1, 0, 0, dt, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                {0, 0, 0, 0, 0, 0, 1, 0,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                {0, 0, 0, 0, 0, 0, 0, 1,  0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                                {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0}
                                {0, 0, 0, 0, 0, 0, 0, 0, 0, alpha, 0, 0, 0, 0, 0, 0, 0, 0},         
                                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, beta, 0, 0, 0, 0, 0, 0, 0},
                                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, gamma, 0, 0, 0, 0, 0, 0},
                                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, delta, 0, 0, 0, 0, 0},
                                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0,   1, 0, 0, 0, 0},
                                {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,   0,   0, 1, 0, dt, 0},
                                {},
                                {},
                                {}}

    return A;
}

float constant(float velocity){
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



class KalmanFilter {
    public:
        vector<vector<double>> A_matrix;
        vector<vector<double>> H_matrix;
        vector<vector<double>> Q_matrix;
        vector<vector<double>> R_matrix;

        KalmanFilter(vector<vector<double>> A, vector<vector<double>> H, 
            vector<vector<double>> Q, vector<vector<double>> R){
            A_matrix = A;
            H_matrix = H;
            Q_matrix = Q;
            R_matrix = R;
        }
        ~KalmanFilter(){}

        vector<vector<double>> predict_state(vector<vector<double>> state_0){
            return matrix_multiplication(A_matrix, state_0);
          }

        vector<vector<double>> predict_error_covariance(vector<vector<double>> err_co_0){
            vector<vector<double>> result = matrix_multiplication(A_matrix, err_co_0);
            return matrix_addition(matrix_multiplication(result, transposeMat(A_matrix)), Q_matrix);
          }
        vector<vector<double>> kalman_gain(vector<vector<double>> pred_co){
            vector<vector<double>> result = matrix_multiplication(H_matrix, pred_co);
            vector<vector<double>> result_two = transposeMat(H_matrix); 
            vector<vector<double>> result_three = matrix_multiplication(result, result_two);
            vector<vector<double>> result_four = matrix_addition(result_three, R_matrix);
            vector<vector<double>> result_five = matrix_multiplication(pred_co, result_two);
            vector<vector<double>> result_six = get_inverse(result_four, result_four.size());
            return matrix_multiplication(result_five, result_six);

          }
        vector<vector<double>> estimate(vector<vector<double>> kal_gain, vector<vector<double>> pred_state, vector<vector<double>> z_0){
            vector<vector<double>> result = subtractMat(z_0, matrix_multiplication(H_matrix, pred_state));
            return matrix_addition(pred_state, matrix_multiplication(kal_gain, result));
          }
          
        vector<vector<double>> compute_error_covariance(vector<vector<double>> pred_co, vector<vector<double>> kal_gain){
            vector<vector<double>> result = matrix_multiplication(kal_gain, H_matrix);
            return subtractMat(pred_co, matrix_multiplication(result, pred_co));
        }

        vector<vector<double>> run_kalman_filter_estimate(vector<vector<double>> err_co_estimate, 
            vector<vector<double>> state_0, vector<vector<double>> measurement){
            vector<vector<double>> predicted_state = predict_state(state_0);
            vector<vector<double>> predicted_err_co = predict_error_covariance(err_co_estimate);
            vector<vector<double>> kal_gain = kalman_gain(predicted_err_co);
            vector<vector<double>> est_state = estimate(kal_gain, predicted_state, measurement);
            vector<vector<double>> comp_err_co = compute_error_covariance(predicted_err_co, kal_gain);
            return est_state;
        }

        vector<vector<double>> run_kalman_filter_covar(vector<vector<double>> err_co_estimate, 
          vector<vector<double>> state_0, vector<vector<double>> measurement){
          vector<vector<double>> predicted_state = predict_state(state_0);
          vector<vector<double>> predicted_err_co = predict_error_covariance(err_co_estimate);
          vector<vector<double>> kal_gain = kalman_gain(predicted_err_co);
          vector<vector<double>> comp_err_co = compute_error_covariance(predicted_err_co, kal_gain);
          return comp_err_co;
      }
};

int main(){
  vector<vector<double>> Q = {{1.0, 0.0}, {0.0, 3.0}};
  vector<vector<double>> R = {{10.0}};
  vector<vector<double>> A = {{1.0, 0.1}, {0.0, 1.0}};
  vector<vector<double>> H = {{1.0, 0.0}};
  KalmanFilter myObj(A, H, Q, R);

  vector<vector<double>> P = {{5.0, 0.0}, {0.0, 5.0}};
  vector<vector<double>> x = {{0.0}, {20.0}};
  vector<vector<double>> z(1, vector<double>(1,0));
  vector<vector<double>> mat(1, vector<double>(1,0));
  vector<vector<double>> mat2(2, vector<double>(2,0));
  int n = 10;
  for (int i = 0; i < 100; i++){
    z[0][0] = i + 3.5;
    mat = myObj.run_kalman_filter_estimate(P, x, z);
    mat2 = myObj.run_kalman_filter_covar(P, x, z);
    x = mat;
    P = mat2;
    cout << mat[0][0] << endl;
  }

}
