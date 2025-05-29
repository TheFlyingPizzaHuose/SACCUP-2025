// Programed by Elizabeth McGhee

#ifndef kalman_filter_h
#define kalman_filter_h
#include <iostream>
#include <vector>
#include <algorithm>
using namespace std;

vector<vector<double>> swap_rows(vector<vector<double>> myvec, int n, int m){
  vector<double> temp = myvec[n];
  myvec[n] = myvec[m];
  myvec[m] = temp;

  return myvec;
}

double triangulate(vector<vector<double>> my_mat, int f){
    int n = my_mat.size();
    int w = 0;
    for (int i = 0; i < n-1; i++){
        int pivotRow = i;
        for (int j = i+1; j < n; j++){
            if (fabs(my_mat[j][i]) > fabs(my_mat[pivotRow][i])){
                pivotRow = j;
            }
        }

        if (pivotRow != i){
            vector<vector<double>> result = swap_rows(my_mat, i, pivotRow);
            my_mat = result;
            w++;
        }

        double diag = my_mat[i][i];
        for (int j = i+1; j<n; j++){
            double mult = my_mat[j][i] / my_mat[i][i];
            for (int k = i; k < n; k++){
              my_mat[j][k] -= mult * my_mat[i][k];
              if (my_mat[j][k] == -0.0){
                my_mat[j][k] = 0.0;
              }
            }
        }
    }

    double my_det = 1.0;
    for (int i = 0; i < f; i++){
        my_det *= my_mat[i][i];                           
    }
    my_det = my_det * pow(-1, w);

    return my_det;
}

vector<vector<double>> transposeMat(vector<vector<double>> mat) {
  int n = mat.size();
  int m = mat[0].size();
  vector<vector<double>> my_result(m, vector<double>(n, 0));
  for (int i = 0; i < my_result.size(); i++) {
    for (int j = 0; j < my_result[i].size(); j++) {
      my_result[i][j] = mat[j][i];
    }
  }
  return my_result;
}


double getMinor(vector<vector<double>> mat, int row, int col) {
    int n = mat.size();

    vector<vector<double>> submatrix(n - 1, vector<double>(n - 1));
    int sub_i = 0, sub_j = 0;

    for (int i = 0; i < n; i++) {
        if (i == row) continue; 

        sub_j = 0;
        for (int j = 0; j < n; j++) {
            if (j == col) continue; 

            submatrix[sub_i][sub_j] = mat[i][j];
            sub_j++;
        }
        sub_i++;
    }
    return triangulate(submatrix, n-1);
}

vector<vector<double>> get_Minor_matrix(vector<vector<double>> my_vector, int n){
    vector<vector<double>> result(n, vector<double>(n, 0)); 
    for (int i = 0; i < n; i++){
        for (int j = 0; j < n; j++){
            result[i][j] =  getMinor(my_vector, i, j);
        }
    }
    return result;
}

vector<vector<double>> cofactor(vector<vector<double>> my_vector, int n){
    vector<vector<double>> result(n, vector<double>(n, 0)); 
    vector<vector<double>> minor(n, vector<double>(n, 0));
    minor = get_Minor_matrix(my_vector, n);
    for (int i = 0; i < n; i++){
        for (int j = 0; j < n; j++){
            result[i][j] =  pow(-1, i+j) * minor[i][j];
        }
    }

    return result;
}

vector<vector<double>> get_inverse(vector<vector<double>> my_vector, int n){
    vector<vector<double>> result(n, vector<double>(n, 0)); 
    double det = triangulate(my_vector, n);
    result = transposeMat(cofactor(my_vector, n));

    for (int i = 0; i < n; i++){
        for (int j = 0; j < n; j++){
            result[i][j] = result[i][j] / det;
        }
    }

    return result;
}

vector<vector<double>> matrix_addition(vector<vector<double>> mat1, vector<vector<double>> mat2){
    int m = mat1.size();
    int n = mat1[0].size();
    vector<vector<double>> my_result(m, vector<double>(n, 0));
    for (int i = 0; i < m; i++) {
      for (int j = 0; j < n; j++) {
        my_result[i][j] = mat1[i][j] + mat2[i][j];
      }
    }
    return my_result;
  }

vector<vector<double>> matrix_multiplication(vector<vector<double>> mat1, vector<vector<double>> mat2){
    int r1 = mat1.size();
    int c1 = mat1[0].size();
    int r2 = mat2.size();
    int c2 = mat2[0].size();
    if (c1 != r2){
      cout << "Aborting Program" << endl;
      abort();
    }
    vector<vector<double>> my_result(r1, vector<double>(c2, 0));
    for (int i = 0; i < r1; i++) {
      for (int j = 0; j < c2; j++) {
        my_result[i][j] = 0;
        for (int k = 0; k < r2; k++) {
          my_result[i][j] += mat1[i][k] * mat2[k][j];
      }
    }
  }
  return my_result;
}

vector<vector<double>> subtractMat(vector<vector<double>> mat1, vector<vector<double>> mat2) {
  int r1 = mat1.size();
  int c1 = mat1[0].size();
  vector<vector<double>> my_result(r1, vector<double>(c1, 0));
  for (int i = 0; i < my_result.size(); i++) {
      for (int j = 0; j < my_result[i].size(); j++) {
        my_result[i][j] = mat1[i][j] - mat2[i][j];
      }
    }
  return my_result;
}

class KalmanFilter {
  public:
      vector<vector<double>> A_matrix;
      vector<vector<double>> H_matrix = {{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                            {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};

      vector<vector<double>> Q_matrix = {{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                            {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0},
                            {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1}};
      vector<vector<double>> R = {{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                            {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0},
                            {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0},
                            {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1}};

      KalmanFilter(vector<vector<double>> A, vector<vector<double>> H, 
          vector<vector<double>> Q, vector<vector<double>> R){
          A_matrix = A;
      }
      ~KalmanFilter(){}
      // seems fine here
      vector<vector<double>> predict_state(vector<vector<double>> state_0){
        return matrix_multiplication(A_matrix, state_0);
        }
        // seems fine here
      vector<vector<double>> predict_error_covariance(vector<vector<double>> err_co_0){
          vector<vector<double>> result = matrix_multiplication(A_matrix, err_co_0);
          return matrix_addition(matrix_multiplication(result, transposeMat(A_matrix)), Q_matrix);
        }

      // working fine
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

double constant_alpha(double velocity, double altitude){
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

    double rho = 1.225*pow(1 - 6.875e-6 * altitude, 5.2561); //Replace with equation
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


#endif /* kalman_filter_h */
