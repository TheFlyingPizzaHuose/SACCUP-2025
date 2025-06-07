// Programed by Elizabeth McGhee

#ifndef kalman_filter_h
#define kalman_filter_h
#include <iostream>
#include <vector>
//#include <algorithm>
using namespace std;

vector<vector<double>> swap_rows(vector<vector<double>> myvec, int n, int m){
  vector<double> temp = myvec[n];
  myvec[n] = myvec[m];
  myvec[m] = temp;

  return myvec;
}

vector<vector<double>> inverse(int n, vector<vector<double>> mat){
  vector<vector<double>> I(n, vector<double>(n,0.0));

  // We have the indentity matrix
  for (int i = 0; i < n; i++){
    for (int j = 0; j < n; j++){
      if (i == j){
        I[i][j] = 1.0;
      }
    }
  }

  // Next check for zeros on diagonal and swap rows
  for (int i = 0; i < n; i++){
    if (i == n - 1){
      if (mat[i][i] == 0.0 && mat[i-1][i] != 0.0){
        mat = swap_rows(mat, i, i- 1);
        I = swap_rows(I, i, i-1);
      }
  }else if (i != n - 1){
    if (mat[i][i] == 0.0 && mat[i+1][i] != 0.0){
      mat = swap_rows(mat, i, i+1);
      I = swap_rows(I, i, i +1);
      }
    }
  }
  

  double ratio;
  
  // Now we row reduce that shit !! Make sure it can handle edge cases
  for (int i = 0; i < n; i++){
    for (int j = i+1; j < n; j++){
      ratio = mat[j][i] / mat[i][i] * -1;
      for (int k = 0; k < n; k++){
        mat[j][k] = mat[j][k] + ratio * mat[i][k];
        I[j][k] = I[j][k] + ratio * I[i][k];
        }
      }
  }

  // Now reverse

  for (int i = n - 1; i >= 0; i--){
    for (int j = i-1; j >= 0; j--){
      ratio = mat[j][i] / mat[i][i] * -1;
      for (int k = n - 1; k >= 0; k--){
        mat[j][k] = mat[j][k] + ratio * mat[i][k];
        I[j][k] = I[j][k] + ratio * I[i][k];
        }
      }
  }

    // Next put ones along the diagonal

  for (int i = 0; i < n; i++){
    ratio = mat[i][i];
    for (int j = 0; j < n; j++){
      mat[i][j] = mat[i][j] / ratio;
      I[i][j] = I[i][j] / ratio;
      if (I[i][j] == -0.0){
        I[i][j] = 0.0;
      }
    }
  }
  return I;
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

vector<vector<double>> getSubmatrix(vector<vector<double>> matrix, int rowToRemove, int colToRemove) {
    int n = matrix.size();

    vector<vector<double>> test(n-1, vector<double>(n-1,2));

    for (int i = 0; i < n; ++i) {
      if (i == rowToRemove){
          matrix.erase(matrix.begin() + i);
        } 
      }

      int row = 0;
      for (int k = 0; k < matrix.size(); k++){
        int col = 0;
        for (int j = 0; j < matrix[0].size(); j++){
            if (j != colToRemove){
              test[row][col] = matrix[row][j];
              col++;
            }
          }
        row++;
      }

    return test;
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
          vector<vector<double>> result_six = inverse(result_four.size(), result_four); //this is the problem function :(

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


class runKalmanFilter {
  private:
    double constant_alpha(double velocity, double altitude){
    // These constants are good
    double pi = 3.1415926536;
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

vector<vector<double>> state_transition(double gx, double gy, double gz){

    double dt = 0.001;
    double a = (1/2) * pow(dt,2);
    double t = dt / 2;
    double w1 = gx * t;
    double w2 = gy * t;
    double w3 = gz * t;
  
    vector<vector<double>> A = {{1, 0, 0, dt, 0, 0,  a,  0,   0,   0,  0,    0,  0,  0,   0,   0,   0,  0},
                                {0, 1, 0, 0, dt, 0,  0,  a,   0,   0,  0,    0,  0,  0,   0,   0,   0,  0},
                                {0, 0, 1, 0, 0,  dt, 0,  0,   a,   0,  0,    0,  0,  0,   0,   0,   0,  0},
                                {0, 0, 0, 1, 0,  0,  dt, 0,   0,   0,  0,    0,  0,  0,   0,   0,   0,  0},
                                {0, 0, 0, 0, 1,  0,  0,  dt,  0,   0,  0,    0,  0,  0,   0,   0,   0,  0},
                                {0, 0, 0, 0, 0,  1,  0,  0,   dt,  0,  0,    0,  0,  0,   0,   0,   0,  0},
                                {0, 0, 0, 0, 0,  0,  1,  0,   0,   0,  0,    0,  0,  0,   0,   0,   0,  0},
                                {0, 0, 0, 0, 0,  0,  0,  1,   0,   0,  0,    0,  0,  0,   0,   0,   0,  0},
                                {0, 0, 0, 0, 0,  0,  0,  0,   1,   0,  0,    0,  0,  0,   0,   0,   0,  0},
                                {0, 0, 0, 0, 0,  0,  0,  0,   0,   1,  0,    0,  0,  0,   0,   0,   0,  0},
                                {0, 0, 0, 0, 0,  0,  0,  0,   0,   0,  1,    0,  0,  0,   0,   0,   dt, 0},
                                {0, 0, 0, 0, 0,  0,  0,  0,   0,   0,  0,    1,  0,  0,   0,   0,   0,  dt},
                                {0, 0, 0, 0, 0,  0,  0,  0,   0,   0,  0,    0,  1,  -w1, -w2, -w3, 0,  0},         
                                {0, 0, 0, 0, 0,  0,  0,  0,   0,   0,  0,    0,  w1, 1,   w3,  w2,  0,  0},
                                {0, 0, 0, 0, 0,  0,  0,  0,   0,   0,  0,    0,  w2, -w3, 1,   w1,  0,  0},
                                {0, 0, 0, 0, 0,  0,  0,  0,   0,   0,  0,    0,  w3, -w2, -w1, 1,   0,  0},
                                {0, 0, 0, 0, 0,  0,  0,  0,   0,   0,  -dt,  0,  0,  0,   0,   0,   1,  0},
                                {0, 0, 0, 0, 0,  0,  0,  0,   0,   0,  0,  -dt,  0,  0,   0,   0,   0,  1}};
    return A;
}
  public:
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
                                           {0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0},
                                           {0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0},
                                           {0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0},
                                           {0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0},
                                           {0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0},
                                           {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                           {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                           {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                           {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                           {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}};

      vector<vector<double>> Q_matrix = {{0.001,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                         {0,0.001,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                         {0,0,0.001,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                         {0,0,0,0.001,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                         {0,0,0,0,0.001,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                         {0,0,0,0,0,0.001,0,0,0,0,0,0,0,0,0,0,0,0},
                                         {0,0,0,0,0,0,0.001,0,0,0,0,0,0,0,0,0,0,0},
                                         {0,0,0,0,0,0,0,0.001,0,0,0,0,0,0,0,0,0,0},
                                         {0,0,0,0,0,0,0,0,0.001,0,0,0,0,0,0,0,0,0},
                                         {0,0,0,0,0,0,0,0,0,0.001,0,0,0,0,0,0,0,0},
                                         {0,0,0,0,0,0,0,0,0,0,0.001,0,0,0,0,0,0,0},
                                         {0,0,0,0,0,0,0,0,0,0,0,0.001,0,0,0,0,0,0},
                                         {0,0,0,0,0,0,0,0,0,0,0,0,0.001,0,0,0,0,0},
                                         {0,0,0,0,0,0,0,0,0,0,0,0,0,0.001,0,0,0,0},
                                         {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.001,0,0,0},
                                         {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.001,0,0},
                                         {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.001,0},
                                         {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0.001}};

      vector<vector<double>> R_matrix = {{10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                         {0,10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                         {0,0,10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                         {0,0,0,10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                         {0,0,0,0,10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                         {0,0,0,0,0,10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                         {0,0,0,0,0,0,10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                         {0,0,0,0,0,0,0,10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                         {0,0,0,0,0,0,0,0,10,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                         {0,0,0,0,0,0,0,0,0,10,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                         {0,0,0,0,0,0,0,0,0,0,10,0,0,0,0,0,0,0,0,0,0,0,0},
                                         {0,0,0,0,0,0,0,0,0,0,0,10,0,0,0,0,0,0,0,0,0,0,0},
                                         {0,0,0,0,0,0,0,0,0,0,0,0,10,0,0,0,0,0,0,0,0,0,0},
                                         {0,0,0,0,0,0,0,0,0,0,0,0,0,10,0,0,0,0,0,0,0,0,0},
                                         {0,0,0,0,0,0,0,0,0,0,0,0,0,0,10,0,0,0,0,0,0,0,0},
                                         {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,10,0,0,0,0,0,0,0},
                                         {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,10,0,0,0,0,0,0},
                                         {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,10,0,0,0,0,0},
                                         {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,10,0,0,0,0},
                                         {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,10,0,0,0},
                                         {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,10,0,0},
                                         {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,10,0},
                                         {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,10}};
                                      
      double x, y, z, vx, vy, vz, ax, ay, az, gx, gy, gz, a, b, c, d, alpha_phi, alpha_gamma;
      double lat, lon, altitude_1, altitude_2, press1, press2, press3, velocity, vx1, vy1, vz1, 
             ax1, ay1, az1, ax2, ay2, az2, ax3, ay3, az3, A, B, C, D, ang1, ang2, angle_1, angle_2,
             gx1, gy1, gz1, gx2, gy2, gz2, MX, MY, MZ;
      double state_var[18] = {x, y, z, vx, vy, vz, ax, ay, az, gx, gy, gz, a, b, c, d, alpha_phi, alpha_gamma};
      
      vector<vector<double>> A_matrix = {{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
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
      vector<vector<double>> state = {{0},
                                      {0},
                                      {0},
                                      {0},
                                      {0},
                                      {0},
                                      {0},
                                      {0},
                                      {0},
                                      {0},
                                      {0},
                                      {0},
                                      {0},
                                      {0},
                                      {0},
                                      {0},
                                      {0},
                                      {0}};
      vector<vector<double>> covar = {{1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
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

      vector<vector<double>> measurement = {{0},
                                            {0},
                                            {0},
                                            {0},
                                            {0},
                                            {0},
                                            {0},
                                            {0},
                                            {0},
                                            {0},
                                            {0},
                                            {0},
                                            {0},
                                            {0},
                                            {0},
                                            {0},
                                            {0},
                                            {0},
                                            {0},
                                            {0},
                                            {0},
                                            {0},
                                            {0},
                                            {0}};
                                      

      runKalmanFilter(){}
      ~runKalmanFilter(){}

      float velocity_magnitude(){
        float T = 280;
        float gamma = 1.4;
        float R = 287;

        float velocity = sqrt(((2*gamma*R*T)/(gamma - 1))*(pow((press1/press2),(gamma - 1)/1)) - 1);
        return velocity;
      }

      vector<double> quaternion_to_speed(double a, double b, double c, double d){
        vector<double> q = {a, b, c, d};
        vector<double> direction = {0, 0, 1};
        vector<double> q_star = {a, -b, -c, -d};
        vector<double> new_q = {q[1]*direction[1]*q_star[1], q[2]*direction[2]*q_star[2], q[3]*direction[3]*q_star[3]};

        return new_q;}

      double x_speed(vector<double> quaternion, double velocity){
        return velocity * quaternion[1];
      }

      double y_speed(vector<double> quaternion, double velocity){
        return velocity * quaternion[2];
      }

      double z_speed(vector<double> quaternion, double velocity){
        return velocity * quaternion[3];
      }

       double<double<vector>> updateR(angle_1, angle_2){
          vector<vector<double>> R_matrix = {{10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                             {0,10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                             {0,0,10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                             {0,0,0,10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                             {0,0,0,0,10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                             {0,0,0,0,0,10,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                             {0,0,0,0,0,0,10,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                             {0,0,0,0,0,0,0,10,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                             {0,0,0,0,0,0,0,0,10,0,0,0,0,0,0,0,0,0,0,0,0},
                                             {0,0,0,0,0,0,0,0,0,10,0,0,0,0,0,0,0,0,0,0,0},
                                             {0,0,0,0,0,0,0,0,0,0,10,0,0,0,0,0,0,0,0,0,0},
                                             {0,0,0,0,0,0,0,0,0,0,0,10,0,0,0,0,0,0,0,0,0},
                                             {0,0,0,0,0,0,0,0,0,0,0,0,10,0,0,0,0,0,0,0,0},
                                             {0,0,0,0,0,0,0,0,0,0,0,0,0,10,0,0,0,0,0,0,0},
                                             {0,0,0,0,0,0,0,0,0,0,0,0,0,0,10,0,0,0,0,0,0},
                                             {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,10,0,0,0,0,0},
                                             {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,10,0,0,0,0},
                                             {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,10,0,0,0},
                                             {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,10,0,0},
                                             {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,angle_1,0},
                                             {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,angle_2}};
       }
        
  

      void updateData(double GPS_LAT, double GPS_LON, double alt1, double alt2, double PRESS1, 
                      double PRESS2, double PRESS3, double MPU_AX, double MPU_AY, double MPU_AZ, 
                      double LSM_AX, double LSM_AY, double LSM_AZ, double ADXL345_AX, double ADXL345_AY, 
                      double ADXL345_AZ, double a, double b, double c, double d, double LSM_GX, 
                      double LSM_GY,double LSM_GZ, double MPU_GX, double MPU_GY, double MPU_GZ, double MPU_MX, 
                      double MPU_MY, double MPU_MZ, double ang1, double ang2){
          vector<double> q = {a, b, c, d};
          lat = GPS_LAT;
          lon = GPS_LON;
          altitude_1 = alt1;
          altitude_2 = alt2;
          press1 = PRESS1;
          press2 = PRESS2;
          press3 = PRESS3;
          velocity = velocity_magnitude();
          vx1 = x_speed(q, velocity);
          vy1 = y_speed(q, velocity);
          vz1 = z_speed(q, velocity);
          ax1 = MPU_AX;
          ay1 = MPU_AY;
          az1 = MPU_AZ;
          ax2 = LSM_AX;
          ay2 = LSM_AY;
          az2 = LSM_AZ;
          ax3 = ADXL345_AX;
          ay3 = ADXL345_AY;
          az3 = ADXL345_AZ;
          gx1 = LSM_GX;
          gy1 = LSM_GY;
          gz1 = LSM_GZ;
          gx2 = MPU_GX;
          gy2 = MPU_GY;
          gz2 = MPU_GZ;
          MX = MPU_MX;
          MY = MPU_MY;
          MZ = MPU_MZ;
          angle_1 = ang1;
          angle_2 = ang2;


          
      };


      void updateStateTransition(){
          A_matrix = state_transition(gx, gy, gz);
      }

      void updateRMatrix(double ang1_err, double ang2_err){
          R_matrix = updateR(ang1_err, ang2_err);
      }

      void updateMeasurement(){
         measurement = {{lat},
                        {lon},
                        {altitude_1},
                        {altitude_2},
                        {vx1},
                        {vy1},
                        {vz1},
                        {ax1},
                        {ay1},
                        {az1},
                        {ax2},
                        {ay2},
                        {az2},
                        {ax3},
                        {ay3},
                        {az3},
                        {gx1},
                        {gy1},
                        {gz1},
                        {gx2},
                        {gy2},
                        {gz2},
                        {MX},
                        {MY},
                        {MZ}};
      }

      void updateState_and_Covar(){
          KalmanFilter myObj = KalmanFilter(A_matrix, H_matrix, Q_matrix, R_matrix);
          state = myObj.run_kalman_filter_estimate(covar, state, measurement);

          for (int i = 0; i < 18; i++){
            state_var[i] = state[i][0];
            
          };
      }
  
};




#endif /* kalman_filter_h */
