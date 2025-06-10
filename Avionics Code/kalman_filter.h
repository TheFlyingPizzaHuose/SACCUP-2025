// Programed by Elizabeth McGhee
#ifndef kalman_filter_h
#define kalman_filter_h
#include <iostream>
#include <vector>
using namespace std;

// This is good ! ----------------------------------------------------------
vector<vector<double>> swap_rows(vector<vector<double>> myvec, int n, int m){
  vector<double> temp = myvec[n];
  myvec[n] = myvec[m];
  myvec[m] = temp;

  return myvec;
}

// This is good ! ----------------------------------------------------------
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


// This is good! --------------------------------------------------
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

// This is good ! -----------------------------------------------
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

// This is good ! -----------------------------------------------------------------------
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

// This is good ! -----------------------------------------------------------------------------
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

// This class is good ! ---------------------------------------------------------------------------------------------
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
// ---------------------------------------------------------------------------------

class runKFIMU{
  private:

  public:

    vector<vector<double>> A = {{1,0,0,0,0,0,0,0,0},{0,1,0,0,0,0,0,0,0},{0,0,1,0,0,0,0,0,0},
                                {0,0,0,1,0,0,0,0,0},{0,0,0,0,1,0,0,0,0},{0,0,0,0,0,1,0,0,0},
                                {0,0,0,0,0,0,1,0,0},{0,0,0,0,0,0,0,1,0},{0,0,0,0,0,0,0,0,1}};

    vector<vector<double>> H = {{1,0,0,0,0,0,0,0,0},{0,1,0,0,0,0,0,0,0},{0,0,1,0,0,0,0,0,0},
                                {0,0,0,1,0,0,0,0,0},{0,0,0,0,1,0,0,0,0},{0,0,0,0,0,1,0,0,0},
                                {0,0,0,0,0,0,1,0,0},{0,0,0,0,0,0,0,1,0},{0,0,0,0,0,0,0,0,1}};

    vector<vector<double>> Q = {{0.001,0,0,0,0,0,0,0,0},{0,0.001,0,0,0,0,0,0,0},{0,0,0.001,0,0,0,0,0,0},
                                {0,0,0,0.001,0,0,0,0,0},{0,0,0,0,0.001,0,0,0,0},{0,0,0,0,0,0.001,0,0,0},
                                {0,0,0,0,0,0,0.001,0,0},{0,0,0,0,0,0,0,0.001,0},{0,0,0,0,0,0,0,0,0.001}};

    vector<vector<double>> P = {{0.3,0,0,0,0,0,0,0,0},{0,0.3,0,0,0,0,0,0,0},{0,0,0.3,0,0,0,0,0,0},
                                {0,0,0,0.3,0,0,0,0,0},{0,0,0,0,0.3,0,0,0,0},{0,0,0,0,0,0.3,0,0,0},
                                {0,0,0,0,0,0,0.3,0,0},{0,0,0,0,0,0,0,0.3,0},{0,0,0,0,0,0,0,0,0.3}};
    

    vector<vector<double>> R = {{5,0,0,0,0,0,0,0,0},{0,5,0,0,0,0,0,0,0},{0,0,5,0,0,0,0,0,0},
                                {0,0,0,5,0,0,0,0,0},{0,0,0,0,5,0,0,0,0},{0,0,0,0,0,5,0,0,0},
                                {0,0,0,0,0,0,5,0,0},{0,0,0,0,0,0,5,0,0},{0,0,0,0,0,0,0,0,5}};
                                                               
    vector<vector<double>> R_fuse = {{5,0,0},{0,5,0},{0,0,5}};
    vector<vector<double>> R_end = {{0.3,0,0},{0,0.3,0},{0,0,0.3}};

    vector<vector<double>> z = {{0},{0},{0},{0},{0},{0},{0},{0},{0}};
    vector<vector<double>> a1 = {{0},{0},{0}};
    vector<vector<double>> a2 = {{0},{0},{0}};
    vector<vector<double>> a3 = {{0},{0},{0}};
    vector<vector<double>> g1 = {{0},{0},{0}};
    vector<vector<double>> g2 = {{0},{0},{0}};
    vector<vector<double>> M = {{0},{0},{0}};
    vector<vector<double>> accel = {{0},{0},{0}};
    vector<vector<double>> gyro = {{0},{0},{0}};
    vector<vector<double>> x = {{0},{0},{0},{0},{0},{0},{0},{0},{0}};

    void updateData(double ax1, double ax2, double ax3, double ay1, double ay2, double ay3, double az1, double az2, double az3,
                    double gx1, double gx2, double gy1, double gy2, double gz1, double gz2, double mx, double my, double mz){
      a1[0][0] = ax1;
      a1[1][0] = ay1;
      a1[2][0] = az1;
      a2[0][0] = ax2;
      a2[1][0] = ay2;
      a2[2][0] = az2;
      a3[0][0] = ax3;
      a3[1][0] = ay3;
      a3[2][0] = az3;

      g1[0][0] = gx1;
      g1[1][0] = gy1;
      g1[2][0] = gz1;
      g2[0][0] = gx2;
      g2[1][0] = gy2;
      g2[2][0] = gz2;

      M[0][0] = mx;
      M[1][0] = my;
      M[2][0] = mz;

    }

    void fuseDataAcceleration_and_Gyro(){
      accel = matrix_multiplication(inverse(R_fuse.size(),R_fuse), a1);
      accel = matrix_addition(matrix_multiplication(inverse(R_fuse.size(),R_fuse), a2), accel);
      accel = matrix_addition(matrix_multiplication(inverse(R_fuse.size(),R_fuse), a3), accel);
      accel = matrix_multiplication(R_fuse, accel);

      gyro = matrix_multiplication(R_fuse, matrix_multiplication(inverse(R_fuse.size(),R_fuse), g1));
      gyro = matrix_addition(matrix_multiplication(R_fuse, matrix_multiplication(inverse(R_fuse.size(),R_fuse), g2)), gyro);  
      

      z[0][0] = accel[0][0];
      z[1][0] = accel[1][0];
      z[2][0] = accel[2][0];
      z[3][0] = gyro[0][0];
      z[4][0] = gyro[1][0];
      z[5][0] = gyro[2][0];
      z[6][0] = M[0][0];
      z[7][0] = M[1][0];
      z[8][0] = M[2][0];

    }

    void getFiltered(){
      KalmanFilter myObj(A,H,Q,R);

      x = myObj.run_kalman_filter_estimate(P, x, z);
      P = myObj.run_kalman_filter_covar(P, x, z);
    };
};


class runKFOrientation {
  private:

  public:
    vector<vector<double>> A = {{1,0,0,0},
                                {0,1,0,0},
                                {0,0,1,0},
                                {0,0,0,1}};
    vector<vector<double>> Q = {{0.001,0,0,0},
                                {0,0.001,0,0},
                                {0,0,0.001,0},
                                {0,0,0,0.001}};
    vector<vector<double>> H = {{0,0,0,0}, 
                                {0,0,0,0},
                                {0,0,0,0}};
    vector<vector<double>> R = {{1.0,0,0},
                                {0,1.0,0},
                                {0,0,1.0}};
    vector<vector<double>> z = {{0},{0},{0}};
    vector<vector<double>> x = {{0},{0},{0},{0}};
    vector<vector<double>> P = {{0.001,0,0,0},
                                {0,0.001,0,0},
                                {0,0,0.001,0},
                                {0,0,0,0.001}};

    double gx, gy, gz;

    void updateData(double ax1, double ax2, double ax3, double ay1, double ay2, double ay3, double az1, double az2, double az3,
                    double gx1, double gx2, double gy1, double gy2, double gz1, double gz2, double mx, double my, double mz){
      runKFIMU obj;
      obj.updateData(ax1, ax2, ax3, ay1, ay2, ay3, az1, az2, az3, gx1, gx2, gy1, gy2, gz1, gz2, mx, my, mz);
      obj.fuseDataAcceleration_and_Gyro();
      obj.getFiltered();                
      gx = obj.x[3][0];
      gy = obj.x[4][0];
      gz = obj.x[5][0];
    
      z[0][0] = gx;
      z[1][0] = gy;
      z[2][0] = gz;
    }
    void update_state_transition(){
      double t = 0.8;
      A = {{t/2,  -t/2 * gx, -t/2 * gy, -t/2 * gz},         
          {t/2 * gx, t/2, t/2 * gz,  t/2 * gy},
          {t/2 * gy, -t/2 * gz, t/2, t/2 * gx},
          {t/2 * gz, -t/2 * gy, -t/2 * gx, t/2}};

    }

    void getOrientation(){
      KalmanFilter myObj(A, H, Q, R);
      x = myObj.run_kalman_filter_covar(P, x, z);
      P = myObj.run_kalman_filter_covar(P, x, z);
    }



};

    
#endif /* kalman_filter_h */
