// Programed by Elizabeth McGhee
#ifndef kalman_filter_h
#define kalman_filter_h
#include <iostream>
#include <vector>
using namespace std;

// This is good ! ----------------------------------------------------------
vector<vector<float>> swap_rows(vector<vector<float>> myvec, int n, int m){
  vector<float> temp = myvec[n];
  myvec[n] = myvec[m];
  myvec[m] = temp;

  return myvec;
}

// This is good ! ----------------------------------------------------------
vector<vector<float>> inverse(int n, vector<vector<float>> mat){
  vector<vector<float>> I(n, vector<float>(n,0.0));

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
  

  float ratio;
  
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
vector<vector<float>> transposeMat(vector<vector<float>> mat) {
  int n = mat.size();
  int m = mat[0].size();
  vector<vector<float>> my_result(m, vector<float>(n, 0));
  for (int i = 0; i < my_result.size(); i++) {
    for (int j = 0; j < my_result[i].size(); j++) {
      my_result[i][j] = mat[j][i];
    }
  }
  return my_result;
}

// This is good ! -----------------------------------------------
vector<vector<float>> matrix_addition(vector<vector<float>> mat1, vector<vector<float>> mat2){
    int m = mat1.size();
    int n = mat1[0].size();
    vector<vector<float>> my_result(m, vector<float>(n, 0));
    for (int i = 0; i < m; i++) {
      for (int j = 0; j < n; j++) {
        my_result[i][j] = mat1[i][j] + mat2[i][j];
      }
    }
    return my_result;
  }

// This is good ! -----------------------------------------------------------------------
vector<vector<float>> matrix_multiplication(vector<vector<float>> mat1, vector<vector<float>> mat2){
    int r1 = mat1.size();
    int c1 = mat1[0].size();
    int r2 = mat2.size();
    int c2 = mat2[0].size();
    if (c1 != r2){
      cout << "Aborting Program" << endl;
      abort();
    }
    vector<vector<float>> my_result(r1, vector<float>(c2, 0));
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
vector<vector<float>> subtractMat(vector<vector<float>> mat1, vector<vector<float>> mat2) {
  int r1 = mat1.size();
  int c1 = mat1[0].size();
  vector<vector<float>> my_result(r1, vector<float>(c1, 0));
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
      vector<vector<float>> A_matrix;
      vector<vector<float>> H_matrix;
      vector<vector<float>> Q_matrix;
      vector<vector<float>> R_matrix;
      

      KalmanFilter(vector<vector<float>> A, vector<vector<float>> H, 
          vector<vector<float>> Q, vector<vector<float>> R){
          A_matrix = A;
          H_matrix = H;
          Q_matrix = Q;
          R_matrix = R;
      }
      ~KalmanFilter(){}
      
      vector<vector<float>> predict_state(vector<vector<float>> state_0){
        return matrix_multiplication(A_matrix, state_0);
        }
        
      vector<vector<float>> predict_error_covariance(vector<vector<float>> err_co_0){
          vector<vector<float>> result = matrix_multiplication(A_matrix, err_co_0);
          return matrix_addition(matrix_multiplication(result, transposeMat(A_matrix)), Q_matrix);
        }

      
      vector<vector<float>> kalman_gain(vector<vector<float>> pred_co){
          vector<vector<float>> result = matrix_multiplication(H_matrix, pred_co);
          vector<vector<float>> result_two = transposeMat(H_matrix); 
          vector<vector<float>> result_three = matrix_multiplication(result, result_two);
          vector<vector<float>> result_four = matrix_addition(result_three, R_matrix);
          vector<vector<float>> result_five = matrix_multiplication(pred_co, result_two);
          vector<vector<float>> result_six = inverse(result_four.size(), result_four); //this is the problem function :(

          return matrix_multiplication(result_five, result_six);

        }
      vector<vector<float>> estimate(vector<vector<float>> kal_gain, vector<vector<float>> pred_state, vector<vector<float>> z_0){
          vector<vector<float>> result = subtractMat(z_0, matrix_multiplication(H_matrix, pred_state));
          return matrix_addition(pred_state, matrix_multiplication(kal_gain, result));
        }
        
      vector<vector<float>> compute_error_covariance(vector<vector<float>> pred_co, vector<vector<float>> kal_gain){
          vector<vector<float>> result = matrix_multiplication(kal_gain, H_matrix);
          return subtractMat(pred_co, matrix_multiplication(result, pred_co));
      }

      vector<vector<float>> run_kalman_filter_estimate(vector<vector<float>> err_co_estimate, 
          vector<vector<float>> state_0, vector<vector<float>> measurement){
          vector<vector<float>> predicted_state = predict_state(state_0);
          vector<vector<float>> predicted_err_co = predict_error_covariance(err_co_estimate);
          vector<vector<float>> kal_gain = kalman_gain(predicted_err_co);
          vector<vector<float>> est_state = estimate(kal_gain, predicted_state, measurement);
          vector<vector<float>> comp_err_co = compute_error_covariance(predicted_err_co, kal_gain);
          return est_state;
      }

      vector<vector<float>> run_kalman_filter_covar(vector<vector<float>> err_co_estimate, 
        vector<vector<float>> state_0, vector<vector<float>> measurement){
        vector<vector<float>> predicted_state = predict_state(state_0);
        vector<vector<float>> predicted_err_co = predict_error_covariance(err_co_estimate);
        vector<vector<float>> kal_gain = kalman_gain(predicted_err_co);
        vector<vector<float>> comp_err_co = compute_error_covariance(predicted_err_co, kal_gain);
        return comp_err_co;
    }
};
// ---------------------------------------------------------------------------------

class runKFIMU{
  private:

  public:

    vector<vector<float>> A =  {{1,0,0,0,0,0,0,0,0},
                                {0,1,0,0,0,0,0,0,0},
                                {0,0,1,0,0,0,0,0,0},
                                {0,0,0,1,0,0,0,0,0},
                                {0,0,0,0,1,0,0,0,0},
                                {0,0,0,0,0,1,0,0,0},
                                {0,0,0,0,0,0,1,0,0},
                                {0,0,0,0,0,0,0,1,0},
                                {0,0,0,0,0,0,0,0,1}};

    vector<vector<float>> H = {{1,0,0,0,0,0,0,0,0},
                                {1,0,0,0,0,0,0,0,0},
                                {1,0,0,0,0,0,0,0,0},
                                {0,1,0,0,0,0,0,0,0},
                                {0,1,0,0,0,0,0,0,0},
                                {0,1,0,0,0,0,0,0,0},
                                {0,0,1,0,0,0,0,0,0},
                                {0,0,1,0,0,0,0,0,0},
                                {0,0,1,0,0,0,0,0,0},
                                {0,0,0,1,0,0,0,0,0},
                                {0,0,0,1,0,0,0,0,0},
                                {0,0,0,0,1,0,0,0,0},
                                {0,0,0,0,1,0,0,0,0},
                                {0,0,0,0,0,1,0,0,0},
                                {0,0,0,0,0,1,0,0,0},
                                {0,0,0,0,0,0,1,0,0},
                                {0,0,0,0,0,0,0,1,0},
                                {0,0,0,0,0,0,0,0,1}};


    vector<vector<float>> Q = {{0.001,0,0,0,0,0,0,0,0},
                                {0,0.001,0,0,0,0,0,0,0},
                                {0,0,0.001,0,0,0,0,0,0},
                                {0,0,0,0.001,0,0,0,0,0},
                                {0,0,0,0,0.001,0,0,0,0},
                                {0,0,0,0,0,0.001,0,0,0},
                                {0,0,0,0,0,0,0.001,0,0},
                                {0,0,0,0,0,0,0,0.001,0},
                                {0,0,0,0,0,0,0,0,0.001}};

    vector<vector<float>> P = {{0.3,0,0,0,0,0,0,0,0},
                                {0,0.3,0,0,0,0,0,0,0},
                                {0,0,0.3,0,0,0,0,0,0},
                                {0,0,0,0.3,0,0,0,0,0},
                                {0,0,0,0,0.3,0,0,0,0},
                                {0,0,0,0,0,0.3,0,0,0},
                                {0,0,0,0,0,0,0.3,0,0},
                                {0,0,0,0,0,0,0,0.3,0},
                                {0,0,0,0,0,0,0,0,0.3}};
    

    vector<vector<float>> R = {{5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,5,0,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,5,0,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,5,0,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,5,0,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,5,0,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,5,0,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,5,0,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,5,0,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,5,0,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,5,0,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5,0,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5,0},
                                {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,5}};
                                                              

    vector<vector<float>> z = {{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0}};
    vector<vector<float>> x = {{0},{0},{0},{0},{0},{0},{0},{0},{0}};

    void updateData(float ax1, float ax2, float ax3, float ay1, float ay2, float ay3, float az1, float az2, float az3,
                    float gx1, float gx2, float gy1, float gy2, float gz1, float gz2, float mx, float my, float mz){
      z[0][0] = ax1;
      z[1][0] = ay1;
      z[2][0] = az1;
      z[3][0] = ax2;
      z[4][0] = ay2;
      z[5][0] = az2;
      z[6][0] = ax3;
      z[7][0] = ay3;
      z[8][0] = az3;
      z[9][0] = gx1;
      z[10][0] = gy1;
      z[11][0] = gz1;
      z[12][0] = gx2;
      z[13][0] = gy2;
      z[14][0] = gz2;
      z[15][0] = mx;
      z[16][0] = my;
      z[17][0] = mz;

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
    vector<vector<float>> A =  {{1,0,0,0},
                                {0,1,0,0},
                                {0,0,1,0},
                                {0,0,0,1}};
                                
    vector<vector<float>> Q =   {{0.001,0,0,0},
                                {0,0.001,0,0},
                                {0,0,0.001,0},
                                {0,0,0,0.001}};

    vector<vector<float>> H =  {{0,0,0,0}, 
                                {0,0,0,0},
                                {0,0,0,0},
                                {0,0,0,0}, 
                                {0,0,0,0},
                                {0,0,0,0}};

    vector<vector<float>> R = {{3,0,0,0,0,0},
                               {0,3,0,0,0,0},
                               {0,0,3,0,0,0},
                               {0,0,0,3,0,0},
                               {0,0,0,0,3,0},
                               {0,0,0,0,0,3}};

    vector<vector<float>> z = {{0},{0},{0},{0},{0},{0}};
    vector<vector<float>> x = {{0},{0},{0},{1}};
    vector<vector<float>> P = {{0.001,0,0,0},
                               {0,0.001,0,0},
                               {0,0,0.001,0},
                               {0,0,0,0.001}};
    float gx, gy, gz;

    void updateData(float ax1, float ax2, float ax3, float ay1, float ay2, float ay3, float az1, float az2, float az3,
                    float gx1, float gx2, float gy1, float gy2, float gz1, float gz2, float mx, float my, float mz){
      runKFIMU obj;
      obj.updateData(ax1, ax2, ax3, ay1, ay2, ay3, az1, az2, az3, gx1, gx2, gy1, gy2, gz1, gz2, mx, my, mz);
      obj.getFiltered();                
      gx = obj.x[3][0];
      gy = obj.x[4][0];
      gz = obj.x[5][0];
    
      z[0][0] = gx;
      z[1][0] = gy;
      z[2][0] = gz;
      z[3][0] = mx;
      z[4][0] = my;
      z[5][0] = mz;

    }
    void update_state_transition(){
      float t = 0.04;
      A = {{1,  -t/2 * gx, -t/2 * gy, -t/2 * gz},         
          {t/2 * gx, 1, t/2 * gz,  t/2 * gy},
          {t/2 * gy, -t/2 * gz, 1, t/2 * gx},
          {t/2 * gz, -t/2 * gy, -t/2 * gx, 1}};

    }

    void getOrientation(){
      KalmanFilter myObj(A, H, Q, R);
      x = myObj.run_kalman_filter_covar(P, x, z);
      P = myObj.run_kalman_filter_covar(P, x, z);
      float q_mag = sqrt(pow(x[0][0],2) + pow(x[1][0],2) + pow(x[2][0],2) + pow(x[3][0],2));
      x[0][0] = x[0][0]/q_mag;
      x[1][0] = x[1][0]/q_mag;
      x[2][0] = x[2][0]/q_mag;
      x[3][0] = x[3][0]/q_mag;
      
      }



};

    
#endif /* kalman_filter_h */
