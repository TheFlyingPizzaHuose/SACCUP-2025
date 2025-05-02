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

// double get_determinate(vector<vector<double>> my_mat, int n){
//     double my_det = 1.0;
//     for (int i = 0; i < n; i++){
//         my_det *= my_mat[i][i];                           
//     }
//     cout << my_det << endl;
//     return my_det;
// }

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

#endif /* kalman_filter_h */
