#include <iostream>
#include <vector>
#include <cmath>
#include "kalman_filter.h"
using namespace std;

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
