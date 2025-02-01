#include <iostream>
#include <vector>
#include <cmath>
// #include <chrono>
using namespace std;
// Elizabeth McGhee Kalman Filter code still in progress

float find_det(vector<vector<float>> my_arr, int n){
    float det = 1.0; 
 
    for (int i = 0; i < n - 1; i++){
        int x = i+1;
        for (int k = x; k < n; k++){
            float val = my_arr[k][i]/my_arr[i][i];
            for (int j = 0; j < n; j++){
                my_arr[k][j] = -1 * val * my_arr[i][j] + my_arr[k][j]; 
            }
        }
    }

    for (int i = 0; i < n; i++){
        det = my_arr[i][i] * det;
    }

    return det;
}

vector<vector<float>> transposeMat(vector<vector<float>> mat) {
  int n = mat.size();
  vector<vector<float>> my_result(n, vector<float>(n, 0));
  for (int i = 0; i < my_result.size(); i++) {
    for (int j = 0; j < my_result[i].size(); j++) {
      my_result[i][j] = mat[j][i];
    }
  }
  return my_result;
}


float getMinor(const vector<vector<float>>& mat, int row, int col) {
    int n = mat.size();

    vector<vector<float>> submatrix(n - 1, vector<float>(n - 1));
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
    return find_det(submatrix, n-1);
}

vector<vector<float>> get_Minor_matrix(vector<vector<float>> my_vector, int n){
    vector<vector<float>> result(n, vector<float>(n, 0)); 
    for (int i = 0; i < n; i++){
        for (int j = 0; j < n; j++){
            result[i][j] =  getMinor(my_vector, i, j);
        }
    }
    return result;
}

vector<vector<float>> cofactor(vector<vector<float>> my_vector, int n){
    vector<vector<float>> result(n, vector<float>(n, 0)); 
    vector<vector<float>> minor(n, vector<float>(n, 0));
    minor = get_Minor_matrix(my_vector, n);
    for (int i = 0; i < n; i++){
        for (int j = 0; j < n; j++){
            result[i][j] =  pow(-1, i+j) * minor[i][j];
        }
    }

    return result;
}

vector<vector<float>> get_inverse(vector<vector<float>> my_vector, int n){
    vector<vector<float>> result(n, vector<float>(n, 0)); 
    float det = find_det(my_vector, n);
    result = transposeMat(cofactor(my_vector, n));
    for (int i = 0; i < n; i++){
        for (int j = 0; j < n; j++){
            result[i][j] = result[i][j] / det;
        }
    }

    return result;
}

vector<vector<float>> matrix_addition(vector<vector<float>> mat1, vector<vector<float>> mat2){
    int m = mat1.size();
    int n = mat2.size();
    vector<vector<float>> my_result(m, vector<float>(n, 0));
    for (int i = 0; i < m; i++) {
      for (int j = 0; j < n; j++) {
        my_result[i][j] = mat1[j][i] + mat2[i][j];
      }
    }
    return my_result;
  }

vector<vector<float>> matrix_multiplication(vector<vector<float>> mat1, vector<vector<float>> mat2){
    int r1 = mat1.size();
    int c1 = mat1[0].size();
    int r2 = mat2.size();
    int c2 = mat2[0].size();
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

vector<vector<float>> subtractMat(vector<vector<float>> mat1, vector<vector<float>> mat2) {
  int r1 = mat1.size();
  int c1 = mat1[0].size();
  vector<vector<float>> my_result(r1, vector<float>(c1, 0));
  for (int i = 0; i < my_result.size(); i++) {
      for (int j = 0; j < my_result[i].size(); j++) {
        my_result[i][j] = mat1[j][i] - mat2[i][j];
      }
    }
  return my_result;
}

// Theoretically works
vector<vector<float>> predict_state(vector<vector<float>> state_0, vector<vector<float>> a_matrix){
  return matrix_multiplication(a_matrix, state_0);
}

// 
vector<vector<float>> predict_error_covariance(vector<vector<float>> a_matrix, vector<vector<float>> p_0, vector<vector<float>> q_matrix){
  vector<vector<float>> result = matrix_multiplication(a_matrix, p_0);
  return matrix_addition(matrix_multiplication(result, transposeMat(a_matrix)), q_matrix);
}

vector<vector<float>> kalman_gain(vector<vector<float>> p_0, vector<vector<float>> h_matrix, vector<vector<float>> r_matrix){
  vector<vector<float>> result = matrix_multiplication(h_matrix, p_0);
  vector<vector<float>> result_two = transposeMat(h_matrix);
  vector<vector<float>> result_three = matrix_multiplication(result, result_two);
  vector<vector<float>> result_four = matrix_multiplication(p_0, result_two);
  return matrix_multiplication(result_three, get_inverse(matrix_addition(result_three, r_matrix), r_matrix.size()));
}

vector<vector<float>> estimate(vector<vector<float>> x_0, vector<vector<float>> z_0, vector<vector<float>> h_matrix, vector<vector<float>>K_matrix){
  vector<vector<float>> result = subtractMat(z_0, matrix_multiplication(h_matrix, x_0));
  return matrix_addition(x_0, matrix_multiplication(K_matrix, result));
}

vector<vector<float>> compute_error_covariance(vector<vector<float>> p_0, vector<vector<float>> K_matrix, vector<vector<float>> h_matrix){
  vector<vector<float>> result = matrix_multiplication(K_matrix, h_matrix);
  return matrix_addition(p_0, matrix_multiplication(result, p_0));
}


int main(){


    vector<vector<float>> matrix_a = {{1, 2, 3},{9, 45, 3}, {9,13, 3}};
    vector<vector<float>> x_0 = {{1}, {-9.0}, {2.3}};
    vector<vector<float>> P_0 = {{5, 9.3, 9.0}, {-1.9, -1.3, -.09}, {2.3, -3.4, -6.5}};
    vector<vector<float>> matrix_q = {{-3.4, -4.3, 7.5}, {8.4, -4.8, 9.32}, {-9.2, 3.4, 3.11}};
    vector<vector<float>> matrix_h = {{1, 3, 2}, {8.3, 2.01, 9.2}, {11.2, -93.3, 74.2}};
    vector<vector<float>> matrix_r = {{2.1, 3.1, 0.9}, {-1.2, -11.2, -98.2}, {0.07, -0.2, 3.34}};
    vector<vector<float>> predicted_state = predict_state(x_0, matrix_a);
    vector<vector<float>> predicted_error_covariance = predict_error_covariance(matrix_a, P_0, matrix_q);
    vector<vector<float>> kalman_gain_matrix = kalman_gain(P_0, matrix_h, matrix_r);

    for (int i = 0; i < kalman_gain_matrix.size(); i++){
      for (int j = 0; j < kalman_gain_matrix[i].size(); j++){
        cout << kalman_gain_matrix[i][j]<< " ";
        }
      cout << endl;
    }
    return 0;
}

