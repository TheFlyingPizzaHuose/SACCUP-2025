#include <iostream>
#include <vector>
#include "kalman_filter.h"
using namespace std;



vector<vector<double>> predict_state(vector<vector<double>> state_0, vector<vector<double>> a_matrix){
  return matrix_multiplication(a_matrix, state_0);
}

// 
vector<vector<double>> predict_error_covariance(vector<vector<double>> a_matrix, vector<vector<double>> p_0, vector<vector<double>> q_matrix){
  vector<vector<double>> result = matrix_multiplication(a_matrix, p_0);
  return matrix_addition(matrix_multiplication(result, transposeMat(a_matrix)), q_matrix);
}

vector<vector<double>> kalman_gain(vector<vector<double>> p_0, vector<vector<double>> h_matrix, vector<vector<double>> r_matrix){
  vector<vector<double>> result = matrix_multiplication(h_matrix, p_0);
  vector<vector<double>> result_two = transposeMat(h_matrix);
  vector<vector<double>> result_three = matrix_multiplication(result, result_two);
  vector<vector<double>> result_four = matrix_multiplication(p_0, result_two);
  return matrix_multiplication(result_three, get_inverse(matrix_addition(result_three, r_matrix), r_matrix.size()));
}

vector<vector<double>> estimate(vector<vector<double>> x_0, vector<vector<double>> z_0, vector<vector<double>> h_matrix, vector<vector<double>>K_matrix){
  vector<vector<double>> result = subtractMat(z_0, matrix_multiplication(h_matrix, x_0));
  return matrix_addition(x_0, matrix_multiplication(K_matrix, result));
}

vector<vector<double>> compute_error_covariance(vector<vector<double>> p_0, vector<vector<double>> K_matrix, vector<vector<double>> h_matrix){
  vector<vector<double>> result = matrix_multiplication(K_matrix, h_matrix);
  return matrix_addition(p_0, matrix_multiplication(result, p_0));
}