#ifndef matrix_operations_h
#define matrix_operations_h
#include <iostream>
#include <vector>
using namespace std;
// Coded by Elizabeth McGhee -----------------------------------------------------------------------------------------------
// Matrix operations for using a Kalman filter. Still in progress. 
float find_det(vector<vector<float>> my_arr, int n){
    long double det = 1.0; 
 
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

    for (int i = 0; i < n; i++){
      for (int j = 0; j < n; j++){
        cout << my_arr[i][j] << " ";
      }
      cout << endl;
    }
    return det;
}

vector<vector<float>> transposeMat(vector<vector<float>> mat, int n) {
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
    result = transposeMat(cofactor(my_vector, n), n);
    for (int i = 0; i < n; i++){
        for (int j = 0; j < n; j++){
            result[i][j] = result[i][j] / det;
        }
    }

    return result;
}

vector<vector<float>> matrix_addition(vector<vector<float>> mat1, vector<vector<float>> mat2, int m, int n){
    vector<vector<float>> my_result(m, vector<float>(n, 0));
    for (int i = 0; i < m; i++) {
      for (int j = 0; j < n; j++) {
        my_result[i][j] = mat1[j][i] + mat2[i][j];
      }
    }
    return my_result;
  }

vector<vector<float>> matrix_multiplication(vector<vector<float>> mat1, vector<vector<float>> mat2, int r1, int r2, int c1, int c2){
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

vector<vector<float>> subtractMat(vector<vector<float>> mat1,
                                           vector<vector<float>> mat2, int r1,
                                           int c1, int r2, int c2) {
  vector<vector<float>> my_result(r1, vector<float>(c1, 0));
  for (int i = 0; i < my_result.size(); i++) {
      for (int j = 0; j < my_result[i].size(); j++) {
        my_result[i][j] = mat1[j][i] - mat2[i][j];
      }
    }
  return my_result;
}

#endif /* matrix_operations_h */