#include <iostream>
#include <vector>
#include <cmath>
#include <chrono>

using namespace std;
using namespace std::chrono;
// Coded by Elizabeth McGhee -----------------------------------------------------------------------------------------------
// Matrix operations for using a Kalman filter. Still in progress. 
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

    
int main(){


    vector<vector<float>> matrix; 
    matrix = {{-1.0, 2.2, -3.5, 5.4, -9.2, 6.5, -1.2, 11.1123, 1.02359, 13.9}, 
              {1.1, -2.1, -3.1, 5.1, -9.2, 6.5, 4.05, 23.83, 0.32, 13.38}, 
              {1.2, -2.32, 3.2, -5.2, -9.2, 6.5, -1.02, 11.13422, 1.0159, 13.8},
              {-1.3, 2.35, 3.3, 5.3, -9.2, 6.5, 4.125, -1.9292, -4.802, -1.0902},
              {-1.4, -2.4, 6.85, 4.25, -1.944, -2.3002, -1.87652, 11.6781, 1.09234, 13.7},
              {5.5, -9.2, 6.5, -4.35, -1.93, -1.002, -1.989752, 11.1687, 1.0952, 13.36},
              {-1.6, 2.6, 3.6, 5.6, -9.2, 6.5, -14.5, 0.112, 1.09, 13.6},
              {1.7, -2.7, 3.77, 5.7, -9.2, 6.5, 32.0025, 0.322, -0.11112, 1.9992},
              {-1.8, -2.8, -3.8, 5.8, -9.2, 6.5, 14.532, 11.6781, 3.009, 13.4},
              {1.9, 2.9, -3.98, -5.9, -0.2432, -5.402, -1.2, 11.1, 1.09, 13.34}};
    auto start = high_resolution_clock::now();
    get_inverse(matrix, 10);
    auto stop = high_resolution_clock::now();
    auto duration = duration_cast<microseconds>(stop - start);

    cout << duration.count() << " microseconds" << endl;   
    cout << find_det(matrix, 10) << endl;

    return 0;
}

