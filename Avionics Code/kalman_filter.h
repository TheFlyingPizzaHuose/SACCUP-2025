// Programed by Elizabeth McGhee

#ifndef kalman_filter_h
#define kalman_filter_h
#include <iostream>
#include <vector>
using namespace std;

vector<double> quaternion_mult(vector<double> mat1, vector<double> mat2){
    vector<double> result(4, 1);
    result[0] = mat1[0] * mat2[0] - mat1[1] * mat2[1] - mat1[2] * mat2[2] - mat1[3] * mat2[3];
    result[1] = mat1[0] * mat2[1] + mat1[1] * mat2[0] + mat1[2] * mat2[3] - mat1[3] * mat2[2];
    result[2] = mat1[0] * mat2[2] + mat1[2] * mat2[0] - mat1[1] * mat2[3] + mat1[3] * mat2[1];
    result[3] = mat1[0] * mat2[3] + mat1[3] * mat2[0] + mat1[1] * mat2[2] - mat1[2] * mat2[1];

    return result;
}

vector<vector<double>> triangulate(vector<vector<double>> my_mat){
    int n = my_mat.size();
    for (int i = 0; i < n-1; i++){
        // int pivotRow = i;
        // for (int j = i+1; j < n; j++){
        //     if (fabs(my_mat[j][i]) > fabs(my_mat[pivotRow][i])){
        //         pivotRow = j;
        //     }
        // }

        // // if (pivotRow != i){
        // //     // cout << "WRITE SWAP Function" << endl;
        // // }
        double diag = my_mat[i][i];
        for (int j = i+1; j<n; j++){
            double mult = my_mat[j][i] / my_mat[i][i];
            for (int k = i; k < n; k++){
            my_mat[j][k] -= mult * my_mat[i][k];
            }
        }
    }
    return my_mat;
}

double get_determinate(vector<vector<double>> my_mat, int n){
    double my_det = 1.0;
    for (int i = 0; i < n; i++){
        my_det *= my_mat[i][i];                           
    }
    return my_det;
}

vector<vector<double>> transposeMat(vector<vector<double>> mat) {
  int n = mat.size();
  vector<vector<double>> my_result(n, vector<double>(n, 0));
  for (int i = 0; i < my_result.size(); i++) {
    for (int j = 0; j < my_result[i].size(); j++) {
      my_result[i][j] = mat[j][i];
    }
  }
  return my_result;
}


double getMinor(const vector<vector<double>>& mat, int row, int col) {
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
    return get_determinate(submatrix, n-1);
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
    double det = get_determinate(my_vector, n);
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
    int n = mat2.size();
    vector<vector<double>> my_result(m, vector<double>(n, 0));
    for (int i = 0; i < m; i++) {
      for (int j = 0; j < n; j++) {
        my_result[i][j] = mat1[j][i] + mat2[i][j];
      }
    }
    return my_result;
  }

vector<vector<double>> matrix_multiplication(vector<vector<double>> mat1, vector<vector<double>> mat2){
    int r1 = mat1.size();
    int c1 = mat1[0].size();
    int r2 = mat2.size();
    int c2 = mat2[0].size();
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
        my_result[i][j] = mat1[j][i] - mat2[i][j];
      }
    }
  return my_result;
}
vector<vector<double>> cross_product(vector<vector<double>> mat1, vector<vector<double>> mat2){
    vector<vector<double>> result(3, vector<double>(1,0));
    result[0][0] = mat1[1][0]*mat2[2][0] - mat1[2][0] * mat2[1][0];
    result[1][0] = mat1[0][0]*mat2[2][0] - mat1[2][0] * mat2[0][0];
    result[2][0] = mat1[0][0]*mat2[1][0] - mat1[1][0] * mat2[0][0];

    return result;
}

#endif /* kalman_filter_h */
