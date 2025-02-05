#include <iostream>
#include <math.h>
#include <vector>
using namespace std; 

float taylor_series_cosine(float z, int n){
    float factorial;
    float sum = 1;
    float cosine = 0;
    float pi = 3.14;
    float x;
    int m;
    x = z * (3.14159265359/180);
    for (int i = 0; i < n; i++){
        if (i == 0){
            cosine = pow(-1, i) * pow(x, 2*i)/1 + cosine;
        } else if (i > 0){
            factorial = 2*i;
            m = 2*i;
            cosine = pow(-1, i) * pow(x, 2*i)/(factorial*(m-1)*sum) + cosine;
            sum = factorial*(m-1)*sum;
        }
    }
    if (z == 90 || z == 270){
        cosine = 0;
    } else if (z == 360 || z == 0){
        cosine = 1;
    }
    return cosine;
}

vector<vector<float>> get_cosine(){
    vector<vector<float>> my_vec(3601,vector<float>(1,0));
    for (int i = 0; i < 3601; i++){
        my_vec[i][0] = taylor_series_cosine(i*0.1, 100);
    }
    return my_vec;
}
vector<vector<float>> initialize_cosine(bool start){ 
    vector<vector<float>> val; 
    if (start == true){
        return get_cosine();
    } else{
        val = {{0}};
        return val;
    }
}

vector<vector<float>> get_sine(){
    vector<vector<float>> my_vec(3601,vector<float>(1,0));
    for (int i = 0; i < 3601; i++){
        my_vec[i][0] = taylor_series_cosine(90-i*0.1, 100);
    }
    return my_vec;
}

vector<vector<float>> initialize_sine(bool start){
    if (start == true){
        return get_sine();
    } else{
        vector<vector<float>> null_val = {{0}};
        return null_val;
    }
}

float taylor_series_arcsine(float z, int n){
    float factorial;
    float factorial_two;
    float sum = 1;
    float sum_2 = 1;
    float arcsine = 0;
    float pi = 3.14159265359;
    int m;
    for (int i = 0; i < n; i++){
        if (i == 0){
            arcsine = ((1/(pow(2, 2*i) * 1 * (2*i+1))) * pow(z, 2*i+1)) + arcsine;
        } else if (i > 0){
            factorial = 2*i;
            m = 2*i;
            arcsine = ((factorial*(m-1)*sum)/(pow(2, 2*i) * (i*sum_2) * (2*i+1))* pow(z, 2*i+1)) + arcsine;
            sum_2 = i*sum_2;
            sum = factorial*(m-1)*sum;
        }
    }
    arcsine = arcsine * (180/pi);

    return arcsine;
}

float taylor_series_arccosine(float z, int n){
    float arccosine; 
    float pi = 3.14159265359;
    arccosine = pi/2 - taylor_series_arcsine(z, n);
    return arccosine;
}

vector<vector<float>> get_arccosine(){
    vector<vector<float>> my_vec(3601,vector<float>(1,0));
    for (int i = 0; i < 3601; i++){
        my_vec[i][0] = taylor_series_arccosine(i*0.1, 10);
    }
    return my_vec;
}
vector<vector<float>> initialize_arccosine(bool start){ 
    vector<vector<float>> val; 
    if (start == true){
        return get_arccosine();
    } else{
        val = {{0}};
        return val;
    }
}

vector<vector<float>> get_arcsine(){
    vector<vector<float>> my_vec(3601,vector<float>(1,0));
    for (int i = 0; i < 3601; i++){
        my_vec[i][0] = taylor_series_arccosine(90-i*0.1, 10);
    }
    return my_vec;
}

vector<vector<float>> initialize_arcsine(bool start){
    if (start == true){
        return get_arcsine();
    } else{
        vector<vector<float>> null_val = {{0}};
        return null_val;
    }
}

int main(){
    bool start = true;
    cout << get_arcsine()[100][0] << endl;
    return 0;
}