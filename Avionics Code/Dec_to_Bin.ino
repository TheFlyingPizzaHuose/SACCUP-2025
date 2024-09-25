#include <cmath>
#include <iostream>
using namespace std;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
}

int* dec_to_binary(float my_dec, int my_bit){
  int my_dec_int = static_cast<int>(round(my_dec));
  int my_rem;
  static int my_arr[32];
  // if (my_bit>32){
  //  cout << "Array requested too large" << endl;
  //  }
  for (int i = my_bit-1; i >= 0; i--){
    int my_exp = pow(2.0, i);
    my_rem = my_dec_int%my_exp;
    if (my_dec_int >= my_exp){
      my_arr[i] = 1;
    }else if (my_dec_int < my_exp){
      my_arr[i] = 0;
    }
    my_dec_int = my_rem;
  }

  // for (int i = 0; i < my_bit; i++){
  //   if (my_arr[i] == 0){
  //     my_arr[i] = 1;
  //   } else if (my_arr[i] == 1){
  //     my_arr[i] = 0;
  //   } 
  // }
  // my_arr[7] = 1;
  return my_arr;
}


void loop() {
  // put your main code here, to run repeatedly 
  for (int i = 15; i >= 0; i--){ //The array returns the binary number in reverse order so it needs to be printed in reverse order
    Serial.print(*(dec_to_binary(24649, 16)+i));
  }
  Serial.println();
}  
