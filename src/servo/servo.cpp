#include <dynamixel.h>
#include <iostream>
using namespace std;
int main(){
  dxl_initialize(0,1);
  dxl_write_word(3,30,512);
  return 0;
}
