#include "/home/pi/systemTest/library/allLib.h"
#include "/home/pi/systemTest/library/movement/moveArray.h"
#include "/home/pi/systemTest/library/dynamixel/syncWrite.cpp"



int main(){

if(dxl_initialize(0,1)==0){
        cout<<"Error connecting"<<dxl_initialize(1,1)<<endl;
        return 0;
    }

else{
                cout<<"connect success!"<<endl;
        }
 cout<<"connect success!"<<dxl_initialize(0,1)<<endl;

}
