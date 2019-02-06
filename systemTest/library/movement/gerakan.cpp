//#ifndef GERAKAN_C
//#define GREAKAN_C
//#include "/home/pi/systemTest/library/movement/moveArray.h"

#include "/home/pi/systemTest/library/movement/bangun.h"

#include "/home/pi/systemTest/library/dynamixel/syncWrite.cpp"
#include "/home/pi/systemTest/library/allLib.h"
//#include "inverseTestA.cpp"

int main(){

if(dxl_initialize(0,1)==0){
        cout<<"Error connecting"<<endl;
        return 0;
    }

else{
                cout<<"connect success!"<<endl;
}
while(1) {

bangun_belakang2();
// miringKanan(10, 3  ,4,4, 4, 4);

sleep(60);
	}
}

//#endif

