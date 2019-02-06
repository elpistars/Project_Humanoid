#ifndef TESTREAD_C
#define TESTREAD_C

#include "/home/pi/systemTest/library/allLib.h"
#include "/home/pi/systemTest/library/movement/moveArray.h"
#include "/home/pi/systemTest/library/dynamixel/syncWrite.cpp"

        double x,y,z,sudutX,sudutY,sudutZ,x2,y2,z2,sudutX2,sudutY2,sudutZ2,sudutNgurang;
        int a1,speed,a2,a3,a4,a5,a6,a7,a8,a9,a10,a11,a12;

/*        cout<<"Masukkan sudut X: ";
        cin>>sudutX;
        cout<<"Masukkan sudut Y: ";
        cin>>sudutY;
        cout<<"Masukkan sudut Z: ";
        cin>>sudutZ;

        cout<<"Masukkan X: ";
        cin>>x;
        cout<<"Masukkan Y: ";
        cin>>y;
        cout<<"Masukkan Z: ";
        cin>>z;*/
       //while(1){
       
	int main(){
if(dxl_initialize(0,1)==0){
        cout<<"Error connecting"<<endl;
        return 0;
    }

else{
                cout<<"connect success!"<<endl;
        }

/*	  x=-9;//-sin(sudutNgurang*3.14/180)*2;
        y=3;
        z=-0;
        sudutX=-6;
        sudutY=16;
        sudutZ=0;
        x2=-11;
        y2=3;
        z2=0;
        sudutX2=-12;
        sudutY2=16;
        sudutZ2=0;

	 inverseAndre(sudutX,sudutY,sudutZ,x,y,z,sudutX2,sudutY2,sudutZ2,x2,y2,z2,50,380,680,'2');
*/

while(1){
a1=dxl_read_word( 8, 40 );
a2=dxl_read_word( 10, 40 );
a3=dxl_read_word( 12, 40 );
a4=dxl_read_word( 14, 40 );
a5=dxl_read_word( 16, 40 );
a6=dxl_read_word( 18, 40 );
a7=dxl_read_word( 7, 40 );
a8=dxl_read_word( 9, 40 );
a9=dxl_read_word( 11, 40 );
a10=dxl_read_word( 13, 40 );
a11=dxl_read_word( 15, 40 );
a12=dxl_read_word( 17, 40 );

cout<<"id8 "<<a1<<" id10 "<<a2<<" id12 "<<a3<<" id14 "<<a4<<" id16 "<<a5<<" id18 "<<a6<<" id7 "<<a7<<" id9 "<<a8<<" id11 "<<a9<<" id13 "<<a10<<" id15 "<<a11<<" id17 "<<a12<<endl;
cout<<"  "<<endl;


}
}




#endif
