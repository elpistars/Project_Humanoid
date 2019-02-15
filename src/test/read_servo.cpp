#include <dynamixel.h>
#include <iostream>
#include <unistd.h>
#include <stdio.h>
using namespace std;
struct servo_t{
	char label[20];
	float torque;
	int pos;
	char load_dir[3];
}servo[255];
int comstat=0;
int rpos(int id);
void read_torque(int id);
void read_pos(int id);
int main(){
	int i=0;
	dxl_initialize(0,1);
/*	if(dxl_initialize(0,1)==1){
		cout<<"Servo Access Succes"<<endl;
	}
	else{
		cout<<"Servo Access Failed"<<endl;
		return 0;
	}
*/
//	for(i=1;i<=18;i++){
	/*	read_torque(i);
		cout<<i-1<<": "<<servo[i-1].torque<<"% "<<servo[i-1].load_dir<<endl;*/
	/*	read_pos(i);
		cout<<i<<": "<<servo[i-1].pos<<endl;*/
//	}
	while(1){
		i=8;
		int x;
		for(i=1;i<=20;i++){
			//read_pos(i);
			//while((x=dxl_get_result())==6);//cout<<x<<endl;
			//cout<<dxl_get_result()<<endl;
			servo[i-1].pos=rpos(i);
			cout<<i<<": "<<servo[i-1].pos<<"  stat "<<comstat<<endl;//<<"% "<<servo[i-1].load_dir;
			//if(i!=18)cout<<"  ||  ";
			//sleep(1);
	//		usleep(1000);
		}
		cout<<endl;
		usleep(500000);
	}
	dxl_terminate();
	return 0;
}
void read_torque(int id){
	int x=0;
	int temp=0;
	int torque[2]={0,0};
	//x=dxl_get_result();
	temp=dxl_read_byte(id,3);//dxl_read_word(id,40);
	x=dxl_get_result();
	cout<<x<<"__";
	if(x!=1){
		//1temp=dxl_read_byte(id,3);//dxl_read_word(id,40);
		//x=dxl_get_result();
		cout<<dxl_get_rxpacket_error(x)<<endl;
	}	
	cout<<id<<": "<<temp<<" ";//<<endl;
	torque[1]=temp & 0xFF; // low byte
	torque[0]=(temp>>8) & 0xFF; // high byte
	/*cout<<torque[1]<<endl;
	cout<<torque[0]<<endl;*/
	if(torque[0]==0)
		sprintf(servo[id-1].load_dir,"CCW");
	else
		sprintf(servo[id-1].load_dir,"CW");
	servo[id-1].torque=(float)torque[1]*0.1f;
}
void read_pos(int id){
	int temp=0;
	comstat=0;
	temp=dxl_read_byte(id,3);
	comstat=dxl_get_result();
	while(comstat==6){
	  temp=dxl_read_byte(id,3);
	  comstat=dxl_get_result();
	  //cout<<comstat<<endl;	  
	  //comstat=dxl_get_result();
	}
        while(comstat==7){
          temp=dxl_read_byte(id,3);
          comstat=dxl_get_result();
          //cout<<comstat<<endl;          
          //comstat=dxl_get_result();
        }
	servo[id-1].pos=temp;
	//cout<<servo[id-1].pos;
}
int rpos(int id){
        int temp=0;
        comstat=0;
        temp=dxl_read_word(id,30);
        comstat=dxl_get_result();
	if(comstat==6){
//	  cout<<"Timeout"<<endl;
	  return rpos(id);
	}
	else if(comstat==7){
//	  cout<<"Corrupt"<<endl;
	  return rpos(id);
	}
	else{
	  return temp;
	}
}

