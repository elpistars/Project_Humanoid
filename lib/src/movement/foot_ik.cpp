#ifndef SYNCWRITE_CPP
#define SYNCWRITE_CPP

#include <iostream>
#include "dynamixel/dynamixel.h"
#include "movement/foot_ik.h"
#include <unistd.h>
#include <math.h>

using namespace std;

// definition for joint class
Joint::Joint(joint_param init){
        Joint::setParam(init.x,init.y,init.z,init.sudutX,init.sudutY,init.sudutZ);
};
Joint::Joint(){
        Joint::setParam(0,0,0,0,0,0);
}
void Joint::setParam(double x, double y, double z, double sudutX, double sudutY, double sudutZ){
        param.x=x; param.y=y; param.z=z;
        param.sudutX=sudutX; param.sudutY=sudutY; param.sudutZ=param.sudutZ;
}
void Joint::setParam(joint_param newParam){
	param=newParam;
}
inline Joint::joint_param Joint::getParam(){
	return param;
}
inline void Joint::getParam(joint_param &p){
	p=param;
}
//definition for Invers Kinematics class
IK::IK(){
	IK::setParam(0,0,0);
}
IK::IK(double teta, double d_teta, int Speed){
	IK::setParam(teta,d_teta,Speed);
}
void IK::setParam(double teta, double d_teta, int Speed){
	param.sudut=teta;
	param.inc_sudut=d_teta;
	param.speed=Speed;
}
void IK::setParam(ik_param newParam){
	param=newParam;
}
inline IK::ik_param IK::getParam(){
	return param;
}
inline void IK::getParam(ik_param &p){
	p=param;
}
int IK::cek_move(){
	int moving=0;
	int temp=0;
	int i=0;
	int comstat=0;
	for(i=1;i<=20;i++){
		temp=dxl_read_byte(i,46);
		if(temp==1){
		  break;
		  moving=1;
		  return moving; 
		}
		else{
		  comstat=dxl_get_result();
		  if(comstat==6){
		    break;
		    return cek_move();
		  }
		  else if(comstat==7){
		    break;
		    return cek_move();
		  }
		  else{}
		}
	}
	return moving;
}
void IK::kirimPacketSpeed(int totalServo, int id[], int sp){ //Buat ngeset kecepatan servo
        int l=2;
        int n=totalServo;

        dxl_set_txpacket_id(BROADCAST_ID); //Jangan diubah 0xFE
        //dxl_set_txpacket_length(); //jumlah paket yang dikirim 0x18
        dxl_set_txpacket_instruction(INST_SYNC_WRITE);//instruksi 0x83
        dxl_set_txpacket_parameter(0,P_GOAL_SPEED_L);//yang mau ditulis 0x1E
        dxl_set_txpacket_parameter(1,2);//Jumlah data per servo
        for( int i=0; i<totalServo;i++){
                dxl_set_txpacket_parameter(2+3*i+0,id[i]);
                dxl_set_txpacket_parameter(2+3*i+1,dxl_get_lowbyte(sp));
                dxl_set_txpacket_parameter(2+3*i+2,dxl_get_highbyte(sp));
        }

        dxl_set_txpacket_length((l+1)*n+4);
        dxl_txrx_packet();

}


void IK::kirimPacketGerak(int totalServo, int totalStep, int start,  int id[],int data[], bool active, int delay){ //Buat gerakin servo
        if (active){
                int l=2;
                int n=totalServo;
                        dxl_set_txpacket_id(BROADCAST_ID); //Jangan diubah 0xFE
                        dxl_set_txpacket_instruction(INST_SYNC_WRITE);//instruksi 0x83
                        dxl_set_txpacket_parameter(0,P_GOAL_POSITION);//yang mau ditulis 0x1E
                        dxl_set_txpacket_parameter(1,2);//Jumlah data per servo
                                for(int j=start-1; j<totalServo;j++){
                                        dxl_set_txpacket_parameter(2+3*j+0,id[j]);
                                        dxl_set_txpacket_parameter(2+3*j+1,dxl_get_lowbyte(data[j]));
                                        dxl_set_txpacket_parameter(2+3*j+2,dxl_get_highbyte(data[j]));
                                }
                        dxl_set_txpacket_length((l+1)*n+4);
                        dxl_txrx_packet();//jumlah paket yang dikirim 0x18
                        usleep(delay);
                        
        }
}


void IK::kirimPacketGerak(int totalServo, int totalStep, int start,  int id[],int data[][18],bool active,int delay[],int speed[]){ //Buat gerakin servo
        if (active){
                int l=2;
                int n=totalServo;
                for(int i=start-1; i<totalStep;i++){
			IK::kirimPacketSpeed(totalServo, id, speed[i]);
                        dxl_set_txpacket_id(BROADCAST_ID); //Jangan diubah 0xFE
                        dxl_set_txpacket_instruction(INST_SYNC_WRITE);//instruksi 0x83
                        dxl_set_txpacket_parameter(0,P_GOAL_POSITION);//yang mau ditulis 0x1E
                        dxl_set_txpacket_parameter(1,2);//Jumlah data per servo
                                for(int j=0; j<totalServo;j++){
                                        dxl_set_txpacket_parameter(2+3*j+0,id[j]);
                                        dxl_set_txpacket_parameter(2+3*j+1,dxl_get_lowbyte(data[i][j]));
                                        dxl_set_txpacket_parameter(2+3*j+2,dxl_get_highbyte(data[i][j]));
                                }
                        dxl_set_txpacket_length((l+1)*n+4);
                        dxl_txrx_packet();//jumlah paket yang dikirim 0x18
                        usleep(delay[i]);
                }
        }
}


void IK::kirimPacketGerak(int totalServo, int totalStep, int start, int id[],int data[][18], bool active, int delay){ //Buat gerakin servo
        if (active){
		int l = 2;
        	int n = totalServo;
		for(int i=start-1; i<totalStep; i++){
			dxl_set_txpacket_id(BROADCAST_ID); //Jangan diubah 0xFE
        		dxl_set_txpacket_instruction(INST_SYNC_WRITE);//instruksi 0x83
        		dxl_set_txpacket_parameter(0,P_GOAL_POSITION);//yang mau ditulis 0x1E
        		dxl_set_txpacket_parameter(1,2);//Jumlah data per servo
        			for(int j=0; j<totalServo;j++){
                			dxl_set_txpacket_parameter(2+3*j+0,id[j]);
                			dxl_set_txpacket_parameter(2+3*j+1,dxl_get_lowbyte(data[i][j]));
                			dxl_set_txpacket_parameter(2+3*j+2,dxl_get_highbyte(data[i][j]));
				}
        		dxl_set_txpacket_length((l+1)*n+4);
        		dxl_txrx_packet();//jumlah paket yang dikirim 0x18
			usleep(delay);
		}
	}
}
                                //total servo yg digerakin //array ID yang mau digerakin //kecepatan

void IK::setAllSpeed(int speed){
        dxl_write_word(BROADCAST_ID,32,speed);
}




void IK::kirimPacketTorque(int totalServo, int id[], int torque[]){ //Buat ngeset kecepatan servo
        int l=2;
        int n=totalServo;

        dxl_set_txpacket_id(BROADCAST_ID); //Jangan diubah 0xFE
        //dxl_set_txpacket_length(); //jumlah paket yang dikirim 0x18
        dxl_set_txpacket_instruction(INST_SYNC_WRITE);//instruksi 0x83
        dxl_set_txpacket_parameter(0,P_TORQUE);//yang mau ditulis 0x1E
        dxl_set_txpacket_parameter(1,2);//Jumlah data per servo
        for( int i=0; i<totalServo;i++){
        dxl_set_txpacket_parameter(2+3*i+0,id[i]);
              dxl_set_txpacket_parameter(2+3*i+1,dxl_get_lowbyte(torque[i]));
	      dxl_set_txpacket_parameter(2+3*i+2,dxl_get_highbyte(torque[i]));
        }

        dxl_set_txpacket_length((l+1)*n+4);
        dxl_txrx_packet();
	
}


void IK::inverseAndre( Joint::joint_param Kanan, Joint::joint_param Kiri, int speed,int no7, int no8){

	double l3, l5, l4, psi; 
	double sudutLutut, sudutAnkle, sudutAnkle2, sudutSelangkang, sudutPantat, sudut345, sudutPaha, sudutLutut2, sudutAnkle22, sudutAnkle222, sudutSelangkang2, sudutPantat2, sudut3452, sudutPaha2;
	int com7;
	int servo7 = no7;
	int servo8 = no8;
	int com8;
	double nx, ny, nz, sx, sy, sz, ax, ay, az, nx2, ny2, nz2, sx2, sy2, sz2, ax2, ay2, az2;
	double s6, c6, c2, s2, s4, c4,s62, c62, c22, s22, s42, c42;
	double nilaiSelangkang,nilaiPantat, nilaiPaha, nilaiLutut, nilaiAnkle, nilaiAnkle2, nilaiDengkul, nilaiSelangkang2,nilaiPantat2, nilaiPaha2, nilaiLutut2, nilaiAnkle22, nilaiAnkle222, nilaiDengkul2;
	double sudutNgurang=0;
	double tempX = 13;
	double maxOperator = 1.2;
	l3 = 8.4;
	l4 = 8;
	l5 = 0;

 	

	c4 = (pow(Kanan.x + l5,2)+pow(Kanan.y,2)+pow(Kanan.z,2)-pow(l3,2)-pow(l4,2))/(2*l3*l4);
	s4 = 1 * sqrt(1-pow(c4,2));
	sudutLutut = atan2(s4,c4);
/*
	cout<<"C4 :"<<c4<<endl;
	cout<<"S4 :"<<s4<<endl;
	cout<<"sudutLutut :"<<sudutLutut<<endl;
*/
	psi = atan2(s4*l3,c4*l3+l4);
	sudutAnkle = atan2(Kanan.z,1*sqrt(pow(Kanan.x+l5,2)+pow(Kanan.y,2)))-psi;

	sudutAnkle2 = atan2(Kanan.y,-Kanan.x-l5);

	Kanan.sudutX=Kanan.sudutX*3.14/180;
	Kanan.sudutY=Kanan.sudutY*3.14/180;
	Kanan.sudutZ=(Kanan.sudutZ*3.14/180)-(4*Kanan.y*3.14/180);

	nx = cos(Kanan.sudutZ) * cos(Kanan.sudutY);
	ny = sin(Kanan.sudutZ) * cos(Kanan.sudutY);
	nz = sin(Kanan.sudutY);

	sx = (sin(Kanan.sudutZ) * cos(Kanan.sudutX)) + (cos(Kanan.sudutZ) * sin(Kanan.sudutY) * sin(Kanan.sudutX));
	sy = (cos(Kanan.sudutZ) * cos(Kanan.sudutX)) + (sin(Kanan.sudutZ) * sin(Kanan.sudutX) * sin(Kanan.sudutY));
	sz = (cos(Kanan.sudutY) * sin(Kanan.sudutX));

	ax = (sin(Kanan.sudutZ) * sin(Kanan.sudutX)) + (cos(Kanan.sudutZ) * sin(Kanan.sudutY) * cos(Kanan.sudutX));
	ay = (-cos(Kanan.sudutZ)* sin(Kanan.sudutX)) + (sin(Kanan.sudutZ) * sin(Kanan.sudutY) * cos(Kanan.sudutX));
	az = (cos(Kanan.sudutY) * cos(Kanan.sudutX));

	s6 = sin(sudutAnkle2);
	c6 = cos(sudutAnkle2);
	c2 = (s6 * ax) + (c6 * ay);
	s2 = 1*sqrt(1-pow(c2,2));
	sudutPantat = atan2(s2,c2);

	sudutSelangkang = atan2(-s6 * sx - c6 * sy, -s6 * nx - c6 * ny);

	sudut345 = atan2(az,(c6 * ax) - (s6 * ay));
	sudutPaha = sudut345 - sudutLutut - sudutAnkle;


	nilaiSelangkang = 668.502239 + ((sudutSelangkang*180/3.14)/0.29);
	nilaiPantat = 196.497761 +  ((sudutPantat*180/3.14)/0.29);
	nilaiPaha = 822.50224 - ((sudutPaha*180/3.14)/0.29);
	nilaiLutut = 512 - ((sudutLutut*180/3.14)/0.29);
	nilaiAnkle  = 512 + ((sudutAnkle*180/3.14)/0.29);
	nilaiAnkle2 =517 -  ((sudutAnkle2*180/3.14)/0.29);

	c42 = (pow(Kiri.x + l5,2)+pow(Kiri.y,2)+pow(Kiri.z,2)-pow(l3,2)-pow(l4,2))/(2*l3*l4);
        s42 = 1 * sqrt(1-pow(c42,2));
        sudutLutut2 = atan2(s42,c42);

        psi = atan2(s42*l3,c42*l3+l4);
        sudutAnkle22 = atan2(Kiri.z,1*sqrt(pow(Kiri.x+l5,2)+pow(Kiri.y,2)))-psi;
        sudutAnkle222 = atan2(Kiri.y,-Kiri.x-l5);

        Kiri.sudutX=Kiri.sudutX*3.14/180;
        Kiri.sudutY=Kiri.sudutY*3.14/180;
        Kiri.sudutZ=(Kiri.sudutZ*3.14/180)-(4*Kiri.y*3.14/180); // y nya ga tau kanan atau kiri

        nx2 = cos(Kiri.sudutZ) * cos(Kiri.sudutY);
        ny2 = sin(Kiri.sudutZ) * cos(Kiri.sudutY);
        nz2 = sin(Kiri.sudutY);

        sx2 = (sin(Kiri.sudutZ) * cos(Kiri.sudutX)) + (cos(Kiri.sudutZ) * sin(Kiri.sudutY) * sin(Kiri.sudutX));
        sy2 = (cos(Kiri.sudutZ) * cos(Kiri.sudutX)) + (sin(Kiri.sudutZ) * sin(Kiri.sudutX) * sin(Kiri.sudutY));
        sz2 = (cos(Kiri.sudutY) * sin(Kiri.sudutX));

        ax2 = (sin(Kiri.sudutZ) * sin(Kiri.sudutX)) + (cos(Kiri.sudutZ) * sin(Kiri.sudutY) * cos(Kiri.sudutX));
        ay2 = (-cos(Kiri.sudutZ)* sin(Kiri.sudutX)) + (sin(Kiri.sudutZ) * sin(Kiri.sudutY) * cos(Kiri.sudutX));
        az2 = (cos(Kiri.sudutY) * cos(Kiri.sudutX));

        s62 = sin(sudutAnkle2);
        c62 = cos(sudutAnkle2);
        c22 = (s62 * ax2) + (c62 * ay2);
        s22 = 1*sqrt(1-pow(c22,2));
        sudutPantat2 = atan2(s22,c22);

        sudutSelangkang2 = atan2(-s62 * sx2 - c62 * sy2, -s62 * nx2 - c62 * ny2);

        sudut3452 = atan2(az2,(c62 * ax2) - (s62 * ay2));
        sudutPaha2 = sudut3452 - sudutLutut2 - sudutAnkle22;
	// nilaiSelangkang2 = 668.502239 + ((sudutSelangkang2*180/3.14)/0.29);

	nilaiSelangkang2 = 365.497761 - ((sudutSelangkang2*180/3.14)/0.29);
        nilaiPantat2 = 1024-(196.497761 +  ((sudutPantat2*180/3.14)/0.29));
        nilaiPaha2 = 201.497761 + ((sudutPaha2*180/3.14)/0.29);
       	//nilaiPantat2 =512 -((sudutPantat2*180/3.14)/0.29);
  

	 nilaiLutut2 = 512 +((sudutLutut2*180/3.14)/0.29);// kalo pake ms 0.29 diganti jadi 0.088 
        nilaiAnkle22  = 512 - ((sudutAnkle22*180/3.14)/0.29);
        nilaiAnkle222 = 517 -  ((sudutAnkle222*180/3.14)/0.29);

	int id[12] = {7,8,9,10,11,12,13,14,15,16,17,18};
        int start=0;
        int totalServo=20;
        int l=2;
        int q=2;
	int n=totalServo;
	int tq=1023;

        int pos[12]= {servo7,/*nilaiSelangkang,*/servo8/*nilaiSelangkang2*/,(int) nilaiPantat,(int)nilaiPantat2,(int)nilaiPaha,(int)nilaiPaha2,(int)nilaiLutut,(int)nilaiLutut2,(int)nilaiAnkle,(int)nilaiAnkle22,(int)nilaiAnkle2,(int)nilaiAnkle222};
//                       nilaiSelangkang,1023-nilaiSelangkang, nilaiPantat,1023-nilaiPantat,nilaiPaha,1023-nilaiPaha,nilaiLutut,4095-nilaiLutut,nilaiAnkle,4095-nilaiAnkle,nilaiAnkle2,1023-nilaiAnkle2};

						     //7  8  9  10 11 12 13 14 15 16 17 18
	int tor[12]={1023,1023,1023,1023,1023,1023}/*q,tq,tq,tq,tq,tq,tq,tq,tq,tq,tq,tq}*/;
/*	 dxl_set_txpacket_id(BROADCAST_ID); //Jangan diubah 0xFE
        dxl_set_txpacket_length(); //jumlah paket yang dikirim 0x18
        dxl_set_txpacket_instruction(INST_SYNC_WRITE);//instruksi 0x83
        dxl_set_txpacket_parameter(0,P_TORQUE);//yang mau ditulis 0x1E
        dxl_set_txpacket_parameter(1,2);//Jumlah data per servo
        	for( int i=0; i<totalServo;i++){
	 	dxl_set_txpacket_parameter(2+3*i+0,id[i]);
                dxl_set_txpacket_parameter(2+3*i+1,dxl_get_lowbyte(tor[i]));
        	 dxl_set_txpacket_parameter(2+3*i+2,dxl_get_highbyte(tor[i]));
	}

        dxl_set_txpacket_length((q+1)*n+4);
        dxl_txrx_packet();
	com7=4;
	com8=4;*/

       setAllSpeed(speed);
	// dxl_write_word(BROADCAST_ID, P_GOAL_SPEED_L,150);
                dxl_set_txpacket_id(BROADCAST_ID); //Jangan diubah 0xFE
                        dxl_set_txpacket_instruction(INST_SYNC_WRITE);//instruksi 0x83
                        dxl_set_txpacket_parameter(0,P_GOAL_POSITION);//yang mau ditulis 0x1E
                        dxl_set_txpacket_parameter(1,2);//Jumlah data per servo
                                for(int j=start; j<=totalServo;j++){
                                        dxl_set_txpacket_parameter(2+3*j+0,id[j]);
                                        dxl_set_txpacket_parameter(2+3*j+1,dxl_get_lowbyte(pos[j]));
                                        dxl_set_txpacket_parameter(2+3*j+2,dxl_get_highbyte(pos[j]));
					
                                }
                        dxl_set_txpacket_length((l+1)*n+4);
                        dxl_txrx_packet();//jumlah paket yang dikirim 0x18
                       // usleep(0);//100 speed 500
	
}
void IK::inverseAndre( Joint::joint_param Kanan, Joint::joint_param Kiri, int speed,int no7, int no8, int x_head, int y_head){

	double l3, l5, l4, psi; 
	double sudutLutut, sudutAnkle, sudutAnkle2, sudutSelangkang, sudutPantat, sudut345, sudutPaha, sudutLutut2, sudutAnkle22, sudutAnkle222, sudutSelangkang2, sudutPantat2, sudut3452, sudutPaha2;
	int com7;
	int servo7 = no7;
	int servo8 = no8;
	int com8;
	double nx, ny, nz, sx, sy, sz, ax, ay, az, nx2, ny2, nz2, sx2, sy2, sz2, ax2, ay2, az2;
	double s6, c6, c2, s2, s4, c4,s62, c62, c22, s22, s42, c42;
	double nilaiSelangkang,nilaiPantat, nilaiPaha, nilaiLutut, nilaiAnkle, nilaiAnkle2, nilaiDengkul, nilaiSelangkang2,nilaiPantat2, nilaiPaha2, nilaiLutut2, nilaiAnkle22, nilaiAnkle222, nilaiDengkul2;
	double sudutNgurang=0;
	double tempX = 13;
	double maxOperator = 1.2;
	l3 = 8.4;
	l4 = 8;
	l5 = 0;

 	

	c4 = (pow(Kanan.x + l5,2)+pow(Kanan.y,2)+pow(Kanan.z,2)-pow(l3,2)-pow(l4,2))/(2*l3*l4);
	s4 = 1 * sqrt(1-pow(c4,2));
	sudutLutut = atan2(s4,c4);
/*
	cout<<"C4 :"<<c4<<endl;
	cout<<"S4 :"<<s4<<endl;
	cout<<"sudutLutut :"<<sudutLutut<<endl;
*/
	psi = atan2(s4*l3,c4*l3+l4);
	sudutAnkle = atan2(Kanan.z,1*sqrt(pow(Kanan.x+l5,2)+pow(Kanan.y,2)))-psi;

	sudutAnkle2 = atan2(Kanan.y,-Kanan.x-l5);

	Kanan.sudutX=Kanan.sudutX*3.14/180;
	Kanan.sudutY=Kanan.sudutY*3.14/180;
	Kanan.sudutZ=(Kanan.sudutZ*3.14/180)-(4*Kanan.y*3.14/180);

	nx = cos(Kanan.sudutZ) * cos(Kanan.sudutY);
	ny = sin(Kanan.sudutZ) * cos(Kanan.sudutY);
	nz = sin(Kanan.sudutY);

	sx = (sin(Kanan.sudutZ) * cos(Kanan.sudutX)) + (cos(Kanan.sudutZ) * sin(Kanan.sudutY) * sin(Kanan.sudutX));
	sy = (cos(Kanan.sudutZ) * cos(Kanan.sudutX)) + (sin(Kanan.sudutZ) * sin(Kanan.sudutX) * sin(Kanan.sudutY));
	sz = (cos(Kanan.sudutY) * sin(Kanan.sudutX));

	ax = (sin(Kanan.sudutZ) * sin(Kanan.sudutX)) + (cos(Kanan.sudutZ) * sin(Kanan.sudutY) * cos(Kanan.sudutX));
	ay = (-cos(Kanan.sudutZ)* sin(Kanan.sudutX)) + (sin(Kanan.sudutZ) * sin(Kanan.sudutY) * cos(Kanan.sudutX));
	az = (cos(Kanan.sudutY) * cos(Kanan.sudutX));

	s6 = sin(sudutAnkle2);
	c6 = cos(sudutAnkle2);
	c2 = (s6 * ax) + (c6 * ay);
	s2 = 1*sqrt(1-pow(c2,2));
	sudutPantat = atan2(s2,c2);

	sudutSelangkang = atan2(-s6 * sx - c6 * sy, -s6 * nx - c6 * ny);

	sudut345 = atan2(az,(c6 * ax) - (s6 * ay));
	sudutPaha = sudut345 - sudutLutut - sudutAnkle;


	nilaiSelangkang = 668.502239 + ((sudutSelangkang*180/3.14)/0.29);
	nilaiPantat = 196.497761 +  ((sudutPantat*180/3.14)/0.29);
	nilaiPaha = 822.50224 - ((sudutPaha*180/3.14)/0.29);
	nilaiLutut = 512 - ((sudutLutut*180/3.14)/0.29);
	nilaiAnkle  = 512 + ((sudutAnkle*180/3.14)/0.29);
	nilaiAnkle2 =517 -  ((sudutAnkle2*180/3.14)/0.29);

	c42 = (pow(Kiri.x + l5,2)+pow(Kiri.y,2)+pow(Kiri.z,2)-pow(l3,2)-pow(l4,2))/(2*l3*l4);
        s42 = 1 * sqrt(1-pow(c42,2));
        sudutLutut2 = atan2(s42,c42);

        psi = atan2(s42*l3,c42*l3+l4);
        sudutAnkle22 = atan2(Kiri.z,1*sqrt(pow(Kiri.x+l5,2)+pow(Kiri.y,2)))-psi;
        sudutAnkle222 = atan2(Kiri.y,-Kiri.x-l5);

        Kiri.sudutX=Kiri.sudutX*3.14/180;
        Kiri.sudutY=Kiri.sudutY*3.14/180;
        Kiri.sudutZ=(Kiri.sudutZ*3.14/180)-(4*Kiri.y*3.14/180); // y nya ga tau kanan atau kiri

        nx2 = cos(Kiri.sudutZ) * cos(Kiri.sudutY);
        ny2 = sin(Kiri.sudutZ) * cos(Kiri.sudutY);
        nz2 = sin(Kiri.sudutY);

        sx2 = (sin(Kiri.sudutZ) * cos(Kiri.sudutX)) + (cos(Kiri.sudutZ) * sin(Kiri.sudutY) * sin(Kiri.sudutX));
        sy2 = (cos(Kiri.sudutZ) * cos(Kiri.sudutX)) + (sin(Kiri.sudutZ) * sin(Kiri.sudutX) * sin(Kiri.sudutY));
        sz2 = (cos(Kiri.sudutY) * sin(Kiri.sudutX));

        ax2 = (sin(Kiri.sudutZ) * sin(Kiri.sudutX)) + (cos(Kiri.sudutZ) * sin(Kiri.sudutY) * cos(Kiri.sudutX));
        ay2 = (-cos(Kiri.sudutZ)* sin(Kiri.sudutX)) + (sin(Kiri.sudutZ) * sin(Kiri.sudutY) * cos(Kiri.sudutX));
        az2 = (cos(Kiri.sudutY) * cos(Kiri.sudutX));

        s62 = sin(sudutAnkle2);
        c62 = cos(sudutAnkle2);
        c22 = (s62 * ax2) + (c62 * ay2);
        s22 = 1*sqrt(1-pow(c22,2));
        sudutPantat2 = atan2(s22,c22);

        sudutSelangkang2 = atan2(-s62 * sx2 - c62 * sy2, -s62 * nx2 - c62 * ny2);

        sudut3452 = atan2(az2,(c62 * ax2) - (s62 * ay2));
        sudutPaha2 = sudut3452 - sudutLutut2 - sudutAnkle22;
	// nilaiSelangkang2 = 668.502239 + ((sudutSelangkang2*180/3.14)/0.29);

	nilaiSelangkang2 = 365.497761 - ((sudutSelangkang2*180/3.14)/0.29);
        nilaiPantat2 = 1024-(196.497761 +  ((sudutPantat2*180/3.14)/0.29));
        nilaiPaha2 = 201.497761 + ((sudutPaha2*180/3.14)/0.29);
       	//nilaiPantat2 =512 -((sudutPantat2*180/3.14)/0.29);
  

	 nilaiLutut2 = 512 +((sudutLutut2*180/3.14)/0.29);// kalo pake ms 0.29 diganti jadi 0.088 
        nilaiAnkle22  = 512 - ((sudutAnkle22*180/3.14)/0.29);
        nilaiAnkle222 = 517 -  ((sudutAnkle222*180/3.14)/0.29);

	int id[14] = {7,8,9,10,11,12,13,14,15,16,17,18,20,19};
        int start=0;
        int totalServo=20;
        int l=2;
        int q=2;
	int n=totalServo;
	int tq=1023;

        int pos[14]= {servo7,/*nilaiSelangkang,*/servo8/*nilaiSelangkang2*/,(int) nilaiPantat,(int)nilaiPantat2,(int)nilaiPaha,(int)nilaiPaha2,(int)nilaiLutut,(int)nilaiLutut2,(int)nilaiAnkle,(int)nilaiAnkle22,(int)nilaiAnkle2,(int)nilaiAnkle222,x_head,y_head};
//                       nilaiSelangkang,1023-nilaiSelangkang, nilaiPantat,1023-nilaiPantat,nilaiPaha,1023-nilaiPaha,nilaiLutut,4095-nilaiLutut,nilaiAnkle,4095-nilaiAnkle,nilaiAnkle2,1023-nilaiAnkle2};

						     //7  8  9  10 11 12 13 14 15 16 17 18
	int tor[12]={1023,1023,1023,1023,1023,1023}/*q,tq,tq,tq,tq,tq,tq,tq,tq,tq,tq,tq}*/;
/*	 dxl_set_txpacket_id(BROADCAST_ID); //Jangan diubah 0xFE
        dxl_set_txpacket_length(); //jumlah paket yang dikirim 0x18
        dxl_set_txpacket_instruction(INST_SYNC_WRITE);//instruksi 0x83
        dxl_set_txpacket_parameter(0,P_TORQUE);//yang mau ditulis 0x1E
        dxl_set_txpacket_parameter(1,2);//Jumlah data per servo
        	for( int i=0; i<totalServo;i++){
	 	dxl_set_txpacket_parameter(2+3*i+0,id[i]);
                dxl_set_txpacket_parameter(2+3*i+1,dxl_get_lowbyte(tor[i]));
        	 dxl_set_txpacket_parameter(2+3*i+2,dxl_get_highbyte(tor[i]));
	}

        dxl_set_txpacket_length((q+1)*n+4);
        dxl_txrx_packet();
	com7=4;
	com8=4;*/

       setAllSpeed(speed);
	// dxl_write_word(BROADCAST_ID, P_GOAL_SPEED_L,150);
                dxl_set_txpacket_id(BROADCAST_ID); //Jangan diubah 0xFE
                        dxl_set_txpacket_instruction(INST_SYNC_WRITE);//instruksi 0x83
                        dxl_set_txpacket_parameter(0,P_GOAL_POSITION);//yang mau ditulis 0x1E
                        dxl_set_txpacket_parameter(1,2);//Jumlah data per servo
                                for(int j=start; j<=totalServo;j++){
                                        dxl_set_txpacket_parameter(2+3*j+0,id[j]);
                                        dxl_set_txpacket_parameter(2+3*j+1,dxl_get_lowbyte(pos[j]));
                                        dxl_set_txpacket_parameter(2+3*j+2,dxl_get_highbyte(pos[j]));
					
                                }
                        dxl_set_txpacket_length((l+1)*n+4);
                        dxl_txrx_packet();//jumlah paket yang dikirim 0x18
                       // usleep(0);//100 speed 500
	
}
void IK::inverseAndre( Joint::joint_param Kanan, Joint::joint_param Kiri, int speed,int no7, int no8, int tk1,int tk2,int tk3,int tk4,int tk5,int tk6, int x_head, int y_head){

	double l3, l5, l4, psi; 
	double sudutLutut, sudutAnkle, sudutAnkle2, sudutSelangkang, sudutPantat, sudut345, sudutPaha, sudutLutut2, sudutAnkle22, sudutAnkle222, sudutSelangkang2, sudutPantat2, sudut3452, sudutPaha2;
	int com7;
	int servo7 = no7;
	int servo8 = no8;
	int com8;
	double nx, ny, nz, sx, sy, sz, ax, ay, az, nx2, ny2, nz2, sx2, sy2, sz2, ax2, ay2, az2;
	double s6, c6, c2, s2, s4, c4,s62, c62, c22, s22, s42, c42;
	double nilaiSelangkang,nilaiPantat, nilaiPaha, nilaiLutut, nilaiAnkle, nilaiAnkle2, nilaiDengkul, nilaiSelangkang2,nilaiPantat2, nilaiPaha2, nilaiLutut2, nilaiAnkle22, nilaiAnkle222, nilaiDengkul2;
	double sudutNgurang=0;
	double tempX = 13;
	double maxOperator = 1.2;
	l3 = 8.4;
	l4 = 8;
	l5 = 0;

 	

	c4 = (pow(Kanan.x + l5,2)+pow(Kanan.y,2)+pow(Kanan.z,2)-pow(l3,2)-pow(l4,2))/(2*l3*l4);
	s4 = 1 * sqrt(1-pow(c4,2));
	sudutLutut = atan2(s4,c4);
/*
	cout<<"C4 :"<<c4<<endl;
	cout<<"S4 :"<<s4<<endl;
	cout<<"sudutLutut :"<<sudutLutut<<endl;
*/
	psi = atan2(s4*l3,c4*l3+l4);
	sudutAnkle = atan2(Kanan.z,1*sqrt(pow(Kanan.x+l5,2)+pow(Kanan.y,2)))-psi;

	sudutAnkle2 = atan2(Kanan.y,-Kanan.x-l5);

	Kanan.sudutX=Kanan.sudutX*3.14/180;
	Kanan.sudutY=Kanan.sudutY*3.14/180;
	Kanan.sudutZ=(Kanan.sudutZ*3.14/180)-(4*Kanan.y*3.14/180);

	nx = cos(Kanan.sudutZ) * cos(Kanan.sudutY);
	ny = sin(Kanan.sudutZ) * cos(Kanan.sudutY);
	nz = sin(Kanan.sudutY);

	sx = (sin(Kanan.sudutZ) * cos(Kanan.sudutX)) + (cos(Kanan.sudutZ) * sin(Kanan.sudutY) * sin(Kanan.sudutX));
	sy = (cos(Kanan.sudutZ) * cos(Kanan.sudutX)) + (sin(Kanan.sudutZ) * sin(Kanan.sudutX) * sin(Kanan.sudutY));
	sz = (cos(Kanan.sudutY) * sin(Kanan.sudutX));

	ax = (sin(Kanan.sudutZ) * sin(Kanan.sudutX)) + (cos(Kanan.sudutZ) * sin(Kanan.sudutY) * cos(Kanan.sudutX));
	ay = (-cos(Kanan.sudutZ)* sin(Kanan.sudutX)) + (sin(Kanan.sudutZ) * sin(Kanan.sudutY) * cos(Kanan.sudutX));
	az = (cos(Kanan.sudutY) * cos(Kanan.sudutX));

	s6 = sin(sudutAnkle2);
	c6 = cos(sudutAnkle2);
	c2 = (s6 * ax) + (c6 * ay);
	s2 = 1*sqrt(1-pow(c2,2));
	sudutPantat = atan2(s2,c2);

	sudutSelangkang = atan2(-s6 * sx - c6 * sy, -s6 * nx - c6 * ny);

	sudut345 = atan2(az,(c6 * ax) - (s6 * ay));
	sudutPaha = sudut345 - sudutLutut - sudutAnkle;


	nilaiSelangkang = 668.502239 + ((sudutSelangkang*180/3.14)/0.29);
	nilaiPantat = 196.497761 +  ((sudutPantat*180/3.14)/0.29);
	nilaiPaha = 822.50224 - ((sudutPaha*180/3.14)/0.29);
	nilaiLutut = 512 - ((sudutLutut*180/3.14)/0.29);
	nilaiAnkle  = 512 + ((sudutAnkle*180/3.14)/0.29);
	nilaiAnkle2 =517 -  ((sudutAnkle2*180/3.14)/0.29);

	c42 = (pow(Kiri.x + l5,2)+pow(Kiri.y,2)+pow(Kiri.z,2)-pow(l3,2)-pow(l4,2))/(2*l3*l4);
        s42 = 1 * sqrt(1-pow(c42,2));
        sudutLutut2 = atan2(s42,c42);

        psi = atan2(s42*l3,c42*l3+l4);
        sudutAnkle22 = atan2(Kiri.z,1*sqrt(pow(Kiri.x+l5,2)+pow(Kiri.y,2)))-psi;
        sudutAnkle222 = atan2(Kiri.y,-Kiri.x-l5);

        Kiri.sudutX=Kiri.sudutX*3.14/180;
        Kiri.sudutY=Kiri.sudutY*3.14/180;
        Kiri.sudutZ=(Kiri.sudutZ*3.14/180)-(4*Kiri.y*3.14/180); // y nya ga tau kanan atau kiri

        nx2 = cos(Kiri.sudutZ) * cos(Kiri.sudutY);
        ny2 = sin(Kiri.sudutZ) * cos(Kiri.sudutY);
        nz2 = sin(Kiri.sudutY);

        sx2 = (sin(Kiri.sudutZ) * cos(Kiri.sudutX)) + (cos(Kiri.sudutZ) * sin(Kiri.sudutY) * sin(Kiri.sudutX));
        sy2 = (cos(Kiri.sudutZ) * cos(Kiri.sudutX)) + (sin(Kiri.sudutZ) * sin(Kiri.sudutX) * sin(Kiri.sudutY));
        sz2 = (cos(Kiri.sudutY) * sin(Kiri.sudutX));

        ax2 = (sin(Kiri.sudutZ) * sin(Kiri.sudutX)) + (cos(Kiri.sudutZ) * sin(Kiri.sudutY) * cos(Kiri.sudutX));
        ay2 = (-cos(Kiri.sudutZ)* sin(Kiri.sudutX)) + (sin(Kiri.sudutZ) * sin(Kiri.sudutY) * cos(Kiri.sudutX));
        az2 = (cos(Kiri.sudutY) * cos(Kiri.sudutX));

        s62 = sin(sudutAnkle2);
        c62 = cos(sudutAnkle2);
        c22 = (s62 * ax2) + (c62 * ay2);
        s22 = 1*sqrt(1-pow(c22,2));
        sudutPantat2 = atan2(s22,c22);

        sudutSelangkang2 = atan2(-s62 * sx2 - c62 * sy2, -s62 * nx2 - c62 * ny2);

        sudut3452 = atan2(az2,(c62 * ax2) - (s62 * ay2));
        sudutPaha2 = sudut3452 - sudutLutut2 - sudutAnkle22;
	// nilaiSelangkang2 = 668.502239 + ((sudutSelangkang2*180/3.14)/0.29);

	nilaiSelangkang2 = 365.497761 - ((sudutSelangkang2*180/3.14)/0.29);
        nilaiPantat2 = 1024-(196.497761 +  ((sudutPantat2*180/3.14)/0.29));
        nilaiPaha2 = 201.497761 + ((sudutPaha2*180/3.14)/0.29);
       	//nilaiPantat2 =512 -((sudutPantat2*180/3.14)/0.29);
  

	 nilaiLutut2 = 512 +((sudutLutut2*180/3.14)/0.29);// kalo pake ms 0.29 diganti jadi 0.088 
        nilaiAnkle22  = 512 - ((sudutAnkle22*180/3.14)/0.29);
        nilaiAnkle222 = 517 -  ((sudutAnkle222*180/3.14)/0.29);

	int id[20] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,20,19};
        int start=0;
        int totalServo=20;
        int l=2;
        int q=2;
	int n=totalServo;
	int tq=1023;

        int pos[20]= {tk1,tk2,tk3,tk4,tk5,tk6,servo7,/*nilaiSelangkang,*/servo8/*nilaiSelangkang2*/,(int) nilaiPantat,(int)nilaiPantat2,(int)nilaiPaha,(int)nilaiPaha2,(int)nilaiLutut,(int)nilaiLutut2,(int)nilaiAnkle,(int)nilaiAnkle22,(int)nilaiAnkle2,(int)nilaiAnkle222,x_head,y_head};
//                       nilaiSelangkang,1023-nilaiSelangkang, nilaiPantat,1023-nilaiPantat,nilaiPaha,1023-nilaiPaha,nilaiLutut,4095-nilaiLutut,nilaiAnkle,4095-nilaiAnkle,nilaiAnkle2,1023-nilaiAnkle2};

						     //7  8  9  10 11 12 13 14 15 16 17 18
	int tor[12]={1023,1023,1023,1023,1023,1023}/*q,tq,tq,tq,tq,tq,tq,tq,tq,tq,tq,tq}*/;
/*	 dxl_set_txpacket_id(BROADCAST_ID); //Jangan diubah 0xFE
        dxl_set_txpacket_length(); //jumlah paket yang dikirim 0x18
        dxl_set_txpacket_instruction(INST_SYNC_WRITE);//instruksi 0x83
        dxl_set_txpacket_parameter(0,P_TORQUE);//yang mau ditulis 0x1E
        dxl_set_txpacket_parameter(1,2);//Jumlah data per servo
        	for( int i=0; i<totalServo;i++){
	 	dxl_set_txpacket_parameter(2+3*i+0,id[i]);
                dxl_set_txpacket_parameter(2+3*i+1,dxl_get_lowbyte(tor[i]));
        	 dxl_set_txpacket_parameter(2+3*i+2,dxl_get_highbyte(tor[i]));
	}

        dxl_set_txpacket_length((q+1)*n+4);
        dxl_txrx_packet();
	com7=4;
	com8=4;*/

       setAllSpeed(speed);
	// dxl_write_word(BROADCAST_ID, P_GOAL_SPEED_L,150);
                dxl_set_txpacket_id(BROADCAST_ID); //Jangan diubah 0xFE
                        dxl_set_txpacket_instruction(INST_SYNC_WRITE);//instruksi 0x83
                        dxl_set_txpacket_parameter(0,P_GOAL_POSITION);//yang mau ditulis 0x1E
                        dxl_set_txpacket_parameter(1,2);//Jumlah data per servo
                                for(int j=start; j<=totalServo;j++){
                                        dxl_set_txpacket_parameter(2+3*j+0,id[j]);
                                        dxl_set_txpacket_parameter(2+3*j+1,dxl_get_lowbyte(pos[j]));
                                        dxl_set_txpacket_parameter(2+3*j+2,dxl_get_highbyte(pos[j]));
					
                                }
                        dxl_set_txpacket_length((l+1)*n+4);
                        dxl_txrx_packet();//jumlah paket yang dikirim 0x18
                       // usleep(0);//100 speed 500
	
}


void IK::kirimPacketTorque(int totalServo, int id[], int torque){ //Buat ngeset kecepatan servo
        int l=1;
        int n=totalServo;

        dxl_set_txpacket_id(BROADCAST_ID); //Jangan diubah 0xFE
        //dxl_set_txpacket_length(); //jumlah paket yang dikirim 0x18
        dxl_set_txpacket_instruction(INST_SYNC_WRITE);//instruksi 0x83
        dxl_set_txpacket_parameter(0,P_TORQUE);//yang mau ditulis 0x1E
        dxl_set_txpacket_parameter(1,1);//Jumlah data per servo
        for( int i=0; i<totalServo;i++){
        dxl_set_txpacket_parameter(2+3*i+0,id[i]);
                dxl_set_txpacket_parameter(2+3*i+1,torque);
        }

        dxl_set_txpacket_length((l+1)*n+4);
        dxl_txrx_packet();

}



void IK::PrintErrorCode() {
	if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
		printf("Input voltage error!\n");
	if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
		printf("Angle limit error!\n");
	if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
		printf("Overheat error!\n");
	if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
		printf("Out of range error!\n");
	if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
		printf("Checksum error!\n");
	if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
		printf("Overload error!\n");
	if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
		printf("Instruction code error!\n");
}

void IK::PrintCommStatus(int CommStatus) {
	switch(CommStatus)
	{
	case COMM_TXFAIL:
		printf("COMM_TXFAIL: Failed transmit instruction packet!\n");
		break;
	case COMM_TXERROR:
		printf("COMM_TXERROR: Incorrect instruction packet!\n");
		break;
	case COMM_RXFAIL:
		printf("COMM_RXFAIL: Failed get status packet from device!\n");
		break;
	case COMM_RXWAITING:
		printf("COMM_RXWAITING: Now recieving status packet!\n");
		break;
	case COMM_RXTIMEOUT:
		printf("COMM_RXTIMEOUT: There is no status packet!\n");
		break;
	case COMM_RXCORRUPT:
		printf("COMM_RXCORRUPT: Incorrect status packet!\n");
		break;
	default:
		printf("This is unknown error code!\n");
		break;
	}
}


void IK::setAllTorque(int torque){
	dxl_write_word(BROADCAST_ID,34,torque);
}


#endif
