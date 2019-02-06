#include "/home/pi/systemTest/library/allLib.h"
#include "library/sensor/LSM303.h"
#include "library/sensor/L3G.h"
#include "library/sensor/sensor.c"
#include "library/dynamixel/syncWrite.cpp"
#include <pthread.h>
#include "library/movement/moveArray.h"
#include "library/camera/Camera.cpp"
#include "array.h"
#include "gerakan.cpp"
//#include "library/wifi/wifi.cpp"

pthread_mutex_t lock,lock2,lock3;

int commandSensor=0;
//int commandMotor=0;
//int command=0;
int commandCamera=0;

//pthread_t thread1, thread2;

void *reading(void * arg){
	//pthread_join(thread2,NULL);
	//commandSensor=1;
	pthread_mutex_lock(&lock);
	if(command==0){
		//startWifi();
		readWifi();
		commandSensor=1;
		string status=bacaACC(Pacc_raw);
		//pthread_t thread2=(pthread_t)arg;
		cout<<"Status: "<<status<<endl;
		//moving=6;
		char *a=new char[status.size()+1];
		a[status.size()]=0;
		memcpy(a,status.c_str(),status.size());
		//cout<<"masuk1"<<endl;	
		if(strcmp(a,"Depan")==0||strcmp(a,"Kiri")==0||strcmp(a,"Kanan")==0){
			//setAllTorque(0);
			commandSearch=2;
			sleep(5);
			destroyWindow(windowName);
			//setAllTorque(1023);
                        dxl_write_word(19,30,512);
                        dxl_write_word(20,30,512);
			moving=0;
			//cap.release();
			cout<<"masuk3"<<endl;
			command=1;
			bangun=1;
			while(command==1){
				//setAllTorque(0);
				sleep(3);
				//setAllTorque(200);
				dxl_write_word(19,24,200);
				dxl_write_word(20,24,200);
	                        dxl_write_word(19,30,512);
	                        dxl_write_word(20,30,512);
				sleep(3);
				kasus(18);
				//pthread_mutex_lock(&lock);
				cout<<"berdiri tunggu 5 detik"<<endl;
				//kasus(18);
				//sleep(50);
				cout<<"lanjutkan"<<" Command: "<<command<<endl;
				status=bacaACC(Pacc_raw);
				a[status.size()]=0;
	       			memcpy(a,status.c_str(),status.size());
			
				if(strcmp(a,"Depan")!=0){kasus(28);pengecekan=0;command=0;commandMotor=0;commandSearch=0;}
				//pthread_mutex_unlock(&lock);	
			}
		}

		if(strcmp(a,"Belakang")==0){
			//setAllTorque(0);
			commandSearch=2;
            		//sleep(5);
			destroyWindow(windowName);
                        //setAllTorque(200);
			//dxl_write_word(19,30,512);
                        //dxl_write_word(20,30,512);
			//cap.release();
            		moving=0;
			cout<<"masuk3"<<endl;
            		command=1;
			bangun=1;
            		while(command==1){
				//setAllTorque(0);
				sleep(3);
				//setAllTorque(200);
                        	dxl_write_word(19,24,200);
                                dxl_write_word(20,24,200);
				dxl_write_word(19,30,512);
                        	dxl_write_word(20,30,512);
				sleep(3);
				kasus(19);
				//pthread_mutex_lock(&lock);
            			cout<<"berdiri tunggu 5 detik"<<endl;
				//kasus(18);
				//sleep(50);
            			cout<<"lanjutkan"<<" Command: "<<command<<endl;
            			status=bacaACC(Pacc_raw);
            			a[status.size()]=0;
            			memcpy(a,status.c_str(),status.size());
            			if(strcmp(a,"Belakang")!=0){kasus(28);pengecekan=0;command=0;commandMotor=0;commandSearch=0;}
			//pthread_mutex_unlock(&lock);
            }
        }

		cout<<"masuk2"<<endl;
		sleep(1.5);
		//pthread_mutex_unlock(&lock);
	}
	commandSensor=0;
	//command=0;
        //set_siap_jalan(0,0,0,0,-10,-10,0,0,30,30,0,0);
	//pid(175);
        //set_siap_jalan(0,0,0,0,10,10,0,0,-30,-30,0,0);
	pthread_mutex_unlock(&lock);
}

//int id[2]={1,2};
//int id2[18]={1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18};
//int pos1[2][2]={{512,512},{0,1000}};
//int pos[2]={512,512};

void *jalan(void * arg){
	if (command==0){
		pthread_mutex_lock(&lock2);
		commandMotor=1;
		//int perintah;
		//cout<<"command: ";
		//cin>>perintah;
		//cout<<"jalan "<<perintah<<endl;
		//if(commandCamera==0){
		//commandCamera=1;
		//kasus(perintah);
		search();
		//kirimPacketTorque(18,id2,1);
		//kirimPacketSpeed(18,id2,40);
		//kirimPacketGerak(18,6,1,id2,gerakAndre2);
		//commandMotor=0;
		sleep(1);
		//}
		//commandMotor=0;
		//pthread_mutex_unlock(&lock2);
	}
	//pthread_exit(&thread2);
	bangun=1;
	commandMotor=0;
	pthread_mutex_unlock(&lock2);
}


void *list_thread(void *threadid){
	long tid;
	tid=(long)threadid;
	switch(tid){
		case 0: reading(NULL); break;
		case 1: jalan(NULL); break;
	}
	pthread_exit(NULL);
}

void *test1(void *arg){
	cout<<"1"<<endl;
}
void *test2(void *arg){
	cout<<"2"<<endl;
}


int main(){
    string status;
    enableIMU();
    startWifi();
    if(dxl_initialize(0,1)==0){
        cout<<"Error Connecting Servo"<<endl;
    }


    else{
        cout<<"Connect Servo Berhasil"<<endl;
		//kasus(28);
		//pthread_t thread1,thread2;
        while(1){
		
		while(WIFI!=1){
			readWifi();
			if(WIFI==2||WIFI==3){
				kasus(1);
				//slkirimPacketGerak(18,7,1,id,FMR2,1,40000);eep(1);
				readWifi();
			}
		}
			
			//string status=bacaACC(Pacc_raw);
			//cout<<status<<endl;
			//sleep(1);
			//jalan(NULL);
            pthread_t thread1,thread2;
			//if(commandSensor==0){
			//cout<<"Masuk Sensor"<<endl;
			//pthread_create(&thread1, NULL, reading,NULL);
		//}
		if(WIFI==1&&commandMotor==0&&command==0){
			cout<<"Masuk jalan"<<endl;
			pthread_create(&thread2, NULL, jalan, NULL);
			//jalan(NULL);
		}
		readWifi();

            if(WIFI==1&&commandSensor==0){
                cout<<"Masuk Sensor"<<endl;
                //pthread_create(&thread1, NULL, reading,NULL);
              	reading(NULL);
	    }
		readWifi();
//
		//kasus(5);
		//sleep(5);
        pthread_detach(thread2);
		//pthread_join(thread2,NULL);
		//sleep(2);
		}		
	}
}

