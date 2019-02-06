
#ifndef INVERSKINEMATIK_C
#define INVERSKINEMATIK_C

#include "/home/pi/systemTest/library/allLib.h"
#include <math.h>

#define P_TORQUE 24
#define P_GOAL_SPEED_L 32
#define P_GOAL_POSITION 30

using namespace std;

void kirimPacketSpeed(int totalServo, int id, int speed){ //Buat ngeset kecepatan servo
        int l=2;
        int n=totalServo;

        dxl_set_txpacket_id(BROADCAST_ID); //Jangan diubah 0xFE
        //dxl_set_txpacket_length(); //jumlah paket yang dikirim 0x18
        dxl_set_txpacket_instruction(INST_SYNC_WRITE);//instruksi 0x83
        dxl_set_txpacket_parameter(0,P_GOAL_SPEED_L);//yang mau ditulis 0x1E
        dxl_set_txpacket_parameter(1,2);//Jumlah data per servo
        for( int i=0; i<totalServo; i++){
                dxl_set_txpacket_parameter(2+3*i+0,id);
                dxl_set_txpacket_parameter(2+3*i+1,dxl_get_lowbyte(speed));
                dxl_set_txpacket_parameter(2+3*i+2,dxl_get_highbyte(speed));
        }

        dxl_set_txpacket_length((l+1)*n+4);
        dxl_txrx_packet();

}

void kirimPacketGerak(int totalServo, int totalStep, int start, int id[], int data[], bool active, int delay[], int speed[]){ //Buat gerakin servo
        if (active){
                int l = 2;
                int n = totalServo;
                for(int i=start-1; i<totalStep; i++){
			kirimPacketSpeed(totalServo, id[i], speed[i]);
                        dxl_set_txpacket_id(BROADCAST_ID); //Jangan diubah 0xFE
                        dxl_set_txpacket_instruction(INST_SYNC_WRITE);//instruksi 0x83
                        dxl_set_txpacket_parameter(0,P_GOAL_POSITION);//yang mau ditulis 0x1E
                        dxl_set_txpacket_parameter(1,2);//Jumlah data per servo
                                for(int j=0; j<totalServo; j++){
                                        dxl_set_txpacket_parameter(2+3*j+0,id[j]);
                                        dxl_set_txpacket_parameter(2+3*j+1,dxl_get_lowbyte(data[j]));
                                        dxl_set_txpacket_parameter(2+3*j+2,dxl_get_highbyte(data[j]));
                                }
                        dxl_set_txpacket_length((l+1)*n+4);
                        dxl_txrx_packet();//jumlah paket yang dikirim 0x18
                        usleep(delay[i]);
                }
        }
}

//total servo yg digerakin //array ID yang mau digerakin //kecepatan

void setAllSpeed(int sped1){
        dxl_write_word(BROADCAST_ID,32,sped1);
}

void kirimPacketInverse(double sudutX, double sudutY, double sudutZ, double x, double y, double z, double sudutX2, double sudutY2, double sudutZ2, double x2, double y2, double z2, int delay[], int speed[]){

	double l3, l5, l4, psi,tkan,tki; 
	double sudutLutut, sudutAnkle, sudutAnkle2, sudutSelangkang, sudutPantat, sudut345, sudutPaha, sudutLutut2, sudutAnkle22, sudutAnkle222, sudutSelangkang2, sudutPantat2, sudut3452, sudutPaha2;
	double nx, ny, nz, sx, sy, sz, ax, ay, az, nx2, ny2, nz2, sx2, sy2, sz2, ax2, ay2, az2;
	double s6, c6, c2, s2, s4, c4,s62, c62, c22, s22, s42, c42;
	double nilaiSelangkang,nilaiPantat, nilaiPaha, nilaiLutut, nilaiAnkle, nilaiAnkle2, nilaiDengkul, nilaiSelangkang2,nilaiPantat2, nilaiPaha2, nilaiLutut2, nilaiAnkle22, nilaiAnkle222, nilaiDengkul2;
	double sudutNgurang=0;
	double tempX = 13;
	double maxOperator = 1.2;
	l3 = 8.4;
	l4 = 8;
	l5 = 0;

	c4 = (pow(x + l5,2)+pow(y,2)+pow(z,2)-pow(l3,2)-pow(l4,2))/(2*l3*l4);
	s4 = 1 * sqrt(1-pow(c4,2));
	sudutLutut = atan2(s4,c4);

	cout<<"C4 :"<<c4<<endl;
	cout<<"S4 :"<<s4<<endl;
	cout<<"sudutLutut :"<<sudutLutut<<endl;

	psi = atan2(s4*l3,c4*l3+l4);
	sudutAnkle = atan2(z,1*sqrt(pow(x+l5,2)+pow(y,2)))-psi;

	sudutAnkle2 = atan2(y,-x-l5);

	sudutX=sudutX*3.14/180;
	sudutY=sudutY*3.14/180;
	sudutZ=(sudutZ*3.14/180)-(4*y*3.14/180);

	nx = cos(sudutZ) * cos(sudutY);
	ny = sin(sudutZ) * cos(sudutY);
	nz = sin(sudutY);

	sx = (sin(sudutZ) * cos(sudutX)) + (cos(sudutZ) * sin(sudutY) * sin(sudutX));
	sy = (cos(sudutZ) * cos(sudutX)) + (sin(sudutZ) * sin(sudutX) * sin(sudutY));
	sz = (cos(sudutY) * sin(sudutX));

	ax = (sin(sudutZ) * sin(sudutX)) + (cos(sudutZ) * sin(sudutY) * cos(sudutX));
	ay = (-cos(sudutZ)* sin(sudutX)) + (sin(sudutZ) * sin(sudutY) * cos(sudutX));
	az = (cos(sudutY) * cos(sudutX));

	s6 = sin(sudutAnkle2);
	c6 = cos(sudutAnkle2);
	c2 = (s6 * ax) + (c6 * ay);
	s2 = 1*sqrt(1-pow(c2,2));
	sudutPantat = atan2(s2,c2);

	sudutSelangkang = atan2(-s6 * sx - c6 * sy, -s6 * nx - c6 * ny);

	sudut345 = atan2(az,(c6 * ax) - (s6 * ay));
	sudutPaha = sudut345 - sudutLutut - sudutAnkle;

	cout<<"psi :"<<psi<<endl;
	cout<<"sudutAnkle :"<<sudutAnkle<<endl;
	cout<<"sudutAnkle2 :"<<sudutAnkle2<<endl;
	cout<<"sudutX :"<<sudutX<<endl;
	cout<<"sudutY :"<<sudutY<<endl;
	cout<<"sudutZ :"<<sudutZ<<endl;
	cout<<"nx :"<<nx<<endl;
	cout<<"ny :"<<ny<<endl;
	cout<<"nz :"<<nz<<endl;
	cout<<"sx :"<<sx<<endl;
	cout<<"sy :"<<sy<<endl;
	cout<<"sz :"<<sz<<endl;
	cout<<"ax :"<<ax<<endl;
	cout<<"ay :"<<ay<<endl;
	cout<<"az :"<<az<<endl;

	cout<<"s6 :"<<s6<<endl;
	cout<<"c6 :"<<c6<<endl;
	cout<<"s2 :"<<s2<<endl;
	cout<<"c2 :"<<c2<<endl;

	cout<<"sudutPantat :"<<sudutPantat<<endl;
	cout<<"sudutSelangkang :"<<sudutSelangkang<<endl;
	cout<<"sudut345 :"<<sudut345<<endl;
	cout<<"sudutPaha :"<<sudutPaha<<endl;

	nilaiSelangkang = 668.502239 + ((sudutSelangkang*180/3.14)/0.29);
	nilaiPantat = 196.497761 +  ((sudutPantat*180/3.14)/0.29);
	nilaiPaha = 201.497761 + ((sudutPaha*180/3.14)/0.29);
	nilaiLutut = 2048 - ((sudutLutut*180/3.14)/0.08797);
	nilaiAnkle  = 2048 - ((sudutAnkle*180/3.14)/0.08797);
	nilaiAnkle2 =517 -  ((sudutAnkle2*180/3.14)/0.29);

	cout<<"nilaiSelangkang :"<<nilaiSelangkang<<endl;
	cout<<"nilaiPantat :"<<nilaiPantat<<endl;
	cout<<"nilaiPaha :"<<nilaiPaha<<endl;
	cout<<"nilaiLutut :"<<nilaiLutut<<endl;
	cout<<"nilaiAnkle :"<<nilaiAnkle<<endl;
	cout<<"nilaiAnkle2 :"<<nilaiAnkle2<<endl;
	cout<<"Nilai X :"<<x<<endl;
	cout<<"Nilai Sin :"<<sin(sudutNgurang*3.14/180)*2<<endl;
	
	c42 = (pow(x2 + l5,2)+pow(y2,2)+pow(z2,2)-pow(l3,2)-pow(l4,2))/(2*l3*l4);
        s42 = 1 * sqrt(1-pow(c42,2));
        sudutLutut2 = atan2(s42,c42);

        cout<<"C42 :"<<c42<<endl;
        cout<<"S42 :"<<s42<<endl;
        cout<<"sudutLutut2 :"<<sudutLutut2<<endl;

        psi = atan2(s42*l3,c42*l3+l4);
        sudutAnkle22 = atan2(z2,1*sqrt(pow(x2+l5,2)+pow(y2,2)))-psi;

        sudutAnkle222 = atan2(y2,-x2-l5);

        sudutX2=sudutX2*3.14/180;
        sudutY2=sudutY2*3.14/180;
        sudutZ2=(sudutZ2*3.14/180)-(4*y*3.14/180);

        nx2 = cos(sudutZ2) * cos(sudutY2);
        ny2 = sin(sudutZ2) * cos(sudutY2);
        nz2 = sin(sudutY2);

        sx2 = (sin(sudutZ2) * cos(sudutX2)) + (cos(sudutZ2) * sin(sudutY2) * sin(sudutX2));
        sy2 = (cos(sudutZ2) * cos(sudutX2)) + (sin(sudutZ2) * sin(sudutX2) * sin(sudutY2));
        sz2 = (cos(sudutY2) * sin(sudutX2));

        ax2 = (sin(sudutZ2) * sin(sudutX2)) + (cos(sudutZ2) * sin(sudutY2) * cos(sudutX2));
        ay2 = (-cos(sudutZ2)* sin(sudutX2)) + (sin(sudutZ2) * sin(sudutY2) * cos(sudutX2));
        az2 = (cos(sudutY2) * cos(sudutX2));

        s62 = sin(sudutAnkle2);
        c62 = cos(sudutAnkle2);
        c22 = (s6 * ax) + (c6 * ay);
        s22 = 1*sqrt(1-pow(c22,2));
        sudutPantat2 = atan2(s22,c22);

        sudutSelangkang2 = atan2(-s62 * sx2 - c62 * sy2, -s62 * nx2 - c62 * ny2);

        sudut3452 = atan2(az2,(c62 * ax2) - (s62 * ay2));
        sudutPaha2 = sudut3452 - sudutLutut2 - sudutAnkle22;
	
	nilaiSelangkang2 = 365.497761 - ((sudutSelangkang2*180/3.14)/0.29);
        nilaiPantat2 = 1024-(196.497761 +  ((sudutPantat*180/3.14)/0.29));
        nilaiPaha2 = 822.50224 - ((sudutPaha2*180/3.14)/0.29);
        nilaiLutut2 = 2048 + ((sudutLutut2*180/3.14)/0.08797);
        nilaiAnkle22  = 2048 + ((sudutAnkle22*180/3.14)/0.08797);
        nilaiAnkle222 = 517 -  ((sudutAnkle222*180/3.14)/0.29);
	cout<<"nilaiSelangkang2 :"<<nilaiSelangkang2<<endl;
        cout<<"nilaiPantat2 :"<<nilaiPantat2<<endl;
        cout<<"nilaiPaha2 :"<<nilaiPaha2<<endl;
        cout<<"nilaiLutut2 :"<<nilaiLutut2<<endl;
        cout<<"nilaiAnkle22 :"<<nilaiAnkle22<<endl;
        cout<<"nilaiAnkle222 :"<<nilaiAnkle222<<endl;
	tkan=235+80*sin(sudutNgurang*3.14/180);
	tki=788+80*sin(sudutNgurang*3.14/180);

				   //744
	int id[] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18};
        int data[18]= {235,780,300,744,291,733,365/*nilaiSelangkang*/,668/*nilaiSelangkang2*/, nilaiPantat,nilaiPantat2,nilaiPaha,nilaiPaha2,nilaiLutut,nilaiLutut2,nilaiAnkle,nilaiAnkle22,nilaiAnkle2,nilaiAnkle222};
	kirimPacketGerak(18, 1, 1, id, data, 1, delay, speed);
}


void kirimPacketTorque(int totalServo, int id[], int torque){ //Buat ngeset kecepatan servo
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



void PrintErrorCode() {
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

void PrintCommStatus(int CommStatus) {
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


void setAllTorque(int torque){
	dxl_write_word(BROADCAST_ID,34,torque);
}


#endif
