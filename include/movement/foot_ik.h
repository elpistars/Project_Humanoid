#ifndef SYNCWRITE_H
#define SYNCWRITE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <iostream>
#include "dynamixel/dynamixel.h"
#include <unistd.h>
#include <math.h>

#define P_TORQUE 34
#define P_GOAL_SPEED_L 32
#define P_GOAL_POSITION 30
class Joint{
    public:
        struct joint_param{
            double x,y,z,sudutX,sudutY,sudutZ;
        };
        Joint(joint_param init);
        Joint();
        void setParam(double x, double y, double z, double sudutX, double sudutY, double sudutZ);
        void setParam(joint_param newParam);
        void printParam(char* label);

        Joint::joint_param getParam() {return param;}
        void getParam(joint_param &p) {p=param;}
        
    private:
        joint_param param;
};
class IK{
    public:
        struct ik_param{
            double sudut,inc_sudut;
            int speed;
        };
        IK();
        IK(double teta, double d_teta, int Speed);   
        void kirimPacketSpeed(int totalServo, int id[], int sp);
        void kirimPacketGerak(int totalServo, int totalStep, int start,  int id[],int data[], bool active, int delay);
        void kirimPacketGerak(int totalServo, int totalStep, int start,  int id[],int data[][18],bool active,int delay[],int speed[]);
        void kirimPacketGerak(int totalServo, int totalStep, int start, int id[],int data[][18], bool active, int delay);
        void setAllSpeed(int Speed);
        void kirimPacketTorque(int totalServo, int id[], int torque[]);
        void inverseAndre( Joint::joint_param Kanan, Joint::joint_param Kiri, int speed,int no7, int no8); //just leg
        void inverseAndre( Joint::joint_param Kanan, Joint::joint_param Kiri, int speed,int no7, int no8, int x_head, int y_head);//leg and head
        void inverseAndre( Joint::joint_param Kanan, Joint::joint_param Kiri, int speed,int no7, int no8, int tk1,int tk2,int tk3,int tk4,int tk5,int tk6, int x_head, int y_head);//all body
        void kirimPacketTorque(int totalServo, int id[], int torque); 
        void PrintErrorCode();
        void PrintCommStatus(int CommStatus);
        void setAllTorque(int torque);
        void setParam(double teta, double d_teta, int Speed);
        void setParam(ik_param newParam);
        IK::ik_param getParam(){return param;}
        void getParam(ik_param &p){p=param;}
        void printParam(char* label);
        ik_param param;

};

extern int x_head,y_head;
extern double f2;

int cek_move();

Joint Kanan,Kiri;
Joint::joint_param temp_joint[2];
IK Invers;

#ifdef __cplusplus
}
#endif

#endif