#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include "std_msgs/Float32.h"
#include <iostream>
#include <stdlib.h>
#include <thread>
#include <fstream>
#include <math.h>
#include <std_msgs/Bool.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Pose2D.h"
#include "sensor_msgs/LaserScan.h"


float Angle[1152];
float Jarak[1152];

int closestIndex = -1;
float closestDistance = std::numeric_limits<float>::infinity();
float closestAngle;
float ChangeAngle;
// Komunikasi ROS
std_msgs::Float32 Fnmsg;
std_msgs::Float32 Falp;
std_msgs::Float32 FVelo;
ros::Publisher publisherF, publisherAl, PublisherStart, PublisherStop, PublisherResultant, PublisherStatis, PublisherTarget;
std::thread thread_runThread;
std_msgs::Bool startMsg;
bool stopThread;
int nilai = 1;
// Input Gain
float Gain = 3;
float Psi = 1;
double thetaY, thetaX, thetaZ;
// Input Goal dan Obstacle
ros::Time startTime;
float VectorX = 0;
float VectorY = 0;
float VectorGX = 0;
float VectorGY = 0;
float Pitch = 0;
float massa = 20;
float koef = 0.1;
float Mass = 1;
float Speed = 10;
float SpeedV = 1;
float Time = 1;
float Radius = 3.6;
float Roll, Yaw;
float radianAngle;

double degreesToRadians(double degrees) {
    return degrees * M_PI / 180.0;
}
void multiplyMatrices(double mat1[4][4], double mat2[4][4], double result[4][4]) {
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            result[i][j] = 0;
            for (int k = 0; k < 4; k++) {
                result[i][j] += mat1[i][k] * mat2[k][j];
            }
        }
    }
}

void lidarCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
    closestIndex = -1;
    closestDistance = std::numeric_limits<float>::infinity();

    for (int i = 0; i < msg->ranges.size(); ++i) {
        if (std::isinf(msg->ranges[i])) {
            Jarak[i] = 12;
        } else {
            Jarak[i] = msg->ranges[i];
        }
        Angle[i] = i * (msg->angle_increment * 180 / M_PI);

        if (Jarak[i] < closestDistance) {
            closestIndex = i;
            closestDistance = Jarak[i];
            closestAngle = Angle[closestIndex];
            ChangeAngle = closestAngle - 120; 
            radianAngle = ChangeAngle * (M_PI / 180); // Konversi sudut ke radian

        }
    }
            VectorX = closestDistance * cos(radianAngle);
            VectorY = closestDistance * sin(radianAngle);
}

void arraySFM(const std_msgs::Float32MultiArray::ConstPtr &array_msg)
{
  const auto &data = array_msg->data;
  int rows = array_msg->layout.dim[0].size;
  int cols = array_msg->layout.dim[4].size;
  VectorGX = data[0 * cols + 0];
  VectorGY = data[0 * cols + 1];
  Roll = data[0 * cols + 2];
  Pitch = data[0 * cols + 3];
  Yaw = data[0 * cols + 4];
}

float GoalForce(float VectorG, float Mass, float Speed, float CurrentSpeed, float Time)
{
  float equation = VectorG * Mass * (Speed - CurrentSpeed) / Time;
  ROS_INFO_STREAM("GForce : " << equation);
  return equation;
}

float Distance(float VectorX, float VectorY)
{
  float equation = sqrt(pow(VectorX, 2) + pow(VectorY, 2));
  return equation;
}

float SocialForce(float Gain, float Radius, float Psi)
{
  float Dist = Distance(VectorX, VectorY);
  float equation = pow(Gain, (Radius - Dist)) / Psi;
  return equation;
}

float PhysicalForce(float Gain, float Radius)
{
  float Dist = Distance(VectorX, VectorY);
  float equation = Gain * (Radius - Dist);
  return equation;
}

double hitungFgParalel(double massa, double sudut_kemiringan) {
    const double percepatan_gravitasi = 9.8; 
    double sudut_radian = sudut_kemiringan * (M_PI / 180);
    return massa * percepatan_gravitasi * sin(sudut_radian);
}

double hitungFgesekan(double koefisien_gesekan, double massa, double sudut_kemiringan) {
    const double percepatan_gravitasi = 9.8;
    double sudut_radian = sudut_kemiringan * (M_PI / 180);
    double fg_tegak_lurus = massa * percepatan_gravitasi * cos(sudut_radian);
    return koefisien_gesekan * fg_tegak_lurus;
}

float StaticForce(float Vector)
{
  float Dist = Distance(VectorX, VectorY);
  float SocForce = SocialForce(Gain, Radius, Psi);
  float PhyForce = PhysicalForce(Gain, Radius);
  float equation = (SocForce + PhyForce) * Vector;
  if (Dist > Radius)
  {
    equation = 0;
  }
  ROS_INFO_STREAM("Distance : " << Dist);
  ROS_INFO_STREAM("Force : " << equation);
  return equation;
}

float NavForce(float Vector, float VectorG, float SpeedV)
{
  float SocForce = SocialForce(Gain, Radius, Psi);
  float PhyForce = PhysicalForce(Gain, Radius);
  float GForce = GoalForce(VectorG, Mass, Speed, SpeedV, Time);
  float StaForce = StaticForce(Vector);
  float equation = GForce - StaForce;
  return equation;
}


void runThread()
{
  while (ros::ok())
  {
  
    while (nilai == 1)
    {
      
      bool Start = true;
      startMsg.data = Start;
      PublisherStart.publish(startMsg);
      nilai = 0;
      startTime = ros::Time::now(); 

    }

    while (nilai == 2){
      continue;

    }


    double angleX_rad = degreesToRadians(Roll);
    double angleY_rad = degreesToRadians(Pitch);
    double angleZ_rad = degreesToRadians(Yaw);

    double Rx[4][4] = {{1, 0, 0, 0}, {0, cos(angleX_rad), -sin(angleX_rad), 0}, {0, sin(angleX_rad), cos(angleX_rad), 0}, {0, 0, 0, 1}};
    double Ry[4][4] = {{cos(angleY_rad), 0, sin(angleY_rad), 0}, {0, 1, 0, 0}, {-sin(angleY_rad), 0, cos(angleY_rad), 0}, {0, 0, 0, 1}};
    double Rz[4][4] = {{cos(angleZ_rad), -sin(angleZ_rad), 0, 0}, {sin(angleZ_rad), cos(angleZ_rad), 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    double RTz[4][4] = {{cos(-angleZ_rad), -sin(-angleZ_rad), 0, 0}, {sin(-angleZ_rad), cos(-angleZ_rad), 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};
    double TT[4][4] = {{1, 0, 0, 0}, {0, 1, 0, 0}, {0, 0, 1, 0}, {0, 0, 0, 1}};

    double FK[4][4]  ;
    double RPY1[4][4] ;
    double RYZ[4][4] ;
    double Pose1[4][4] ;

    multiplyMatrices(RTz, TT, FK);
    multiplyMatrices(Ry, Rz, RYZ);
    multiplyMatrices(RYZ, Rx, RPY1);
    multiplyMatrices(RPY1, FK, Pose1);


    if (Pose1[0][2] < 1) {
        if (Pose1[0][2] > -1) {
            thetaY = asin(Pose1[0][2])*180/M_PI;
            thetaX = atan2(-Pose1[1][2], Pose1[2][2])*180/M_PI;
            thetaZ = atan2(-Pose1[0][1], Pose1[0][0])*180/M_PI;
        } else {
            thetaY = (-M_PI/2)*180/M_PI;
            thetaX = -atan2(Pose1[1][0], Pose1[1][1])*180/M_PI;
            thetaZ = 0.0*180/M_PI;
        }
    } else {
        thetaY = (M_PI/2)*180/M_PI;
        thetaX = atan2(Pose1[1][0], Pose1[1][1])*180/M_PI;
        thetaZ = 0.0*180/M_PI;
    }
    float Npitch = thetaY;
    bool Start = false;
    startMsg.data = Start;
    PublisherStart.publish(startMsg);
    float NavX = NavForce(VectorX, VectorGX, SpeedV);
    float NavY = NavForce(VectorY, VectorGY, SpeedV);
    float VObs = Distance(VectorX, VectorY);
    float VNav = Distance(NavX, NavY);
    float Vgoal = Distance(VectorGX, VectorGY);
    float AngleObs = (atan2(VectorY, VectorX)) * 180 / 3.14;
    float Angleg = (atan2(VectorGY, VectorGX)) * 180 / 3.14;
    float Angle = (atan2(NavY, NavX)) * 180 / 3.14;
    float Alpha = (atan2(((VNav * sin(Angle)) + (VObs * sin(AngleObs))), ((VNav * cos(Angle)) + (VObs * cos(AngleObs)))));
    double fg_paralel = hitungFgParalel(massa, thetaY);
    double f_gesekan = hitungFgesekan(koef, massa, thetaY);
    double jumlah_gaya = fg_paralel - f_gesekan;
    float Fi = jumlah_gaya/20;
    geometry_msgs::Point Statis, Target, Resultant;
    Statis.x = VectorX;
    Statis.y = VectorY;
    Statis.z = 0;
    Resultant.x = NavX;
    Resultant.y = NavY;
    Resultant.z = 0;
    Target.x = VectorGX;
    Target.y = VectorGY;
    Target.z = 0;
    PublisherResultant.publish(Resultant);
    PublisherStatis.publish(Statis);
    PublisherTarget.publish(Target);
    Falp.data = Angle;
    publisherAl.publish(Falp);
    float Vn = VNav/5;
    float Velo = (VNav/5)+Fi;
    FVelo.data = Velo;
    publisherF.publish(FVelo);
    float VgoalAbs = abs(Vgoal);
    ROS_INFO_STREAM("RPY : " << thetaX << " : " << thetaY << " : " << thetaZ);
    std::ofstream outputFile;
    outputFile.imbue(std::locale("")); // Tambahkan pengaturan locale di sini

    outputFile.open("/home/nana/dataAngle.csv", std::ios_base::app); // Mode append
    if (outputFile.is_open()) 
    {
        outputFile << std::fixed << std::setprecision(2) << Npitch << ";" << Velo << ";" << Vn << ";" << Fi << std::endl;
        outputFile.close();
    } else 
    {
        ROS_ERROR_STREAM("Failed to open file dataAngle.csv");
    }

    if (VgoalAbs > 0.1 && VgoalAbs < 0.5)
    {
        bool Stop = true;
        std_msgs::Bool stopMsg;
        stopMsg.data = Stop;
        PublisherStop.publish(stopMsg);
        ros::Time endTime = ros::Time::now(); // Waktu akhir loop
        ros::Duration duration = endTime - startTime; // Durasi eksekusi loop
        ROS_INFO("Waktu awal: %f", startTime.toSec());
        ROS_INFO("Waktu akhir: %f", endTime.toSec());
        ROS_INFO("Durasi eksekusi: %f", (duration.toSec()));
        Angle = 0;
        nilai = 2;
    }
  }
}
void startThread()
{
  stopThread = false;
  thread_runThread = std::thread(runThread);
}
void closeThread()
{
  stopThread = true;
  thread_runThread.join();
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "SFM");
  ros::NodeHandlePtr nh(new ros::NodeHandle());
  ros::Subscriber sub = nh->subscribe<sensor_msgs::LaserScan>("/hokuyo", 1000, lidarCallback);
  ros::Subscriber subSFM = nh->subscribe("social", 10, arraySFM);
  publisherF = nh->advertise<std_msgs::Float32>("/Fnav", 10);
  publisherAl = nh->advertise<std_msgs::Float32>("/Alp", 10);
  PublisherStart = nh->advertise<std_msgs::Bool>("/startSimulation", 10);
  PublisherStop = nh->advertise<std_msgs::Bool>("/stopSimulation", 10);
  PublisherTarget = nh->advertise<geometry_msgs::Point>("target", 10);
  PublisherStatis = nh->advertise<geometry_msgs::Point>("Statis", 10);
  PublisherResultant = nh->advertise<geometry_msgs::Point>("Resultant", 10);
  if (nilai == 1) 
  {
    startThread();
  }
  else 
  {
    closeThread(); 
  }
  ros::spin();
  return 0;
}