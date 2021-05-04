#include "Robot.h"
#include "Marvelmind.h"
#include "ros.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseStamped.h"


/******************** CONFIG ****************************/
/****** Define Motor connection to the MotorShield ******/
#define	MOTOR_PIN_FR		  1
#define MOTOR_PIN_FL		  2
#define MOTOR_PIN_RR		  3         
#define MOTOR_PIN_RL		  4         

/**************** Define Robot Dimensions ***************/
#define WHEEL_DIAMETER_MM	100		
#define WHEEL_TO_CENTER_X 157.5
#define WHEEL_TO_CENTER_Y 157.5

/********* Define Marvelmind mounting position **********/
#define MARVELMIND_OFST_X -78
#define MARVELMIND_OFST_Y 100

/********* Define Lidar mounting position **********/
#define LIDAR_OFST_X  0    
#define LIDAR_OFST_Y  50
#define LIDAR_ODO_CORRECTION_PERIOD 1000

/******* Define Encoder connection to the Arduino *******/
#define ENC_PIN_FR_A    49
#define ENC_PIN_FR_B    48
#define ENC_PIN_FL_A    50
#define ENC_PIN_FL_B    51 
#define ENC_PIN_RR_A    27
#define ENC_PIN_RR_B    26 
#define ENC_PIN_RL_A    24
#define ENC_PIN_RL_B    25 

/********** Define Fixed Scan Period of Arduino *********/
#define T_PERIOD_MS     20

/************ Define ROS Communication ************/
#define ROS_UPDATE_PERIOD  50

/********************************************************/

Robot myRobot = Robot(MOTOR_PIN_FR, MOTOR_PIN_FL, MOTOR_PIN_RR, MOTOR_PIN_RL,
                      ENC_PIN_FR_A, ENC_PIN_FR_B, ENC_PIN_FL_A, ENC_PIN_FL_B,
                      ENC_PIN_RR_A, ENC_PIN_RR_B, ENC_PIN_RL_A, ENC_PIN_RL_B,
                      T_PERIOD_MS, WHEEL_TO_CENTER_X, WHEEL_TO_CENTER_Y, WHEEL_DIAMETER_MM);


/*************** ROS CONFIGURATION *********************/

// Raspberry and Arduino communicate through Serial3
class NewHardware : public ArduinoHardware {
      public:
        NewHardware(): ArduinoHardware(&Serial3, 57600) {};
};

//Callbacks function prototype
void cmdModeCallback( const std_msgs::UInt8& msg);
void cmdManualCallback( const std_msgs::UInt8& msg);
void cmdSpeedCallback( const std_msgs::Float32& msg);
void cmdTrayCallback( const geometry_msgs::Point32& msg);
void lidarCallback( const geometry_msgs::PoseStamped& msg);
void odometryCorrectionPeriodCallback( const std_msgs::UInt16& msg);

//Node Handle
ros::NodeHandle_<NewHardware> nh;   

// ROS Subscriber
ros::Subscriber<std_msgs::UInt8> subCmdMode("/cmdMode", &cmdModeCallback);
ros::Subscriber<std_msgs::UInt8> subCmdManual("/cmdManual", &cmdManualCallback);
ros::Subscriber<std_msgs::Float32> subCmdSpeed("/cmdSpeed", &cmdSpeedCallback);
ros::Subscriber<geometry_msgs::Point32> subCmdTray("/cmdTray", &cmdTrayCallback);
ros::Subscriber<geometry_msgs::PoseStamped> subLidarPos("/slam_out_pose", &lidarCallback);
ros::Subscriber<std_msgs::UInt16> subOdometryCorrectionPeriod("/odometryCorrectionPeriod", &odometryCorrectionPeriodCallback);

// ROS publisher
geometry_msgs::Point32 robotPoseMsg;
geometry_msgs::Point32 robotSpeedMsg;
geometry_msgs::Point32 robotImuMsg;
geometry_msgs::Point32 marvelmindHedgeMsg;
std_msgs::UInt8 robotOdometryCorrectionMsg;  // msg.data = set to 1 when odometry is corrected 
ros::Publisher pubRobotPose("/robotPose", &robotPoseMsg);
ros::Publisher pubRobotSpeed("/robotSpeed", &robotSpeedMsg);
ros::Publisher pubRobotImu("/robotImu", &robotImuMsg);
ros::Publisher pubMarvelmindHedge("/marvelmindHedge", &marvelmindHedgeMsg);
ros::Publisher pubRobotOdometryCorrection("/robotOdometryCorrection", &robotOdometryCorrectionMsg);

//Marvelmind Indoor GPS
MarvelmindHedge hedge;
PositionValue hedgePosition;
float hedgePositionOffsetX = 0, hedgePositionOffsetY = 0;
                      
unsigned long timeScanStart;
uint8_t updateROSCounter=0, updateLidarCorrectionCounter=0;
long baudRate = 115200;
float lidarOdometryCorrectionPeriod = 1000;

void setup(){
  Serial.begin(baudRate);
  Serial2.begin(baudRate);
  hedge.begin(&Serial2);
  myRobot.init();
  nh.initNode();
  nh.subscribe(subCmdMode);
  nh.subscribe(subCmdManual);
  nh.subscribe(subCmdSpeed);
  nh.subscribe(subCmdTray);
  nh.subscribe(subLidarPos);
  nh.subscribe(subOdometryCorrectionPeriod);
  nh.advertise(pubRobotPose);
  nh.advertise(pubRobotSpeed);
  nh.advertise(pubRobotImu);
  nh.advertise(pubMarvelmindHedge);
  nh.advertise(pubRobotOdometryCorrection);
  delay(1000);
  myRobot.motorFR->setPID(0.05,0.8,0.2);
  myRobot.motorFL->setPID(0.05,0.8,0.2);
  myRobot.motorRR->setPID(0.05,0.8,0.2);
  myRobot.motorRL->setPID(0.05,0.8,0.2);
  attachInterrupt(ENC_PIN_FR_A,updateEncoderFRChA,CHANGE);
  attachInterrupt(ENC_PIN_FR_B,updateEncoderFRChB,CHANGE);
  attachInterrupt(ENC_PIN_FL_A,updateEncoderFLChA,CHANGE);
  attachInterrupt(ENC_PIN_FL_B,updateEncoderFLChB,CHANGE);
  attachInterrupt(ENC_PIN_RR_A,updateEncoderRRChA,CHANGE);
  attachInterrupt(ENC_PIN_RR_B,updateEncoderRRChB,CHANGE);
  attachInterrupt(ENC_PIN_RL_A,updateEncoderRLChA,CHANGE);
  attachInterrupt(ENC_PIN_RL_B,updateEncoderRLChB,CHANGE);
  delay(1000);
}

void loop(){
  timeScanStart=millis(); 

  myRobot.ROSControl();
  myRobot.updateSpeed(); 
  myRobot.updatePos();
 

  // Lidar correction
  if(updateLidarCorrectionCounter >= (lidarOdometryCorrectionPeriod / T_PERIOD_MS)){
    if(myRobot.lidarOdometryCorrection()){
      robotOdometryCorrectionMsg.data = 1;   //Odometry corrected
    }
    updateLidarCorrectionCounter = 0; 
  }
  
  //Every ROS_UPDATE_PERIOD get-send data to ROS
  if(updateROSCounter >= (ROS_UPDATE_PERIOD / T_PERIOD_MS)){

    //Debug
    //myRobot.serialDebug();

    // Robot
    myRobot.getDataSendToROS(&robotPoseMsg, &robotSpeedMsg, &robotImuMsg);

    // Marvelmind 
    hedge.read();
    if(hedge.getPositionFromMarvelmindHedge(&hedgePosition)){
      //Calculate Offset so hedge position will be (x,y) = (0,0) when a ROS_CMD_ODO_RST is received
      if(myRobot.setOdometryPosition(0,0,0)){ 
        hedgePositionOffsetX = (hedgePosition.x - MARVELMIND_OFST_X)/ 1000.0 ;
        hedgePositionOffsetY = (hedgePosition.y - MARVELMIND_OFST_Y)/ 1000.0 ;
      }
      marvelmindHedgeMsg.x = (hedgePosition.x - MARVELMIND_OFST_X)/ 1000.0 - hedgePositionOffsetX;
      marvelmindHedgeMsg.y = (hedgePosition.y - MARVELMIND_OFST_Y)/ 1000.0 - hedgePositionOffsetY;
      marvelmindHedgeMsg.z = hedgePosition.z / 1000.0; 
      pubMarvelmindHedge.publish(&marvelmindHedgeMsg);
    }
    
    // Publish to ROS node
    pubRobotPose.publish(&robotPoseMsg);
    pubRobotSpeed.publish(&robotSpeedMsg);
    pubRobotImu.publish(&robotImuMsg);  
    pubRobotOdometryCorrection.publish(&robotOdometryCorrectionMsg); 
    nh.spinOnce(); 

    if(robotOdometryCorrectionMsg.data == 1)   robotOdometryCorrectionMsg.data = 0;  //Set to zero once published
    updateROSCounter = 0;
  }
  else{
    updateROSCounter++;
    updateLidarCorrectionCounter++;
  }
  
  while((millis()- timeScanStart) < T_PERIOD_MS){ ;}
    
}

void updateEncoderFRChA(){ myRobot.motorFR->encoder->updateEncoderChA();}
void updateEncoderFRChB(){ myRobot.motorFR->encoder->updateEncoderChB();}
void updateEncoderFLChA(){ myRobot.motorFL->encoder->updateEncoderChA();}
void updateEncoderFLChB(){ myRobot.motorFL->encoder->updateEncoderChB();}
void updateEncoderRRChA(){ myRobot.motorRR->encoder->updateEncoderChA();}
void updateEncoderRRChB(){ myRobot.motorRR->encoder->updateEncoderChB();}
void updateEncoderRLChA(){ myRobot.motorRL->encoder->updateEncoderChA();}
void updateEncoderRLChB(){ myRobot.motorRL->encoder->updateEncoderChB();}
void cmdModeCallback( const std_msgs::UInt8& msg)           {myRobot.setCmdModeFromROS(msg.data);   }
void cmdManualCallback( const std_msgs::UInt8& msg)         {myRobot.setCmdManualFromROS(msg.data); }
void cmdSpeedCallback( const std_msgs::Float32& msg)        {myRobot.setCmdSpeedFromROS(msg.data);  }
void cmdTrayCallback( const geometry_msgs::Point32& msg)    {myRobot.setCmdTrayFromROS(msg.x, msg.y, msg.z);}
void lidarCallback( const geometry_msgs::PoseStamped& msg)  {myRobot.setLidarPosFromROS(msg.pose.position.x - (LIDAR_OFST_X/1000.0), msg.pose.position.y - (LIDAR_OFST_Y/1000.0));}
void odometryCorrectionPeriodCallback( const std_msgs::UInt16& msg)   {lidarOdometryCorrectionPeriod = msg.data;}
