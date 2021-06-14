#include "Robot.h"
#include "Marvelmind.h"
#include "ros.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/Float32.h"
#include "geometry_msgs/Point32.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Pose.h"


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

/********* Define Lidar parameters **********/
#define LIDAR_OFST_X  0    
#define LIDAR_OFST_Y  0


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
#define ROS_UPDATE_PERIOD  60

/********************************************************/

Robot myRobot = Robot(MOTOR_PIN_FR, MOTOR_PIN_FL, MOTOR_PIN_RR, MOTOR_PIN_RL,
                      ENC_PIN_FR_A, ENC_PIN_FR_B, ENC_PIN_FL_A, ENC_PIN_FL_B,
                      ENC_PIN_RR_A, ENC_PIN_RR_B, ENC_PIN_RL_A, ENC_PIN_RL_B,
                      T_PERIOD_MS, WHEEL_TO_CENTER_X, WHEEL_TO_CENTER_Y, WHEEL_DIAMETER_MM);


/*************** ROS CONFIGURATION *********************/

// Raspberry and Arduino communicate through Serial3
class NewHardware : public ArduinoHardware {
      public:
        NewHardware(): ArduinoHardware(&Serial3, 250000) {};
};

//Callbacks function prototype
void cmdModeCallback( const std_msgs::UInt8& msg);
void cmdManualCallback( const std_msgs::UInt8& msg);
void cmdSpeedCallback( const std_msgs::Float32& msg);
void cmdTrayCallback( const geometry_msgs::Point32& msg);
void lidarCallback( const geometry_msgs::PoseStamped& msg);

//Node Handle
ros::NodeHandle_<NewHardware> nh;   

// ROS Subscriber
ros::Subscriber<std_msgs::UInt8> subCmdMode("/cmdMode", &cmdModeCallback);
ros::Subscriber<std_msgs::UInt8> subCmdManual("/cmdManual", &cmdManualCallback);
ros::Subscriber<std_msgs::Float32> subCmdSpeed("/cmdSpeed", &cmdSpeedCallback);
ros::Subscriber<geometry_msgs::Point32> subCmdTray("/cmdTray", &cmdTrayCallback);
ros::Subscriber<geometry_msgs::PoseStamped> subLidarPos("/slam_out_pose", &lidarCallback);

// ROS publisher
geometry_msgs::Pose robotPoseMsg;   //msg.position: robot position     msg.orientation: wheel speed reference
geometry_msgs::Pose robotSpeedMsg;  //msg.position: robot speed        msg.orientation: wheel speed 
geometry_msgs::Point32 robotImuMsg;
geometry_msgs::Point32 robotLidarMsg;
geometry_msgs::Point32 marvelmindHedgeMsg;
geometry_msgs::Point32 robotKalmanFilterMsg;
ros::Publisher pubRobotPose("/robotPose", &robotPoseMsg);
ros::Publisher pubRobotSpeed("/robotSpeed", &robotSpeedMsg);
ros::Publisher pubRobotImu("/robotImu", &robotImuMsg);
ros::Publisher pubRobotLidar("/robotLidar", &robotLidarMsg);
ros::Publisher pubMarvelmindHedge("/marvelmindHedge", &marvelmindHedgeMsg);
ros::Publisher pubKalmanFilter("/robotKalmanFilter", &robotKalmanFilterMsg);

//Marvelmind Indoor GPS
MarvelmindHedge hedge;
PositionValue hedgePosition;
float mX, mY, hedgePositionOffsetX = 0, hedgePositionOffsetY = 0;                      
unsigned long timeScanStart, startTime;
uint8_t updateROSCounter=0, updateLidarCounter=0;
long baudRate = 115200;

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
  nh.advertise(pubRobotPose);
  nh.advertise(pubRobotSpeed);
  nh.advertise(pubRobotImu);
  nh.advertise(pubRobotLidar);
  nh.advertise(pubMarvelmindHedge);
  nh.advertise(pubKalmanFilter);
  delay(1000);
  myRobot.motorFR->setPID(0.1,1.6,0.4);
  myRobot.motorFL->setPID(0.1,1.6,0.4);
  myRobot.motorRR->setPID(0.1,1.6,0.4);
  myRobot.motorRL->setPID(0.1,1.6,0.4);
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
  myRobot.updateKalmanFilter();
 
  
  //Every ROS_UPDATE_PERIOD get-send data to ROS
  if(updateROSCounter >= (ROS_UPDATE_PERIOD / T_PERIOD_MS)){

    //Debug
    //myRobot.serialDebug();

    // Robot
    myRobot.getDataSendToROS(&robotPoseMsg, &robotSpeedMsg, &robotImuMsg, &robotLidarMsg, &robotKalmanFilterMsg);

    // Marvelmind 
    hedge.read();
    if(hedge.getPositionFromMarvelmindHedge(&hedgePosition)){
      //Calculate Offset so hedge position will be (x,y) = (0,0) when a ROS_CMD_ODO_RST is received
      if(myRobot.isPositionResetted()){ 
        hedgePositionOffsetX = (hedgePosition.x - MARVELMIND_OFST_X)/ 1000.0 ;
        hedgePositionOffsetY = (hedgePosition.y - MARVELMIND_OFST_Y)/ 1000.0 ;
      }
      mX = (hedgePosition.x - MARVELMIND_OFST_X)/ 1000.0 - hedgePositionOffsetX;
      mY = (hedgePosition.y - MARVELMIND_OFST_Y)/ 1000.0 - hedgePositionOffsetY;
      marvelmindHedgeMsg.z = hedgePosition.z / 1000.0; 
      // System reference change. robotX = -gpsY, robotY = gpsX.
      marvelmindHedgeMsg.x = -mY;
      marvelmindHedgeMsg.y = mX;
      pubMarvelmindHedge.publish(&marvelmindHedgeMsg);
    }
    
    // Publish to ROS node
    pubRobotPose.publish(&robotPoseMsg);
    pubRobotSpeed.publish(&robotSpeedMsg);
    pubRobotImu.publish(&robotImuMsg);  
    pubRobotLidar.publish(&robotLidarMsg);
    pubKalmanFilter.publish(&robotKalmanFilterMsg);
    nh.spinOnce(); 
    
    updateROSCounter = 0;
  }
  else{
    updateROSCounter++;
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
void lidarCallback( const geometry_msgs::PoseStamped& msg)  {myRobot.setLidarPosFromROS(msg.pose.position.x, 
                                                                                        msg.pose.position.y, 
                                                                                        msg.pose.orientation.x,
                                                                                        msg.pose.orientation.y,
                                                                                        msg.pose.orientation.z,
                                                                                        msg.pose.orientation.w,
                                                                                        LIDAR_OFST_X/1000.0,
                                                                                        LIDAR_OFST_Y/1000.0); }
