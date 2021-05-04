#pragma once
#include <stdint.h>
#include "Arduino.h"

// THIS DEFINES CAN BE TUNE
#define MAX_BUFFERED_POSITIONS 1
#define MAX_STATIONARY_BEACONS 30
#define MESSAGE_BUFFER_SIZE 256

// THIS DEFINES CAN'T BE TUNE
#define NDISTANCES 4

#define POSITION_DATAGRAM_ID 0x0001
#define BEACONS_POSITIONS_DATAGRAM_ID 0x0002
#define POSITION_DATAGRAM_HIGHRES_ID 0x0011
#define BEACONS_POSITIONS_DATAGRAM_HIGHRES_ID 0x0012
#define IMU_RAW_DATAGRAM_ID 0x0003
#define BEACON_RAW_DISTANCE_DATAGRAM_ID 0x0004
#define IMU_FUSION_DATAGRAM_ID 0x0005
#define TELEMETRY_DATAGRAM_ID 0x0006
#define QUALITY_DATAGRAM_ID 0x0007
#define WAYPOINT_DATAGRAM_ID 0x0201

enum message_state
{
    RECV_HDR,
    RECV_DGRAM
};

class PositionValue
{
public:
    uint8_t address;
    uint32_t timestamp;
    int32_t x, y, z; // coordinates in millimeters

    float angle;

    bool highResolution;

    bool ready;
    bool processed;
};

class RawIMUValue
{
public:
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;

    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;

    int16_t compass_x;
    int16_t compass_y;
    int16_t compass_z;

    uint32_t timestamp;

    bool updated;
};

class FusionIMUValue
{
public:
    int32_t x;
    int32_t y;
    int32_t z; // coordinates in mm

    int16_t qw;
    int16_t qx;
    int16_t qy;
    int16_t qz; // quaternion, normalized to 10000

    int16_t vx;
    int16_t vy;
    int16_t vz; // velocity, mm/s

    int16_t ax;
    int16_t ay;
    int16_t az; // acceleration, mm/s^2

    uint32_t timestamp;

    bool updated;
};

class RawDistanceItem
{
public:
    uint8_t address_beacon;
    uint32_t distance; // distance, mm
};
class RawDistances
{
public:
    uint8_t address_hedge;
    RawDistanceItem distances[NDISTANCES];

    uint32_t timestamp;
    uint16_t timeShift;

    bool updated;
};

class StationaryBeaconPosition
{
public:
    uint8_t address;
    int32_t x, y, z; // coordinates in millimeters

    bool highResolution;
};
class StationaryBeaconsPositions
{
public:
    uint8_t numBeacons;
    StationaryBeaconPosition beacons[MAX_STATIONARY_BEACONS];

    bool updated;
};

class TelemetryData
{
public:
    uint16_t vbat_mv;
    int8_t rssi_dbm;

    bool updated;
};

class QualityData
{
public:
    uint8_t address;
    uint8_t quality_per;

    bool updated;
};

//////////////////////////////////////////////////////////////////////////////
class PositionValuePro
{
public:
    float x, y, z; // coordinates in meters
    float angle;
};

class RawDistancesPro
{
public:
    uint8_t address_hedge;
    float distances[NDISTANCES]; // distance in meters 
    uint8_t translate_beacons[NDISTANCES]; // Sorted from lowest to high address beacon
    uint8_t address_beacons[NDISTANCES]; // Sorted from lowest to high address beacon
    bool sorted = false;
};

class FusionIMUValuePro
{
public:
    float x;
    float y;
    float z; // coordinates in meters

    float roll;
    float pitch;
    float yaw; // euler angles in radians

    float vx;
    float vy;
    float vz; // velocity, m/s

    float ax;
    float ay;
    float az; // acceleration, m/s^2
};

const float g = 9.80665f; // m/s2
const float cfAcc = 0.001f * g; // mg (g - gravity earth unit value 1g = 9.8 m/s^2)
const float d2r = (PI / 180.0f); // degrees to radians
const float r2d = (180.0f / PI ); // radians to degrees  
const float cfGyro = 0.0175f * d2r; // conversion factor to dps (degrees per second) -> radians
const float g2t = 0.0001f; // 1 gauss = 0.0001 tesla
const float cfMagXY = (1/980) * g2t * 1000000.0f; // 1 Tesla [T] = 1 000 000 Microtesla [µT]
const float cfMagZ  = (1/1100) * g2t * 1000000.0f;

class RawIMUValuePro
{
public:
    float acc_x; // m/s^2
    float acc_y; // m/s^2
    float acc_z; // m/s^2

    float gyro_x; // rad/s
    float gyro_y; // rad/s
    float gyro_z; // rad/s

    float compass_x; // µT
    float compass_y; // µT
    float compass_z; // µT
};
//////////////////////////////////////////////////////////////////////////////

class MarvelmindHedge
{
public:
    void begin(Stream *serial);
    void begin(Stream *serial, Stream *serialDebug);

    void read();

    bool getPositionFromMarvelmindHedgeByAddress(PositionValue *position, uint8_t address);
    bool getPositionFromMarvelmindHedge(PositionValue *position);
    bool getStationaryBeaconsPositionsFromMarvelmindHedge(StationaryBeaconsPositions *positions);
    bool getRawDistancesFromMarvelmindHedge(RawDistances *rawDistances);
    bool getRawIMUFromMarvelmindHedge(RawIMUValue *rawIMU);
    bool getFusionIMUFromMarvelmindHedge(FusionIMUValue *fusionIMU);
    bool getTelemetryFromMarvelmindHedge(TelemetryData *telemetry);
    bool getQualityFromMarvelmindHedge(QualityData *quality);

    void printPositionFromMarvelmindHedge(bool onlyNew);
    void printStationaryBeaconsPositionsFromMarvelmindHedge(bool onlyNew);
    void printRawDistancesFromMarvelmindHedge(bool onlyNew);
    void printRawIMUFromMarvelmindHedge(bool onlyNew);
    void printFusionIMUFromMarvelmindHedge(bool onlyNew);
    void printTelemetryFromMarvelmindHedge(bool onlyNew);
    void printQualityFromMarvelmindHedge(bool onlyNew);

    bool getPositionFromMarvelmindHedge(bool onlyNew, PositionValuePro *positionValuePro);
    bool getRawDistancesFromMarvelmindHedge(bool onlyNew, RawDistancesPro *rawDistancesPro);
    bool getRawIMUFromMarvelmindHedge(bool onlyNew, RawIMUValue *rawIMU);
    bool getRawIMUFromMarvelmindHedge(bool onlyNew, RawIMUValuePro *rawIMUPro);
    bool getFusionIMUFromMarvelmindHedge(bool onlyNew, FusionIMUValuePro *fusionIMUValuePro);
private:
    char print_buffer [128]; // must be large enough for your whole string!

    uint8_t input_buffer[MESSAGE_BUFFER_SIZE];
    uint8_t recvState;             // current state of receive data
    uint8_t nBytesInBlockReceived; // bytes received
    uint16_t dataId;

    PositionValue curPosition;                            // Current Position
    PositionValue positionBuffer[MAX_BUFFERED_POSITIONS]; // buffer of measurements

    StationaryBeaconsPositions positionsBeacons;

    RawIMUValue rawIMU;
    FusionIMUValue fusionIMU;

    RawDistances rawDistances;

    TelemetryData telemetry;
    QualityData quality;

    // private variables
    uint8_t lastValuesCount_;
    uint8_t lastValues_next;
    bool haveNewValues_;

    // Additional arduino
    Stream *serialRX;
    bool debug = false;
    Stream *serialDebug;

    uint16_t CalcCrcModbus_(uint8_t *buf, int len);

    uint16_t get_uint16(uint8_t *buffer);
    int16_t get_int16(uint8_t *buffer);
    uint32_t get_uint32(uint8_t *buffer);
    int32_t get_int32(uint8_t *buffer);

    uint8_t markPositionReady();
    PositionValue process_position_datagram(uint8_t *buffer);
    PositionValue process_position_highres_datagram(uint8_t *buffer);
    StationaryBeaconPosition *getOrAllocBeacon(uint8_t address);
    void process_beacons_positions_datagram(uint8_t *buffer);
    void process_beacons_positions_highres_datagram(uint8_t *buffer);
    void process_imu_raw_datagram(uint8_t *buffer);
    void process_imu_fusion_datagram(uint8_t *buffer);
    void process_raw_distances_datagram(uint8_t *buffer);
    void process_telemetry_datagram(uint8_t *buffer);
    void process_quality_datagram(uint8_t *buffer);
    void process_waypoint_data(uint8_t *buffer);
};
