#include "Marvelmind.h"

void MarvelmindHedge::begin(Stream *serial)
{

    this->serialRX = serial;

    for (int i = 0; i < MESSAGE_BUFFER_SIZE; i++)
        input_buffer[i] = 0;

    recvState = RECV_HDR;      // current state of receive data
    nBytesInBlockReceived = 0; // bytes received
    dataId = 0;

    curPosition.address = 0;
    curPosition.timestamp = 0;
    curPosition.x = 0;
    curPosition.y = 0;
    curPosition.z = 0;
    curPosition.angle = 0;
    curPosition.highResolution = false;
    curPosition.processed = false;
    curPosition.ready = false;

    for (int i = 0; i < MAX_BUFFERED_POSITIONS; i++)
    {
        positionBuffer[i].address = 0;
        positionBuffer[i].timestamp = 0;
        positionBuffer[i].x = 0;
        positionBuffer[i].y = 0;
        positionBuffer[i].z = 0;
        positionBuffer[i].angle = 0;
        positionBuffer[i].highResolution = false;
        positionBuffer[i].processed = false;
        positionBuffer[i].ready = false;
    }

    positionsBeacons.numBeacons = 0;
    positionsBeacons.updated = false;
    for (int i = 0; i < MAX_STATIONARY_BEACONS; i++)
    {
        positionsBeacons.beacons[i].address = 0;
        positionsBeacons.beacons[i].x = 0;
        positionsBeacons.beacons[i].y = 0;
        positionsBeacons.beacons[i].z = 0;
        positionsBeacons.beacons[i].highResolution = false;
    }

    rawIMU.acc_x = 0;
    rawIMU.acc_y = 0;
    rawIMU.acc_z = 0;
    rawIMU.compass_x = 0;
    rawIMU.compass_y = 0;
    rawIMU.compass_z = 0;
    rawIMU.gyro_x = 0;
    rawIMU.gyro_y = 0;
    rawIMU.gyro_z = 0;
    rawIMU.timestamp = 0;
    rawIMU.updated = false;

    fusionIMU.x = 0;
    fusionIMU.y = 0;
    fusionIMU.z = 0;
    fusionIMU.vx = 0;
    fusionIMU.vy = 0;
    fusionIMU.vz = 0;
    fusionIMU.qw = 0;
    fusionIMU.qx = 0;
    fusionIMU.qy = 0;
    fusionIMU.qz = 0;
    fusionIMU.ax = 0;
    fusionIMU.ay = 0;
    fusionIMU.az = 0;
    fusionIMU.timestamp = 0;
    fusionIMU.updated = false;

    rawDistances.address_hedge = 0;
    rawDistances.timestamp = 0;
    rawDistances.timeShift = 0;
    rawDistances.updated = false;
    for (int i = 0; i < NDISTANCES; i++)
    {
        rawDistances.distances[i].address_beacon = 0;
        rawDistances.distances[i].distance = 0;
    }

    telemetry.vbat_mv = 0;
    telemetry.rssi_dbm = 0;
    telemetry.updated = false;

    quality.address = 0;
    quality.quality_per = 0;
    quality.updated = 0;
}

void MarvelmindHedge::begin(Stream *serial, Stream *serialDebug)
{

    this->debug = true;
    this->serialDebug = serialDebug;

    this->begin(serial);
}

void MarvelmindHedge::read()
{
    while (this->serialRX->available() > 0)
    {
        uint8_t receivedChar = this->serialRX->read();
        bool goodByte = false;
        input_buffer[nBytesInBlockReceived] = receivedChar;
        switch (recvState)
        {
        case RECV_HDR:
            switch (nBytesInBlockReceived)
            {
            case 0:
                goodByte = (receivedChar == 0xff);
                break;
            case 1:
                goodByte = (receivedChar == 0x47) || (receivedChar == 0x4a);
                break;
            case 2:
                goodByte = true;
                break;
            case 3:
                dataId = (((uint16_t)receivedChar) << 8) + input_buffer[2];
                goodByte = (dataId == POSITION_DATAGRAM_ID) ||
                           (dataId == BEACONS_POSITIONS_DATAGRAM_ID) ||
                           (dataId == POSITION_DATAGRAM_HIGHRES_ID) ||
                           (dataId == BEACONS_POSITIONS_DATAGRAM_HIGHRES_ID) ||
                           (dataId == IMU_RAW_DATAGRAM_ID) ||
                           (dataId == IMU_FUSION_DATAGRAM_ID) ||
                           (dataId == BEACON_RAW_DISTANCE_DATAGRAM_ID) ||
                           (dataId == TELEMETRY_DATAGRAM_ID) ||
                           (dataId == QUALITY_DATAGRAM_ID) ||
                           (dataId == WAYPOINT_DATAGRAM_ID);
                break;
            case 4:
                switch (dataId)
                {
                case POSITION_DATAGRAM_ID:
                    goodByte = (receivedChar == 0x10);
                    break;
                case BEACONS_POSITIONS_DATAGRAM_ID:
                case BEACONS_POSITIONS_DATAGRAM_HIGHRES_ID:
                    goodByte = true;
                    break;
                case POSITION_DATAGRAM_HIGHRES_ID:
                    goodByte = (receivedChar == 0x16);
                    break;
                case IMU_RAW_DATAGRAM_ID:
                    goodByte = (receivedChar == 0x20);
                    break;
                case IMU_FUSION_DATAGRAM_ID:
                    goodByte = (receivedChar == 0x2a);
                    break;
                case BEACON_RAW_DISTANCE_DATAGRAM_ID:
                    goodByte = (receivedChar == 0x20);
                    break;
                case TELEMETRY_DATAGRAM_ID:
                    goodByte = (receivedChar == 0x10);
                    break;
                case QUALITY_DATAGRAM_ID:
                    goodByte = (receivedChar == 0x10);
                    break;
                case WAYPOINT_DATAGRAM_ID:
                    goodByte = (receivedChar == 0x0c);
                    break;
                }
                if (goodByte)
                    recvState = RECV_DGRAM;
                break;
            }
            if (goodByte)
            {
                // correct header byte
                nBytesInBlockReceived++;
            }
            else
            {
                // ...or incorrect
                recvState = RECV_HDR;
                nBytesInBlockReceived = 0;
            }
            break;
        case RECV_DGRAM:
            nBytesInBlockReceived++;
            if (nBytesInBlockReceived >= 7 + input_buffer[4])
            {
                // parse dgram
                uint16_t blockCrc = this->CalcCrcModbus_(input_buffer, nBytesInBlockReceived);
                if (blockCrc == 0)
                {

                    switch (dataId)
                    {
                    case POSITION_DATAGRAM_ID:
                        // add to positionBuffer
                        curPosition = this->process_position_datagram(input_buffer);
                        break;
                    case BEACONS_POSITIONS_DATAGRAM_ID:
                        this->process_beacons_positions_datagram(input_buffer);
                        break;
                    case POSITION_DATAGRAM_HIGHRES_ID:
                        // add to positionBuffer
                        curPosition = this->process_position_highres_datagram(input_buffer);
                        break;
                    case BEACONS_POSITIONS_DATAGRAM_HIGHRES_ID:
                        this->process_beacons_positions_highres_datagram(input_buffer);
                        break;
                    case IMU_RAW_DATAGRAM_ID:
                        this->process_imu_raw_datagram(input_buffer);
                        break;
                    case IMU_FUSION_DATAGRAM_ID:
                        this->process_imu_fusion_datagram(input_buffer);
                        break;
                    case BEACON_RAW_DISTANCE_DATAGRAM_ID:
                        this->process_raw_distances_datagram(input_buffer);
                        break;
                    case TELEMETRY_DATAGRAM_ID:
                        this->process_telemetry_datagram(input_buffer);
                        break;
                    case QUALITY_DATAGRAM_ID:
                        this->process_quality_datagram(input_buffer);
                        break;
                    case WAYPOINT_DATAGRAM_ID:
                        this->process_waypoint_data(input_buffer);
                        break;
                    }
                }
                // and repeat
                recvState = RECV_HDR;
                nBytesInBlockReceived = 0;
            }
        }
    }
}

//////////////////////////////////////////////////////////////////////////////
// Calculate CRC (Modbus) for array of bytes
// buf: input buffer
// len: size of buffer
// returncode: CRC value
//////////////////////////////////////////////////////////////////////////////
uint16_t MarvelmindHedge::CalcCrcModbus_(uint8_t *buf, int len)
{
    uint16_t crc = 0xFFFF;
    int pos;
    for (pos = 0; pos < len; pos++)
    {
        crc ^= (uint16_t)buf[pos]; // XOR byte into least sig. byte of crc
        int i;
        for (i = 8; i != 0; i--) // Loop over each bit
        {
            if ((crc & 0x0001) != 0) // If the LSB is set
            {
                crc >>= 1; // Shift right and XOR 0xA001
                crc ^= 0xA001;
            }
            else           // Else LSB is not set
                crc >>= 1; // Just shift right
        }
    }
    return crc;
}

//////////////////////////////////////////////////////////////////////////////

uint16_t MarvelmindHedge::get_uint16(uint8_t *buffer)
{
    uint16_t res = buffer[0] |
                   (((uint16_t)buffer[1]) << 8);

    return res;
}

int16_t MarvelmindHedge::get_int16(uint8_t *buffer)
{
    int16_t res = buffer[0] |
                  (((uint16_t)buffer[1]) << 8);

    return res;
}

uint32_t MarvelmindHedge::get_uint32(uint8_t *buffer)
{
    uint32_t res = buffer[0] |
                   (((uint32_t)buffer[1]) << 8) |
                   (((uint32_t)buffer[2]) << 16) |
                   (((uint32_t)buffer[3]) << 24);

    return res;
}

int32_t MarvelmindHedge::get_int32(uint8_t *buffer)
{
    int32_t res = buffer[0] |
                  (((uint32_t)buffer[1]) << 8) |
                  (((uint32_t)buffer[2]) << 16) |
                  (((uint32_t)buffer[3]) << 24);

    return res;
}

//////////////////////////////////////////////////////////////////////////////

uint8_t MarvelmindHedge::markPositionReady()
{
    uint8_t ind = this->lastValues_next;
    uint8_t indCur = ind;

    this->positionBuffer[ind].ready =
        true;
    this->positionBuffer[ind].processed =
        false;
    ind++;
    if (ind >= MAX_BUFFERED_POSITIONS)
        ind = 0;
    if (this->lastValuesCount_ < MAX_BUFFERED_POSITIONS)
        this->lastValuesCount_++;
    this->haveNewValues_ = true;

    this->lastValues_next = ind;

    return indCur;
}

PositionValue MarvelmindHedge::process_position_datagram(uint8_t *buffer)
{
    uint8_t ind = this->lastValues_next;

    this->positionBuffer[ind].address =
        buffer[16];
    this->positionBuffer[ind].timestamp =
        buffer[5] |
        (((uint32_t)buffer[6]) << 8) |
        (((uint32_t)buffer[7]) << 16) |
        (((uint32_t)buffer[8]) << 24);

    int16_t vx = buffer[9] |
                 (((uint16_t)buffer[10]) << 8);
    this->positionBuffer[ind].x = vx * 10; // millimeters

    int16_t vy = buffer[11] |
                 (((uint16_t)buffer[12]) << 8);
    this->positionBuffer[ind].y = vy * 10; // millimeters

    int16_t vz = buffer[13] |
                 (((uint16_t)buffer[14]) << 8);
    this->positionBuffer[ind].z = vz * 10; // millimeters

    uint16_t vang = buffer[17] |
                    (((uint16_t)buffer[18]) << 8);
    this->positionBuffer[ind].angle = ((float)(vang & 0x0fff)) / 10.0f;

    this->positionBuffer[ind].highResolution = false;

    ind = this->markPositionReady();

    return this->positionBuffer[ind];
}

PositionValue MarvelmindHedge::process_position_highres_datagram(uint8_t *buffer)
{
    uint8_t ind = this->lastValues_next;

    this->positionBuffer[ind].address =
        buffer[22];
    this->positionBuffer[ind].timestamp =
        buffer[5] |
        (((uint32_t)buffer[6]) << 8) |
        (((uint32_t)buffer[7]) << 16) |
        (((uint32_t)buffer[8]) << 24);

    int32_t vx = buffer[9] |
                 (((uint32_t)buffer[10]) << 8) |
                 (((uint32_t)buffer[11]) << 16) |
                 (((uint32_t)buffer[12]) << 24);
    this->positionBuffer[ind].x = vx;

    int32_t vy = buffer[13] |
                 (((uint32_t)buffer[14]) << 8) |
                 (((uint32_t)buffer[15]) << 16) |
                 (((uint32_t)buffer[16]) << 24);
    this->positionBuffer[ind].y = vy;

    int32_t vz = buffer[17] |
                 (((uint32_t)buffer[18]) << 8) |
                 (((uint32_t)buffer[19]) << 16) |
                 (((uint32_t)buffer[20]) << 24);
    this->positionBuffer[ind].z = vz;

    uint16_t vang = buffer[23] |
                    (((uint16_t)buffer[24]) << 8);
    this->positionBuffer[ind].angle = ((float)(vang & 0x0fff)) / 10.0f;

    this->positionBuffer[ind].highResolution = true;

    ind = markPositionReady();

    return this->positionBuffer[ind];
}

StationaryBeaconPosition *MarvelmindHedge::getOrAllocBeacon(uint8_t address)
{
    uint8_t i;
    uint8_t n_used = this->positionsBeacons.numBeacons;

    if (n_used != 0)
        for (i = 0; i < n_used; i++)
        {
            if (this->positionsBeacons.beacons[i].address == address)
            {
                return &this->positionsBeacons.beacons[i];
            }
        }

    if (n_used >= (MAX_STATIONARY_BEACONS - 1))
        return NULL;

    this->positionsBeacons.numBeacons = (n_used + 1);
    return &this->positionsBeacons.beacons[n_used];
}

void MarvelmindHedge::process_beacons_positions_datagram(uint8_t *buffer)
{
    uint8_t n = buffer[5]; // number of beacons in packet
    uint8_t i, ofs;
    uint8_t address;
    int16_t x, y, z;
    StationaryBeaconPosition *b;

    if ((1 + n * 8) != buffer[4])
        return; // incorrect size

    for (i = 0; i < n; i++)
    {
        ofs = 6 + i * 8;

        address = buffer[ofs + 0];
        x = buffer[ofs + 1] |
            (((uint16_t)buffer[ofs + 2]) << 8);
        y = buffer[ofs + 3] |
            (((uint16_t)buffer[ofs + 4]) << 8);
        z = buffer[ofs + 5] |
            (((uint16_t)buffer[ofs + 6]) << 8);

        b = this->getOrAllocBeacon(address);
        if (b != NULL)
        {
            b->address = address;
            b->x = x * 10; // millimeters
            b->y = y * 10; // millimeters
            b->z = z * 10; // millimeters

            b->highResolution = false;

            this->positionsBeacons.updated = true;
        }
    }
}

void MarvelmindHedge::process_beacons_positions_highres_datagram(uint8_t *buffer)
{
    uint8_t n = buffer[5]; // number of beacons in packet
    uint8_t i, ofs;
    uint8_t address;
    int32_t x, y, z;
    StationaryBeaconPosition *b;

    if ((1 + n * 14) != buffer[4])
        return; // incorrect size

    for (i = 0; i < n; i++)
    {
        ofs = 6 + i * 14;

        address = buffer[ofs + 0];
        x = buffer[ofs + 1] |
            (((uint32_t)buffer[ofs + 2]) << 8) |
            (((uint32_t)buffer[ofs + 3]) << 16) |
            (((uint32_t)buffer[ofs + 4]) << 24);
        y = buffer[ofs + 5] |
            (((uint32_t)buffer[ofs + 6]) << 8) |
            (((uint32_t)buffer[ofs + 7]) << 16) |
            (((uint32_t)buffer[ofs + 8]) << 24);
        z = buffer[ofs + 9] |
            (((uint32_t)buffer[ofs + 10]) << 8) |
            (((uint32_t)buffer[ofs + 11]) << 16) |
            (((uint32_t)buffer[ofs + 12]) << 24);

        b = this->getOrAllocBeacon(address);
        if (b != NULL)
        {
            b->address = address;
            b->x = x;
            b->y = y;
            b->z = z;

            b->highResolution = true;

            this->positionsBeacons.updated = true;
        }
    }
}

void MarvelmindHedge::process_imu_raw_datagram(uint8_t *buffer)
{
    uint8_t *dataBuf = &buffer[5];

    this->rawIMU.acc_x = this->get_int16(&dataBuf[0]);
    this->rawIMU.acc_y = this->get_int16(&dataBuf[2]);
    this->rawIMU.acc_z = this->get_int16(&dataBuf[4]);

    //
    this->rawIMU.gyro_x = this->get_int16(&dataBuf[6]);
    this->rawIMU.gyro_y = this->get_int16(&dataBuf[8]);
    this->rawIMU.gyro_z = this->get_int16(&dataBuf[10]);

    //
    this->rawIMU.compass_x = this->get_int16(&dataBuf[12]);
    this->rawIMU.compass_y = this->get_int16(&dataBuf[14]);
    this->rawIMU.compass_z = this->get_int16(&dataBuf[16]);

    this->rawIMU.timestamp = this->get_uint32(&dataBuf[24]);

    this->rawIMU.updated = true;
}

void MarvelmindHedge::process_imu_fusion_datagram(uint8_t *buffer)
{
    uint8_t *dataBuf = &buffer[5];

    this->fusionIMU.x = this->get_int32(&dataBuf[0]);
    this->fusionIMU.y = this->get_int16(&dataBuf[4]);
    this->fusionIMU.z = this->get_int16(&dataBuf[8]);

    this->fusionIMU.qw = this->get_int16(&dataBuf[12]);
    this->fusionIMU.qx = this->get_int16(&dataBuf[14]);
    this->fusionIMU.qy = this->get_int16(&dataBuf[16]);
    this->fusionIMU.qz = this->get_int16(&dataBuf[18]);

    this->fusionIMU.vx = this->get_int16(&dataBuf[20]);
    this->fusionIMU.vy = this->get_int16(&dataBuf[22]);
    this->fusionIMU.vz = this->get_int16(&dataBuf[24]);

    this->fusionIMU.ax = this->get_int16(&dataBuf[26]);
    this->fusionIMU.ay = this->get_int16(&dataBuf[28]);
    this->fusionIMU.az = this->get_int16(&dataBuf[30]);

    this->fusionIMU.timestamp = this->get_uint32(&dataBuf[34]);

    this->fusionIMU.updated = true;
}

void MarvelmindHedge::process_raw_distances_datagram(uint8_t *buffer)
{
    uint8_t *dataBuf = &buffer[5];
    uint8_t ofs, i;

    this->rawDistances.address_hedge = dataBuf[0];

    ofs = 1;
    for (i = 0; i < 4; i++)
    {
        this->rawDistances.distances[i].address_beacon = dataBuf[ofs + 0];
        this->rawDistances.distances[i].distance = this->get_uint32(&dataBuf[ofs + 1]);
        ofs += 6;
    }

    this->rawDistances.timestamp = this->get_uint32(&dataBuf[25]);
    this->rawDistances.timeShift = this->get_uint16(&dataBuf[29]);

    this->rawDistances.updated = true;
}

void MarvelmindHedge::process_telemetry_datagram(uint8_t *buffer)
{
    uint8_t *dataBuf = &buffer[5];

    this->telemetry.vbat_mv = this->get_uint16(&dataBuf[0]);
    this->telemetry.rssi_dbm = (int8_t)dataBuf[2];

    this->telemetry.updated = true;
}

void MarvelmindHedge::process_quality_datagram(uint8_t *buffer)
{
    uint8_t *dataBuf = &buffer[5];

    this->quality.address = dataBuf[0];
    this->quality.quality_per = dataBuf[1];

    this->quality.updated = true;
}

void MarvelmindHedge::process_waypoint_data(uint8_t *buffer)
{
    /*
    uint8_t i;
    for (i = 0; i < 16; i++)
    {
        SERIAL_DEBUG.printf("%03d, ", buffer[i]);
    }
    SERIAL_DEBUG.printf("\n");
*/
}

//////////////////////////////////////////////////////////////////////////////
// Write average position coordinates
// position:   pointer to PositionValue for write coordinates
// returncode: true if position is valid
//////////////////////////////////////////////////////////////////////////////
bool MarvelmindHedge::getPositionFromMarvelmindHedgeByAddress(PositionValue *position, uint8_t address)
{
    uint8_t i;
    int32_t avg_x = 0, avg_y = 0, avg_z = 0;
    float avg_ang = 0.0;
    uint32_t max_timestamp = 0;
    bool position_valid;
    bool highRes = false;

    if (this->lastValuesCount_)
    {
        uint8_t real_values_count = MAX_BUFFERED_POSITIONS;
        uint8_t nFound = 0;
        if (this->lastValuesCount_ < real_values_count)
            real_values_count = this->lastValuesCount_;
        for (i = 0; i < real_values_count; i++)
        {
            if (address != 0)
                if (this->positionBuffer[i].address != address)
                    continue;
            if (!this->positionBuffer[i].ready)
                continue;
            if (this->positionBuffer[i].processed)
                continue;
            if (address == 0)
                address = this->positionBuffer[i].address;
            nFound++;
            avg_x += this->positionBuffer[i].x;
            avg_y += this->positionBuffer[i].y;
            avg_z += this->positionBuffer[i].z;
            avg_ang += this->positionBuffer[i].angle;
            if (this->positionBuffer[i].highResolution)
                highRes = true;
            this->positionBuffer[i].processed = true;
            if (this->positionBuffer[i].timestamp > max_timestamp)
                max_timestamp = this->positionBuffer[i].timestamp;
        }
        if (nFound != 0)
        {
            avg_x /= nFound;
            avg_y /= nFound;
            avg_z /= nFound;
            avg_ang /= nFound;
            position_valid = true;
        }
        else
        {
            position_valid = false;
        }
    }
    else
        position_valid = false;

    position->address = address;
    position->x = avg_x;
    position->y = avg_y;
    position->z = avg_z;
    position->angle = avg_ang;
    position->timestamp = max_timestamp;
    position->ready = position_valid;
    position->highResolution = highRes;
    return position_valid;
}

bool MarvelmindHedge::getPositionFromMarvelmindHedge(PositionValue *position)
{
    return getPositionFromMarvelmindHedgeByAddress(position, 0);
};

//////////////////////////////////////////////////////////////////////////////
// Print average position coordinates
// onlyNew: print only new positions
//////////////////////////////////////////////////////////////////////////////
void MarvelmindHedge::printPositionFromMarvelmindHedge(bool onlyNew)
{
    if (this->debug)
    {
        uint8_t i, j;
        float xm, ym, zm;

        if (this->haveNewValues_ || (!onlyNew))
        {
            struct PositionValue position;
            uint8_t addresses[MAX_BUFFERED_POSITIONS];
            uint8_t addressesNum = 0;

            for (i = 0; i < MAX_BUFFERED_POSITIONS; i++)
            {
                uint8_t address = this->positionBuffer[i].address;
                bool alreadyProcessed = false;
                if (addressesNum != 0)
                    for (j = 0; j < addressesNum; j++)
                    {
                        if (address == addresses[j])
                        {
                            alreadyProcessed = true;
                            break;
                        }
                    }
                if (alreadyProcessed)
                    continue;
                addresses[addressesNum++] = address;

                this->getPositionFromMarvelmindHedgeByAddress(&position, address);
                xm = ((float)position.x) / 1000.0f;
                ym = ((float)position.y) / 1000.0f;
                zm = ((float)position.z) / 1000.0f;
                if (position.ready)
                {
                    int precision = 2;

                    if (position.highResolution) precision = 3;

                    this->serialDebug->print("Address: ");
                    this->serialDebug->print(position.address);
                    this->serialDebug->print(", X: ");
                    this->serialDebug->print(xm, precision);
                    this->serialDebug->print(", Y: ");
                    this->serialDebug->print(ym, precision);
                    this->serialDebug->print(", Z: ");
                    this->serialDebug->print(zm, precision);
                    this->serialDebug->print(", Angle: ");
                    this->serialDebug->print(position.angle, 1);
                    this->serialDebug->print("  at time T: ");
                    this->serialDebug->print(position.timestamp);
                    this->serialDebug->print(" \n");
                }
                this->haveNewValues_ = false;
            }
        }
    }
}

/////////////////////////////////////////////

bool MarvelmindHedge::getStationaryBeaconsPositionsFromMarvelmindHedge(StationaryBeaconsPositions *positions)
{

    *positions = this->positionsBeacons;

    return true;
}

void MarvelmindHedge::printStationaryBeaconsPositionsFromMarvelmindHedge(bool onlyNew)
{
    if (this->debug)
    {
        StationaryBeaconsPositions positions;
        float xm, ym, zm;

        this->getStationaryBeaconsPositionsFromMarvelmindHedge(&positions);

        if (positions.updated || (!onlyNew))
        {
            uint8_t i;
            uint8_t n = this->positionsBeacons.numBeacons;
            StationaryBeaconPosition *b;

            for (i = 0; i < n; i++)
            {
                b = &positions.beacons[i];
                xm = ((float)b->x) / 1000.0f;
                ym = ((float)b->y) / 1000.0f;
                zm = ((float)b->z) / 1000.0f;

                int precision = 2;
                if (positions.beacons[i].highResolution) precision = 3;

                this->serialDebug->print("Stationary beacon: address: ");
                this->serialDebug->print(b->address);
                this->serialDebug->print(", X: ");
                this->serialDebug->print(xm, precision);
                this->serialDebug->print(", Y: ");
                this->serialDebug->print(ym, precision);
                this->serialDebug->print(", Z: ");
                this->serialDebug->print(zm, precision);
                this->serialDebug->print(" \n");
            }

            this->positionsBeacons.updated = false;
        }
    }
}

//////////////

bool MarvelmindHedge::getRawDistancesFromMarvelmindHedge(RawDistances *rawDistances)
{

    *rawDistances = this->rawDistances;

    return true;
}

void MarvelmindHedge::printRawDistancesFromMarvelmindHedge(bool onlyNew)
{
    if (this->debug)
    {
        RawDistances rawDistances;
        uint8_t i;
        float d_m;

        this->getRawDistancesFromMarvelmindHedge(&rawDistances);

        if (rawDistances.updated || (!onlyNew))
        {
            for (i = 0; i < 4; i++)
            {
                if (rawDistances.distances[i].address_beacon != 0)
                {
                    d_m = rawDistances.distances[i].distance / 1000.0;

                    sprintf(print_buffer, "Raw distance: %02d ==> %02d,  Distance= ",
                            (int)rawDistances.address_hedge,
                            (int)rawDistances.distances[i].address_beacon);
                    this->serialDebug->print(print_buffer);

                    this->serialDebug->print(d_m, 3);

                    sprintf(print_buffer, ", Timestamp= %d, Time shift= %d \n",
                            (int)rawDistances.timestamp,
                            (int)rawDistances.timeShift);
                    this->serialDebug->print(print_buffer);
                }
            }

            this->rawDistances.updated = false;
        }
    }
}

//////////////

bool MarvelmindHedge::getRawIMUFromMarvelmindHedge(RawIMUValue *rawIMU)
{

    *rawIMU = this->rawIMU;

    return true;
}

void MarvelmindHedge::printRawIMUFromMarvelmindHedge(bool onlyNew)
{
    if (this->debug)
    {
        RawIMUValue rawIMU;

        this->getRawIMUFromMarvelmindHedge(&rawIMU);

        if (rawIMU.updated || (!onlyNew))
        {
            sprintf(print_buffer, "Raw IMU: Timestamp: %08d, aX=%05d aY=%05d aZ=%05d  gX=%05d gY=%05d gZ=%05d  cX=%05d cY=%05d cZ=%05d \n",
                    (int)rawIMU.timestamp,
                    (int)rawIMU.acc_x, (int)rawIMU.acc_y, (int)rawIMU.acc_z,
                    (int)rawIMU.gyro_x, (int)rawIMU.gyro_y, (int)rawIMU.gyro_z,
                    (int)rawIMU.compass_x, (int)rawIMU.compass_y, (int)rawIMU.compass_z);
            this->serialDebug->print(print_buffer);

            this->rawIMU.updated = false;
        }
    }
}

//////////////

bool MarvelmindHedge::getFusionIMUFromMarvelmindHedge(FusionIMUValue *fusionIMU)
{

    *fusionIMU = this->fusionIMU;

    return true;
}

void MarvelmindHedge::printFusionIMUFromMarvelmindHedge(bool onlyNew)
{
    if (this->debug)
    {
        FusionIMUValue fusionIMU;
        float x_m, y_m, z_m;
        float qw, qx, qy, qz;
        float vx, vy, vz, ax, ay, az;

        this->getFusionIMUFromMarvelmindHedge(&fusionIMU);

        if (fusionIMU.updated || (!onlyNew))
        {
            x_m = fusionIMU.x / 1000.0f;
            y_m = fusionIMU.y / 1000.0f;
            z_m = fusionIMU.z / 1000.0f;

            qw = fusionIMU.qw / 10000.0f;
            qx = fusionIMU.qx / 10000.0f;
            qy = fusionIMU.qy / 10000.0f;
            qz = fusionIMU.qz / 10000.0f;

            vx = fusionIMU.vx / 1000.0f;
            vy = fusionIMU.vy / 1000.0f;
            vz = fusionIMU.vz / 1000.0f;

            ax = fusionIMU.ax / 1000.0f;
            ay = fusionIMU.ay / 1000.0f;
            az = fusionIMU.az / 1000.0f;

            sprintf(print_buffer, "IMU fusion: Timestamp: %08d,", (int)fusionIMU.timestamp);
            this->serialDebug->print(print_buffer);

            this->serialDebug->print("  X=");
            this->serialDebug->print(x_m, 3);
            this->serialDebug->print("  Y=");
            this->serialDebug->print(y_m, 3);
            this->serialDebug->print("  Z=");
            this->serialDebug->print(z_m, 3);

            this->serialDebug->print("  q=");
            this->serialDebug->print(qw, 3);
            this->serialDebug->print(",");
            this->serialDebug->print(qx, 3);
            this->serialDebug->print(",");
            this->serialDebug->print(qy, 3);
            this->serialDebug->print(",");
            this->serialDebug->print(qz, 3);

            this->serialDebug->print("  v=");
            this->serialDebug->print(vx, 3);
            this->serialDebug->print(",");
            this->serialDebug->print(vy, 3);
            this->serialDebug->print(",");
            this->serialDebug->print(vz, 3);

            this->serialDebug->print("  a=");
            this->serialDebug->print(ax, 3);
            this->serialDebug->print(",");
            this->serialDebug->print(ay, 3);
            this->serialDebug->print(",");
            this->serialDebug->print(az, 3);

            this->serialDebug->print(" \n");

            this->fusionIMU.updated = false;
        }
    }
}

//////

bool MarvelmindHedge::getTelemetryFromMarvelmindHedge(TelemetryData *telemetry)
{

    *telemetry = this->telemetry;

    return true;
}

void MarvelmindHedge::printTelemetryFromMarvelmindHedge(bool onlyNew)
{
    if (this->debug)
    {
        TelemetryData telemetry;
        this->getTelemetryFromMarvelmindHedge(&telemetry);

        if (telemetry.updated || (!onlyNew))
        {
            sprintf(print_buffer, "Telemetry: Vbat= ");
            this->serialDebug->print(print_buffer);

            float v_bat = ((float) telemetry.vbat_mv) / 1000.0f;
            this->serialDebug->print(v_bat, 3);

            sprintf(print_buffer, " V,    RSSI= %d dBm \n", (int)telemetry.rssi_dbm);
            this->serialDebug->print(print_buffer);

            this->telemetry.updated = false;
        }
    }
}

//////////

bool MarvelmindHedge::getQualityFromMarvelmindHedge(QualityData *quality)
{

    *quality = this->quality;

    return true;
}

void MarvelmindHedge::printQualityFromMarvelmindHedge(bool onlyNew)
{
    if (this->debug)
    {
        QualityData quality;
        this->getQualityFromMarvelmindHedge(&quality);

        if (quality.updated || (!onlyNew))
        {
            sprintf(print_buffer, "Quality: Address= %d,  Q= %d %% \n", (int)quality.address, (int)quality.quality_per);
            this->serialDebug->print(print_buffer);

            this->quality.updated = false;
        }
    }
}

/////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
// Get average position coordinates
// onlyNew: print only new positions
//////////////////////////////////////////////////////////////////////////////
bool MarvelmindHedge::getPositionFromMarvelmindHedge(bool onlyNew, PositionValuePro *positionValuePro)
{
    if (this->haveNewValues_ || (!onlyNew))
    {
        uint8_t i, j;
        PositionValue position;
        uint8_t addresses[MAX_BUFFERED_POSITIONS];
        uint8_t addressesNum = 0;

        for (i = 0; i < MAX_BUFFERED_POSITIONS; i++)
        {
            uint8_t address = this->positionBuffer[i].address;
            bool alreadyProcessed = false;
            if (addressesNum != 0)
                for (j = 0; j < addressesNum; j++)
                {
                    if (address == addresses[j])
                    {
                        alreadyProcessed = true;
                        break;
                    }
                }
            if (alreadyProcessed)
                continue;
            addresses[addressesNum++] = address;

            this->getPositionFromMarvelmindHedgeByAddress(&position, address);

            positionValuePro->x = ((float)position.x) / 1000.0f;
            positionValuePro->y = ((float)position.y) / 1000.0f;
            positionValuePro->z = ((float)position.z) / 1000.0f;

            positionValuePro->angle = position.angle;

            this->haveNewValues_ = false;
        }
        return true;
    }
    return false;
}

bool MarvelmindHedge::getRawDistancesFromMarvelmindHedge(bool onlyNew, RawDistancesPro *rawDistancesPro)
{

    RawDistances rawDistances;
    this->getRawDistancesFromMarvelmindHedge(&rawDistances);

    if (rawDistances.updated || (!onlyNew))
    {
        uint8_t i, j, id;
        float d_m;

        if(!rawDistancesPro->sorted) {
            rawDistancesPro->address_hedge = rawDistances.address_hedge;

            uint8_t max_id_sorted = 0;
            for (j = 0; j < 4; j++) {
                id = 0;
                uint8_t min_id = 255;
                for (i = 0; i < 4; i++) {
                    if(rawDistances.distances[i].address_beacon < min_id &&
                    rawDistances.distances[i].address_beacon > max_id_sorted) {
                        min_id = rawDistances.distances[i].address_beacon;
                        id = i;
                    }
                }
                max_id_sorted = rawDistances.distances[id].address_beacon;
                rawDistancesPro->translate_beacons[j] = id;
                rawDistancesPro->address_beacons[j] = rawDistances.distances[id].address_beacon;
            }        
            rawDistancesPro->sorted = true;
        }

        for (i = 0; i < 4; i++)
        {
            id = rawDistancesPro->translate_beacons[i];
            if (rawDistances.distances[id].address_beacon != 0)
            { 
                rawDistancesPro->distances[i] = ((float) rawDistances.distances[id].distance) / 1000.0f;  
            }
        }

        this->rawDistances.updated = false;

        return true;
    }
    return false;
}

bool MarvelmindHedge::getRawIMUFromMarvelmindHedge(bool onlyNew, RawIMUValue *rawIMU){

    this->getRawIMUFromMarvelmindHedge(rawIMU);

    if (rawIMU->updated || (!onlyNew))
    {
        this->rawIMU.updated = false;
        return true;
    }
    return false;
}

bool MarvelmindHedge::getRawIMUFromMarvelmindHedge(bool onlyNew, RawIMUValuePro *rawIMUPro){

    RawIMUValue rawIMU;
    this->getRawIMUFromMarvelmindHedge(&rawIMU);

    if (rawIMU.updated || (!onlyNew))
    {
        rawIMU.acc_x = ((float) rawIMU.acc_x) * cfAcc;
        rawIMU.acc_y = ((float) rawIMU.acc_y) * cfAcc;
        rawIMU.acc_z = ((float) rawIMU.acc_z) * cfAcc;

        rawIMU.gyro_x = ((float) rawIMU.gyro_x) * cfGyro;
        rawIMU.gyro_y = ((float) rawIMU.gyro_y) * cfGyro;
        rawIMU.gyro_z = ((float) rawIMU.gyro_z) * cfGyro;

        rawIMU.compass_x = ((float) rawIMU.compass_x) * cfMagXY;
        rawIMU.compass_y = ((float) rawIMU.compass_y) * cfMagXY;
        rawIMU.compass_z = ((float) rawIMU.compass_z) * cfMagZ;

        return true;
    }
    return false;
}

bool MarvelmindHedge::getFusionIMUFromMarvelmindHedge(bool onlyNew, FusionIMUValuePro *fusionIMUValuePro){

    FusionIMUValue fusionIMU;

    this->getFusionIMUFromMarvelmindHedge(&fusionIMU);

    if (fusionIMU.updated || (!onlyNew))
    {
        fusionIMUValuePro->x = ((float) fusionIMU.x) / 1000.0f;
        fusionIMUValuePro->y = ((float) fusionIMU.y) / 1000.0f;
        fusionIMUValuePro->z = ((float) fusionIMU.z) / 1000.0f;

        float qw = ((float) fusionIMU.qw) / 10000.0f;
        float qx = ((float) fusionIMU.qx) / 10000.0f;
        float qy = ((float) fusionIMU.qy) / 10000.0f;
        float qz = ((float) fusionIMU.qz) / 10000.0f;

        fusionIMUValuePro->roll = atan2(2.0*(qy*qz + qw*qx), qw*qw - qx*qx - qy*qy + qz*qz);
        fusionIMUValuePro->pitch = asin(-2.0*(qx*qz - qw*qy));
        fusionIMUValuePro->yaw = atan2(2.0*(qx*qy + qw*qz), qw*qw + qx*qx - qy*qy - qz*qz);

        fusionIMUValuePro->vx = ((float) fusionIMU.vx) / 1000.0f;
        fusionIMUValuePro->vy = ((float) fusionIMU.vy) / 1000.0f;
        fusionIMUValuePro->vz = ((float) fusionIMU.vz) / 1000.0f;

        fusionIMUValuePro->ax = ((float) fusionIMU.ax) / 1000.0f;
        fusionIMUValuePro->ay = ((float) fusionIMU.ay) / 1000.0f;
        fusionIMUValuePro->az = ((float) fusionIMU.az) / 1000.0f;

        this->fusionIMU.updated = false;

        return true;
    }
    return false;
}
