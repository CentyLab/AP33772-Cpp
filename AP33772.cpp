/* Structs and Resgister list ported from "AP33772 I2C Command Tester" by Joseph Liang
 * Created 11 April 2022
 * Added class and class functions by VicentN for PicoPD evaluation board
 * Created 8 August 2023
 * Updated 6 Oct 2024 - Expose more internal class variable and include better PPS functions
 */

#include "AP33772.h"

/**
 * @brief Class constuctor
 * @param &wire reference of Wire class. Pass in Wire or Wire1
 */
AP33772::AP33772(TwoWire &wire)
{
    _i2cPort = &wire;
}

/**
 * @brief Check if power supply is good and fetch the PDO profile
 */
void AP33772::begin()
{
    i2c_read(AP33772_ADDRESS, CMD_STATUS, 1); // CMD: Read Status
    ap33772_status.readStatus = readBuf[0];

    if (ap33772_status.isOvp)
        event_flag.ovp = 1;
    if (ap33772_status.isOcp)
        event_flag.ocp = 1;

    if (ap33772_status.isReady) // negotiation finished
    {
        if (ap33772_status.isNewpdo) // new PDO
        {
            if (ap33772_status.isSuccess) // negotiation success
                event_flag.newNegoSuccess = 1;
            else
                event_flag.newNegoFail = 1;
        }
        else
        {
            if (ap33772_status.isSuccess)
                event_flag.negoSuccess = 1;
            else
                event_flag.negoFail = 1;
        }
    }
    delay(10);

    // If negotiation is good, let's load in some PDOs from charger
    if (event_flag.newNegoSuccess | event_flag.negoSuccess)
    {
        event_flag.newNegoSuccess = 0;

        i2c_read(AP33772_ADDRESS, CMD_PDONUM, 1); // CMD: Read PDO Number
        numPDO = readBuf[0];

        i2c_read(AP33772_ADDRESS, CMD_SRCPDO, SRCPDO_LENGTH); // CMD: Read PDOs
        // copy PDOs to pdoData[]
        for (byte i = 0; i < numPDO; i++)
        {
            pdoData[i].byte0 = readBuf[i * 4];
            pdoData[i].byte1 = readBuf[i * 4 + 1];
            pdoData[i].byte2 = readBuf[i * 4 + 2];
            pdoData[i].byte3 = readBuf[i * 4 + 3];

            if ((pdoData[i].byte3 & 0xF0) == 0xC0) // PPS profile found
            {
                PPSindices[numPPS] = i; // Store index
                numPPS += 1;
            }
        }
    }
}

/**
 * @brief Set VBUS voltage and max current
 * Current will automatically be in limit mode
 * @param targetVoltage, targetCurrent, - mV and mA
 */
void AP33772::setSupplyVoltageCurrent(int targetVoltage, int targetCurrent)
{
    int8_t ppsIndex = getPPSIndexByVoltageCurrent(targetVoltage, targetCurrent);
    if (ppsIndex != -1) {
        setPPSPDO(ppsIndex, targetVoltage, targetCurrent);
    }
}

/**
 * @brief Set VBUS voltage
 * Current is set to MAX all the time
 * @param targetVoltage in mV
 */
void AP33772::setVoltage(int targetVoltage)
{
    // Step 1: Check if PPS can satify request voltage
    int8_t ppsIndex = getPPSIndexByVoltageCurrent(targetVoltage, 0);
    if (ppsIndex != -1) {
        setPPSPDO(ppsIndex, targetVoltage, getPPSMaxCurrent(ppsIndex));
        return;
    }

    int8_t bestPDOIndex = -1; // Set -1 to indicate no PDO
    int bestPPSVoltage = -1; // Set -1 to indicate PPS is not used
    int bestPDOVoltageDist; // Tracks the distance between the target voltage and the voltage of the closest PDO

    // Step 2: Scan PDOs to see what is the closest voltage to the request
    for (byte i = 0; i < numPDO; i++)
    {
        int voltageDist;
        bool isPPS = isIndexPPS(i);
        int ppsVoltage = -1;

        if (isPPS) 
        {
            int minVoltage = getPPSMinVoltage(i);
            int maxVoltage = getPPSMaxVoltage(i);
            ppsVoltage = maxVoltage; // Default to max voltage

            // If minVoltage is closer to targetVoltage than maxVoltage set the ppsVoltage used to minVoltage
            if (abs(minVoltage - targetVoltage) < abs(maxVoltage - targetVoltage)) {
                ppsVoltage = minVoltage;
            }
            voltageDist = abs(ppsVoltage - targetVoltage);            
        }
        else
        {
            voltageDist = abs(getPDOVoltage(i) - targetVoltage);
        }

        if (bestPDOIndex == -1 || voltageDist < bestPDOVoltageDist) {
            bestPDOIndex = i;
            bestPDOVoltageDist = voltageDist;
            bestPPSVoltage = ppsVoltage;
        }
    }

    if (bestPDOIndex == -1) {
        return;
    }

    // Step 3: Set PDO
    // If no PPS voltage is defined, it is a fixed PDO
    if (bestPPSVoltage == -1) {
        setPDO(bestPDOIndex);
    } else {
        setPPSPDO(bestPDOIndex, bestPPSVoltage, getPPSMaxCurrent(bestPDOIndex));
        
    }
}

/**
 * @brief Request PDO profile.
 * @param PDOindex start from index 0 to (PDONum - 1) if no PPS, (PDONum -2) if PPS found
 */
void AP33772::setPDO(uint8_t PDOindex)
{
    uint8_t guarding;

    if (numPPS > 0)
        {guarding = numPDO - numPPS - 1;}
    else
        guarding = numPDO - 1; // Example array[4] only exist index 0,1,2,3

    if (PDOindex <= guarding)
    {
        indexPDO = PDOindex;
        rdoData.fixed.objPosition = PDOindex + 1; // Index 0 to Index 1
        rdoData.fixed.maxCurrent = pdoData[PDOindex].fixed.maxCurrent;
        rdoData.fixed.opCurrent = pdoData[PDOindex].fixed.maxCurrent;
        writeRDO();
    }
}

/**
 * @brief Request PPS PDO profile at a target voltage and maximum current.
 * @param targetVoltage in mV
 * @param targetMaxCurrent in mA
 */
void AP33772::setPPSPDO(uint8_t PPSindex, int targetVoltage, int maxCurrent)
{
    uint8_t guarding;

    if (numPPS == 0) {
        return;
    }
    else {
        guarding = numPDO - numPPS - 1; // Uses the assumption from setPDO that PPS PDOs are always at the end
    }

    if (PPSindex > guarding)
    {
        indexPDO = PPSindex;
        reqPpsVolt = targetVoltage / 20;
        rdoData.pps.objPosition = PPSindex + 1; // index 1
        rdoData.pps.opCurrent = maxCurrent / 50; // 50mA/LBS
        rdoData.pps.voltage = reqPpsVolt;
        writeRDO();
    }
}

/**
 * @brief Set max current before tripping at wall plug
 * @param targetMaxCurrent in mA
 */
void AP33772::setMaxCurrent(int targetMaxCurrent)
{
    /*
    Step 1: Check if current profile is PPS, check if max current is lower than request
        If yes, set new max current
        If no, report fault
    Step 2: If profile is PDO, check if max current is lower than request
        If yes, set new max current
        If no, report fault
    */
    if (isIndexPPS(indexPDO))
    {
        if (targetMaxCurrent <= pdoData[indexPDO].pps.maxCurrent * 50)
        {
            rdoData.pps.objPosition = indexPDO + 1;        // index 1
            rdoData.pps.opCurrent = targetMaxCurrent / 50; // 50mA/LBS
            rdoData.pps.voltage = reqPpsVolt;
            writeRDO();
        }
        else
        {
        } // Do nothing
    }
    else
    {
        if (targetMaxCurrent <= pdoData[indexPDO].fixed.maxCurrent * 10)
        {
            rdoData.fixed.objPosition = indexPDO + 1;         // Index 0 to Index 1
            rdoData.fixed.maxCurrent = targetMaxCurrent / 10; // 10mA/LBS
            rdoData.fixed.opCurrent = targetMaxCurrent / 10;  // 10mA/LBS
            writeRDO();
        }
        else
        {
        } // Do nothing
    }
}

/**
 * @brief Set resistance value of 10K NTC at 25C, 50C, 75C and 100C.
 *          Default is 10000, 4161, 1928, 974Ohm
 * @param TR25, TR50, TR75, TR100 unit in Ohm
 * @attention Blocking function due to long I2C write, min blocking time 15ms
 */
void AP33772::setNTC(int TR25, int TR50, int TR75, int TR100) // Parameter NOT DONE
{
    writeBuf[0] = TR25 & 0xff;
    writeBuf[1] = (TR25 >> 8) & 0xff;
    i2c_write(AP33772_ADDRESS, 0x28, 2);
    delay(5);
    writeBuf[0] = TR50 & 0xff;
    writeBuf[1] = (TR50 >> 8) & 0xff;
    i2c_write(AP33772_ADDRESS, 0x2A, 2);
    delay(5);
    writeBuf[0] = TR75 & 0xff;
    writeBuf[1] = (TR75 >> 8) & 0xff;
    i2c_write(AP33772_ADDRESS, 0x2C, 2);
    delay(5);
    writeBuf[0] = TR100 & 0xff;
    writeBuf[1] = (TR100 >> 8) & 0xff;
    i2c_write(AP33772_ADDRESS, 0x2E, 2);
}

/**
 * @brief Set target temperature (C) when output power through USB-C is reduced
 *          Default is 120 C
 * @param temperature (unit in Celcius)
 */
void AP33772::setDeratingTemp(int temperature)
{
    writeBuf[0] = temperature;
    i2c_write(AP33772_ADDRESS, CMD_DRTHR, 1);
}

void AP33772::setMask(AP33772_MASK flag)
{
    // First read in what is currently in the MASK
    i2c_read(AP33772_ADDRESS, CMD_MASK, 1);
    writeBuf[0] = readBuf[0] | flag;
    delay(5); // Short break between read/write
    i2c_write(AP33772_ADDRESS, CMD_MASK, 1);
}

void AP33772::clearMask(AP33772_MASK flag)
{
    // First read in what is currently in the MASK
    i2c_read(AP33772_ADDRESS, CMD_MASK, 1);
    writeBuf[0] = readBuf[0] & ~flag;
    delay(5); // Short break between read/write
    i2c_write(AP33772_ADDRESS, CMD_MASK, 1);
}

void AP33772::i2c_read(byte slvAddr, byte cmdAddr, byte len)
{
    // clear readBuffer
    for (byte i = 0; i < READ_BUFF_LENGTH; i++)
    {
        readBuf[i] = 0;
    }
    byte i = 0;
    Wire.beginTransmission(slvAddr); // transmit to device SLAVE_ADDRESS
    Wire.write(cmdAddr);             // sets the command register
    Wire.endTransmission();          // stop transmitting

    Wire.requestFrom(slvAddr, len); // request len bytes from peripheral device
    if (len <= Wire.available())
    { // if len bytes were received
        while (Wire.available())
        {
            readBuf[i] = (byte)Wire.read();
            i++;
        }
    }
}

void AP33772::i2c_write(byte slvAddr, byte cmdAddr, byte len)
{
    Wire.beginTransmission(slvAddr); // transmit to device SLAVE_ADDRESS
    Wire.write(cmdAddr);             // sets the command register
    Wire.write(writeBuf, len);       // write data with len
    Wire.endTransmission();          // stop transmitting

    // clear readBuffer
    for (byte i = 0; i < WRITE_BUFF_LENGTH; i++)
    {
        writeBuf[i] = 0;
    }
}

/**
 * @brief Write the desire power profile back to the power source
 */
void AP33772::writeRDO()
{
    writeBuf[3] = rdoData.byte3;
    writeBuf[2] = rdoData.byte2;
    writeBuf[1] = rdoData.byte1;
    writeBuf[0] = rdoData.byte0;
    i2c_write(AP33772_ADDRESS, CMD_RDO, 4); // CMD: Write RDO
}

/**
 * @brief Read VBUS voltage
 * @return voltage in mV
 */
int AP33772::readVoltage()
{
    i2c_read(AP33772_ADDRESS, CMD_VOLTAGE, 1);
    return readBuf[0] * 80; // I2C read return 80mV/LSB
}

/**
 * @brief Read VBUS current
 * @return current in mA
 */
int AP33772::readCurrent()
{
    i2c_read(AP33772_ADDRESS, CMD_CURRENT, 1);
    return readBuf[0] * 24; // I2C read return 24mA/LSB
}

/**
 * @brief Read maximum VBUS current
 * @return current in mA
 */
int AP33772::getMaxCurrent()
{
    if (isIndexPPS(indexPDO))
    {
        return pdoData[indexPDO].pps.maxCurrent * 50;
    }
    else
    {
        return pdoData[indexPDO].fixed.maxCurrent * 10;
    }
}

/**
 * @brief Read NTC temperature
 * @return tempearture in C
 */
int AP33772::readTemp()
{
    i2c_read(AP33772_ADDRESS, CMD_TEMP, 1);
    return readBuf[0]; // I2C read return 1C/LSB
}

/**
 * @brief Hard reset the power supply. Will temporary cause power outage
 */
void AP33772::reset()
{
    writeBuf[0] = 0x00;
    writeBuf[1] = 0x00;
    writeBuf[2] = 0x00;
    writeBuf[3] = 0x00;
    i2c_write(AP33772_ADDRESS, CMD_RDO, 4);
}

/**
 * @brief Debug code to quickly check power supply profile PDOs
 */
void AP33772::printPDO()
{
    Serial.print("Source PDO Number = ");
    Serial.print(numPDO);
    Serial.println();

    for (byte i = 0; i < numPDO; i++)
    {
        if ((pdoData[i].byte3 & 0xF0) == 0xC0) // PPS PDO
        {
            Serial.print("PDO[");
            Serial.print(i + 1); // PDO position start from 1
            Serial.print("] - PPS : ");
            Serial.print((float)(pdoData[i].pps.minVoltage) * 100 / 1000);
            Serial.print("V~");
            Serial.print((float)(pdoData[i].pps.maxVoltage) * 100 / 1000);
            Serial.print("V @ ");
            Serial.print((float)(pdoData[i].pps.maxCurrent) * 50 / 1000);
            Serial.println("A");
        }
        else if ((pdoData[i].byte3 & 0xC0) == 0x00) // Fixed PDO
        {
            Serial.print("PDO[");
            Serial.print(i + 1);
            Serial.print("] - Fixed : ");
            Serial.print((float)(pdoData[i].fixed.voltage) * 50 / 1000);
            Serial.print("V @ ");
            Serial.print((float)(pdoData[i].fixed.maxCurrent) * 10 / 1000);
            Serial.println("A");
        }
    }
    Serial.println("===============================================");
}

/**
 * Add on for PPS Bench Power Supply
 */

/**
 * @brief Get the number of power profile, include PPS if exist
 */
int AP33772::getNumPDO()
{
    return numPDO;
}

/**
 * @brief Get index of the first PPS profile
 */
int AP33772::getPPSIndex()
{
    return PPSindices[0] ? numPPS > 0 : -1;
}

/**
 * @brief Get the index of the first PPS PDO that can provide the target voltage and target current. 
 * @param targetVoltage in mV
 * @param targetCurrent in mA
 * @return The index of the suitable PPS PDO. Returns -1 if no suitable PPS PDO exists
 */
int AP33772::getPPSIndexByVoltageCurrent(int targetVoltage, int targetCurrent)
{
    for (int i = 0; i < numPPS; i++) {
        int8_t ppsIndex = PPSindices[i];
        if (pdoData[ppsIndex].pps.maxVoltage * 100 >= targetVoltage && pdoData[ppsIndex].pps.minVoltage * 100 <= targetVoltage && pdoData[ppsIndex].pps.maxCurrent * 50 >= targetCurrent) {
            return ppsIndex;
        }
    }
    return -1;
}

/**
 * @brief Checks if the provided index is a PPS PDO index
 * @param index The PDO index to be checked
 * @return True if the index is a PPS PDO, otherwise False
 */
bool AP33772::isIndexPPS(uint8_t index) {
    for (int i = 0; i < numPPS; i++) {
        if (PPSindices[i] == index) {
            return true;
        }
    }
    return false;
}

/**
 * @brief MaxCurrent for fixed voltage PDO
 * @return Current in mAmp
 */
int AP33772::getPDOMaxcurrent(uint8_t PDOindex)
{
    return pdoData[PDOindex].fixed.maxCurrent * 10;
}

/**
 * @brief Get fixed PDO voltage
 * @return Voltage in mVolt
 */
int AP33772::getPDOVoltage(uint8_t PDOindex)
{
    return pdoData[PDOindex].fixed.voltage * 50;
}

/**
 * @brief Get PPS min votlage
 * @return Voltage in mVolt
 */
int AP33772::getPPSMinVoltage(uint8_t PPSindex)
{
    return pdoData[PPSindex].pps.minVoltage * 100;
}

/**
 * @brief Get PPS max votlage
 * @return Voltage in mVolt
 */
int AP33772::getPPSMaxVoltage(uint8_t PPSindex)
{
    return pdoData[PPSindex].pps.maxVoltage * 100;
}

/**
 * @brief Get PPS max current
 * @return Current in mAmp
 */
int AP33772::getPPSMaxCurrent(uint8_t PPSindex)
{
    return pdoData[PPSindex].pps.maxCurrent * 50;
}