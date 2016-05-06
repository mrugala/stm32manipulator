/****************************************************************
 * Program sterowania manipulatora na podstawie otrzymanych     *
 * przez UART sygna³ów z akcelerometru, mo¿liwa tak¿e opcja     *
 * sterowania przy pomocy komputera (jako dane wejœciowe s³u¿¹  *
 * w takim wypadku wspó³rzêdne [mm].                            *
 *                                                              *
 * Autor: Jêdrzej Mruga³a                                       *
 *                                                              *
 ****************************************************************/
#include "periph.h"                 // inicjalizacja i sterowanie peryferiami
#include "servo.h"                  // ruch serwomechanizmow (wykorzystuje periph.h)
#include "comm.h"                   // komunikacja USART
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "zodm.h"
#include "stm32f4_adc.h"

#define ever (;;)

// __attribute__((unused)) static MPU6050_Data_t MPU6050Data, MPU6050Mean;
// __attribute__((unused)) static MPU6050_Data_t globalMPUData;
static input_coord_t globalInputData, __attribute__((unused)) measData;
static float robotAngles[] = { .0f, .0f, .0f, .0f, .0f };
static float globalGData[3] = { 0 };
static float globalAData[3] = { 0 };
static float globalAOffset[3] = { 0 };
static float globalVData[3] = { 0 };
static matrix_t globalRotData = { 3, 3, {0} };
static float globalADCData[4] = { 0 };
static uint8_t globalADCAddr =  0;

static bool is_data_ok = false, is_rot_received = false, are_coords_calculated = false;
static bool accel_data_ready = false, grav_data_ready = false, offset_calculated = false;

static unsigned data_samples = 0;

#define INPUT_DATA_CALCULATED (is_rot_received && are_coords_calculated)
#define ACCEL_GRAV_RECEIVED (accel_data_ready && grav_data_ready)

#define G_KRK 9.8105

#define SYSCLK_FREQ 1000

/*
void mean() {
    MPU6050Mean = divideData(MPU6050Mean, samplesForMean);
    globalMPUData = MPU6050Mean;
    MPU6050Mean = zeroData(MPU6050Mean);
}

void noiseAndGravityFilter() {
    static uint8_t it = 0;
    uint8_t i;
    static float vectX[samplesForA] = { 0 }, vectY[samplesForA] = { 0 }, vectZ[samplesForA] = { 0 };
    float sum[3] = { 0 };

    vectX[it] = globalMPUData.AccelX-globalGData[0];
    vectY[it] = globalMPUData.AccelY-globalGData[1];
    vectZ[it] = globalMPUData.AccelZ-globalGData[2];

    ++it;
    if (it == samplesForA)
        it = 0;

    for(i = 0; i < samplesForA; ++i)
    {
        sum[0] += vectX[i];
        sum[1] += vectY[i];
        sum[2] += vectZ[i];
    }

    globalInputData.x_mm = sum[0]/samplesForA;
    globalInputData.x_mm = sum[1]/samplesForA;
    globalInputData.x_mm = sum[2]/samplesForA;
}

void accel() {
    static float accelVect[3] = { 0 };
    static float accelData[3] = { 0 };
    static float accelMeanData[3] = { 0 };
    static float accel_ms2[3] = { 0 };


    accelVect[0] = globalMPUData.AccelX;
    accelVect[1] = globalMPUData.AccelY;
    accelVect[2] = globalMPUData.AccelZ;

    accelData[0] = korxa(accelVect, globalInputData.alpha_deg, globalInputData.beta_deg, globalInputData.gamma_deg);
    accelData[1] = korya(accelVect, globalInputData.alpha_deg, globalInputData.beta_deg, globalInputData.gamma_deg);
    accelData[2] = korza(accelVect, globalInputData.alpha_deg, globalInputData.beta_deg, globalInputData.gamma_deg) - 1;

    accelMeanData[0] = 0.9*accelData[0] + 0.1*accelMeanData[0];
    accelMeanData[1] = 0.9*accelData[1] + 0.1*accelMeanData[1];
    accelMeanData[2] = 0.9*accelData[2] + 0.1*accelMeanData[2];

    accel_ms2[0] = (accelData[0] - accelMeanData[0])*9.8105;
    accel_ms2[1] = (accelData[1] - accelMeanData[1])*9.8105;
    accel_ms2[2] = (accelData[2] - accelMeanData[2])*9.8105;

    globalAData[0] = 0.2*accel_ms2[0] + 0.8*globalAData[0];
    globalAData[1] = 0.2*accel_ms2[1] + 0.8*globalAData[1];
    globalAData[2] = 0.2*accel_ms2[2] + 0.8*globalAData[2];
}

void velocity() {
    static float velocityVect[3] = { 0 };
    static float velocityData[3] = { 0 };
    static float velocityMeanData[3] = { 0 };

    velocityVect[0] += 0.1*globalAData[0];
    velocityVect[1] += 0.1*globalAData[1];
    velocityVect[2] += 0.1*globalAData[2];


    velocityMeanData[0] = 0.9*velocityVect[0] + 0.1*velocityMeanData[0];
    velocityMeanData[1] = 0.9*velocityVect[1] + 0.1*velocityMeanData[1];
    velocityMeanData[2] = 0.9*velocityVect[2] + 0.1*velocityMeanData[2];

    velocityData[0] = (velocityVect[0] - velocityMeanData[0]);
    velocityData[1] = (velocityVect[1] - velocityMeanData[1]);
    velocityData[2] = (velocityVect[2] - velocityMeanData[2]);

    globalVData[0] = 0.1*velocityData[0] + 0.9*globalVData[0];
    globalVData[1] = 0.1*velocityData[1] + 0.9*globalVData[1];
    globalVData[2] = 0.1*velocityData[2] + 0.9*globalVData[2];
}

void coord() {
    static int i = 0;
    if(i < 50)
    {
        globalInputData.x_mm = 0;
        globalInputData.y_mm = 0;
        globalInputData.z_mm = 0;
        ++i;
    }
    else
    {
        globalInputData.x_mm += globalVData[0];
        globalInputData.y_mm += globalVData[1];
        globalInputData.z_mm += globalVData[2];
    }
}

void angles() {
    static float drift0;
    static float gyroZ;
    static uint8_t i = 0;
    static float alfa = 0.3;
    static float alfa2 = 0.2;

    if(i == 0) {
        printToUSART(USART3,"drift\n");
        i = 1;
        driftZ = globalMPUData.GyroZ;
        gyroZ = globalMPUData.GyroZ;
        drift0 = driftZ;
    }

    if(globalMPUData.GyroZ >= driftZ - 8.0 && globalMPUData.GyroZ <= driftZ + 8.0)
        driftZ = (1.0-0.01) * driftZ + 0.01 * globalMPUData.GyroZ;

    gyroZ = (1.0-alfa2)*gyroZ + alfa2 * globalMPUData.GyroZ;

    globalInputData.alpha_deg = filterFi1(globalMPUData.AccelY, globalMPUData.AccelZ, globalMPUData.GyroX, &measData.alpha_deg);
    globalInputData.beta_deg = - filterFi2(globalMPUData.AccelX, globalMPUData.AccelZ, globalMPUData.GyroY, &measData.beta_deg);
    globalInputData.gamma_deg += (globalMPUData.GyroZ - driftZ)/2.0*(float)samplesForMean/1000.0;

}
*/

void calculateOffsets()
{
    if(!ACCEL_GRAV_RECEIVED) return;

    int i;
    for(i = 0; i < 3; i++)
    {
        globalAOffset[i] = globalAData[i] - globalGData[i]/G_KRK;
    }
    accel_data_ready = false;
    grav_data_ready = false;
    offset_calculated = true;
}

void calculateCoordinates()
{
    if(!ACCEL_GRAV_RECEIVED || !offset_calculated) return;
    float *coord; 

    int i;
    for(i = 0; i < 3; i++)
    {
        switch(i)
        {
            case 0:
                coord = &globalInputData.x_mm;
                break;
            case 1:
                coord = &globalInputData.y_mm;
                break;
            case 2:
                coord = &globalInputData.z_mm;
                break;
            default: break;
        }
        globalVData[i] += ((globalAData[i] - globalAOffset[i])*G_KRK - globalGData[i])/SYSCLK_FREQ*data_samples;
        *coord += globalVData[i]/SYSCLK_FREQ*data_samples/1000;
    }
    data_samples = 0;
    accel_data_ready = false;
    grav_data_ready = false;
    are_coords_calculated = true;
}

void calculateAngles() {
    if(INPUT_DATA_CALCULATED)
    {
        if(!calc_zodm_5_rot(&globalRotData, &globalInputData, robotAngles))
        {
            is_data_ok = true;
            are_coords_calculated = false;
            is_rot_received = false;
        }
        else
        {
            is_data_ok = false;
            printToUSART(USART3, "angles not valid");
        }
    }

}

void printADCData() {
    char str[128];
//  sprintf(str, "%d %3.2f %3.2f %3.2f %3.8f %3.8f %3.8f %3.2f %3.2f %3.2f %3.2f %3.2f %3.2f %3.2f %3.2f %3.2f\n", globalADCAddr, globalInputData.fiX, globalInputData.fiY, globalInputData.fiZ,
//          globalInputData.x, globalInputData.y, globalInputData.z, globalADCData[0], globalADCData[1], globalADCData[2], globalADCData[3], robotAngles.fi1, robotAngles.fi2, robotAngles.fi3, robotAngles.fi4, robotAngles.fi5);

    sprintf(str, "%d %3.2f %3.2f %3.2f %3.2f\n", globalADCAddr, globalADCData[0], globalADCData[1], globalADCData[2], globalADCData[3]);

//  sprintf(str, "gyro/kalman: %3.2f %3.2f %3.4f\n", globalMPUData.GyroZ, driftZ, globalInputData.fiZ);
    printToUSART(USART3,str);


//  sprintf(str, "G: %3.4f %3.4f %3.4f  ", globalGData[0], globalGData[1], globalGData[2]);
//  printToUSART(USART3,str);
//  sprintf(str, "kalman: %3.4f %3.4f %3.4f\n", globalInputData.fiX, globalInputData.fiY, globalInputData.fiZ);
//  printToUSART(USART3,str);

}

void receivePath() {
    int i;
    if(UARTStringReceived) {
        if(!strcmp(UARTdataRx,"mode1")) {}
            //TODO: do nothing
        else {
            for(i = 0; i < bytesReceived; ++i) {
                if(UARTdataRx[i] == 'w') globalInputData.x_mm += 10;
                if(UARTdataRx[i] == 's') globalInputData.x_mm -= 10;
                if(UARTdataRx[i] == 'a') globalInputData.y_mm += 10;
                if(UARTdataRx[i] == 'd') globalInputData.y_mm -= 10;
                if(UARTdataRx[i] == 'r') globalInputData.z_mm += 10;
                if(UARTdataRx[i] == 'f') globalInputData.z_mm -= 10;
            }
        }
        UARTStringReceived = false;
    }
}

void receiveDMPdata() {
    int status;
    int i;
    uint32_t temp;
    if(GravStringReceived) {
        status = sscanf(GravStringRx, "%*s %*s %f %f %f", &globalGData[0], &globalGData[1], &globalGData[2]);
        if(status != 3)
            printToUSART(USART3, "Gravity Vector sscanf error.\n");

        GravStringReceived = false;
        grav_data_ready = true;
    }
    if(AccelDataReceived) {
        int32_t temp_accel;
        for(i = 0; i < 3; ++i)
        {
            temp = ((uint32_t)DMPdataRx[DATA_OFFSET + i*ACCEL_DATA_LENGTH] << 24)
                 + ((uint32_t)DMPdataRx[DATA_OFFSET + i*ACCEL_DATA_LENGTH + 1] << 16)
                 + ((uint32_t)DMPdataRx[DATA_OFFSET + i*ACCEL_DATA_LENGTH + 2] << 16)
                 + ((uint32_t)DMPdataRx[DATA_OFFSET + i*ACCEL_DATA_LENGTH + 3]);
            if(temp > (uint32_t)2147483648)
                temp_accel = temp - (uint32_t)4294967296;
            else
                temp_accel = temp;
            globalAData[i] = (float)temp_accel/(1<<14);
        }
        AccelDataReceived = false;
        accel_data_ready = true;
    }
    if(GyroDataReceived) {
        int32_t temp_gyro;
        for(i = 0; i < 3; ++i)
        {
            temp = ((uint32_t)DMPdataRx[DATA_OFFSET + i*GYRO_DATA_LENGTH] << 24)
                 + ((uint32_t)DMPdataRx[DATA_OFFSET + i*GYRO_DATA_LENGTH + 1] << 16)
                 + ((uint32_t)DMPdataRx[DATA_OFFSET + i*GYRO_DATA_LENGTH + 2] << 16)
                 + ((uint32_t)DMPdataRx[DATA_OFFSET + i*GYRO_DATA_LENGTH + 3]);
            if(temp > (uint32_t)2147483648)
                temp_gyro = temp - (uint32_t)4294967296;
            else
                temp_gyro = temp;
            globalAData[i] = (float)temp_gyro/(1<<14);
        }
        GyroDataReceived = false;
    }
    if(RotDataReceived) {
        int16_t temp_rot;
        for(i = 0; i < 9; ++i)
        {
            temp = ((uint16_t)DMPdataRx[DATA_OFFSET + i*ROT_DATA_LENGTH] << 8)
                 + ((uint16_t)DMPdataRx[DATA_OFFSET + i*ROT_DATA_LENGTH + 1]);
            if(temp > (uint16_t)32767)
                temp_rot = temp - (uint16_t)65536;
            else
                temp_rot = temp;
            *access_matrix_cell(&globalRotData, i/3, i%3) = (float)temp_rot/(1<<14);
        }
        RotDataReceived = false;
        is_rot_received = true;
    }
}

/*
void moveServoTest() {
    static int i = 0;
    uint16_t dataRead;
    float dataFloat0;

    Move (1, (i-90)*1.0, HD_1201MG);
    Move (2, (i-90)*1.0, HD_1201MG);
    Move (3, (i-90)*1.0, HD_1201MG);
    Move (4, (i-90)*1.0, HD_1201MG);
    Move (5, (i-90)*1.0, HD_1201MG);
    Move (6, (i-90)*1.0, HD_1201MG);
    Move (7, (i-90)*1.0, HD_1201MG);
    Move (8, (i-90)*1.0, HD_1201MG);
    Move (9, (i-90)*1.0, HD_1201MG);

    Delayms(3000);

    dataRead = TM_ADC_Read(ADC1, 0);
    dataFloat0 = (float)dataRead*3.0/0xFFF;
    char a[256];
    sprintf(a,"ADC0: %3.4f krok: %d\n",dataFloat0, i);
    printToUSART(USART3, a);

    ++i;
    if(i == 180)
        i = 0;
}
*/

void moveRobot() {
    if(!is_data_ok) return;

    Move (1, robotAngles[0], HD_1501MG);
    Move (2, robotAngles[1], HD_1501MG);
    Move (3, robotAngles[2], HD_1201MG);
    Move (4, robotAngles[3], HD_1201MG);
    Move (5, robotAngles[4], HS_645MG);

    char a[256];
    sprintf(a,"robot moved, angles: %.2f, %.2f, %.2f, %.2f, %.2f",
            robotAngles[0], robotAngles[1], robotAngles[2], robotAngles[3], robotAngles[4]);
    printToUSART(USART3, a);

    is_data_ok = false;
}

void testADC() {
    uint16_t dataRead;
    float dataFloat0, dataFloat1;

    dataRead = TM_ADC_Read(ADC1, 0);
    dataFloat0 = (float)dataRead*3.0/0xFFF;
    dataRead = TM_ADC_Read(ADC1, 1);
    dataFloat1 = (float)dataRead*3.0/0xFFF;
    char a[256];
    sprintf(a,"ADC0: %3.4f ADC1: %3.4f\n",dataFloat0, dataFloat1);
    printToUSART(USART3, a);
}

void readADC() {
    uint16_t dataRead;

    ++globalADCAddr;
    if(globalADCAddr > 7)
    {
        globalADCAddr = 0;
    }

    setADCaddr(globalADCAddr);

    dataRead = TM_ADC_Read(ADC1, 3);
    globalADCData[0] = (float)dataRead*3.2/0xFFF;
    dataRead = TM_ADC_Read(ADC1, 5);
    globalADCData[1] = (float)dataRead*3.2/0xFFF;
    dataRead = TM_ADC_Read(ADC1, 7);
    globalADCData[2] = (float)dataRead*3.2/0xFFF;
    dataRead = TM_ADC_Read(ADC1, 1);
    globalADCData[3] = (float)dataRead*3.2/0xFFF;
}

void executeOnTick(uint16_t *tick, uint16_t when, void (*func)()) {
    *tick=*tick+1;
    if (*tick >= when) {
        (*func)();
        *tick = 0;
    }
}

void executeOnTickWithFlag(uint16_t *tick, uint16_t when, void (*func)(int), uint8_t flag) {
    *tick=*tick+1;
    if (*tick >= when) {
        (*func)(flag);
        *tick = 0;
    }
}

int main(void)
{
    __attribute__((unused)) uint8_t sensorOK = 0;
    __attribute__((unused)) uint16_t startup = 0;
    uint16_t ticksA=0, ticksRecvDMP=0, ticksPrint=0, ticksFi=0, ticksMove=0, ticksADC=0;

    // MPU6050_Config_t MPU6050Config = {MPU6050_DEV_0, MPU6050_Gyroscope_500s, MPU6050_Accelerometer_2G};

    waitForHSEStartUp();

//  initUserButton();
//  initLEDs();
    initADC();
    initSysTick();
//  PWMLEDInit();
    initServoPins();
    initUSART3(115200);
    initUSART1(115200);
    initServos();

    // if (initMPU6050(MPU6050Config) == MPU6050_Result_Ok) {
    //     printToUSART(USART3, "MPU6050 ready.\n");
    //     sensorOK = 1;
    // }
    // else {
    //     printToUSART(USART3, "MPU6050 error.\n");
    // }

    printToUSART(USART1, "agor\n"); //init motion driver to send raw accel, gyro, gravity and rotation data

    do
    {
        calculateOffsets();  
    }
    while(!offset_calculated);

    for ever 
    {
        if (Tickms(1))
        {
            // if(/*sensorOK*/ 1)
            // {
                // MPU6050Data = readMPU6050Data(MPU6050Config.Address);
                // MPU6050Mean = addData(MPU6050Mean, MPU6050Data);
                // executeOnTick(&ticksMean, samplesForMean, mean);
                // executeOnTick(&ticksA, samplesForMean, accel);
                // executeOnTick(&ticksFi, samplesForMean, angles);
                // executeOnTick(&ticksS, samplesForMean, coord);
                // executeOnTick(&ticksADC, 1000, readADC);
                // executeOnTick(&ticksPrint, 1000, printADCData);
                // if(startup<2000)
                //     ++startup;
                // else {
                //     //executeOnTick(&ticksV, samplesForMean, velocity);
                // }

            // }
            data_samples++;
            executeOnTick(&ticksADC, 1000, readADC);
            executeOnTick(&ticksPrint, 1000, printADCData);
            executeOnTick(&ticksRecvDMP, 1, receiveDMPdata);
            executeOnTick(&ticksA, 5, calculateCoordinates);
            executeOnTick(&ticksFi, 5, calculateAngles);
            executeOnTick(&ticksMove, 20, moveRobot);
//              executeOnTick(&ticksADC, 1000, testADC);

//              executeOnTick(&ticksIn, samplesForMean, inputData);
//              executeOnTick(&ticksOut, samplesForMean, outputData);
//              executeOnTick(&ticksPrint, 1000, printAllData);
//              executeOnTick(&ticksRecv, 1000, receivePath);

//          }
        }
    }
}
