/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "usb_device.h"
#include <string.h>
#include <stdint.h>
#define LORA_NSS_PORT GPIOA
#define LORA_NSS_PIN  GPIO_PIN_4

#define LORA_RST_PORT GPIOB
#define LORA_RST_PIN  GPIO_PIN_0

#define LORA_DIO0_PIN GPIO_PIN_1
#define MPU6050_ADD (0x68 << 1)
#define PWR_MGMT_1 0x6B
#define ACCL_START 0x3B
#define CALIB_SAMPLES 2000
#define SAMPLE_DELAY 5
#include "usbd_cdc_if.h"
#include "math.h"
extern uint8_t CDC_Transmit_FS(uint8_t* Buf, uint16_t Len);

#define MS5611_ADDR (0x76 << 1)  // REMOVE << 1 ! Standard MS5611 address
#define MS5611_RESET     0x1E
#define MS5611_ADC_READ  0x00
#define MS5611_CONV_D1   0x48   // Pressure conversion
#define MS5611_CONV_D2   0x58   // Temperature conversion
#define MS5611_PROM_READ 0xA2
#define LED_PORT GPIOC
#define LED_PIN  GPIO_PIN_13
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
float kalman_alt = 0;
float kalman_vel = 0;
uint8_t armed = 0;
uint8_t released = 0;
uint8_t servo_done = 0;
uint32_t drop_time = 0;

float P[2][2] = {{1,0},{0,1}};   // Error covariance
float Q[2][2] = {{0.15,0},{0,0.15}};
float R = 1.0; // Measurement noise (MS5611 noise)

float dt = 0.02; // 50 Hz loop
uint8_t raw_data[14];
int16_t Ax,Ay,Az;
int16_t Gx,Gy,Gz;
int32_t Ax_offset=0, Ay_offset=0, Az_offset=0;
int32_t Gx_offset=0, Gy_offset=0, Gz_offset=0;
float Ax_g, Ay_g, Az_g;
float Gx_dps, Gy_dps, Gz_dps;
uint16_t C[7];  // Calibration constants
uint32_t D1, D2;
float pressure = 0;
float temperature = 0;
float altitude = 0;
float pressure_baseline = 0;
uint8_t gps_data;
float altitude_offset = 0;
char gps_buffer[120];
int gps_index = 0;
int satellites = 0;
int gps_fix = 0;
float latitude = 0;
float longitude = 0;
uint16_t lora_packet = 0;
float gps_altitude = 0;
char teamID[] = "TEAM1";
char state[16] = "IDLE"; // Add your team ID
int time = 0;
int packetCount = 0;
float voltage = 3.3f;  // Placeholder - add ADC later

/* USER CODE END PV */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE BEGIN PFP */
void usb_print(char *msg);  // forward declaration
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

extern SPI_HandleTypeDef hspi1;

void lora_select()
{
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET);
    HAL_Delay(1);
}

void lora_unselect()
{
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);
    HAL_Delay(1);
}

uint8_t lora_read(uint8_t reg)
{
    uint8_t tx[2];
    uint8_t rx[2];

    tx[0] = reg & 0x7F;
    tx[1] = 0x00;

    lora_select();
    HAL_SPI_TransmitReceive(&hspi1, tx, rx, 2, 100);
    lora_unselect();

    return rx[1];
}

void led_blink(int times, int delay_ms)
{
    for(int i = 0; i < times; i++)
    {
        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET); // ON
        HAL_Delay(delay_ms);

        HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);   // OFF
        HAL_Delay(delay_ms);
    }
    HAL_Delay(500);
}

void error_blink(int code)
{
    while(1)
    {
        led_blink(code, 200);
        HAL_Delay(1000);
    }
}

void lora_write(uint8_t reg, uint8_t data)
{
    uint8_t buf[2];
    buf[0] = reg | 0x80;
    buf[1] = data;

    lora_select();
    HAL_SPI_Transmit(&hspi1, buf, 2, 100);
    lora_unselect();
}

void lora_reset()
{
    HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(LORA_RST_PORT, LORA_RST_PIN, GPIO_PIN_SET);
    HAL_Delay(10);
}

void servo_lock(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 1500);   // locked position
}

void servo_open(void)
{
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 2000);   // open position
}

void lora_init()
{
    lora_reset();

    // LoRa sleep
    lora_write(0x01, 0x80);
    HAL_Delay(10);

    // LoRa standby
    lora_write(0x01, 0x81);
    HAL_Delay(10);

    // Frequency = 433.000 MHz
    lora_write(0x06, 0x6C);
    lora_write(0x07, 0x80);
    lora_write(0x08, 0x00);

    // PA config: max power
    lora_write(0x09, 0x8F);

    // LNA boost
    lora_write(0x0C, 0x23);

    // FIFO base addresses
    lora_write(0x0E, 0x00);   // TX base
    lora_write(0x0F, 0x00);   // RX base

    // Explicit header, BW=125kHz, CR=4/5
    lora_write(0x1D, 0x72);

    // SF7, CRC ON
    lora_write(0x1E, 0x74);

    // AGC auto on
    lora_write(0x26, 0x04);

    // Preamble length = 8
    lora_write(0x20, 0x00);
    lora_write(0x21, 0x08);

    // Sync word = 0x12 (must match receiver)
    lora_write(0x39, 0x34);

    // DIO0 = TxDone
    lora_write(0x40, 0x40);

    // Clear all IRQ flags
    lora_write(0x12, 0xFF);

    // Standby
    lora_write(0x01, 0x81);
    HAL_Delay(10);
}

void ms5611_init() {
    uint8_t cmd;

    // Reset
    cmd = MS5611_RESET;
    HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDR, &cmd, 1, 100);
    HAL_Delay(20);

    uint8_t buf[2];
    for (int i = 0; i < 6; i++) {
        cmd = MS5611_PROM_READ + 2*i;   // A2, A4, A6, A8, AA, AC
        HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDR, &cmd, 1, 100);
        HAL_I2C_Master_Receive(&hi2c1, MS5611_ADDR, buf, 2, 100);
        C[i+1] = (buf[0] << 8) | buf[1];

        char msg[30];
        sprintf(msg, "C%d=%u\r\n", i+1, C[i+1]);
        usb_print(msg);
    }
}

void kalman_update(float measured_alt, float accel_z)
{
    /* ===== Predict ===== */
    kalman_alt += kalman_vel * dt + 0.5f * accel_z * dt * dt;
    kalman_vel += accel_z * dt;

    P[0][0] += dt*(2*P[1][0] + dt*P[1][1]) + Q[0][0];
    P[0][1] += dt*P[1][1];
    P[1][0] += dt*P[1][1];
    P[1][1] += Q[1][1];

    /* ===== Update ===== */
    float y = measured_alt - kalman_alt;
    float S = P[0][0] + R;

    float K0 = P[0][0] / S;
    float K1 = P[1][0] / S;

    kalman_alt += K0 * y;
    kalman_vel += K1 * y;

    float P00 = P[0][0];
    float P01 = P[0][1];

    P[0][0] -= K0 * P00;
    P[0][1] -= K0 * P01;
    P[1][0] -= K1 * P00;
    P[1][1] -= K1 * P01;
}

void ms5611_read()
{
    uint8_t buf[3];
    uint8_t cmd;

    /* --- D1: Pressure --- */
    cmd = MS5611_CONV_D1;
    HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDR, &cmd, 1, 100);
    HAL_Delay(15);  // increased from 12 to 15ms to be safe

    cmd = MS5611_ADC_READ;
    HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDR, &cmd, 1, 100);
    memset(buf, 0, 3);
    HAL_I2C_Master_Receive(&hi2c1, MS5611_ADDR, buf, 3, 100);
    uint32_t new_D1 = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];

    /* --- D2: Temperature --- */
    cmd = MS5611_CONV_D2;
    HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDR, &cmd, 1, 100);
    HAL_Delay(15);

    cmd = MS5611_ADC_READ;
    HAL_I2C_Master_Transmit(&hi2c1, MS5611_ADDR, &cmd, 1, 100);
    memset(buf, 0, 3);
    HAL_I2C_Master_Receive(&hi2c1, MS5611_ADDR, buf, 3, 100);
    uint32_t new_D2 = ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2];

    /* Reject zero reads (ADC not ready) - keep last valid value */
    if(new_D1 == 0 || new_D2 == 0) return;

    D1 = new_D1;
    D2 = new_D2;

    /* MS5611 compensation */
    int64_t dT   = (int64_t)D2 - ((int64_t)C[5] << 8);
    int64_t TEMP = 2000LL + (dT * (int64_t)C[6]) / (1LL << 23);

    int64_t OFF  = ((int64_t)C[2] << 16) + ((int64_t)C[4] * dT) / (1LL << 7);
    int64_t SENS = ((int64_t)C[1] << 15) + ((int64_t)C[3] * dT) / (1LL << 8);

    /* Second order compensation */
    if(TEMP < 2000)
    {
        int64_t T2    = (dT * dT) / (1LL << 31);
        int64_t diff  = TEMP - 2000LL;
        int64_t OFF2  = 5LL * diff * diff / 2LL;
        int64_t SENS2 = 5LL * diff * diff / 4LL;
        if(TEMP < -1500)
        {
            int64_t diff2  = TEMP + 1500LL;
            OFF2  += 7LL * diff2 * diff2;
            SENS2 += 11LL * diff2 * diff2 / 2LL;
        }
        TEMP -= T2;
        OFF  -= OFF2;
        SENS -= SENS2;
    }

    int64_t P = ((int64_t)D1 * SENS / (1LL << 21) - OFF) / (1LL << 15);

    float new_temp = (float)TEMP / 100.0f;
    float new_pres = (float)P    / 100.0f;

    /* Sanity check before accepting */
    if(new_pres > 300.0f && new_pres < 1100.0f &&
       new_temp > -40.0f && new_temp < 85.0f)
    {
        temperature = new_temp;
        pressure    = new_pres;
    }
    /* else: silently discard, keep last valid reading */
}

void i2c_scan()
{
    char msg[50];

    for(uint8_t addr = 1; addr < 128; addr++)
    {
        if(HAL_I2C_IsDeviceReady(&hi2c1, addr << 1, 1, 10) == HAL_OK)
        {
            sprintf(msg, "I2C Device Found at 0x%X\r\n", addr);
            usb_print(msg);
        }
    }
}
float convert_deg(float raw)
{
    int deg = (int)(raw / 100);
    float min = raw - (deg * 100);
    return deg + (min / 60.0);
}

float get_altitude(float press)
{
    return 44330.0 * (1.0 - pow(press / pressure_baseline, 0.1903));
}

void parse_gps(char *data)
{
    if(strncmp(data, "$GPGGA", 6) != 0 && strncmp(data, "$GNGGA", 6) != 0)
        return;

    char local[128];
    strncpy(local, data, sizeof(local) - 1);
    local[sizeof(local) - 1] = '\0';

    char  *token;
    int    field   = 0;
    float  raw_lat = 0, raw_lon = 0;
    char   lat_dir = 'N', lon_dir = 'E';
    int    fix = 0, sats = 0;
    float  alt = 0;

    token = strtok(local, ",");
    while(token != NULL)
    {
        field++;
        switch(field)
        {
            case 3:  raw_lat = atof(token); break;
            case 4:  lat_dir = token[0];    break;
            case 5:  raw_lon = atof(token); break;
            case 6:  lon_dir = token[0];    break;
            case 7:  fix     = atoi(token); break;
            case 8:  sats    = atoi(token); break;
            case 10: alt     = atof(token); break;
        }
        token = strtok(NULL, ",");
    }

    satellites = sats;
    gps_fix    = fix;
    if(fix > 0)
    {
        gps_altitude = alt;
        latitude     = convert_deg(raw_lat);
        longitude    = convert_deg(raw_lon);
        if(lat_dir == 'S') latitude  = -latitude;
        if(lon_dir == 'W') longitude = -longitude;
    }
}

void wake_mpu(void){
    uint8_t wake = 0x00;
    HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADD, PWR_MGMT_1, 1, &wake, 1, 100);
    HAL_Delay(50);
}

HAL_StatusTypeDef read_raw(void){
    return HAL_I2C_Mem_Read(&hi2c1, MPU6050_ADD, ACCL_START, 1, raw_data, 14, 100);
}

extern USBD_HandleTypeDef hUsbDeviceFS;

void usb_print(char *msg)
{
    if(hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED)
    {
        CDC_Transmit_FS((uint8_t*)msg, strlen(msg));
    }
}

void lora_debug_registers()
{
    char msg[80];

    uint8_t version = lora_read(0x42);
    sprintf(msg, "LoRa Version: 0x%02X\r\n", version);
    usb_print(msg);

    uint8_t opmode = lora_read(0x01);
    sprintf(msg, "OP Mode: 0x%02X\r\n", opmode);
    usb_print(msg);

    uint8_t frf_msb = lora_read(0x06);
    uint8_t frf_mid = lora_read(0x07);
    uint8_t frf_lsb = lora_read(0x08);
    sprintf(msg, "FRF: %02X %02X %02X\r\n", frf_msb, frf_mid, frf_lsb);
    usb_print(msg);

    uint8_t pa = lora_read(0x09);
    sprintf(msg, "PA Config: 0x%02X\r\n", pa);
    usb_print(msg);

    uint8_t lna = lora_read(0x0C);
    sprintf(msg, "LNA: 0x%02X\r\n", lna);
    usb_print(msg);

    uint8_t modem1 = lora_read(0x1D);
    uint8_t modem2 = lora_read(0x1E);
    uint8_t modem3 = lora_read(0x26);
    sprintf(msg, "Modem1: 0x%02X | Modem2: 0x%02X | Modem3: 0x%02X\r\n", modem1, modem2, modem3);
    usb_print(msg);

    uint8_t sync = lora_read(0x39);
    sprintf(msg, "Sync Word: 0x%02X\r\n", sync);
    usb_print(msg);

    uint8_t dio = lora_read(0x40);
    sprintf(msg, "DIO Mapping1: 0x%02X\r\n", dio);
    usb_print(msg);
}

void calibrate_mpu()
{
    int32_t gx=0, gy=0, gz=0;
    int32_t az=0;
    int count = 0;

    for(int i=0;i<CALIB_SAMPLES;i++)
    {
        if(read_raw() == HAL_OK)
        {
            az += (int16_t)((raw_data[4] << 8) | raw_data[5]);

            gx += (int16_t)((raw_data[8] << 8) | raw_data[9]);
            gy += (int16_t)((raw_data[10] << 8) | raw_data[11]);
            gz += (int16_t)((raw_data[12] << 8) | raw_data[13]);

            count++;
        }

        HAL_Delay(SAMPLE_DELAY);
    }

    if(count == 0)
    {
        usb_print("Calibration Failed!\r\n");
        return;
    }

    // DO NOT CALIBRATE X/Y ACCEL
    Ax_offset = 0;
    Ay_offset = 0;

    // REMOVE ONLY Z GRAVITY OFFSET
    Az_offset = (az / count) - 8192;

    // GYRO OFFSETS
    Gx_offset = gx / count;
    Gy_offset = gy / count;
    Gz_offset = gz / count;
}

void lora_send_telemetry()
{
    char dbg[64];
    packetCount++;  // Increment packet counter
    time = HAL_GetTick() / 1000;  // Seconds since boot

    // Determine state
    if(released && servo_done) strcpy(state, "DROPPED");
    else if(released) strcpy(state, "TRIG");
    else if(kalman_alt > 10) strcpy(state, "ARMED");
    else strcpy(state, "IDLE");

    uint8_t packet[80];  // Increased size for full telemetry
    uint8_t i = 0;

    // Team ID (6 chars + null)
    const char* team_str = teamID;
    uint8_t team_len = strlen(teamID);
    packet[i++] = team_len;
    memcpy(&packet[i], teamID, team_len);
    i += team_len;

    // Time (int32_t, 4 bytes)
    int32_t time32 = (int32_t)time;
    memcpy(&packet[i], &time32, 4); i += 4;

    // Packet Count (int32_t, 4 bytes)
    int32_t pkt32 = (int32_t)packetCount;
    memcpy(&packet[i], &pkt32, 4); i += 4;

    // Altitude (float, 4 bytes)
    float alt = kalman_alt;
    memcpy(&packet[i], &alt, 4); i += 4;

    // Pressure (float, 4 bytes)
    float pres = pressure;
    memcpy(&packet[i], &pres, 4); i += 4;

    // Temperature (float, 4 bytes)
    float temp = temperature;
    memcpy(&packet[i], &temp, 4); i += 4;

    // Voltage (float, 4 bytes)
    memcpy(&packet[i], &voltage, 4); i += 4;

    // Latitude (double, 8 bytes)
    double lat = (double)latitude;
    memcpy(&packet[i], &lat, 8); i += 8;

    // Longitude (double, 8 bytes)
    double lon = (double)longitude;
    memcpy(&packet[i], &lon, 8); i += 8;

    // GPS Altitude (float, 4 bytes)
    float gps_alt = gps_altitude;
    memcpy(&packet[i], &gps_alt, 4); i += 4;

    // Satellites (int32_t, 4 bytes)
    int32_t sats = (int32_t)satellites;
    memcpy(&packet[i], &sats, 4); i += 4;

    // Accel X,Y,Z (QVector3D -> 3 floats = 12 bytes)
    float ax = Ax_g;
    float ay = Ay_g;
    float az = Az_g;
    memcpy(&packet[i], &ax, 4); i += 4;
    memcpy(&packet[i], &ay, 4); i += 4;
    memcpy(&packet[i], &az, 4); i += 4;

    // Gyro (single float - RMS or Z-axis)
    float gyro_rms = sqrtf(Gx_dps*Gx_dps + Gy_dps*Gy_dps + Gz_dps*Gz_dps) / sqrtf(3.0f);
    memcpy(&packet[i], &gyro_rms, 4); i += 4;

    // State (6 chars + null)
    const char* state_str = state;
    uint8_t state_len = strlen(state);
    packet[i++] = state_len;
    memcpy(&packet[i], state, state_len);
    i += state_len;

    sprintf(dbg, "LoRa TX %d bytes (pkt#%d)\r\n", i, packetCount);
    usb_print(dbg);

    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);

    // Send using existing LoRa logic
    lora_write(0x01, 0x81);  // Standby
    lora_write(0x12, 0xFF);  // Clear IRQs
    lora_write(0x0E, 0x00);  // TX base
    lora_write(0x0D, 0x00);  // FIFO ptr

    for(uint8_t j = 0; j < i; j++) {
        lora_write(0x00, packet[j]);
    }

    lora_write(0x22, i);  // Payload length
    lora_write(0x01, 0x83);  // Start TX

    // Wait TxDone
    while((lora_read(0x12) & 0x08) == 0);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);

    lora_write(0x12, 0x08);  // Clear TxDone
    lora_write(0x01, 0x81);  // Standby

    usb_print("Telemetry TX Done\r\n");
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_TIM1_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  servo_lock();
  HAL_Delay(1000);
  wake_mpu();
  wake_mpu();

  /* ===== GYRO RANGE ±250 DPS ===== */
  uint8_t gyro_config = 0x00;
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADD, 0x1B, 1,
                    &gyro_config, 1, 100);

  /* ===== ACCEL RANGE ±4G ===== */
  uint8_t accel_config = 0x08;
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADD, 0x1C, 1,
                    &accel_config, 1, 100);

  /* ===== MPU6050 HARDWARE LPF ===== */
  /* 0x03 = ~44Hz */
  uint8_t dlpf = 0x03;
  HAL_I2C_Mem_Write(&hi2c1, MPU6050_ADD, 0x1A, 1,
                    &dlpf, 1, 100);

  HAL_Delay(100);
  usb_print("Keep MPU6050 completely still!\r\n");

  usb_print("Calibrating now...\r\n");
  calibrate_mpu();

  usb_print("Calibration Done!\r\n");

  i2c_scan();
  HAL_Delay(500);

  HAL_I2C_DeInit(&hi2c1);
  HAL_Delay(10);
  MX_I2C1_Init();
  HAL_Delay(100);

  ms5611_init();
  HAL_Delay(200);

  uint8_t version = lora_read(0x42);
  char msg2[50];
  sprintf(msg2, "LoRa Version: 0x%X\r\n", version);
  usb_print(msg2);


  lora_init();

  HAL_Delay(100);
  usb_print("LoRa Initialized\r\n");
  lora_debug_registers();


  usb_print("Ground calibration - keep still...\r\n");
  float sum_p = 0;
  int count = 0;

  for(int i = 0; i < 200; i++)
  {
      ms5611_read();

      if(pressure > 300.0f && pressure < 1100.0f)
      {
          sum_p += pressure;
          count++;
      }

      HAL_Delay(20);
  }

  pressure_baseline = (count > 10) ? (sum_p / count) : 1013.25f;
  altitude_offset = 0.0f;

  char msg[80];
  sprintf(msg, "BASELINE: %.2f hPa (%d samples)\r\n", pressure_baseline, count);
  usb_print(msg);


  HAL_UART_Receive_IT(&huart1, &gps_data, 1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  // Check MPU connection

	        if(HAL_I2C_IsDeviceReady(&hi2c1, MPU6050_ADD, 3, 100) != HAL_OK)
	        {
	            usb_print("MPU NOT FOUND\r\n");
	            HAL_Delay(1000);
	            continue;
	        }

	        /* ================= MPU6050 ================= */
	        if(read_raw() == HAL_OK)
	        {
	            /* ===== OFFSET COMPENSATION ===== */
	            Ax = ((int16_t)(raw_data[0] << 8 | raw_data[1])) - Ax_offset;
	            Ay = ((int16_t)(raw_data[2] << 8 | raw_data[3])) - Ay_offset;
	            Az = ((int16_t)(raw_data[4] << 8 | raw_data[5])) - Az_offset;

	            Gx = ((int16_t)(raw_data[8] << 8 | raw_data[9])) - Gx_offset;
	            Gy = ((int16_t)(raw_data[10] << 8 | raw_data[11])) - Gy_offset;
	            Gz = ((int16_t)(raw_data[12] << 8 | raw_data[13])) - Gz_offset;

	            /* ===== CONVERT TO REAL UNITS ===== */
	            Ax_g = Ax / 8192.0f;
	            Ay_g = Ay / 8192.0f;
	            Az_g = Az / 8192.0f;

	            Gx_dps = Gx / 131.0f;
	            Gy_dps = Gy / 131.0f;
	            Gz_dps = Gz / 131.0f;

	            /* ===== SOFTWARE LOW PASS FILTER ===== */
	            static float gx_f = 0;
	            static float gy_f = 0;
	            static float gz_f = 0;

	            gx_f = gx_f * 0.90f + Gx_dps * 0.10f;
	            gy_f = gy_f * 0.90f + Gy_dps * 0.10f;
	            gz_f = gz_f * 0.90f + Gz_dps * 0.10f;

	            Gx_dps = gx_f;
	            Gy_dps = gy_f;
	            Gz_dps = gz_f;
	        }
	        else
	        {
	            usb_print("MPU READ ERROR\r\n");
	            HAL_Delay(200);
	            continue;
	        }

	        ms5611_read();
	        if(pressure > 300.0f && pressure < 1100.0f && pressure_baseline > 300.0f)
	        {
	        	float new_alt = 44330.0f *
	        	(1.0f - powf(pressure / pressure_baseline, 0.1903f));

	        	/* ===== REMOVE TINY BARO DRIFT ===== */
	        	if(fabs(new_alt) < 0.30f)
	        	{
	        	    new_alt = 0;
	        	}

	        	/* ===== SMOOTH ALTITUDE ===== */
	        	if(!isnan(new_alt) && !isinf(new_alt))
	        	{
	        	    altitude = altitude * 0.95f + new_alt * 0.05f;
	        	}
	        }

	        float accel_z = (Az_g * 9.81f) - 9.81f;

	        /* ===== REMOVE SMALL NOISE ===== */
	        if(fabs(accel_z) < 0.08f)
	        {
	            accel_z = 0;
	        }

	        /* ===== KALMAN UPDATE ===== */
	        kalman_update(altitude, accel_z);

	        /* ===== STOP VELOCITY DRIFT ===== */
	        if(fabs(kalman_vel) < 0.03f)
	        {
	            kalman_vel = 0;
	        }
	        /* ===== DRONE DROP SERVO LOGIC ===== */

	        /* Arm after drone climbs */
	        // ===== ARM =====
	        // ===== TOTAL ACCELERATION =====
	        // ===== TOTAL ACCELERATION =====
	        // ===== FILTERED Z ACCELERATION =====


	        // ===== FREE FALL DETECTION USING Z ONLY =====


	        if(!released)
	        {
	            if(Az_g <= 0)
	            {
	                released = 1;
	                drop_time = HAL_GetTick();
	                usb_print("TRIGGERED\r\n");
	            }
	        }

	        // ===== SERVO OPEN =====
	        if(released && !servo_done)
	        {
	            servo_open();
	            servo_done = 1;
	            usb_print("SERVO OPEN\r\n");
	        }

	        char msg[220];
	        sprintf(msg,
	        "AX:%.2f AY:%.2f AZ:%.2f | "
	        "GX:%.2f GY:%.2f GZ:%.2f | "
	        "P:%.2f mbar | T:%.2f C | ALT:%.2f m | "
	        "LAT:%.5f LON:%.5f | GPS_ALT:%.2f | SAT:%d|VEL:%.2f\r\n",

	        Ax_g, Ay_g, Az_g,
	        Gx_dps, Gy_dps, Gz_dps,
	        pressure, temperature, kalman_alt,
	        latitude, longitude,
	        gps_altitude,
	        satellites, kalman_vel);

	        usb_print(msg);
	        lora_send_telemetry();
	        HAL_Delay(200);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 83;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 19999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LORA_NSS_GPIO_Port, LORA_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LORA_RST_GPIO_Port, LORA_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_NSS_Pin */
  GPIO_InitStruct.Pin = LORA_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_RST_Pin */
  GPIO_InitStruct.Pin = LORA_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA_RST_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_DIO0_Pin */
  GPIO_InitStruct.Pin = LORA_DIO0_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LORA_DIO0_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        if(gps_data != '\n' && gps_index < 119)
        {
            gps_buffer[gps_index++] = gps_data;
        }
        else
        {
            gps_buffer[gps_index] = '\0';
            gps_index = 0;

            parse_gps(gps_buffer);   // 🔥 process sentence
        }

        HAL_UART_Receive_IT(&huart1, &gps_data, 1);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
