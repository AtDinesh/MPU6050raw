
#include <HardWire.h>
#include <SPI.h>

//Pin MAP
#define LED_PIN PA14    
#define POWER_EN_PIN PB1
#define MEM_HOLD_PIN PA3
#define MEM_CS_PIN PA4
#define ADC_PHOT_PIN PA2


//COMMANDS FLASH
#define W_EN   0x06  //write enable
#define W_DE  0x04  //write disable
#define R_SR1 0x05  //read status reg 1
#define R_SR2 0x35  //read status reg 2
#define W_SR  0x01  //write status reg
#define PAGE_PGM  0x02  //page program
#define QPAGE_PGM 0x32  //quad input page program
#define BLK_E_64K 0xD8  //block erase 64KB
#define BLK_E_32K 0x52  //block erase 32KB
#define SECTOR_E  0x20  //sector erase 4KB
#define CHIP_ERASE  0xc7  //chip erase
#define CHIP_ERASE2 0x60  //=CHIP_ERASE
#define E_SUSPEND 0x75  //erase suspend
#define E_RESUME  0x7a  //erase resume
#define PDWN    0xb9  //power down
#define HIGH_PERF_M 0xa3  //high performance mode
#define CONT_R_RST  0xff  //continuous read mode reset
#define RELEASE   0xab  //release power down or HPM/Dev ID (deprecated)
#define R_MANUF_ID  0x90  //read Manufacturer and Dev ID (deprecated)
#define R_UNIQUE_ID 0x4b  //read unique ID (suggested)
#define R_JEDEC_ID  0x9f  //read JEDEC ID = Manuf+ID (suggested)
#define READ    0x03
#define FAST_READ 0x0b



HardWire HWire(1, I2C_FAST_MODE); // I2c1 I2C_FAST_MODE
const int MPU_addr=0x69;//0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int loop_time=0;
int t_tmp = 0;
int i;
int dt = 1000; //(1ms)
int next_time=0;

//#define LED PB1
//Kmeans
float H_mean = 1024;
float L_mean = 1024;

void setup() {
  // Setup SPI 1
  SPI.begin(); //Initialize the SPI_1 port.
  SPI.setBitOrder(MSBFIRST); // Set the SPI_1 bit order
  SPI.setDataMode(SPI_MODE0); //Set the  SPI_2 data mode 0
  SPI.setClockDivider(SPI_CLOCK_DIV16);      // Slow speed (72 / 16 = 4.5 MHz SPI_1 speed)
  SPI.setClockDivider(SPI_CLOCK_DIV128);    
  pinMode(MEM_CS_PIN, OUTPUT);
  pinMode(MEM_HOLD_PIN, OUTPUT);
  digitalWrite(MEM_HOLD_PIN, LOW); 
  
  // initialize digital pin PA14 as an output.
  pinMode(LED_PIN, OUTPUT);
  //pinMode(PA3, OUTPUT);
  for(i=0;i<5;i++) {digitalWrite(LED_PIN, HIGH);delay(10);digitalWrite(LED_PIN, LOW);delay(300);}
  digitalWrite(LED_PIN, LOW);
  Serial.begin(250000);
  delay(1000);
  Serial.println("start!");
  HWire.begin();
  HWire.beginTransmission(MPU_addr);
  HWire.write(0x6B);  // PWR_MGMT_1 register
  HWire.write(0);     // set to zero (wakes up the MPU-6050)
  HWire.endTransmission();
  digitalWrite(LED_PIN, LOW);

  //read memory inique ID
  //print_ID();
  delay(1000);
}


void loop() {
  unsigned char flag_send_raw = 0;
  while(1)
  {
    if (Serial.available())
    {
      unsigned char c = Serial.read();
      switch(c)
      {
        case 'i':
          print_ID();
          break;
        case 'h':
          print_help();
          break;
        case 'r':
          flag_send_raw = 1;
          break;
        case 'R':
          flag_send_raw = 0;
        break;
      }
    }
 
  //blink RED LED
  digitalWrite(LED_PIN, (millis()%1000)<10);
  //Serial.println("mes");
  MPU_read_RAW();
  //write_flash();
  //print_ID();
  if (flag_send_raw)
  {
    Serial.print(AcX);
    Serial.print(","); Serial.print(AcY);
    Serial.print(","); Serial.print(AcZ);
    Serial.print(","); Serial.print(Tmp/340.00+36.53);  //temperature in degrees C
    Serial.print(","); Serial.print(GyX);
    Serial.print(","); Serial.print(GyY);
    Serial.print(","); Serial.print(GyZ);
    Serial.print("\n");
  }

  while(micros()<next_time) delayMicroseconds(1); //1khz

  //digitalWrite(LED, LOW);
  next_time+=dt;

  int photo = analogRead(PA2);
  int e = photo;

  if (e >(H_mean+L_mean)/2 )
  {
    digitalWrite(PA3, HIGH);
    H_mean=H_mean*0.9+e*0.1;
  }
  else
  {
    L_mean=L_mean*0.9+e*0.1;
    digitalWrite(PA3, LOW);
  }
  //Serial.print("H_mean = "); 
  //Serial.print(H_mean);
  //Serial.print("\tL_mean = "); 
  //Serial.println(L_mean);
  }
}


char MPU_read(unsigned char address)
{
  unsigned char data;
  HWire.beginTransmission(MPU_addr);
  HWire.write(address);
  HWire.endTransmission();
  
  HWire.requestFrom(MPU_addr, 1);
  while(!HWire.available())
    ;
  return HWire.read();
}

char MPU_read_RAW()
{
  HWire.beginTransmission(MPU_addr);
  HWire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  HWire.endTransmission();
  HWire.requestFrom(MPU_addr,14);  // request a total of 14 registers
  while(!HWire.available());
  AcX=HWire.read()<<8|HWire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=HWire.read()<<8|HWire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=HWire.read()<<8|HWire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=HWire.read()<<8|HWire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=HWire.read()<<8|HWire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=HWire.read()<<8|HWire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=HWire.read()<<8|HWire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void write_flash()
{
  digitalWrite(MEM_CS_PIN, LOW); // manually take CSN low for SPI_1 transmission
  SPI.transfer(0x55); //Send the HEX data 0x55 over SPI-1 port and store the received byte to the <data> variable.
  SPI.transfer(0x55); //Send the HEX data 0x55 over SPI-1 port and store the received byte to the <data> variable.
  SPI.transfer(0x55); //Send the HEX data 0x55 over SPI-1 port and store the received byte to the <data> variable.
  SPI.transfer(0x55); //Send the HEX data 0x55 over SPI-1 port and store the received byte to the <data> variable.
  SPI.transfer(0x55); //Send the HEX data 0x55 over SPI-1 port and store the received byte to the <data> variable.
  SPI.transfer(0x55); //Send the HEX data 0x55 over SPI-1 port and store the received byte to the <data> variable.

  SPI.transfer(0x55); //Send the HEX data 0x55 over SPI-1 port and store the received byte to the <data> variable.
  SPI.transfer(0x55); //Send the HEX data 0x55 over SPI-1 port and store the received byte to the <data> variable.
  SPI.transfer(0x55); //Send the HEX data 0x55 over SPI-1 port and store the received byte to the <data> variable.
  SPI.transfer(0x55); //Send the HEX data 0x55 over SPI-1 port and store the received byte to the <data> variable.
  SPI.transfer(0x55); //Send the HEX data 0x55 over SPI-1 port and store the received byte to the <data> variable.
  SPI.transfer(0x55); //Send the HEX data 0x55 over SPI-1 port and store the received byte to the <data> variable.
  
  digitalWrite(MEM_CS_PIN, HIGH); // manually take CSN high between spi transmissions
}


void print_ID()
{
  uint64_t uid;
  uint8_t *arr;
  arr = (uint8_t*)&uid;
  digitalWrite(MEM_CS_PIN, LOW); // manually take CSN low for SPI_1 transmission
  SPI.transfer(R_UNIQUE_ID);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  //for little endian machine only
  //Serial.println("Memory ID is:");
  for(int i=7;i>=0;i--)
  {
    arr[i] = SPI.transfer(0x00);
    if (arr[i]<0x10)Serial.print('0');
    Serial.print(arr[i],HEX);
    Serial.print(" ");
  }
  Serial.print('\n');
  digitalWrite(MEM_CS_PIN, HIGH); // manually take CSN high between spi transmissions
}


void print_help()
{
 Serial.println("IMU logger - Thomas FLAYOLS LAAS CNRS");
 Serial.println("Commands are");
 Serial.println(" h: Help ");
 Serial.println(" i: return Unique ID");
 Serial.println(" r: send RAW data at 1Khz (Ax,Ay,Az,Temp,Gx,Gy,Gz)");
 Serial.println(" R: stop sending raw data");
}


