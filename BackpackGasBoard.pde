
//LIBS
#include <WaspSensorGas_v20.h>
//CONSTANTS
#define FRAME_DELAY_TIME 2 //secods

//SENSORS
bool readTemp=true; 
bool readPressure = true; 
bool readCO2=true; 

bool readAccelerometer=true; 
bool isAccelInitialized=false; 
//NH3
//bool readNH3=true; 
bool readO3=true; 
//NO2
bool readNO2 = true;
//Air pollutants
bool readAP1=true; 
bool readAP2=true; 
//
bool isRTCon=false; 


//SENSOR CONSTANTS
#define CO2_GAIN  7  //GAIN of the sensor stage
#define O3_GAIN  1      //GAIN of the sensor stage
#define O3_RESISTOR 20  //LOAD RESISTOR of the sensor stage
#define NO2_GAIN 1
#define NO2_RESISTOR 3
#define AP1_GAIN  1      // GAIN of the sensor stage
#define AP1_RESISTOR 20  // LOAD RESISTOR of the sensor stage
#define AP2_GAIN  1      // GAIN of the sensor stage
#define AP2_RESISTOR 20  // LOAD RESISTOR of the sensor stage
//
float initDelay; 

//parse token
char token='$'; 

void setup() {
  // put your setup code here, to run once:
  USB.ON();
  USB.println(F("TC_Start_GAS_BOARD"));
  //battery  
  getBatteryReading();
  //   
  initDelay= calculateInitializationDelay();
  printSensorSetup(); 
  //
  RTC.ON();
  isRTCon=true;
  
  delay(1000);
  if (readO3){
    initializeO3(); 
  }
  if (readCO2){
    initializeCO2();
  }
  if (readNO2){
    initializeNO2(); 
  }
  if (readAP1){
    initializeAP1(); 
  }
  if (readAP2){
    initializeAP2();  
  }
  //
  doInitializationDelay();
  //
  //
  if (readAccelerometer){
    ACC.ON(); 
    isAccelInitialized=true; 
  }

  USB.println("STARTING");
}

void loop() {
  
  /*
   *  GET SENSOR READINGS
   */
  
  //switch board on
  SensorGasv20.ON();
  //initialize sensors
  if(readPressure){
    SensorGasv20.setSensorMode(SENS_ON, SENS_PRESSURE);
  }
  delay(20); 
  /*
   *Get sensor values
   */
  //TEMPERATURE
  float tempVal=0;
  if (readTemp){
    tempVal =SensorGasv20.readValue(SENS_TEMPERATURE); 
  }
  //PRESSURE
  float pressureVal=0;
  if(readPressure){
    pressureVal = SensorGasv20.readValue(SENS_PRESSURE);
    //set sensors off 
    SensorGasv20.setSensorMode(SENS_OFF, SENS_PRESSURE);
  }
  //O3
  float o3Val_volt=0; 
  float  o3Val_kohms;
  if (readO3){
    o3Val_volt = SensorGasv20.readValue(SENS_SOCKET2B);
    // Conversion from voltage into kiloohms
    o3Val_kohms = SensorGasv20.calculateResistance(SENS_SOCKET2B, o3Val_volt, O3_GAIN, O3_RESISTOR);

  }
  //CO2
  float co2Val=0; 
  if (readCO2){
    // Read the sensor 
    co2Val = SensorGasv20.readValue(SENS_CO2); 
  }
  //NO2 
  float no2Val=0; 
  if (readNO2){
    no2Val = SensorGasv20.readValue(SENS_SOCKET3B); 
  }
  //AP1
  float ap1Val_volt=0; 
  float ap1Val_kohms=0; 
  if (readAP1){
    // Read the sensor 
    ap1Val_volt = SensorGasv20.readValue(SENS_SOCKET4A); 
    // Conversion from voltage into kiloohms  
    ap1Val_kohms = SensorGasv20.calculateResistance(SENS_SOCKET4A, ap1Val_volt, AP1_GAIN, AP1_RESISTOR);
  }
  //AP2 
  float ap2Val_volt=0; 
  float ap2Val_kohms=0; 
  if (readAP2){ //@S3
    // Read the sensor 
    ap2Val_volt = SensorGasv20.readValue(SENS_SOCKET3A);
    ap2Val_kohms = SensorGasv20.calculateResistance(SENS_SOCKET3A, ap2Val_volt, AP2_GAIN, AP2_RESISTOR);

  }
  //switch off board
  SensorGasv20.OFF();
  //

  /*
   *print values
   */
  //temperature
  if (readTemp){
    USB.print(token); 
    USB.print(F("Temprature: ")); 
    USB.print(tempVal); 
    USB.println(F(" Celcius")); 
  }
  //pressure
  if (readPressure){
    USB.print(F("Pressure: ")); 
    USB.print(pressureVal); 
    USB.println(F(" kPa"));
  }
  if (readAccelerometer && isAccelInitialized){
    getAccelValue();
  }

  if (readO3){
    // Print the result through the USB
    USB.print(F("O3 @S2B: "));
    USB.print(o3Val_volt);
    USB.print(F("V - "));


    USB.print(o3Val_kohms);
    USB.println(" kohms"); 
  }

  if (readCO2){
    // Print the result through the USB
    USB.print(F("CO2: "));
    USB.print(co2Val);
    USB.println(F("V")); 
  }

  if (readNO2){
    // Print the result through the USB
    USB.print(F("NO2 @S3B: "));
    USB.print(no2Val);
    USB.print(F("V - "));

    // Conversion from voltage into kiloohms
    float no2Val_kohms = SensorGasv20.calculateResistance(SENS_SOCKET3B, no2Val, NO2_GAIN, NO2_RESISTOR);
    USB.print(no2Val_kohms);
    USB.println("kohms"); 
  }
  if (readAP1){
    // Print the result through the USB
    USB.print(F("AP1 @ S4: "));
    USB.print(ap1Val_volt);
    USB.print(F("V - "));

    USB.print(ap1Val_kohms);
    USB.println(F("kohms"));
  }
  if (readAP2){
    // Print the result through the USB
    USB.print(F("AP2 @S3A: "));
    USB.print(ap2Val_volt);
    USB.print(F("V - "));

    USB.print(ap2Val_kohms);
    USB.println(F("kohms")); 
  }
  //
  delay(FRAME_DELAY_TIME*1000); 
}

void getAccelValue(){
  byte check = ACC.check();

  //----------X Value-----------------------
  int x_acc = ACC.getX();

  //----------Y Value-----------------------
  int y_acc = ACC.getY();
  //----------Z Value-----------------------
  int z_acc = ACC.getZ();
  //-------------------------------
  USB.print(F("ACC: ")); 
  USB.print(x_acc, DEC);
  USB.print(F(" / ")); 
  USB.print(y_acc, DEC);
  USB.print(F(" / ")); 
  USB.println(z_acc, DEC); 
}


































