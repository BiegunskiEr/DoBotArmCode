/****************************************Copyright(c)*****************************************************
**                            Shenzhen Yuejiang Technology Co., LTD.
**
**                                 http://www.dobot.cc
**
**--------------File Info---------------------------------------------------------------------------------
** File name:           main.cpp
** Latest modified Date:2016-10-24
** Latest Version:      V2.0.0
** Descriptions:        main body
**
**--------------------------------------------------------------------------------------------------------
** Modify by:           Edward
** Modified date:       2016-11-25
** Version:             V1.0.0
** Descriptions:        Modified,From DobotDemoForSTM32
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
//Define which libraries the code is using
#include "stdio.h"
#include "Protocol.h"
#include "command.h"
#include "FlexiTimer2.h"

//Set Serial TX&RX Buffer Size
#define SERIAL_TX_BUFFER_SIZE 64
#define SERIAL_RX_BUFFER_SIZE 256

// define the switch pin
#define SWITCH_PIN 27
#define MOVE_DELAY 3000  // Delay between movements in milliseconds

/*********************************************************************************************************
** Global parameters
*********************************************************************************************************/
EndEffectorParams gEndEffectorParams;

JOGJointParams  gJOGJointParams;
JOGCoordinateParams gJOGCoordinateParams;
JOGCommonParams gJOGCommonParams;
JOGCmd          gJOGCmd;

PTPCoordinateParams gPTPCoordinateParams;
PTPCommonParams gPTPCommonParams;
PTPCmd          gPTPCmd;



uint64_t gQueuedCmdIndex;




/*********************************************************************************************************
** Function name:       setup
** Descriptions:        Initializes Serial
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void setup() {
    Serial.begin(115200);
    Serial1.begin(115200); 
    printf_begin();
    
    // Initialize the switch pin as an input:
    pinMode(SWITCH_PIN, INPUT_PULLUP);

    // Protocol and parameter initialization
    InitRAM();
    ProtocolInit();
    
    SetJOGJointParams(&gJOGJointParams, true, &gQueuedCmdIndex);
    SetJOGCoordinateParams(&gJOGCoordinateParams, true, &gQueuedCmdIndex);
    SetJOGCommonParams(&gJOGCommonParams, true, &gQueuedCmdIndex);
    SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);

    // Set Timer Interrupt
    FlexiTimer2::set(500, Serialread); 
    FlexiTimer2::start();
}


/*********************************************************************************************************
** Function name:       Serialread
** Descriptions:        import data to rxbuffer
** Input parametersnone:
** Output parameters:   
** Returned value:      
*********************************************************************************************************/
void Serialread()
{
  while(Serial1.available()) {
        uint8_t data = Serial1.read();
        if (RingBufferIsFull(&gSerialProtocolHandler.rxRawByteQueue) == false) {
            RingBufferEnqueue(&gSerialProtocolHandler.rxRawByteQueue, &data);
        }
  }
}
/*********************************************************************************************************
** Function name:       Serial_putc
** Descriptions:        Remap Serial to Printf
** Input parametersnone:
** Output parameters:   
** Returned value:      
*********************************************************************************************************/
int Serial_putc( char c, struct __file * )
{
    Serial.write( c );
    return c;
}

/*********************************************************************************************************
** Function name:       printf_begin
** Descriptions:        Initializes Printf
** Input parameters:    
** Output parameters:
** Returned value:      
*********************************************************************************************************/
void printf_begin(void)
{
    fdevopen( &Serial_putc, 0 );
}


 
/*********************************************************************************************************
** Function name:       InitRAM
** Descriptions:        Initializes a global variable
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/
void InitRAM(void)
{
    //Set JOG Model
    gJOGJointParams.velocity[0] = 100;
    gJOGJointParams.velocity[1] = 100;
    gJOGJointParams.velocity[2] = 100;
    gJOGJointParams.velocity[3] = 100;
    gJOGJointParams.acceleration[0] = 80;
    gJOGJointParams.acceleration[1] = 80;
    gJOGJointParams.acceleration[2] = 80;
    gJOGJointParams.acceleration[3] = 80;

    gJOGCoordinateParams.velocity[0] = 100;
    gJOGCoordinateParams.velocity[1] = 100;
    gJOGCoordinateParams.velocity[2] = 100;
    gJOGCoordinateParams.velocity[3] = 100;
    gJOGCoordinateParams.acceleration[0] = 80;
    gJOGCoordinateParams.acceleration[1] = 80;
    gJOGCoordinateParams.acceleration[2] = 80;
    gJOGCoordinateParams.acceleration[3] = 80;

    gJOGCommonParams.velocityRatio = 50;
    gJOGCommonParams.accelerationRatio = 50;

    gJOGCmd.isJoint = JOINT_MODEL;
    

    

    //Set PTP Model
    gPTPCoordinateParams.xyzVelocity = 100;
    gPTPCoordinateParams.rVelocity = 100;
    gPTPCoordinateParams.xyzAcceleration = 80;
    gPTPCoordinateParams.rAcceleration = 80;

    gPTPCommonParams.velocityRatio = 50;
    gPTPCommonParams.accelerationRatio = 50;

    gPTPCmd.ptpMode = MOVL_XYZ;
    

    gQueuedCmdIndex = 0;

    
}


void jogToPart(int delayTime) {
    Serial.println("Jogging to part...");

    // Set the joint and direction and store in memory
    gJOGCmd.cmd = AP_DOWN;

    // Packages command to be executed
    SetJOGCmd(&gJOGCmd, true, &gQueuedCmdIndex);

    //Execute stored command
    ProtocolProcess();

    //Delay and stop
    delay(delayTime);
    stopJog();
}


void jogToReject(int delayTime) {
    Serial.println("Jogging to reject...");

    // Set the joint and direction and store in memory
    gJOGCmd.cmd = AN_DOWN;

    // Packages command to be executed
    SetJOGCmd(&gJOGCmd, true, &gQueuedCmdIndex);

    // Execute stored command
    ProtocolProcess();

    // Delay and stop
    delay(delayTime);
    stopJog();
}


void stopJog() {
    Serial.println("Stopping jog...");

    // Set the arm to IDEL and store in memory
    gJOGCmd.cmd = IDEL;
    
    
    // Packages command to be executed
    SetJOGCmd(&gJOGCmd, true, &gQueuedCmdIndex);

    //Execute stored command
    ProtocolProcess();
}


/*********************************************************************************************************
** Function name:       moveArm
** Descriptions:        Takes X, Y, & Z cords and plugs them into gPTPCmd
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/

void moveArm(float x, float y, float z)
{
  

  gPTPCmd.x = x;
  gPTPCmd.y = y;
  gPTPCmd.z = z;



SetPTPCmd(&gPTPCmd, true, &gQueuedCmdIndex);




ProtocolProcess();


}












/*********************************************************************************************************
** Function name:       loop
** Descriptions:        Program entry
** Input parameters:    none
** Output parameters:   none
** Returned value:      none
*********************************************************************************************************/

void loop() 
{
   // Check if switch is pressed
  if (digitalRead(SWITCH_PIN) == LOW) 
  {
    //Print statment to check if arduino found that the switch was pressed
    Serial.println("Switch pressed!");



    //Run the jogToPart function
    jogToPart(12000);

    //Pick the part up
    moveArm(0,0,60);

    //Turn on vacuum

    //Go up
    moveArm(160,110,0);

    //Go in reverse with the jogToReject function
    jogToReject(12000 * 2);

    //Go back to midpoint
    jogToPart(12000);

  }
}



