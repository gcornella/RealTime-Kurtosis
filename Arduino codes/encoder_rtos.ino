#include <Arduino_FreeRTOS.h>
#include <semphr.h>  // add the FreeRTOS functions for Semaphores (or Flags).
#include <Wire.h>

// Declare a mutex Semaphore Handle which we will use to manage the Serial Port.
// It will be used to ensure only one Task is accessing this resource at any time.
SemaphoreHandle_t xSerialSemaphore;

// define three Tasks for DigitalRead for every encoder
void receiveEventTask( void *pvParameters );
void TaskDigitalRead1( void *pvParameters );
void TaskDigitalRead2( void *pvParameters );
void TaskDigitalRead3( void *pvParameters );

// Define the global variables
int countTickRot = 0;
int countIndexRot = 0;
int precTickRot = 0;
int precIndexRot = 0;
int tickRot = 0;
int tickBRot = 0;

int countTickExt = 0;
int countIndexExt = 0;
int precTickExt = 0;
int precIndexExt = 0;
int tickExt = 0;
int tickBExt = 0;

int countTickSup = 0;
int countIndexSup = 0;
int precTickSup = 0;
int precIndexSup = 0;
int tickSup = 0;
int tickBSup = 0;

// Pulses Per Revolution, specified in the encoder settings
int PPR = 192;

bool flagReceived = false;

// the setup function runs once when you press reset or power the board
void setup() {
  Wire.begin(1); 
  Wire.onReceive(receiveEvent);

  // initialize serial communication at 9600 bits per second:
  Serial.begin(115200);
  Serial.println("Moving to start");
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB, on LEONARDO, MICRO, YUN, and other 32u4 based boards.
  }
  
  while(!flagReceived){
    delay(10);
  }
   
  // Semaphores are useful to stop a Task proceeding, where it should be paused to wait,
  // because it is sharing a resource, such as the Serial port.
  // Semaphores should only be used whilst the scheduler is running, but we can set it up here.
  if ( xSerialSemaphore == NULL )  // Check to confirm that the Serial Semaphore has not already been created.
  {
    xSerialSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore we will use to manage the Serial Port
    if ( ( xSerialSemaphore ) != NULL )
      xSemaphoreGive( ( xSerialSemaphore ) );  // Make the Serial Port available for use, by "Giving" the Semaphore.
  }

   
  xTaskCreate(
    TaskDigitalRead1
    ,  "DigitalRead1"  // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL //Parameters for the task
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL ); //Task Handle
    
  xTaskCreate(
    TaskDigitalRead2
    ,  "DigitalRead2"  // A name just for humans
    ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL //Parameters for the task
    ,  1  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL ); //Task Handle
  
  xTaskCreate(
      TaskDigitalRead3
      ,  "DigitalRead3"  // A name just for humans
      ,  128  // This stack size can be checked & adjusted by reading the Stack Highwater
      ,  NULL //Parameters for the task
      ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
      ,  NULL ); //Task Handle 
     
}

void loop()
{
  // Empty. Things are done in Tasks.
}

// read the data that the other arduino sends ans set the ticks to 0
void receiveEvent(int bytes) 
{
  Serial.println("Event received");

  //////////////////////////////////////
  int ROT_A = 13;
  pinMode(ROT_A, INPUT);
  precTickRot = digitalRead(ROT_A);
  
  countTickRot = 0;
  Serial.print("Rotation:");
  Serial.println(0);

  //////////////////////////////////////
  int EXT_C = 10;
  pinMode(EXT_C, INPUT);
  precTickExt = digitalRead(EXT_C);
  
  countTickExt = 0;
  Serial.print("Extension:");
  Serial.println(0);


  //////////////////////////////////////
  int Sup_E = 7;
  pinMode(Sup_E, INPUT);
  precTickSup = digitalRead(Sup_E);
  
  countTickSup = 0;
  Serial.print("Supination:");
  Serial.println(0);
  
  flagReceived = true;
}


/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskDigitalRead1( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  // digital pins
  int ROT_A = 13;
  int ROT_B = 12;
      
  // declare
  pinMode(ROT_A, INPUT);
  pinMode(ROT_B, INPUT);

  for (;;) // A Task shall never return or exit.
  {
    // See if we can obtain or "Take" the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 1 ) == pdTRUE )
    {
      // read the input pin from the encoder:
      tickRot = digitalRead(ROT_A);
      tickBRot = digitalRead(ROT_B);
        
      // We were able to obtain or "Take" the semaphore and can now access the shared resource.
      // We want to have the Serial Port for us alone, as it takes some time to print,
      // so we don't want it getting stolen during the middle of a conversion.
      // print out the state of the button:
      // formula to convert tick to relative angle:
      long rot_mapped = (360L * (countTickRot)) / PPR;
      Serial.print("Rotation:");
      Serial.println(rot_mapped);
  
      // MY CODE GOES HERE
      if(tickRot != precTickRot)
      {
        if(tickRot != tickBRot)
        {
          countTickRot = countTickRot + tickRot;
          precTickRot = tickRot;
        }
      else
      {
        countTickRot = countTickRot - tickRot;
        precTickRot = tickRot;
      }
      
      }
      else{
         if (countTickRot<0) {
            countTickRot = 0;
          }
          else if (countTickRot>90*PPR/360) {
            countTickRot = 90*PPR/360;
          }
      
      }
      xSemaphoreGive(xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }
    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}

/////////////////////////////////////////////////////////////////
void TaskDigitalRead2( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  // digital pins
  int EXT_C = 10;
  int EXT_D = 9;
  
  // declare
  pinMode(EXT_C, INPUT);
  pinMode(EXT_D, INPUT);

  for (;;) // A Task shall never return or exit.
  {
    // See if we can obtain or "Take" the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 1 ) == pdTRUE )
    {
      // We were able to obtain or "Take" the semaphore and can now access the shared resource.
      // We want to have the Serial Port for us alone, as it takes some time to print,
      // so we don't want it getting stolen during the middle of a conversion.
      // print out the state of the button:

      long ext_mapped = (360L * countTickExt) / PPR;
      Serial.print("Extension:");
      Serial.println(ext_mapped);
      
      // read the input pin from the encoder:
      tickExt = digitalRead(EXT_C);
      tickBExt = digitalRead(EXT_D);
      
      // MY CODE GOES HERE
      if(tickExt != precTickExt)
      {
        if(tickExt != tickBExt)
        {
          countTickExt = countTickExt + tickExt;
          precTickExt = tickExt;
        }
        else
        {
          countTickExt = countTickExt - tickExt;
          precTickExt = tickExt;
        }
        
      }
      else{
        
         if (countTickExt>0) {
            countTickExt = 0;
          }
          else if (countTickExt<-(90*PPR/360)) {
            countTickExt = -90*PPR/360;
          }
      
      }
      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }
    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}


/////////////////////////////////////////////////////////////////

void TaskDigitalRead3( void *pvParameters __attribute__((unused)) )  // This is a Task.
{
  // digital pins supination
  int Sup_E = 7;
  int Sup_F = 6;

  // declare
  pinMode(Sup_E, INPUT);
  pinMode(Sup_F, INPUT);

  for (;;) // A Task shall never return or exit.
  {
   
    // See if we can obtain or "Take" the Serial Semaphore.
    // If the semaphore is not available, wait 5 ticks of the Scheduler to see if it becomes free.
    if ( xSemaphoreTake( xSerialSemaphore, ( TickType_t ) 1 ) == pdTRUE )
    {
      // We were able to obtain or "Take" the semaphore and can now access the shared resource.
      // We want to have the Serial Port for us alone, as it takes some time to print,
      // so we don't want it getting stolen during the middle of a conversion.
      // print out the state of the button:
      // read the input pin from the encoder:

      // formula to convert tick to relative angle:
      long sup_mapped = (360L * countTickSup) / PPR;
      Serial.print("Supination:");
      Serial.println(sup_mapped);
      
      tickSup = digitalRead(Sup_E);
      tickBSup = digitalRead(Sup_F);
      
      // MY CODE GOES HERE
      if(tickSup != precTickSup)
      {
        if(tickSup != tickBSup)
        {
          countTickSup = countTickSup + tickSup;
          precTickSup = tickSup;
        }
        else
        {
          countTickSup = countTickSup - tickSup;
          precTickSup = tickSup;
        }
        
      }
      else{
        
         if (countTickSup>90*PPR/360) {
            countTickSup = 90*PPR/360;
          }
          else if (countTickSup<0) {
            countTickSup = 0;
          }
      
      }
      xSemaphoreGive( xSerialSemaphore ); // Now free or "Give" the Serial Port for others.
    }
    vTaskDelay(1);  // one tick delay (15ms) in between reads for stability
  }
}
