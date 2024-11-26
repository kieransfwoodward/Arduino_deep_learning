/*
  IMU Classifier
  This example uses the on-board IMU to start reading acceleration and gyroscope
  data from on-board IMU, once enough samples are read, it then uses a
  TensorFlow Lite (Micro) model to try to classify the movement as a known gesture.
  Note: The direct use of C/C++ pointers, namespaces, and dynamic memory is generally
        discouraged in Arduino examples, and in the future the TensorFlowLite library
        might change to make the sketch simpler.
  The circuit:
  - Arduino Nano 33 BLE or Arduino Nano 33 BLE Sense board.
  Created by Don Coleman, Sandeep Mistry
  Modified by Dominic Pajak, Sandeep Mistry
  This example code is in the public domain.
*/

#include <Arduino_LSM9DS1.h>

#define USE_ARDUINO_INTERRUPTS false
//#include <PulseSensorPlayground.h>

#include <TensorFlowLite.h>
#include <tensorflow/lite/micro/all_ops_resolver.h>
#include <tensorflow/lite/micro/micro_error_reporter.h>
#include <tensorflow/lite/micro/micro_interpreter.h>
#include <tensorflow/lite/schema/schema_generated.h>
#include <tensorflow/lite/version.h>

#include "float16_stress.h"
//#include "HAR.h"


const int GSR = A2;
int gsr;
//const int OUTPUT_TYPE = SERIAL_PLOTTER;

/*
   Pinout:
     PULSE_INPUT = Analog Input. Connected to the pulse sensor
      purple (signal) wire.
     PULSE_BLINK = digital Output. Connected to an LED (and 1K resistor)
      that will flash on each detected pulse.
     PULSE_FADE = digital Output. PWM pin onnected to an LED (and 1K resistor)
      that will smoothly fade with each pulse.
      NOTE: PULSE_FADE must be a pin that supports PWM.
       If USE_INTERRUPTS is true, Do not use pin 9 or 10 for PULSE_FADE,
       because those pins' PWM interferes with the sample timer.
*/
const int PULSE_INPUT = A3;
const int THRESHOLD = 550;   // Adjust this number to avoid noise when idle

/*
   samplesUntilReport = the number of samples remaining to read
   until we want to report a sample over the serial connection.

   We want to report a sample value over the serial port
   only once every 20 milliseconds (10 samples) to avoid
   doing Serial output faster than the Arduino can send.
*/
byte samplesUntilReport;
const byte SAMPLES_PER_SERIAL_SAMPLE = 10;
//PulseSensorPlayground pulseSensor;



const float accelerationThreshold = 2.5; // threshold of significant in G's
const int numSamples = 250;

int samplesRead = numSamples;

// global variables used for TensorFlow Lite (Micro)
tflite::MicroErrorReporter tflErrorReporter;

// pull in all the TFLM ops, you can remove this line and
// only pull in the TFLM ops you need, if would like to reduce
// the compiled size of the sketch.
tflite::AllOpsResolver tflOpsResolver;

const tflite::Model* tflModel = nullptr;
tflite::MicroInterpreter* tflInterpreter = nullptr;
TfLiteTensor* tflInputTensor = nullptr;
TfLiteTensor* tflOutputTensor = nullptr;

// Create a static memory buffer for TFLM, the size may need to
// be adjusted based on the model you are using
constexpr int tensorArenaSize = 100 * 1024;
byte tensorArena[tensorArenaSize] __attribute__((aligned(16)));

// array to map gesture index to a name
const char* GESTURES[] = {
  "Stressed",
  "Relaxed"
};

#define NUM_GESTURES (sizeof(GESTURES) / sizeof(GESTURES[0]))

















tflite::MicroErrorReporter tflErrorReporter1;

// pull in all the TFLM ops, you can remove this line and
// only pull in the TFLM ops you need, if would like to reduce
// the compiled size of the sketch.
tflite::AllOpsResolver tflOpsResolver1;

const tflite::Model* tflModel1 = nullptr;
tflite::MicroInterpreter* tflInterpreter1 = nullptr;
TfLiteTensor* tflInputTensor1 = nullptr;
TfLiteTensor* tflOutputTensor1 = nullptr;

// Create a static memory buffer for TFLM, the size may need to
// be adjusted based on the model you are using
constexpr int tensorArenaSize1 = 32 * 1024;
byte tensorArena1[tensorArenaSize1] __attribute__((aligned(16)));

// array to map gesture index to a name
const char* GESTURES1[] = {
  "Exercise",
  "Rest"
};

#define NUM_GESTURES1 (sizeof(GESTURES1) / sizeof(GESTURES1[0]))


void setup() {
  Serial.begin(9600);
  while (!Serial);


if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

 // pulseSensor.analogInput(PULSE_INPUT);


//  pulseSensor.setSerial(Serial);
 // pulseSensor.setOutputType(OUTPUT_TYPE);
//  pulseSensor.setThreshold(THRESHOLD);

  //   Skip the first SAMPLES_PER_SERIAL_SAMPLE in the loop().
  samplesUntilReport = SAMPLES_PER_SERIAL_SAMPLE;


  //   Now that everything is ready, start reading the PulseSensor signal.
  //if (!pulseSensor.begin()) {
    /*
       PulseSensor initialization failed,
       likely because our Arduino platform interrupts
       aren't supported yet.

       If your Sketch hangs here, try changing USE_PS_INTERRUPT to false.
    */
   // for (;;) {
      // Flash the led to show things didn't work.

   // }
 // }







  // initialize the IMU
  //if (!IMU.begin()) {
  ////  Serial.println("Failed to initialize IMU!");
  //  while (1);
 // }

  // print out the samples rates of the IMUs
  Serial.print("Accelerometer sample rate = ");
  Serial.print(IMU.accelerationSampleRate());
  Serial.println(" Hz");
  Serial.print("Gyroscope sample rate = ");
  Serial.print(IMU.gyroscopeSampleRate());
  Serial.println(" Hz");

  Serial.println();

  // get the TFL representation of the model byte array
  tflModel = tflite::GetModel(HRmodel16);
  if (tflModel->version() != TFLITE_SCHEMA_VERSION) {
    Serial.println("Model schema mismatch!");
    while (1);
  }


  // Create an interpreter to run the model
  tflInterpreter = new tflite::MicroInterpreter(tflModel, tflOpsResolver, tensorArena, tensorArenaSize, &tflErrorReporter);

  // Allocate memory for the model's input and output tensors
  tflInterpreter->AllocateTensors();

  // Get pointers for the model's input and output tensors
  tflInputTensor = tflInterpreter->input(0);
  tflOutputTensor = tflInterpreter->output(0);






//  tflModel1 = tflite::GetModel(HAR);
//  if (tflModel1->version() != TFLITE_SCHEMA_VERSION) {
//    Serial.println("Model schema mismatch!");
//    while (1);
//  }
//
//
//  // Create an interpreter to run the model
//  tflInterpreter1 = new tflite::MicroInterpreter(tflModel1, tflOpsResolver1, tensorArena1, tensorArenaSize1, &tflErrorReporter1);
//
//  // Allocate memory for the model's input and output tensors
//  tflInterpreter1->AllocateTensors();
//
//  // Get pointers for the model's input and output tensors
//  tflInputTensor1 = tflInterpreter1->input(0);
//  tflOutputTensor1 = tflInterpreter1->output(0);



  Serial.print("Number of Dimensions: ");
Serial.println(tflInputTensor1->dims->size);
Serial.print("Dim 1 Size: ");
Serial.println(tflInputTensor1->dims->data[0]);
Serial.print("Dim 2 Size: ");
Serial.println(tflInputTensor1->dims->data[1]);
Serial.print("Dim 3 Size: ");
Serial.println(tflInputTensor1->dims->data[2]);
Serial.print("Input Type: ");
Serial.println(tflInputTensor1->type);
}

void loop() {

  samplesRead = 0;

  while (samplesRead < numSamples) {
   // if (pulseSensor.sawNewSample()) {
      /*
         Every so often, send the latest Sample.
         We don't print every sample, because our baud rate
         won't support that much I/O.
      */
//          if (--samplesUntilReport == (byte) 0) {
//            samplesUntilReport = SAMPLES_PER_SERIAL_SAMPLE;
//      
//           pulseSensor.outputSample();
//      
//            /*
//               At about the beginning of every heartbeat,
//               report the heart rate and inter-beat-interval.
//            */
//            if (pulseSensor.sawStartOfBeat()) {
//              pulseSensor.outputBeat();
//            }
//          }







      float aX, aY, aZ;

IMU.readAcceleration(aX, aY, aZ);

     // uint8_t aX1 = (uint8_t)(pulseSensor.getBeatsPerMinute());
//      uint8_t aY1 = (uint8_t)(pulseSensor.getInterBeatIntervalMs());
   //   uint8_t aZ1 = (uint8_t)(pulseSensor.getLatestSample());
     uint8_t hr = (uint8_t)(analogRead(A3));
      uint8_t eda = (uint8_t)(analogRead(GSR));
    //  Serial.println(pulseSensor.getLatestSample());

      // normalize the IMU data between 0 to 1 and store in the mCircuit Playground Bluefruitodel's
      // input tensor
      tflInputTensor->data.f[samplesRead * 2 + 0] = hr;
      tflInputTensor->data.f[samplesRead * 2 + 1] = eda;



//      tflInputTensor1->data.f[samplesRead * 3 + 0] = aX;
//      tflInputTensor1->data.f[samplesRead * 3 + 1] = aY;
//      tflInputTensor1->data.f[samplesRead * 3 + 2] = aZ;
      Serial.println(aX);


      samplesRead++;
      Serial.println(samplesRead);

      if (samplesRead == numSamples) {
        // Run inferencing
        TfLiteStatus invokeStatus = tflInterpreter->Invoke();
        if (invokeStatus != kTfLiteOk) {
          Serial.println("Invoke failed!");
          while (1);
          return;
        }

        // Loop through the output tensor values from the model
        for (int i = 0; i < NUM_GESTURES; i++) {
          Serial.print(GESTURES[i]);
          Serial.print(": ");
          Serial.println(tflOutputTensor->data.f[i], 3);
        }
        Serial.println();
      }


//      if (samplesRead == numSamples) {
//        // Run inferencing
//        TfLiteStatus invokeStatus1 = tflInterpreter1->Invoke();
//        if (invokeStatus1 != kTfLiteOk) {
//          Serial.println("Invoke failed!");
//          while (1);
//          return;
//        }
//
//        // Loop through the output tensor values from the model
//        for (int i = 0; i < NUM_GESTURES1; i++) {
//          Serial.print(GESTURES1[i]);
//          Serial.print(": ");
//          Serial.println(tflOutputTensor1->data.f[i], 3);
//        }
//        Serial.println();
//      }

      delay(10);

   // }
  }
}
