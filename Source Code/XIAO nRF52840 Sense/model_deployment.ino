#include <TeleMedicine_inferencing.h>

#include <LSM6DS3.h>
#include <U8g2lib.h>
#include <U8X8lib.h>
#include <Wire.h>
#include <SoftwareSerial.h>

SoftwareSerial mySerial(2,3); // RX, TX pins for software serial


/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2    9.80665f
#define MAX_ACCEPTED_RANGE  2.0f        // starting 03/2022, models are generated setting range to +-2, but this example use Arudino library which set range to +-4g. If you are using an older model, ignore this value and use 4.0f instead

float threshold = 0.5;

U8X8_SSD1306_64X48_ER_HW_I2C u8x8(/* reset=*/ U8X8_PIN_NONE); 

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
LSM6DS3 myIMU(I2C_MODE, 0x6A);

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    //u8g2.begin();
    u8x8.begin();
    Serial.println("Edge Impulse Inferencing Demo");

    //if (!IMU.begin()) {
      if (!myIMU.begin()) {
        ei_printf("Failed to initialize IMU!\r\n");
    }
    else {
        ei_printf("IMU initialized\r\n");
    }

    if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 3) {
        ei_printf("ERR: EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME should be equal to 3 (the 3 sensor axes)\n");
        return;
    }
}


float ei_get_sign(float number) {
    return (number >= 0.0) ? 1.0 : -1.0;
}


void loop()
{
    uint8_t buf1[64]="idle";
    uint8_t buf2[64]="left&right";
    uint8_t buf3[64]="up&down";
    
    u8x8.clear();
    u8x8.setFont(u8g2_font_ncenB08_tr);
  
    ei_printf("\nStarting inferencing in 2 seconds...\n");

    delay(2000);

    ei_printf("Sampling...\n");

    // Allocate a buffer here for the values we'll read from the IMU
    float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = { 0 };

    for (size_t ix = 0; ix < EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE; ix += 3) {
        // Determine the next tick (and then sleep later)
        uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);

        buffer[ix] = myIMU.readFloatAccelX();
        buffer[ix+1] = myIMU.readFloatAccelY();
        buffer[ix+2] = myIMU.readFloatAccelZ();
        
        //buffer[ix] = myIMU.readFloatGyroX();
        //buffer[ix+1] = myIMU.readFloatGyroY();
        //buffer[ix+2] = myIMU.readFloatGyroZ();

        for (int i = 0; i < 3; i++) {
            if (fabs(buffer[ix + i]) > MAX_ACCEPTED_RANGE) {
                buffer[ix + i] = ei_get_sign(buffer[ix + i]) * MAX_ACCEPTED_RANGE;
            }
        }

        buffer[ix + 0] *= CONVERT_G_TO_MS2;
        buffer[ix + 1] *= CONVERT_G_TO_MS2;
        buffer[ix + 2] *= CONVERT_G_TO_MS2;

        delayMicroseconds(next_tick - micros());
    }

    // Turn the raw buffer in a signal which we can the classify
    signal_t signal;
    int err = numpy::signal_from_buffer(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
    if (err != 0) {
        ei_printf("Failed to create signal from buffer (%d)\n", err);
        return;
    }

    // Run the classifier
    ei_impulse_result_t result = { 0 };

    err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        return;
    }

    // print the predictions
    ei_printf("Predictions ");
    ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
        result.timing.dsp, result.timing.classification, result.timing.anomaly);
    ei_printf(": \n");
    
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) 
        {
            ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
            String Label = result.classification[ix].label;
            int Value = result.classification[ix].value;
            if(Label == "NORMAL_MOVEMENT")
            { Serial.println(Label);
              if(Value > 0.9)
              { if (result.anomaly<0.5){
                  mySerial.write('C');
                  Serial.println("Normal");
                }
              }
            }
            if(result.anomaly > 0.5)
            {
                mySerial.write('A');
                Serial.println("Anomaly");
            }
        }

#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("    anomaly score: %.3f\n", result.anomaly);

#endif
 
}
