#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <SPI.h>
#include <LSM6DSOXSensor.h>
#include <Ripple_inferencing.h>

LSM6DSOXSensor lsm6dsoxSensor(&SPI1, D6);

/* Constant defines -------------------------------------------------------- */
#define CONVERT_G_TO_MS2 9.80665f
#define MAX_ACCEPTED_RANGE 2.0f // starting 03/2022, models are generated setting range to +-2, but this example use Arudino library which set range to +-4g. If you are using an older model, ignore this value and use 4.0f instead

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static uint32_t run_inference_every_ms = 200;
static rtos::Thread inference_thread(osPriorityLow);
static float buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {0};
static float inference_buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE];

/* Forward declaration */
void run_inference_background();

void setup()
{
    Serial.begin(115200);

    SPI1.begin();

    lsm6dsoxSensor.begin();

    // Enable accelerometer and gyroscope, and check success
    if (lsm6dsoxSensor.Enable_X() != LSM6DSOX_OK)
    {
        while (1)
            ;
    }

    // Read ID of device and check that it is correct
    uint8_t id;
    lsm6dsoxSensor.ReadID(&id);

    if (id != LSM6DSOX_ID)
    {
        while (1)
            ;
    }
    lsm6dsoxSensor.Set_X_FS(2);
    lsm6dsoxSensor.Set_X_ODR(104.0f);

    if (EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME != 3)
    {
        ei_printf("ERR: EI_CLASSIFIER_RAW_SAMPLES_PER_FRAME should be equal to 3 (the 3 sensor axes)\n");
        return;
    }

    inference_thread.start(mbed::callback(&run_inference_background));
}

/**
 * @brief Return the sign of the number
 *
 * @param number
 * @return int 1 if positive (or 0) -1 if negative
 */
float ei_get_sign(float number)
{
    return (number >= 0.0) ? 1.0 : -1.0;
}

/**
 * @brief      Run inferencing in the background.
 */
void run_inference_background()
{
    // wait until we have a full buffer
    delay((EI_CLASSIFIER_INTERVAL_MS * EI_CLASSIFIER_RAW_SAMPLE_COUNT) + 100);

    // This is a structure that smoothens the output result
    // With the default settings 70% of readings should be the same before classifying.
    ei_classifier_smooth_t smooth;
    ei_classifier_smooth_init(&smooth, 10 /* no. of readings */, 7 /* min. readings the same */, 0.8 /* min. confidence */, 0.3 /* max anomaly */);

    while (1)
    {
        // copy the buffer
        memcpy(inference_buffer, buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE * sizeof(float));

        // Turn the raw buffer in a signal which we can the classify
        signal_t signal;
        int err = numpy::signal_from_buffer(inference_buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, &signal);
        if (err != 0)
        {
            ei_printf("Failed to create signal from buffer (%d)\n", err);
            return;
        }

        // Run the classifier
        ei_impulse_result_t result = {0};

        err = run_classifier(&signal, &result, debug_nn);
        if (err != EI_IMPULSE_OK)
        {
            ei_printf("ERR: Failed to run classifier (%d)\n", err);
            return;
        }

        // print the predictions
        ei_printf("Predictions ");
        ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
                  result.timing.dsp, result.timing.classification, result.timing.anomaly);
        ei_printf(": ");

        // ei_classifier_smooth_update yields the predicted label
        const char *prediction = ei_classifier_smooth_update(&smooth, &result);
        ei_printf("%s ", prediction);
        // print the cumulative results
        ei_printf(" [ ");
        for (size_t ix = 0; ix < smooth.count_size; ix++)
        {
            ei_printf("%u", smooth.count[ix]);
            if (ix != smooth.count_size + 1)
            {
                ei_printf(", ");
            }
            else
            {
                ei_printf(" ");
            }
        }
        ei_printf("]\n");

        delay(run_inference_every_ms);
    }

    ei_classifier_smooth_free(&smooth);
}

void loop()
{

    while (1)
    {
        uint8_t acceleroStatus;
        while (1)
        {
            lsm6dsoxSensor.Get_X_DRDY_Status(&acceleroStatus);
            if (acceleroStatus == 1)
            {
                break;
            }
            else
            {
                delay(1);
            }
        }

        if (acceleroStatus == 1)
        { // Status == 1 means a new data is available

            // roll the buffer -3 points so we can overwrite the last one
            numpy::roll(buffer, EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE, -3);

            // Determine the next tick (and then sleep later)
            uint64_t next_tick = micros() + (EI_CLASSIFIER_INTERVAL_MS * 1000);

            int32_t acceleration[3];
            lsm6dsoxSensor.Get_X_Axes(acceleration);

            buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 3] = (acceleration[0] / 1000.0);
            buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 2] = (acceleration[1] / 1000.0);
            buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 1] = (acceleration[2] / 1000.0);

            for (int i = 0; i < 3; i++)
            {
                if (fabs(buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 3 + i]) > MAX_ACCEPTED_RANGE)
                {
                    buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 3 + i] = ei_get_sign(buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 3 + i]) * MAX_ACCEPTED_RANGE;
                }
            }

            buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 3] *= CONVERT_G_TO_MS2;
            buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 2] *= CONVERT_G_TO_MS2;
            buffer[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE - 1] *= CONVERT_G_TO_MS2;

            // and wait for next tick
            uint64_t time_to_wait = next_tick - micros();
            delay((int)floor((float)time_to_wait / 1000.0f));
            delayMicroseconds(time_to_wait % 1000);
        }
    }
}