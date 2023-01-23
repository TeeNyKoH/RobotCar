#include "pico/stdlib.h"
#include <stdio.h>
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/uart.h"
#include <string.h>
#include "pico/binary_info.h"
#include "hardware/i2c.h"
#include <stdlib.h>
#include <time.h>
#include "hardware/irq.h"

/* By default these devices are on bus address 0x68 */ 

static int addr = 0x68;
float ultrasonic_Front = 0, ultrasonic_Avg_Front = 0, ultrasonic_Avg_Left = 0, ultrasonic_Avg_Right = 0, ultrasonic_Left = 0, ultrasonic_Right = 0; 
double accelo_Diff_X = 0, accelo_Diff_Y = 0, accelo_Height = 0, accelo_new_Diff = 0, accelo_Avg_Diff = 0, accelo_All_Height_Average = 0, accelo_Total_All_Height_Average = 0;
double accelo_First_Var_X = 0, accelo_First_Var_Y = 0;
int count = 0;
uint trigPin = 0;
uint echoPin1 = 1;
uint echoPin2 = 15;
uint echoPin3 = 16;

#ifdef i2c_default
static void mpu6050_reset() {
    // Two byte reset. First byte register, second byte data
    // There are a load more options to set up the device in different ways that could be added here
    uint8_t buf[] = {0x6B, 0x00};
    i2c_write_blocking(i2c_default, addr, buf, 2, false);
}

static void mpu6050_read_raw(int16_t accel[3], int16_t gyro[3], int16_t *temp) {
    // For this particular device, we send the device the register we want to read
    // first, then subsequently read from the device. The register is auto incrementing
    // so we don't need to keep sending the register we want, just the first.

    uint8_t buffer[6];

    // Start reading acceleration registers from register 0x3B for 6 bytes
    uint8_t val = 0x3B;
    i2c_write_blocking(i2c_default, addr, &val, 1, true); // true to keep master control of bus
    i2c_read_blocking(i2c_default, addr, buffer, 6, false);

    for (int i = 0; i < 3; i++) 
    {
        accel[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);
    }

    // Now gyro data from reg 0x43 for 6 bytes
    // The register is auto incrementing on each read
    val = 0x43;
    i2c_write_blocking(i2c_default, addr, &val, 1, true);
    i2c_read_blocking(i2c_default, addr, buffer, 6, false);  // False - finished with bus

    for (int i = 0; i < 3; i++) 
    {
        gyro[i] = (buffer[i * 2] << 8 | buffer[(i * 2) + 1]);;
    }

    // Now temperature from reg 0x41 for 2 bytes
    // The register is auto incrementing on each read
    val = 0x41;
    i2c_write_blocking(i2c_default, addr, &val, 1, true);
    i2c_read_blocking(i2c_default, addr, buffer, 2, false);  // False - finished with bus

    *temp = buffer[0] << 8 | buffer[1];
}
#endif

/* Setup for ultrasonic Pins */
void setupUltrasonicPins(uint trigPin, uint echoPin)
{
    gpio_init(trigPin);
    gpio_init(echoPin);
    
    gpio_set_dir(trigPin, GPIO_OUT);
    gpio_set_dir(echoPin, GPIO_IN);

    gpio_pull_up(echoPin);
}

/* Setup for Ultrasonic Pulse */
int getPulse(uint trigPin, uint echoPin)
{
    int loop1 = 0;
    int loop2 = 0;
    
    busy_wait_us(2);
    gpio_put(trigPin, 1);
    busy_wait_us(10);
    gpio_put(trigPin, 0);
    busy_wait_us(2);

    while (gpio_get(echoPin) == 0 ) 
    {
        loop1++;
        busy_wait_us(1);
        if(loop1 > 5000){
            break;
        }
    }
    
    while (gpio_get(echoPin) != 0 ) 
    {
        loop2++;
        busy_wait_us(1);
        if(loop2 > 3000){
            return 0;
        }
    }
    return loop2;
}

/* Converting pulse into cm */
float getCm(uint trigPin, uint echoPin)
{
    int pulseLength = getPulse(trigPin, echoPin);
    return (float) pulseLength / 2 * 0.0343;
}

/* Calculate the distance from the car to the wall on each side */
float getUSDectection(uint trigPin, uint echoPin1, uint echoPin2, uint echoPin3)
{
    /* Getting the cm for each sensor */  
    ultrasonic_Front = getCm(trigPin, echoPin1);
    ultrasonic_Left = getCm(trigPin, echoPin2);
    ultrasonic_Right = getCm(trigPin, echoPin3);

    /* Get the average of the directions for smoothing  */  
    for (int i = 0; i < 50; ++i)
    {
        ultrasonic_Avg_Front = ultrasonic_Front + ultrasonic_Avg_Front;
        ultrasonic_Avg_Left = ultrasonic_Left + ultrasonic_Avg_Left;
        ultrasonic_Avg_Right = ultrasonic_Right + ultrasonic_Avg_Right;
    }

    /* Hardcoded addition as sensors give a set wrong value */
    ultrasonic_Avg_Front = ( ultrasonic_Avg_Front / 50 ) + 2.5;
    ultrasonic_Avg_Left = ( ultrasonic_Avg_Left / 50 ) + 2.5;
    ultrasonic_Avg_Right = ( ultrasonic_Avg_Right / 50 ) + 2.5;
    
    /* Displaying the results obtained after smoothing */
    printf("%.2f cm front avg\n",ultrasonic_Avg_Front);
    printf("%.2f cm left avg\n",ultrasonic_Avg_Left);
    printf("%.2f cm right avg\n",ultrasonic_Avg_Right);
}

/* Setup for accelorometer detection */
float getAcelloDectection()
{
    /* Use I2C0 on the default SDA and SCL pins(4, 5 on a Pico) */
    int16_t acceleration[3], gyro[3], temp;
    i2c_init(i2c_default, 400 * 1000);
    gpio_set_function(PICO_DEFAULT_I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(PICO_DEFAULT_I2C_SDA_PIN);
    gpio_pull_up(PICO_DEFAULT_I2C_SCL_PIN);

    /* Make the I2C pins available to picotool */
    bi_decl(bi_2pins_with_func(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, GPIO_FUNC_I2C));
    mpu6050_reset();

    /* Getting first read value for comparison later */
    mpu6050_read_raw(acceleration, gyro, &temp);
    busy_wait_ms(3000); // wait to remove the noise at the start of the read
    accelo_First_Var_X = accelo_First_Var_X + acceleration[0];
    accelo_First_Var_Y = accelo_First_Var_Y + acceleration[1];
    
}

/* Setup to calculate humps */
float getHump()  
{
    int16_t acceleration[3], gyro[3], temp;
    mpu6050_read_raw(acceleration, gyro, &temp);
    accelo_Diff_X = accelo_First_Var_X - acceleration[0];
    accelo_Diff_Y = accelo_First_Var_Y - acceleration[1];
    accelo_Avg_Diff = (accelo_Diff_X + accelo_Diff_Y) / 2;
    accelo_Height = accelo_Avg_Diff / 931.63;

    /* Filtering away all negative values */
    if (accelo_Height < 0){           
        accelo_Height = 0;
    }

    /* Only count detected height above 0.7cm as a hump */
    if (accelo_Height >= 0.7)
    {
        count = count + 1;
        accelo_All_Height_Average = accelo_Height + accelo_All_Height_Average; // Tally all recorded heights 
        accelo_Total_All_Height_Average = accelo_All_Height_Average / count;   // Averaging out the height

        /* Minimum of 15 samples to go over a hump */
        if (count > 15)
        {
            printf("Hump Detected, avgheight = %f\n", accelo_Total_All_Height_Average , "cm");
        }
    }

    /* Resetting all variables to 0 for next hump detection */
    else
    {
        accelo_Total_All_Height_Average = 0;
        count = 0;
        accelo_All_Height_Average = 0;
    }
}
/* Repeated Timer to call function */
bool repeating_timer_callback(struct repeating_timer *t)
{
    getUSDectection(trigPin, echoPin1, echoPin2, echoPin3);
    getHump();
    return true;
}


int main() {
    stdio_init_all();
    /* Check for all PICO I2C is defined */
#if !defined(i2c_default) || !defined(PICO_DEFAULT_I2C_SDA_PIN) || !defined(PICO_DEFAULT_I2C_SCL_PIN)
    #warning i2c/mpu6050_i2c example requires a board with I2C pins
    puts("Default I2C pins were not defined");
#else
    sleep_ms(1000);

    /* Setting up all ultrasonic pins */
    setupUltrasonicPins(trigPin, echoPin1); //Front
    setupUltrasonicPins(trigPin, echoPin2); //Left
    setupUltrasonicPins(trigPin, echoPin3); //Right

    struct repeating_timer timer;
    getAcelloDectection();
    
    add_repeating_timer_ms(50, repeating_timer_callback, NULL, &timer); 

    while (1) {
        tight_loop_contents();
        getHump();
        sleep_ms(40);
    }
     
    #endif
}



