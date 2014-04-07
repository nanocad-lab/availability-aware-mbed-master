/* main.cpp
 * Tested with mbed board: FRDM-KL46Z
 * Author: Mark Gottscho
 * mgottscho@ucla.edu
 */

#define NDEBUG

#include "mbed.h"
#include "util.h"
#include "SerialManager.h"
#include <string>
#include "MMA8451Q.h" //For talking with the accelerometer
#include "MAG3110.h" //For talking with the magnetometer
#include "ADCSensor.h" //For abstracting ADCs
#include "TouchSensor.h" //For working with the capacitive touch sensor
#include "const.h" //for various constants in global scope
#include "MAX17043.h" //For talking with the battery fuel gauge
#include "INA219.h" //For talking with the power sensors
#include "Timer_LPTMR.h" //Millisecond-resolution hardware timer

using namespace std;

//The console object is global so that all objects can use its generic serial methods.
SerialManager console(PTE16, PTE17, 115200, true); //For serial console
DigitalOut green(LED1);
DigitalOut red(LED2);

typedef struct {
    string name;
    float voltage;
    float current;
    float power;
} power_state_t;

void wake_ISR() { }

int main() {
    //Turn on red and green LEDs during init
    green = 0;
    red = 0;  
    
    __disable_irq(); //No interrupts during init
    
    //Dummy power loads to accompany LEDs
    DigitalOut dummy1(PTA6);
    DigitalOut dummy2(PTA7);
    dummy1 = 0;
    dummy2 = 0;
    
    //System power state
    const float CHARGING_POWER_LEVEL = 0.2;
    const float MAX_POWER_LEVEL = 1;
    const float MIN_POWER_LEVEL = 0;
    float power_level = CHARGING_POWER_LEVEL;
    power_state_t supply_state;
    power_state_t batt_state;
    power_state_t load_state;
    float batt_soc; //percent state of charge
    bool charging = false; //If true, the battery is receiving net power.
    bool supplied = false; //If true, we are plugged in to a supply.
    
    //Init power state
    supply_state.name = "SUPPLY";
    supply_state.voltage = 0;
    supply_state.current = 0;
    supply_state.power = 0;
    batt_state.name = "BATTERY";
    batt_state.voltage = 0;
    batt_state.current = 0;
    batt_state.power = 0;
    load_state.name = "SYSTEM";
    load_state.voltage = 0;
    load_state.current = 0;
    load_state.power = 0;
    batt_soc = 0;
    
    //Power/battery monitoring sensors
    INA219 pwr_sens_supply(PTE0, PTE1, PWR_SENS_SUPPLY_I2C_ADDRESS);
    INA219 pwr_sens_battery(PTE0, PTE1, PWR_SENS_BATTERY_I2C_ADDRESS);
    INA219 pwr_sens_load(PTE0, PTE1, PWR_SENS_LOAD_I2C_ADDRESS);
    MAX17043 fuel_gauge(PTE0, PTE1, FUEL_GAUGE_I2C_ADDRESS);
    
    //Init power sensors
    pwr_sens_supply.selfInit();
    pwr_sens_battery.selfInit();
    pwr_sens_load.selfInit();
    fuel_gauge.selfInit();
 
    //Application Sensors
    MMA8451Q acc(PTE25, PTE24, ACCEL_I2C_ADDRESS);
    MAG3110 mag(PTE25, PTE24, MAG_I2C_ADDRESS);
    ADCSensor light_adc(PTE22); //light sensor
    //TouchSensor touch; //touch sensor. FIXME: Disabled for now, has internal interrupts that break things
    ADCSensor generic_adc(PTB0); // ADC0 input channel 0
    
    //Init application sensors
    acc.selfInit();
    mag.selfInit();
        
    //Turn off red LED when finished init
    green = 1;
    red = 1;

    __enable_irq(); //enable interrupts after initialization

    //Hardware timer interrupt, so that sleep() is possible (can't do it with Ticker!)
    Timer_LPTMR timer;
    timer.enable(&wake_ISR);
    timer.start(500, true, 0); //wake up every 500ms
    
    //The Loop
    while(1) {    
        //Sample power state
        supply_state.voltage = pwr_sens_supply.getBusVoltageFloat(true);
        supply_state.power = pwr_sens_supply.getPowerFloat(true);
        supply_state.current = pwr_sens_supply.getCurrentFloat(false);
        batt_state.voltage = pwr_sens_battery.getBusVoltageFloat(true);
        batt_state.power = pwr_sens_battery.getPowerFloat(true);
        batt_state.current = pwr_sens_battery.getCurrentFloat(false);
        load_state.voltage = pwr_sens_load.getBusVoltageFloat(true);
        load_state.power = pwr_sens_load.getPowerFloat(true);
        load_state.current = pwr_sens_load.getCurrentFloat(false);
        batt_soc = fuel_gauge.getFloatSOC(true);
        
        //Interpret operating power state
        if (supply_state.power > 0.005) //5mW guardband
            supplied = true;
        else
            supplied = false;
            
        if (batt_state.power < -0.005) //-5mW guardband
            charging = true;
        else
            charging = false;
            
        
        
        //Report power state
        console.serial.printf("%s...\r\n", supply_state.name.c_str());
        console.serial.printf("...%0.2f V\r\n", supply_state.voltage);
        console.serial.printf("...%0.2f mA\r\n", supply_state.current * 1000);
        console.serial.printf("...%0.2f mW\r\n", supply_state.power * 1000);
        if (supplied)
            console.serial.printf("...Plugged in!\r\n");
        else
            console.serial.printf("...NOT plugged in!\r\n");
        
        console.serial.printf("%s...\r\n", batt_state.name.c_str());
        console.serial.printf("...%0.2f V\r\n", batt_state.voltage);
        console.serial.printf("...%0.2f mA\r\n", batt_state.current * 1000);
        console.serial.printf("...%0.2f mW\r\n", batt_state.power * 1000);
        console.serial.printf("...%0.02f%% charged\r\n", batt_soc);
        if (charging)
            console.serial.printf("...Charging!\r\n");
        else
            console.serial.printf("...NOT charging!\r\n");
            
        console.serial.printf("%s...\r\n", load_state.name.c_str());
        console.serial.printf("...%0.2f V\r\n", load_state.voltage);
        console.serial.printf("...%0.2f mA\r\n", load_state.current * 1000);
        console.serial.printf("...%0.2f mW\r\n", load_state.power * 1000);
        
        //Make a decision about power load. We have a dynamic range of ~2X
        if (supplied && !charging) //Plugged in and fully charged
            power_level = MAX_POWER_LEVEL;
        else if (supplied && charging) //Plugged in and charging
            power_level = CHARGING_POWER_LEVEL;
        else if (!supplied)
            power_level = MIN_POWER_LEVEL;
        else //uh oh
            warn(&console.serial, "Did not detect power supply, but battery was charging!", 0);
            
        console.serial.printf("Normalized output power level set to %0.02f\r\n", power_level);
            
        //Adjust power output
        green = 1-power_level;
        red = 1-power_level;
        dummy1 = power_level;
        dummy2 = power_level;
        
        sleep(); //sleep until an interrupt arrives
    }
}