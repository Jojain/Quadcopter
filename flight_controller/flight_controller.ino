#include "Wire.h"
#include "Simple_MPU6050.h"
#include <LiquidCrystal.h>


#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS     MPU6050_ADDRESS_AD0_LOW

#define OFFSETS  -1866,    -528,    8580,     395,    -141,      42

#define CHANNEL1 0
#define CHANNEL2 1
#define CHANNEL3 2
#define CHANNEL4 3
#define CHANNEL5 4
#define CHANNEL6 5

#define YAW      0
#define PITCH    1
#define ROLL     2
#define THROTTLE 3
#define SWITCH   4
#define DIAL     5

#define ANGLE_MAX_ROTATION_PITCH_ROLL 25 //change this value in order to get a bigger angle of rotation when joystick is touching limits
#define ROTATION_SPEED_MAX_YAW 360       // 360°/s

#define REFRESH_RATE 4000 // Period in µs at which ESC pulses are sent.
//--------------------------------------------------------------------------------------------------------------------------------------------------------//
//                                                               Définition des variables                                                                 //
//--------------------------------------------------------------------------------------------------------------------------------------------------------//
LiquidCrystal lcd(16,15,3,4,5,6);
Simple_MPU6050 mpu;
Quaternion q;
VectorFloat gravity;
ENABLE_MPU_OVERFLOW_PROTECTION();

volatile unsigned long current_time;

volatile unsigned long timer[6];

// Previous state of each channel (HIGH or LOW)
volatile byte previous_state[6];

// Duration of the pulse on each channel in µs (must be within 1000µs & 2000µs)
volatile unsigned int pulse_duration[6] = {1500, 1500, 1000, 1500, 1000, 1000};
volatile unsigned int motors_pulses[4] = {1000, 1000, 1000, 1000};

unsigned long time_start_loop, loop_timer, display_time, now, while_now, loop_count;

const int LCD_BLINK_DELAY = 500; // Délai de clignottement de l'écran LCD lors de la configuration du PID

float ypr[3] = { 0, 0, 0 };
float zyx[3] = { 0, 0, 0 };
float gyro_yaw, dampened_yaw_speed = 0;
int mode_mapping[6] = {0,0,0,0,0,0};

bool PID_setup_mode = false;
bool yaw = false;
bool pitch = false;
bool roll = false;
bool blink_state = false;
bool flying_display_off = true;

//--------------------------------------------------------------------------------------------------------------------------------------------------------//
//                                                           Définition des structures                                                                    //
//--------------------------------------------------------------------------------------------------------------------------------------------------------//
typedef struct SetupPosition
{
    float yaw;   //speed of rotation in degrees/s
    float pitch; //angle of inclination in degrees
    float roll;  //angle of inclination in degrees
} SetupPosition;

typedef struct  PID
{
    float kp;
    float ki;
    float kd;
} PID;

typedef struct Errors
{   
    float previous;
    float actual;
    float sum;
    float derivate;
    
} Errors;

PID YAW_PID = {2,0.000,0};
PID PITCH_PID = {3,0.001,0};
PID ROLL_PID = {3,0.001,0};

Errors yaw_errors = {0,0,0};
Errors pitch_errors = {0,0,0};
Errors roll_errors = {0,0,0};

SetupPosition pos_required;


//--------------------------------------------------------------------------------------------------------------------------------------------------------//
//                                                           Définition des fonctions                                                                     //
//--------------------------------------------------------------------------------------------------------------------------------------------------------//


void update_angles(int16_t *gyro, int16_t *accel, int32_t *quat, uint32_t *timestamp){

    mpu.GetQuaternion(&q, quat);
    mpu.GetGravity(&gravity, &q);
    mpu.GetYawPitchRoll(ypr, &q, &gravity);
    mpu.ConvertToDegrees(ypr, zyx);
    gyro_yaw = gyro[2];
    
    // Serial.println(gyro[2]);
    // Serial.print(zyx[0]);
    // Serial.print(" ° Z\t");
    // Serial.print(zyx[1]);
    // Serial.print(" ° Y\t");
    // Serial.print(zyx[2]);
    // Serial.print(" ° X\t");
    // Serial.println("");  
}
/* 
Calcule les erreurs en se basant sur la position angulaire du drone
Les angles sont traités en degrés
*/
void calculate_errors(struct SetupPosition pos_required)
{
    // auto [z,y,x] = zyx;
    auto [discarded,y,x] = zyx;
    float yaw_speed = gyro_yaw/16.4; // 16.4 is the sensitivity scale factor for the setup 2000 °/s
    float dampened_yaw_speed = dampened_yaw_speed*0.7 + 0.3* yaw_speed;
    
    yaw_errors.actual = dampened_yaw_speed - pos_required.yaw;
    yaw_errors.sum += yaw_errors.actual;
    yaw_errors.derivate = yaw_errors.actual - yaw_errors.previous;
    yaw_errors.previous = yaw_errors.actual;

    pitch_errors.actual = x - pos_required.pitch;
    pitch_errors.sum += pitch_errors.actual;
    pitch_errors.derivate = pitch_errors.actual - pitch_errors.previous;
    pitch_errors.previous = pitch_errors.actual;

    roll_errors.actual = y - pos_required.roll;
    roll_errors.sum += roll_errors.actual;
    roll_errors.derivate = roll_errors.actual - roll_errors.previous;
    roll_errors.previous = yaw_errors.actual;  

    yaw_errors.sum = min_max(yaw_errors.sum, -400/YAW_PID.ki, 400/YAW_PID.ki);  
    pitch_errors.sum = min_max(pitch_errors.sum, -400/PITCH_PID.ki, 400/PITCH_PID.ki);  
    roll_errors.sum = min_max(roll_errors.sum, -400/ROLL_PID.ki, 400/ROLL_PID.ki);  

}

SetupPosition RF_to_angle(int *yaw_pulse, int *pitch_pulse, int *roll_pulse)
{
    float yaw_rotation_speed = 0;
    float pitch_angle = 0;
    float roll_angle = 0;
    
    SetupPosition position_generated;

    yaw_rotation_speed = *yaw_pulse - 1500;
    if (abs(yaw_rotation_speed) > 12)
    {
        yaw_rotation_speed = yaw_rotation_speed * ROTATION_SPEED_MAX_YAW/500;
        
    }
    pitch_angle = *pitch_pulse - 1500;
    if (abs(pitch_angle) > 12)
    {
        pitch_angle = pitch_angle * ANGLE_MAX_ROTATION_PITCH_ROLL/500;
    }
    roll_angle = *roll_pulse - 1500;
    if (abs(roll_angle) > 12)
    {
        roll_angle = roll_angle * ANGLE_MAX_ROTATION_PITCH_ROLL/500;
    }

    position_generated = {yaw_rotation_speed, pitch_angle, roll_angle};

    return position_generated;
    
}

void calculate_motors_pulses()
{
    int yaw_correction, pitch_correction, roll_correction;
    int throttle = pulse_duration[mode_mapping[THROTTLE]];

    yaw_correction = yaw_errors.actual * YAW_PID.kp + yaw_errors.sum * YAW_PID.ki + yaw_errors.derivate * YAW_PID.kd;
    pitch_correction = pitch_errors.actual * PITCH_PID.kp + pitch_errors.sum * PITCH_PID.ki + pitch_errors.derivate * PITCH_PID.kd;
    roll_correction = roll_errors.actual * ROLL_PID.kp + roll_errors.sum * ROLL_PID.ki + roll_errors.derivate * ROLL_PID.kd;

    throttle = min_max(throttle,1000,1800); // setting throttle to 80% max in order to leave some room for the PID to speed up even more motors if turns are needed

    if (throttle > 1050 )    
    {
        motors_pulses[0] = throttle - roll_correction - pitch_correction - yaw_correction;
        motors_pulses[1] = throttle + roll_correction - pitch_correction + yaw_correction;
        motors_pulses[2] = throttle + roll_correction + pitch_correction - yaw_correction;
        motors_pulses[3] = throttle - roll_correction + pitch_correction + yaw_correction;
        motors_pulses[0] = min_max(motors_pulses[0],1100,2000); // Cap la vitesse basse pour que les moteurs tournent au ralenti
        motors_pulses[1] = min_max(motors_pulses[1],1100,2000);
        motors_pulses[2] = min_max(motors_pulses[2],1100,2000);
        motors_pulses[3] = min_max(motors_pulses[3],1100,2000);
    }

    else
    {
        for (int i = 0; i < 4; i++)
        {
            motors_pulses[i] = 1000;
        }
        
    }    



    for (int i = 0; i < 4; i++)
    {
        motors_pulses[i] = min_max(motors_pulses[i],1000,2000); // setting signals between 1000 µs and 2000 µs
    }
    
}

void configure_channel_mapping() {
    mode_mapping[YAW]      = CHANNEL4;
    mode_mapping[PITCH]    = CHANNEL2;
    mode_mapping[ROLL]     = CHANNEL1;
    mode_mapping[THROTTLE] = CHANNEL3;
    mode_mapping[SWITCH]   = CHANNEL5;
    mode_mapping[DIAL]     = CHANNEL6;
}

void apply_motor_speed() 
{
    unsigned long time_elapsed = 0, now;


    // Refresh rate is 250Hz: send ESC pulses every 4000µs

    while ((now = micros()) - loop_timer < REFRESH_RATE)
        //Waiting 4000µs before actually executing the function;
    
    // Update loop timer
    loop_timer = 0; // Cette ligne est obligatoire sinon loop_timer ne change jamais de valeur, aucune idée de l'origine du bug...
    loop_timer = now;
    // Set pins #49 #48 #47 #46 HIGH

    PORTL |= B00001111;    
 
    // Wait until all pins #49 #48 #47 #46 are LOW
    while (PORTL > 0) 
    {        
        now        = micros();
        time_elapsed = now - loop_timer;         
        
        if (time_elapsed >= motors_pulses[0]) PORTL &= B11111110; // Set pin #49 LOW

        if (time_elapsed >= motors_pulses[1]) PORTL &= B11111101; // Set pin #48 LOW

        if (time_elapsed >= motors_pulses[2]) PORTL &= B11111011; // Set pin #47 LOW

        if (time_elapsed >= motors_pulses[3]) PORTL &= B11110111; // Set pin #46 LOW

    }
    
}
// long count=0;
// void apply_motor_speed() 
// {
//     unsigned long time_elapsed = 0,t1,t2,t3,t4;
    
//     t1=0;
//     t2=0;
//     t3=0;
//     t4=0;
//     // Refresh rate is 250Hz: send ESC pulses every 4000µs

//     while ((now = micros()) - loop_timer < REFRESH_RATE)
//         //Waiting 4000µs before actually executing the function;
//     //PROBLEME DANS LE NOW du while, j'ai beau le print, il ne change jamais pour le loop_timer
//     // Update loop timer
//     loop_timer = 0; // Cette ligne est obligatoire sinon loop_timer ne change jamais de valeur, aucune idée de l'origine du bug...
//     loop_timer = now; 
//     // Set pins #49 #48 #47 #46 HIGH

//     PORTL |= B00001111;    
//     count += 1;
//     // Wait until all pins #49 #48 #47 #46 are LOW
//     while (PORTL > 0) {        
//         now        = micros();
//         time_elapsed = now - loop_timer;  
//         // Serial.print(now);
//         // Serial.print("\t");
//         // Serial.print(loop_timer);
//         // Serial.print("\t");
//         // Serial.println(count);
        

        
//         if (time_elapsed >= motors_pulses[0]) {
//             PORTL &= B11111110 ;// Set pin #49 
//             if (t1==0) t1=time_elapsed;
            

//         }

//         if (time_elapsed >= motors_pulses[1]) 
//         {
//             PORTL &= B11111101;
//             if (t2==0) t2=time_elapsed;

//         } // Set pin #48 LOW

//         if (time_elapsed >= motors_pulses[2]){
//             PORTL &= B11111011; // Set pin #47 LOW
//             if (t3==0) t3=time_elapsed;
//         } 

//         if (time_elapsed >= motors_pulses[3])
//         {
//             PORTL &= B11110111; // Set pin #46 LOW
//             if (t4==0) t4=time_elapsed;
//         } 
//     }
//     Serial.print(t1);
//     Serial.print("\t");
//     Serial.print(t2);
//     Serial.print("\t");
//     Serial.print(t3);
//     Serial.print("\t");
//     Serial.println(t4);
// }
int min_max(int pulse, int min, int max)
{
    if (pulse < min)
    {
        return min;
    }
    else if (pulse > max)
    {
        return max;
    }
    else
    {
        return pulse;
    }
    
}

/* Debug purpose function, print anything on the lcd */
void print(String string)
{
    lcd.clear();
    lcd.print(string);
}
void print(long string)
{
    lcd.clear();
    lcd.print(string);
}
void print_motors_pulses()
{
    for (int i = 0; i < 4; i++)
    {
        Serial.print(motors_pulses[i]);
        Serial.print("\t");
        Serial.print(i+1);
        Serial.print("\t");

    }
    // Serial.println("");
}
void print_coeffs()
{
    Serial.print("Yaw pid : ");
    Serial.print("(kp ");
    Serial.print(YAW_PID.kp);
    Serial.print(")");
    Serial.print(" (ki ");
    Serial.print(YAW_PID.ki);
    Serial.print(")");
    Serial.print(" (kd ");
    Serial.print(YAW_PID.kd);
    Serial.print(")");

    // Serial.print(" pitch pid : ");
    // Serial.print("(kp ");
    // Serial.print(PITCH_PID.kp);
    // Serial.print(")");
    // Serial.print(" (ki ");
    // Serial.print(PITCH_PID.ki);
    // Serial.print(")");
    // Serial.print(" (kd ");
    // Serial.print(PITCH_PID.kd);
    // Serial.print(")");

    // Serial.print(" roll pid : ");
    // Serial.print("(kp ");
    // Serial.print(ROLL_PID.kp);
    // Serial.print(")");
    // Serial.print(" (ki ");
    // Serial.print(ROLL_PID.ki);
    // Serial.print(")");
    // Serial.print(" (kd ");
    // Serial.print(ROLL_PID.kd);
    // Serial.print(")");
    
    Serial.println("");
}
/* 
Function that displays the current value of the PID coefficient being changed
It display the current axis selected by making it blink
*/
void display_lcd()
{
    String full;
    String empty;
    full =  "YAW  PITCH  ROLL";
        
    if (yaw)
    {        
        empty = "     PITCH  ROLL";
    }
    else if (pitch)
    {
        empty = "YAW         ROLL";
    }
    else if (roll)
    {
        empty = "YAW  PITCH      ";
    }

    if ((millis() - display_time) > LCD_BLINK_DELAY)
    {
        blink_state = !blink_state;
        
        if (blink_state)
        {
            lcd.home();
            lcd.print(full); 
            display_time = millis();   
        }
        else
        {
            lcd.home();
            lcd.print(empty);
            display_time = millis();   
        }
    }

}
/* 
This function is only called when the SWITCH is on, making the flight controllor go to setup PID mode
while this mode is active you cannot control the drone but instead you can change the PID values from the radiocommand
It makes the process of finding PID values much more easy and quick.
*/
void setup_PID()
{

    
    int throttle = pulse_duration[mode_mapping[THROTTLE]];
    const float DIV_KP = 100;
    const float DIV_KI = 10000;
    const float DIV_KD = 100;

    if (pulse_duration[mode_mapping[YAW]] > 1800)
    {
        yaw = true;
        pitch = false;
        roll = false;
    }
    else if (pulse_duration[mode_mapping[YAW]] < 1200)
    {
        yaw = false;
    }

    if (pulse_duration[mode_mapping[PITCH]] > 1800)
    {
        yaw = false;
        pitch = true;
        roll = false;
    }
    else if (pulse_duration[mode_mapping[PITCH]] < 1200)
    {
        pitch = false;
    }

    if (pulse_duration[mode_mapping[ROLL]] > 1800)
    {
        yaw = false;
        pitch = false;
        roll = true;
    }
    else if (pulse_duration[mode_mapping[ROLL]] < 1200)
    {
        roll = false;
    }



    
    switch (yaw ? 0 : (pitch ? 1 : roll ? 2 : 3))
    {
        case 0 :
            switch ((throttle < 1200) ? 0 : ((throttle > 1200 && throttle < 1800) ? 1 : 2)) 
            {
            case 0 :
                YAW_PID.kd = (pulse_duration[mode_mapping[DIAL]]-1000) / DIV_KP;
                lcd.setCursor(0,1);
                lcd.print("Kd :            ");
                lcd.setCursor(5,1);
                lcd.print(YAW_PID.kd);
                break;
            case 1 :
                YAW_PID.ki = (pulse_duration[mode_mapping[DIAL]]-1000) / DIV_KI;
                lcd.setCursor(0,1);
                lcd.print("Ki :            ");
                lcd.setCursor(5,1);
                lcd.print(YAW_PID.ki);
                break;   
            case 2 :
                YAW_PID.kp = (pulse_duration[mode_mapping[DIAL]]-1000) / DIV_KD;
                lcd.setCursor(0,1);
                lcd.print("Kp :            ");
                lcd.setCursor(5,1);
                lcd.print(YAW_PID.kp);
                break;
            }
        case 1 :
            switch ((throttle < 1200) ? 0 : ((throttle > 1200 && throttle < 1800) ? 1 : 2)) 
            {
            case 0 :
                PITCH_PID.kd = (pulse_duration[mode_mapping[DIAL]]-1000) / DIV_KP;
                lcd.setCursor(0,1);
                lcd.print("Kd :            ");
                lcd.setCursor(5,1);
                lcd.print(PITCH_PID.kd);
                break;
            case 1 :
                PITCH_PID.ki = (pulse_duration[mode_mapping[DIAL]]-1000) / DIV_KI;
                lcd.setCursor(0,1);
                lcd.print("Ki :            ");
                lcd.setCursor(5,1);
                lcd.print(PITCH_PID.ki);
                break;   
            case 2 :
                PITCH_PID.kp = (pulse_duration[mode_mapping[DIAL]]-1000) / DIV_KD;
                lcd.setCursor(0,1);
                lcd.print("Kp :            ");
                lcd.setCursor(5,1);
                lcd.print(PITCH_PID.kp);
                break;
            }
        case 2 :
            switch ((throttle < 1200) ? 0 : ((throttle > 1200 && throttle < 1800) ? 1 : 2))  
            {
            case 0 :
                ROLL_PID.kd = (pulse_duration[mode_mapping[DIAL]]-1000) / DIV_KP;
                lcd.setCursor(0,1);
                lcd.print("Kd :            ");
                lcd.setCursor(5,1);
                lcd.print(ROLL_PID.kd);
                break;
            case 1 :
                ROLL_PID.ki = (pulse_duration[mode_mapping[DIAL]]-1000) / DIV_KI;
                lcd.setCursor(0,1);
                lcd.print("Ki :            ");
                lcd.setCursor(5,1);
                lcd.print(ROLL_PID.ki);
                break;   
            case 2 :
                ROLL_PID.kp = (pulse_duration[mode_mapping[DIAL]]-1000) / DIV_KD + 1;
                lcd.setCursor(0,1);
                lcd.print("Kp :            ");
                lcd.setCursor(5,1);
                lcd.print(ROLL_PID.kp);
                break;
            }
        case 3 :
                break;
    }
    
    
}


void setup() {
    lcd.begin(16,2);
    lcd.print("----");
    display_time = millis();
    loop_timer = micros();
    // Nom des interruptions des pins :   
    // pin 51 : PCINT2
    // pin 50 : PCINT3
    // pin 10 : PCINT4
    // pin 11 : PCINT5
    // pin 12 : PCINT6
    // pin 13 : PCINT7
    // Voir lien pour infos sur le fonctionnement des interrupts : https://github.com/EnviroDIY/Arduino-SDI-12/wiki/2b.-Overview-of-Interrupts

    DDRL |= B00001111; // Set pin 49,48,47,46 as OUTPUTs
    PCICR  |= (1 << PCIE0);  //Set PCIE0 to enable PCMSK0 scan.
    PCMSK0 |= (1 << PCINT2); 
    PCMSK0 |= (1 << PCINT3); 
    PCMSK0 |= (1 << PCINT4); 
    PCMSK0 |= (1 << PCINT5); 
    PCMSK0 |= (1 << PCINT6); 
    PCMSK0 |= (1 << PCINT7); 

    Wire.begin();
    Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    Serial.begin(115200);
    #ifdef OFFSETS
        Serial.println(F("Using Offsets"));
        mpu.SetAddress(MPU6050_ADDRESS_AD0_LOW).load_DMP_Image(OFFSETS); // Does it all for you
    #else
        mpu.SetAddress(MPU6050_ADDRESS_AD0_LOW).CalibrateMPU().load_DMP_Image();// Does it all for you with Calibration
    #endif
    mpu.on_FIFO(update_angles);

    configure_channel_mapping();



    
}


void loop() {
    time_start_loop = micros();
    mpu.dmp_read_fifo();// Must be in loop


    if (pulse_duration[mode_mapping[SWITCH]] > 1500)
    {
        PID_setup_mode = true ;
        display_lcd();
        flying_display_off = true;
    }
    // else if (pulse_duration[mode_mapping[SWITCH]] < 1500 & flying_display_off)  
    // {
    //     PID_setup_mode = false ;
    //     lcd.clear();
    //     lcd.print("     FLYING     ");
    //     lcd.setCursor(0,1);
    //     lcd.print("      MODE      ");
    //     flying_display_off = false;

    // }

    // else if (pulse_duration[mode_mapping[SWITCH]] < 1500 & loop_count > 100)
    // {
    //     PID_setup_mode = false ;
    //     auto [z,y,x] = zyx;
    //     lcd.clear();
    //     lcd.print("     ROLL     ");
    //     lcd.setCursor(0,1);
    //     lcd.print(motors_pulses[0]);
    //     lcd.setCursor(6,1);
    //     lcd.print(pulse_duration[mode_mapping[THROTTLE]]);
    //     loop_count = 0;
    // }  
    
    if (PID_setup_mode)
    {
        setup_PID();
        for (int i = 0; i < 4; i++)
        {
            motors_pulses[i] = 1000;
        }
    }
    else
    {
        pos_required = RF_to_angle(&pulse_duration[mode_mapping[YAW]], &pulse_duration[mode_mapping[PITCH]], &pulse_duration[mode_mapping[ROLL]]);
        calculate_errors(pos_required);
        calculate_motors_pulses();    
        
    }

    // Sending speed signals to motors even if we are not in flying mode to prevent them from beeping.    
    apply_motor_speed();
    
    // loop_count += 1;



}
















//---------------------------------------------------------------------------------
//                                    Interrupt
//---------------------------------------------------------------------------------
/**
 * This Interrupt Sub Routine is called each time input 10, 11, 12 or 13 changed state.
 * Read the receiver signals in order to get flight instructions.
 *
 * This routine must be as fast as possible to prevent main program to be messed up.
 * The trick here is to use port registers to read pin state.
 * Doing (PINB & B00000001) is the same as digitalRead(8) with the advantage of using less CPU loops.
 * It is less conveniant but more efficient, which is the most important here.
 *
 * @see https://www.arduino.cc/en/Reference/PortManipulation
 */
ISR(PCINT0_vect)
{
    current_time = micros();


    // Channel 1 -------------------------------------------------
    if (PINB & B00010000) {                                        //voir   https://www.arduino.cc/en/Hacking/PinMapping2560     pin B4
                                        
        if (previous_state[CHANNEL1] == LOW) {                     // Input 10 changed from 0 to 1 (rising edge)
            previous_state[CHANNEL1] = HIGH;                       // Save current state
            timer[CHANNEL1]          = current_time;               // Start timer
        }
    } else if(previous_state[CHANNEL1] == HIGH) {                  // Input 10 changed from 1 to 0 (falling edge)
        previous_state[CHANNEL1] = LOW;                            // Save current state
        pulse_duration[CHANNEL1] = current_time - timer[CHANNEL1]; // Stop timer & calculate pulse duration
    }

    // Channel 2 -------------------------------------------------
    if (PINB & B00100000) {                                        //  pin B5
        if (previous_state[CHANNEL2] == LOW) {                     // Input 11 changed from 0 to 1 (rising edge)
            previous_state[CHANNEL2] = HIGH;                       // Save current state
            timer[CHANNEL2]          = current_time;               // Start timer
        }
    } else if(previous_state[CHANNEL2] == HIGH) {                  // Input 11 changed from 1 to 0 (falling edge)
        previous_state[CHANNEL2] = LOW;                            // Save current state
        pulse_duration[CHANNEL2] = current_time - timer[CHANNEL2]; // Stop timer & calculate pulse duration
    }

    // Channel 3 -------------------------------------------------
    if (PINB & B01000000) {                                        //  pin B6
        if (previous_state[CHANNEL3] == LOW) {                     // Input 12 changed from 0 to 1 (rising edge)
            previous_state[CHANNEL3] = HIGH;                       // Save current state
            timer[CHANNEL3]          = current_time;               // Start timer
        }
    } else if(previous_state[CHANNEL3] == HIGH) {                  // Input 12 changed from 1 to 0 (falling edge)
        previous_state[CHANNEL3] = LOW;                            // Save current state
        pulse_duration[CHANNEL3] = current_time - timer[CHANNEL3]; // Stop timer & calculate pulse duration
    }

    // Channel 4 -------------------------------------------------
    if (PINB & B10000000) {                                        //  pin B7
        if (previous_state[CHANNEL4] == LOW) {                     // Input 13 changed from 0 to 1 (rising edge)
            previous_state[CHANNEL4] = HIGH;                       // Save current state
            timer[CHANNEL4]          = current_time;               // Start timer
        }
    } else if(previous_state[CHANNEL4] == HIGH) {                  // Input 13 changed from 1 to 0 (falling edge)
        previous_state[CHANNEL4] = LOW;                            // Save current state
        pulse_duration[CHANNEL4] = current_time - timer[CHANNEL4]; // Stop timer & calculate pulse duration
    }
    // Channel 5 -------------------------------------------------
    if (PINB & B00000100) {                                        //voir   https://www.arduino.cc/en/Hacking/PinMapping2560     pin B2
                                        
        if (previous_state[CHANNEL5] == LOW) {                     // Input 51 changed from 0 to 1 (rising edge)
            previous_state[CHANNEL5] = HIGH;                       // Save current state
            timer[CHANNEL5]          = current_time;               // Start timer
        }
    } else if(previous_state[CHANNEL5] == HIGH) {                  // Input 51 changed from 1 to 0 (falling edge)
        previous_state[CHANNEL5] = LOW;                            // Save current state
        pulse_duration[CHANNEL5] = current_time - timer[CHANNEL5]; // Stop timer & calculate pulse duration
    }
    // Channel 6 -------------------------------------------------
    if (PINB & B00001000) {                                        //voir   https://www.arduino.cc/en/Hacking/PinMapping2560     pin B3
                                        
        if (previous_state[CHANNEL6] == LOW) {                     // Input 50 changed from 0 to 1 (rising edge)
            previous_state[CHANNEL6] = HIGH;                       // Save current state
            timer[CHANNEL6]          = current_time;               // Start timer
        }
    } else if(previous_state[CHANNEL6] == HIGH) {                  // Input 50 changed from 1 to 0 (falling edge)
        previous_state[CHANNEL6] = LOW;                            // Save current state
        pulse_duration[CHANNEL6] = current_time - timer[CHANNEL6]; // Stop timer & calculate pulse duration
    }
}
