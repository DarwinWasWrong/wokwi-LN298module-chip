#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>

#define BOARD_HEIGHT 100
#define BOARD_WIDTH 100

// basic RGBA color
typedef struct {
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t a;
} rgba_t;

typedef struct {
  //Module Pins
  // channel A
  pin_t pin_IN1;
  pin_t pin_IN2;
  pin_t pin_ENA;
  pin_t pin_ENA_5v;   // 5V for enable
  pin_t pin_M1;   // Motor 1
  pin_t pin_M2;   // Motor 2

// Channel B
  pin_t pin_IN3;
  pin_t pin_IN4;
  pin_t pin_ENB;
  pin_t pin_ENB_5v;  // 5V for enable
  pin_t pin_M3;   // Motor 3
  pin_t pin_M4;   // Motor 4


// powers and ground
  pin_t pin_VCC;
  pin_t pin_GND;
  pin_t pin_Motor_V;

  // start of timing checks for the PWM
  uint8_t  use_PWM_ENA; //  pwm used
  uint8_t  use_PWM_ENB; //  pwm used
// Channel A  
  uint32_t  high_ENA;
  uint32_t  low_ENA;
  uint32_t  previous_high_ENA;
  uint32_t  previous_low_ENA;
  uint32_t  high_time_ENA;
  uint32_t  low_time_ENA;
// Channel B
  uint32_t  high_ENB;
  uint32_t  low_ENB;
  uint32_t  previous_high_ENB;
  uint32_t  previous_low_ENB;
  uint32_t  high_time_ENB;
  uint32_t  low_time_ENB;

// check for start state of jumpered enables
  uint32_t  start_state_ENA;
  uint32_t  start_state_ENB;

  uint32_t Vs_attr;  // power

// frame buffer details
  uint32_t fb_w;
  uint32_t fb_h;
  uint32_t row;
  buffer_t framebuffer;

// colors
  rgba_t   white;
  rgba_t   green;
  rgba_t   background;
  rgba_t   purple;
  rgba_t   black;
  rgba_t   red;
  rgba_t   blue;

 
  uint8_t  speed_percent_A;
  uint8_t  speed_percent_B;

  uint8_t  speed_analog_M1;
  uint8_t  speed_analog_M2;
  uint8_t  speed_analog_M3;
  uint8_t  speed_analog_M4;

  uint8_t  previous_speed_percent_A;
  uint8_t  previous_speed_percent_B;

  uint8_t  drive_A_state;
  uint8_t  drive_B_state;
  uint8_t  previous_drive_A_state;
  uint8_t  previous_drive_B_state;

  timer_t timer_motorA;
  timer_t timer_motorB;

  float voltageout_M1;
  float voltageout_M2;
  float voltageout_M3;
  float voltageout_M4;
} chip_state_t;

// screen functions
static void send_signal(chip_state_t *chip);

// timer for watchdog
static void chip_timer_event_Awatchdog(void *user_data);
static void chip_timer_event_Bwatchdog(void *user_data);

// pin change watches
static void chip_pin_change(void *user_data, pin_t pin, uint32_t value);
static void chip_pin_change_PWM_A(void *user_data, pin_t pin, uint32_t value);
static void chip_pin_change_PWM_B(void *user_data, pin_t pin, uint32_t value);

void chip_init(void) {
  printf("*** LN298chip initialising...\n");
  chip_state_t *chip = malloc(sizeof(chip_state_t));

  chip->pin_ENA = pin_init("ENA",INPUT);
  chip->pin_IN1 = pin_init("IN1",INPUT);
  chip->pin_IN2 = pin_init("IN2",INPUT);
  chip->pin_M1 = pin_init("M1",ANALOG);
  chip->pin_M2 = pin_init("M2",ANALOG);

  chip->pin_ENB = pin_init("ENB",INPUT);
  chip->pin_IN3 = pin_init("IN3",INPUT);
  chip->pin_IN4 = pin_init("IN4",INPUT);
  chip->pin_M3 = pin_init("M3",ANALOG);
  chip->pin_M4 = pin_init("M4",ANALOG);

  chip->pin_VCC = pin_init("VCC",INPUT);
  chip->pin_GND = pin_init("GND",INPUT);
  chip->pin_Motor_V = pin_init("Motor_V",INPUT);

  // logic for detecting jumper
  // set the 5v jumper pins to 5v
  chip->pin_ENA_5v= pin_init("ENA_5v",OUTPUT_HIGH );
  chip->pin_ENB_5v= pin_init("ENB_5v",OUTPUT_HIGH );

  // read then ENA pin - if jumper is on - it will be high
  chip->start_state_ENA =pin_read(chip->pin_ENA);
  // read then ENB pin - if jumper is on - it will be high
  chip->start_state_ENB =pin_read(chip->pin_ENB);

 //
  printf( "ENA linked to 5v %d\n",chip->start_state_ENA);
  printf( "ENB linked to 5v %d\n",chip->start_state_ENB);
 
  // if there is no link have to check for PWM or switching
  chip->use_PWM_ENA= !chip->start_state_ENA;
  chip->use_PWM_ENB= !chip->start_state_ENB;

  // pwm timings
  unsigned long  high_ENA;
  unsigned long  low_ENA;
  unsigned long  previous_high_ENA;
  unsigned long  previous_low_ENA;
  unsigned long  high_ENB;
  unsigned long  low_ENB;
  unsigned long  previous_high_ENB;
  unsigned long  previous_low_ENB;
  unsigned long  high_time_ENA;
  unsigned long  low_time_ENA;
  unsigned long  high_time_ENB;
  unsigned long  low_time_ENB;
 
  chip->speed_percent_A=0;
  chip->speed_percent_B=0;
 
const timer_config_t timer_config_Awatchdog = {
    .callback = chip_timer_event_Awatchdog,
    .user_data = chip,
  };
  timer_t timer_Awatchdog = timer_init(&timer_config_Awatchdog);
  timer_start(timer_Awatchdog,100000, true);

const timer_config_t timer_config_Bwatchdog = {
    .callback = chip_timer_event_Bwatchdog,
    .user_data = chip,
  };
  timer_t timer_Bwatchdog = timer_init(&timer_config_Bwatchdog);
  timer_start(timer_Bwatchdog,100000, true);

// config for PWM A watch
const pin_watch_config_t watch_config_PWM_A = {
    .edge = BOTH,
    .pin_change = chip_pin_change_PWM_A,
    .user_data = chip
  };

// config for PWM B watch
const pin_watch_config_t watch_config_PWM_B = {
    .edge = BOTH,
    .pin_change = chip_pin_change_PWM_B,
    .user_data = chip
  };

  // PWM watches
  printf("PWM watches ...\n");
  pin_watch(chip->pin_ENA, &watch_config_PWM_A );
  pin_watch(chip->pin_ENB, &watch_config_PWM_B );
  
  // config for other pins IN1 IN2 IN3 IN4
  const pin_watch_config_t watch_config = {
    .edge = BOTH,
    .pin_change = chip_pin_change,
    .user_data = chip
  };

  // pins watches
  printf("pins watches ...\n");
  pin_watch(chip->pin_IN1, &watch_config);
  pin_watch(chip->pin_IN2, &watch_config);
  pin_watch(chip->pin_IN3, &watch_config);
  pin_watch(chip->pin_IN4, &watch_config);


}

// PWM A pin change function for watch
void chip_pin_change_PWM_A(void *user_data, pin_t pin, uint32_t value) {
  chip_state_t *chip = (chip_state_t*)user_data;
  uint8_t ENA = pin_read(chip->pin_ENA);
  // channel A using PWM
  if (ENA){
    chip->high_ENA = get_sim_nanos();
    chip->low_time_ENA = chip->high_ENA - chip->low_ENA;
  } 
  else 
  {
    chip->low_ENA = get_sim_nanos();
    chip->high_time_ENA = chip->low_ENA - chip->high_ENA ;
  }
  float total_ENA = chip->high_time_ENA + chip->low_time_ENA;
  int duty_cycle_ENA = (chip->high_time_ENA / total_ENA) * 100.0;
  chip->speed_percent_A=duty_cycle_ENA;
  // if a change then redisplay
  if ( chip->previous_speed_percent_A != chip->speed_percent_A)
  {
   send_signal(chip);
 
   chip->previous_speed_percent_A = chip->speed_percent_A;
  }
}

// PWM B pin change function for watch
void chip_pin_change_PWM_B(void *user_data, pin_t pin, uint32_t value) {
  chip_state_t *chip = (chip_state_t*)user_data;
  uint8_t ENB = pin_read(chip->pin_ENB);
  // channel B using PWM
  if (ENB){
    chip->high_ENB= get_sim_nanos();
    chip->low_time_ENB= chip->high_ENB- chip->low_ENB;
  } else {
    chip->low_ENB= get_sim_nanos();
    chip->high_time_ENB= chip->low_ENB- chip->high_ENB;
  }
  float total = chip->high_time_ENB+ chip->low_time_ENB;
  int duty_cycle_ENB = (chip->high_time_ENB / total) * 100.0;
  chip->speed_percent_B=duty_cycle_ENB;
  // if a change then redisplay
  if ( chip->previous_speed_percent_B != chip->speed_percent_B  )
  {
   send_signal(chip);
   chip->previous_speed_percent_B = chip->speed_percent_B;
  }
 }


void chip_pin_change(void *user_data, pin_t pin, uint32_t value) {
  chip_state_t *chip = (chip_state_t*)user_data;
  uint8_t ENA = pin_read(chip->pin_ENA);
  uint8_t ENB = pin_read(chip->pin_ENA);
  uint8_t IN1 = pin_read(chip->pin_IN1);
  uint8_t IN2 = pin_read(chip->pin_IN2);
  uint8_t IN3 = pin_read(chip->pin_IN3);
  uint8_t IN4 = pin_read(chip->pin_IN4);
 
  // read control for PWM used
  // needs to change to detect held HIGH or LOW for NO PWM
  uint8_t use_PWM_ENA= attr_read_float(chip->use_PWM_ENA);
  uint8_t use_PWM_ENB= attr_read_float(chip->use_PWM_ENB);

  if (use_PWM_ENA )
  {
  if ( ENA && IN1 && !IN2) chip-> drive_A_state =  0;
  if ( ENA && !IN1 && IN2) chip-> drive_A_state =  1;
  if ( ENA && IN1 == IN2) chip-> drive_A_state =   2;
  if ( !ENA ) chip-> drive_A_state =               3;
  }
  else
  {
  //drive 1 states
  if ( IN1 && !IN2) chip-> drive_A_state =  0;
  if (!IN1 && IN2) chip->  drive_A_state =  1;
  if ( IN1 == IN2) chip->  drive_A_state =  2;

  }

if (use_PWM_ENB )
  {
  // drive 2 states
  if ( ENB && IN3 && !IN4) chip-> drive_B_state = 0;
  if ( ENB && !IN3 && IN4) chip-> drive_B_state = 1;
  if ( ENB && IN3 == IN4) chip-> drive_B_state =  2;
  if ( !ENB ) chip-> drive_A_state =              3;
  }
  else
 {
  if (IN3 && !IN4) chip-> drive_B_state = 0;
  if ( !IN3 && IN4) chip-> drive_B_state = 1;
  if (IN3 == IN4) chip-> drive_B_state =  2;
 }
  send_signal(chip);
}

void send_signal(chip_state_t *chip) {
  
  //turn off the two timers
  timer_stop(chip->timer_motorA);
  timer_stop(chip->timer_motorB);

// backwards
 if (chip-> drive_A_state == 0) 
 {
  chip->voltageout_M2= chip->speed_percent_A/20.00;
  pin_dac_write(chip->pin_M1, 0);
  pin_dac_write(chip->pin_M2, chip->voltageout_M2);

 }
 //forwards
 if (chip-> drive_A_state == 1) 
 {
  chip->voltageout_M1= chip->speed_percent_A/20.00;
  pin_dac_write(chip->pin_M1, chip->voltageout_M1 );
  pin_dac_write(chip->pin_M2, 0);
 }

 //stopped
 if (chip-> drive_A_state == 2 || chip-> drive_A_state == 3)
 {
  pin_dac_write(chip->pin_M1, 0);
  pin_dac_write(chip->pin_M2, 0);
 }
 if (chip-> drive_B_state == 0)
 { 
    chip->voltageout_M4= chip->speed_percent_B/20.00;
     pin_dac_write(chip->pin_M3, 0);
     pin_dac_write(chip->pin_M4, chip->voltageout_M4) ;
 }

 if (chip-> drive_B_state == 1) 
 {
  chip->voltageout_M3= chip->speed_percent_B/20.00;
  pin_dac_write(chip->pin_M3, chip->voltageout_M3);
  pin_dac_write(chip->pin_M4,  0) ;
 }
 if (chip-> drive_B_state == 2 || chip-> drive_B_state == 3)
 {
     pin_dac_write(chip->pin_M3, 0);
     pin_dac_write(chip->pin_M4, 0);
 }
}
// watch dog A
void chip_timer_event_Awatchdog(void *user_data) {
  chip_state_t *chip = (chip_state_t*)user_data;
 }

// watch dog B
void chip_timer_event_Bwatchdog(void *user_data) {
  chip_state_t *chip = (chip_state_t*)user_data;
 }









