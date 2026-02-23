
// Wokwi Custom Chip - For docs and examples see:
// https://docs.wokwi.com/chips-api/getting-started
//
// SPDX-License-Identifier: MIT
// Copyright 2023 Darwin WasWrong
// src/L298-module.chip.c

// thanks to
// Maverick - for saving my mind with PWM
//
// https://notisrac.github.io/FileToCArray/
// for the conversion for images
//

#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include "graphics.h"
#define BOARD_HEIGHT 100
#define BOARD_WIDTH 100









// the various states the channel can be in
const char drive_state[][17]=
{
  "Backward       ",
  "Forward        ", 
  "Brake Stop     ",
  "Free Stop      "
  };

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
// Channel B
  pin_t pin_IN3;
  pin_t pin_IN4;
  pin_t pin_ENB;
  pin_t pin_ENB_5v;  // 5V for enable
 


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

 // text start and postion
  uint32_t vertical_start;
  uint32_t position_x;
  uint32_t position_y;
  
  uint8_t  speed_percent_A;
  uint8_t  speed_percent_B;
  uint8_t  previous_speed_percent_A;
  uint8_t  previous_speed_percent_B;

  uint8_t  drive_A_state;
  uint8_t  drive_B_state;
  uint8_t  previous_drive_A_state;
  uint8_t  previous_drive_B_state;
  
  // motor graphics position
  int8_t motorAphase;
  int8_t motorBphase;
  uint8_t motor_A_y;
  uint8_t motor_A_x;
  uint8_t motor_B_x;
  uint8_t motor_B_y;

// arrow graphics position
  uint8_t motor_1_2_arrow_y ;

  uint8_t motorA_right_arrow_x;
  uint8_t motorA_left_arrow_x;

  uint8_t motorB_right_arrow_x;
  uint8_t motorB_left_arrow_x;

  uint8_t bar_1_2_y;
  uint8_t bar_left_x;
  uint8_t bar_right_x;

  timer_t timer_motorA;
  timer_t timer_motorB;
} chip_state_t;


// screen functions

static void draw_board(chip_state_t *chip, uint32_t x_start,  uint32_t y_start) ;

static void send_signal(chip_state_t *chip);

// timer for graphics
static void chip_timer_event_motorA(void *user_data);
static void chip_timer_event_motorB(void *user_data);

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
 

  chip->pin_ENB = pin_init("ENB",INPUT);
  chip->pin_IN3 = pin_init("IN3",INPUT);
  chip->pin_IN4 = pin_init("IN4",INPUT);


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
  
  // Display values
  chip->speed_percent_A=0;
  chip->speed_percent_B=0;

  // display colors
  chip-> white      = (rgba_t) { .r = 0xff, .g = 0xff, .b = 0xff, .a = 0xff };
  chip-> green      = (rgba_t) { .r = 0x08, .g = 0x7f, .b = 0x45, .a = 0xff };
  chip-> background = (rgba_t) { .r = 0xf7, .g = 0xf7, .b = 0xf7, .a = 0xff };
  chip-> purple    = (rgba_t) { .r = 0xff, .g=0x00,   .b=0xff,   .a=0xff   };
  chip-> black    =  (rgba_t) { .r = 0x00, .g=0x00,   .b=0x00,   .a=0x00   };
  chip-> red    =  (rgba_t) { .r = 0xff, .g=0x00,   .b=0x00,   .a=0xff   };
  chip-> blue    =  (rgba_t) { .r = 0x00, .g=0x00,   .b=0xff,   .a=0xff   };
  chip->row = 0;
  // get the screen size
  chip->framebuffer = framebuffer_init(&chip->fb_w, &chip->fb_h);
  printf("Framebuffer: fb_w=%d, fb_h=%d\n", chip->fb_w, chip->fb_h);
 
 // settings for gears and displays
 // phase of gear display
  chip->motorAphase=0;
  chip->motorBphase=0;
  // positioning
  chip->motor_A_y=20;
  chip->motor_A_x=113;
  chip->motor_B_x=20;
  chip->motor_B_y=187;
  // arrow and speed base position
  chip->motor_1_2_arrow_y = 70;
  // Motor A arrow positions  relative to motor A position
  chip->motorA_right_arrow_x =chip->motor_A_x + 40;
  chip->motorA_left_arrow_x = chip->motor_A_x  ;
  // Motor B arrow positions  relative to motor B position
  chip->motorB_right_arrow_x = chip->motor_B_y +  40;
  chip->motorB_left_arrow_x = chip->motor_B_y;
  // bars position relative to motor
  chip-> bar_1_2_y=  chip->motor_1_2_arrow_y + 20;
  chip-> bar_left_x=chip->motor_A_x ;
  chip-> bar_right_x=chip->motor_B_y ;
   printf("Framebuffer: fb_w=%d, fb_h=%d\n", chip->fb_w, chip->fb_h);


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


const timer_config_t timer_config_motorA = {
    .callback = chip_timer_event_motorA,
    .user_data = chip,
  };
  chip->timer_motorA = timer_init(&timer_config_motorA);
 // timer_start(chip->timer_motorA,20000, true);

const timer_config_t timer_config_motorB = {
    .callback = chip_timer_event_motorB,
    .user_data = chip,
  };
  chip->timer_motorB = timer_init(&timer_config_motorB);
 // timer_start(timer_motorB, 20000, true);


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
   // printf("chip->previous_speed_percent_A %d chip->speed_percent_A %d \n",chip->previous_speed_percent_A ,chip->speed_percent_A );
 
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
   //printf("chip->previous_speed_percent_B %d chip->speed_percent_B %d \n",chip->previous_speed_percent_B ,chip->speed_percent_B );
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
   
//chip->speed_percent_A

 }
 //forwards
 if (chip-> drive_A_state == 1) 
 {
//chip->speed_percent_A
 }

 //stopped
 if (chip-> drive_A_state == 2 || chip-> drive_A_state == 3)
 {
//chip->speed_percent_A
 }



 if (chip-> drive_B_state == 0)
 { 
    // chip->speed_percent_B
 }
 if (chip-> drive_B_state == 1) 
 {
  //  // chip->speed_percent_B
 }
 if (chip-> drive_B_state == 2 || chip-> drive_B_state == 3)
 {
  //  // chip->speed_percent_B
 }
  if (chip-> drive_A_state == 0 || chip-> drive_A_state == 1)
  {
  timer_start(chip->timer_motorA, (100 - chip->speed_percent_A) * 2400 , true);
 
  }

   if (chip-> drive_B_state == 0 || chip-> drive_B_state == 1)
   {
   timer_start(chip->timer_motorB, (100 - chip->speed_percent_B) * 2400, true);
   }

}


// graphics for motor A
void chip_timer_event_motorA(void *user_data) {
  chip_state_t *chip = (chip_state_t*)user_data;
  
  if ( chip-> drive_A_state == 0 )   chip->motorAphase=chip->motorAphase - 1;
  if ( chip-> drive_A_state == 1) chip->motorAphase=chip->motorAphase + 1;
  if (chip->motorAphase < 0) chip->motorAphase =8;
  if (chip->motorAphase >8) chip->motorAphase = 0;

}

// graphics for motor B
void chip_timer_event_motorB(void *user_data) {
  chip_state_t *chip = (chip_state_t*)user_data;

  if ( chip-> drive_B_state == 0 ) chip->motorBphase=chip->motorBphase - 1;
  if ( chip-> drive_B_state == 1) chip->motorBphase=chip->motorBphase + 1;
  if (chip->motorBphase < 0) chip->motorBphase = 8;
  if (chip->motorBphase > 8) chip->motorBphase = 0;
}

// watch dog A
void chip_timer_event_Awatchdog(void *user_data) {
  chip_state_t *chip = (chip_state_t*)user_data;
 }

// watch dog B
void chip_timer_event_Bwatchdog(void *user_data) {
  chip_state_t *chip = (chip_state_t*)user_data;
 }


 

void draw_board(chip_state_t *chip, uint32_t x_start,  uint32_t y_start) {
// size of our graphic
uint8_t square_size = 100;
uint32_t pixel_spot_data = 0;
for (int x=x_start;x < square_size + x_start; x++)
   {
     for (int y=y_start;y<square_size + y_start ;y++)
    {

    // rgba_t  color1 =  rgba_t(test_pattern[pixel_spot_data] );
      uint32_t pixel_data =  board[pixel_spot_data] ; 
        //uint32_t pixel_data = all_cogs[pos][pixel_spot_data] ;
        rgba_t  color =
            { 
            .r= (pixel_data  & 0xFF000000) >> 24,
            .g = (pixel_data & 0x00FF0000) >> 16,
            .b = (pixel_data & 0x0000FF00) >> 8,
            .a = (pixel_data & 0x000000FF) 
            };
        
  
        uint32_t offset = chip->fb_w * 4 ;  
        buffer_write(chip->framebuffer, (offset * x) + y * 4, (uint8_t*)&color , 4); 
        pixel_spot_data++;
    }
   }
} 







