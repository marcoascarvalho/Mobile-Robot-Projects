#include "16F877.h"
#device adc=8
#use delay(clock=20000000)
#fuses NOWDT,HS, PUT, NOPROTECT, BROWNOUT, NOLVP, NOCPD, NOWRT, NODEBUG
#use rs232(baud=19200,xmit=TX,rcv=RX)


/******************************************************************************
                          Usefull multtask defines

******************************************************************************/
#DEFINE   Version                   1

//***** Motor constants

#DEFINE   n_turns     		         50
#DEFINE   LED_TASK_2                7
#DEFINE   ENC_1                     2
#DEFINE   ENC_2                     1
#DEFINE   ENC_RESOLUTION            4
#DEFINE   CW                        1
#DEFINE   CCW                       0

#DEFINE   MOTOR1                    0
#DEFINE   MOTOR2                    1

#DEFINE   INPUT_CHOPPING            0
#DEFINE   USING_PWM                 0
#DEFINE   PRINT_MODES               0  

#IF INPUT_CHOPPING
//***** Motor Commands constants masks
//                                    76543210        // motor1  dir  motor2  dir
#DEFINE   cmd_STOP                  0b00100100        //   off    X     off    X
#DEFINE   cmd_FWD                   0b01101100        //   on    FWD    on    FWD
#DEFINE   cmd_BKW                   0b10110100        //   on    BCW    on    BCW
#DEFINE   cmd_LEFT_TURN             0b01110100        //   on    FWD    on    BCW
#DEFINE   cmd_RIGHT_TURN            0b10101100        //   on    BCW    on    FWD
#DEFINE   cmd_LEFT_ARC              0b00101100        //   on    FWD    off    X
#DEFINE   cmd_RIGHT_ARC             0b00110100        //   off    X     on    FWD
#DEFINE   cmd_LEFT_ARC_BKW          0b01100100        //   on    BCW    off    X
#DEFINE   cmd_RIGHT_ARC_BKW         0b10100100        //   off    X     on    BCW

#ELSE
//                                    76543210        // motor1  dir  motor2  dir
#DEFINE   cmd_STOP                  0b00000000        //   off    X     off    X
#DEFINE   cmd_FWD_PWM_ON            0b01101100        //   on    FWD    on    FWD
#DEFINE   cmd_FWD_PWM_OFF           0b01001000        //   off   FWD    off   FWD
#DEFINE   cmd_BKW_PWM_ON            0b10110100        //   on    BCW    on    BCW
#DEFINE   cmd_BKW_PWM_OFF           0b10010000        //   off   BCW    off   BCW
#DEFINE   cmd_LEFT_TURN_PWM_ON      0b01110100        //   on    FWD    on    BCW
#DEFINE   cmd_LEFT_TURN_PWM_OFF     0b01010000        //   off   FWD    off   BCW
#DEFINE   cmd_RIGHT_TURN_PWM_ON     0b10101100        //   on    BCW    on    FWD
#DEFINE   cmd_RIGHT_TURN_PWM_OFF    0b10001000        //   off   BCW    off   FWD
#DEFINE   cmd_LEFT_ARC_PWM_ON       0b00101100        //   on    FWD    off    X
#DEFINE   cmd_LEFT_ARC_PWM_OFF      0b00101000        //   off   FWD    off    X
#DEFINE   cmd_RIGHT_ARC_PWM_ON      0b00110100        //   off    X     on    FWD
#DEFINE   cmd_RIGHT_ARC_PWM_OFF     0b00110000        //   off    X     off    FWD
#DEFINE   cmd_LEFT_ARC_BKW_PWM_ON   0b01100100        //   on    BCW    off    X
#DEFINE   cmd_LEFT_ARC_BKW_PWM_OFF  0b01000100        //   off   BCW    off    X
#DEFINE   cmd_RIGHT_ARC_BKW_PWM_ON  0b10100100        //   off    X     on    BCW
#DEFINE   cmd_RIGHT_ARC_BKW_PWM_OFF 0b10000100        //   off    X     off   BCW
#ENDIF

//***** constants for bumper sensor

#DEFINE   ENABLE_BUMPER_BACK        2
#DEFINE   ENABLE_BUMPER_RIGHT       1
#DEFINE   ENABLE_BUMPER_LEFT        0

//***** IR bits --> used in IR_VAR variable and PORTC bits
//#DEFINE   IR_LAST_STATE             4
#DEFINE   IR_FILTER                 100

//***** Analog Channels constants
#DEFINE   ANALOG_0                  0                 //RA0 pin
#DEFINE   ANALOG_1                  0x08              //RA1 pin
#DEFINE   ANALOG_2                  0x10              //RA2 pin
#DEFINE   ANALOG_3                  0x18              //RA3 pin
#DEFINE   ANALOG_4                  0x20              //RA4 pin

//#DEFINE TMR1ON                      0

//***** Motor Port declaration
//MOTOR_PORT      EQU     PORTB

//***** Constants used to perform time slice multitask system

#DEFINE   TASK0_COUNTER_MAX         1    //TASK0 @ 200us
#DEFINE   TASK1_COUNTER_MAX         3    //TASK1 @ 600us
#DEFINE   TASK2_COUNTER_MAX         100  //TASK2 @ 1ms
#DEFINE   TASK3_COUNTER_MAX         500  //TASK3 @ 100ms
#DEFINE   TASK4_COUNTER_MAX         5000 //TASK4 @ 1s


#DEFINE   TICKS_BETWEEN_INTERRUPTS  1000    //interrupts @ 200us
#DEfINE   INTERRUPT_OVERHEAD        19
#DEFINE   TMR1RESET                 (0xFFFF - (TICKS_BETWEEN_INTERRUPTS-INTERRUPT_OVERHEAD))
#DEFINE   TMR1RESET_HIGH            TMR1RESET >> 8
#DEFINE   TMR1RESET_LOW             (TMR1RESET & 0xFF)

/* Global variables */
BYTE motor_cmd, motor_cmd_end, pwm1, pwm2;
BYTE n_pulses_PWM1, n_pulses_PWM2;
long task0_counter, task1_counter, task2_counter, task3_counter, task4_counter;
BOOLEAN task0_enable, task1_enable, task2_enable, task3_enable, task4_enable;

BYTE tmp;

BOOLEAN timeout_1s;

BYTE blind_cmd, avoid_cmd, escape_cmd, cruise_cmd;

unsigned BYTE enable_bumper, ir_rx_read_filter;

unsigned BYTE light_sensor;

//unsigned BYTE bumper_sensor;

BOOLEAN cruise_enable, escape_enable, avoid_enable, blind_enable;

BOOLEAN ir_last_state, ir_rx_state, ir_rx_on, ir_rx_off, obstacle_detected;

unsigned long t2_old, t2, n_rev_2;

#ORG 0x1F00,0x1FFF //for the 8k 16F876/7
void loader() { }

/*=============================================================================
 Timer 1 overflow
 Timer1_isr
=============================================================================*/
#INT_TIMER1
tmr1_isr()
{
   //disable TMR1 load TMR1RESET and start it again
   //setup_timer_1(T1_DISABLED);
   //disable_interrupts(GLOBAL);
   set_timer1(TMR1RESET);
   //setup_timer_1 ( T1_INTERNAL | T1_DIV_BY_1 );

   //TASK_SCHEDULER

   //TASK0
   task0_counter++;
	if (task0_counter >= TASK0_COUNTER_MAX)
   {
      /***********************************************************************
      *   	Code for task0 @ 200us                                           *
      ***********************************************************************/
      //this task perform the pwm duty cycle
      task0_counter = 0;
      if (task0_enable)
      {
         n_pulses_PWM1++;       // n_pulses_PWM++
         n_pulses_PWM2++;
         if(n_pulses_PWM1 == pwm1)
         {
            tmp = input_b();
            tmp &= 0x03;        //strip the six most significant bits
            motor_cmd_end ^= tmp;   //set the motor cmd mask
            output_b(motor_cmd_end);
            //motor_cmd ^= tmp;
            //output_b(motor_cmd);
            n_pulses_PWM1 = 0;
            task0_enable = FALSE;
         }
      }
   }//TASK0

   //TASK1
   task1_counter++;
	if (task1_counter >= TASK1_COUNTER_MAX)
   {
      /***********************************************************************
      *   			Code for task1 @ 600us                            *
      ************************************************************************/
      //this task perform the square wave for infra red sensor modulation
      //when active task1 perform the output signal to IR circuit
      task1_counter = 0;
      if (task1_enable)
      {
         if(ir_last_state == 0)
         {
            ir_rx_on = input(IR_RX);
            output_high(IR_TX);
            ir_last_state = 1;
         }
         else
         {
            ir_rx_off = input(IR_RX);
            output_low(IR_TX);
            ir_last_state = 0;
         }

         ir_rx_state = ir_rx_on & (~ir_rx_off);

         if (ir_rx_state)
         {
            ir_rx_read_filter++;
            if(ir_rx_read_filter > IR_FILTER)
            {
               obstacle_detected = TRUE;
               ir_rx_read_filter = 0;
            }
         }
         else
         {
            obstacle_detected = FALSE;
            ir_rx_read_filter = 0;
         }
         //output_bit(PIN_C3, obstacle_detected);
      }
   }

   //TASK2
   task2_counter++;
	if (task2_counter >= TASK2_COUNTER_MAX)
   {
      /*************************************************************************
      *			Code for task2  @20ms                             *
      ************************************************************************/
      //This task perform the pwm frequency = 50Hz and Bumper sensor read
      task2_counter = 0;
      if (task2_enable)
      {
         tmp = input_b();
         tmp &= 0x03;        //strip the six most significant bits
         motor_cmd ^= tmp;  //set the motor cmd mask
         output_b(motor_cmd);

   	   task0_enable = TRUE;

         //Scheduler for the motor control. Put here the behavior in the highest priority
         //order
         //MOTOR_CONTROL_SCHEDULER
         if(blind_enable)
         {
            switch(blind_cmd)
            {
               case cmd_STOP:
                  motor_cmd = cmd_STOP;
                  motor_cmd_end = cmd_STOP;
                  //printf("STOP");
               break;
               //case cmd_FWD:
               //   motor_cmd = cmd_FWD;
               //   motor_cmd_end = cmd_BKW;
                  //printf("FWD");
               //break;
               //case cmd_BKW:
               //   motor_cmd = cmd_BKW;
               //   motor_cmd_end = cmd_FWD;
                  //printf("BKW");
               //break;
               //case cmd_LEFT_TURN:
               //   motor_cmd = cmd_LEFT_TURN;
               //   motor_cmd_end = cmd_RIGHT_TURN;
                  //printf("TURN LEFT");
               //break;
               //case cmd_RIGHT_TURN:
               //   motor_cmd = cmd_RIGHT_TURN;
               //   motor_cmd_end = cmd_LEFT_TURN;
                  //printf("TURN LEFT");
               //break;
#IF INPUT_CHOPPING
               case cmd_LEFT_ARC:
                  motor_cmd = cmd_LEFT_ARC;
                  motor_cmd_end = cmd_LEFT_ARC_BKW;
                  //printf("LEFT ARC");
               break;
#ELSE
               case cmd_LEFT_ARC_PWM_ON:
                  motor_cmd = cmd_LEFT_ARC_PWM_ON;
                  motor_cmd_end = cmd_LEFT_ARC_PWM_OFF;
                  //printf("LEFT ARC");
               break;
#ENDIF

               //case cmd_RIGHT_ARC:
               //   motor_cmd = cmd_RIGHT_ARC;
               //   motor_cmd_end = cmd_RIGHT_ARC_BKW;
                  //printf("LEFT ARC");
               //break;
            }
         }
         else if(avoid_enable)
         {
            switch(avoid_cmd)
            {
               //case cmd_STOP:
               //   motor_cmd = cmd_STOP;
               //   motor_cmd_end = cmd_STOP;
                  //printf("STOP");
               //break;
               //case cmd_FWD:
               //   motor_cmd = cmd_FWD;
               //   motor_cmd_end = cmd_BKW;
                  //printf("FWD");
               //break;
               //case cmd_BKW:
               //   motor_cmd = cmd_BKW;
               //   motor_cmd_end = cmd_FWD;
                  //printf("BKW");
               //break;
               //case cmd_LEFT_TURN:
               //   motor_cmd = cmd_LEFT_TURN;
               //   motor_cmd_end = cmd_RIGHT_TURN;
                  //printf("TURN LEFT");
               //break;
               //case cmd_RIGHT_TURN:
               //   motor_cmd = cmd_RIGHT_TURN;
               //   motor_cmd_end = cmd_LEFT_TURN;
                  //printf("TURN LEFT");
               //break;
#IF INPUT_CHOPPING
               case cmd_LEFT_ARC:
                  motor_cmd = cmd_LEFT_ARC;
                  motor_cmd_end = cmd_LEFT_ARC_BKW;
                  //printf("LEFT ARC");
               break;
#ELSE
               case cmd_LEFT_ARC_PWM_ON:
                  motor_cmd = cmd_LEFT_ARC_PWM_ON;
                  motor_cmd_end = cmd_LEFT_ARC_PWM_OFF;
                  //printf("LEFT ARC");
               break;
#ENDIF

               //case cmd_RIGHT_ARC:
               //   motor_cmd = cmd_RIGHT_ARC;
               //   motor_cmd_end = cmd_RIGHT_ARC_BKW;
                  //printf("LEFT ARC");
               //break;
            }
         }
         else if(escape_enable)
         {
            switch(escape_cmd)
            {
               //case cmd_STOP:
               //   motor_cmd = cmd_STOP;
               //   motor_cmd_end = cmd_STOP;
                  //printf("STOP");
               //break;
#IF INPUT_CHOPPING
               case cmd_FWD:
                  motor_cmd = cmd_FWD;
                  motor_cmd_end = cmd_BKW;
                //printf("FWD");
               break;
               case cmd_BKW:
                  motor_cmd = cmd_BKW;
                  motor_cmd_end = cmd_FWD;
                  //printf("BKW");
               break;
               case cmd_LEFT_TURN:
                  motor_cmd = cmd_LEFT_TURN;
                  motor_cmd_end = cmd_RIGHT_TURN;
                  //printf("TURN LEFT");
               break;
               case cmd_RIGHT_TURN:
                  motor_cmd = cmd_RIGHT_TURN;
                  motor_cmd_end = cmd_LEFT_TURN;
                  //printf("TURN LEFT");
               break;
#ELSE
               case cmd_FWD_PWM_ON:
                  motor_cmd = cmd_FWD_PWM_ON;
                  motor_cmd_end = cmd_FWD_PWM_OFF;
                //printf("FWD");
               break;
               case cmd_BKW_PWM_ON:
                  motor_cmd = cmd_BKW_PWM_ON;
                  motor_cmd_end = cmd_BKW_PWM_OFF;
                  //printf("BKW");
               break;
               case cmd_LEFT_TURN_PWM_ON:
                  motor_cmd = cmd_LEFT_TURN_PWM_ON;
                  motor_cmd_end = cmd_LEFT_TURN_PWM_OFF;
                  //printf("TURN LEFT");
               break;
               case cmd_RIGHT_TURN_PWM_ON:
                  motor_cmd = cmd_RIGHT_TURN_PWM_ON;
                  motor_cmd_end = cmd_RIGHT_TURN_PWM_OFF;
                  //printf("TURN LEFT");
               break;
#ENDIF
               //case cmd_LEFT_ARC:
               //   motor_cmd = cmd_LEFT_ARC;
               //   motor_cmd_end = cmd_LEFT_ARC_BKW;
                  //printf("LEFT ARC");
               //break;
               //case cmd_RIGHT_ARC:
               //   motor_cmd = cmd_RIGHT_ARC;
               //   motor_cmd_end = cmd_RIGHT_ARC_BKW;
                  //printf("LEFT ARC");
               //break;
            }

         }
         else if(cruise_enable)
         {
            //printf("cruise_cmd = 0x%x\n", cruise_cmd);
            switch(cruise_cmd)
            {
               //case cmd_STOP:
               //   motor_cmd = cmd_STOP;
               //   motor_cmd_end = cmd_STOP;
                  //printf("STOP");
               //break;
#IF INPUT_CHOPPING
               case cmd_FWD:
                  motor_cmd = cmd_FWD;
                  motor_cmd_end = cmd_BKW;
                  //printf("FWD\n");
               break;
#ELSE
               case cmd_FWD_PWM_ON:
                  motor_cmd = cmd_FWD_PWM_ON;
                  motor_cmd_end = cmd_FWD_PWM_OFF;
                  //printf("FWD\n");
               break;
#ENDIF

               //case cmd_BKW:
               //   motor_cmd = cmd_BKW;
               //   motor_cmd_end = cmd_FWD;
                  //printf("BKW");
               //break;
               //case cmd_LEFT_TURN:
               //   motor_cmd = cmd_LEFT_TURN;
               //   motor_cmd_end = cmd_RIGHT_TURN;
                  //printf("TURN LEFT");
               //break;
               //case cmd_RIGHT_TURN:
               //   motor_cmd = cmd_RIGHT_TURN;
               //   motor_cmd_end = cmd_LEFT_TURN;
                  //printf("TURN LEFT");
               //break;
               //case cmd_LEFT_ARC:
               //   motor_cmd = cmd_LEFT_ARC;
               //   motor_cmd_end = cmd_LEFT_ARC_BKW;
                  //printf("LEFT ARC");
               //break;
               //case cmd_RIGHT_ARC:
               //   motor_cmd = cmd_RIGHT_ARC;
               //   motor_cmd_end = cmd_RIGHT_ARC_BKW;
                  //printf("LEFT ARC");
               //break;
            }
         }
         else
         {
            motor_cmd = cmd_STOP;
            motor_cmd_end = cmd_STOP;
         }
      }
   }//TASK2

   //TASK3
   task3_counter++;
	if (task3_counter >= TASK3_COUNTER_MAX)
   {
      /**********************************************************************
		*	Code for task3 @ 100ms                            *
      ***********************************************************************/
      task3_counter = 0;
      if(task3_enable)
      {

      /*Read analog channel 0 - bumper sensor*/
      //set_adc_channel(0);
      //bumper_sensor = read_adc();
      }
   }//TASK3

   //TASK4
   task4_counter++;
	if (task4_counter >= TASK4_COUNTER_MAX)
   {
      /**********************************************************************
      *			Code for task4 @ 1s                               *
      ************************************************************************/
      //this task only set the timer for 1 second to bumper sensor routine
      task4_counter = 0;
      if(task4_enable)
      {
     	   timeout_1s = TRUE;
      }
   }//TASK4

   //enable_interrupts(GLOBAL);

}

/*=============================================================================
 CCP2 IRQ
 ccp2_isr
=============================================================================*/
#INT_CCP2
ccp2_isr()
{
	n_rev_2++;

   if (t2_old >= CCP_2)
   {
      t2= ~t2_old + CCP_2;
   }
   else 
   { 
      t2 = CCP_2 - t2_old;
   }
   t2_old = CCP_2;

}


/*=============================================================================
 TMR0 overflow isr

=============================================================================*/
#INT_RTCC
rtcc_isr()
{
//LCD timer set

//	bsf     TIME_VAR, LCD_TIMER

}

/*=============================================================================
 TMR2 overflow isr

=============================================================================*/
#INT_TIMER2
timer2_isr()
{

}

/*=============================================================================
 init_multtask_var
 Multtask variables setup
=============================================================================*/
void init_multtask_var(BOOLEAN enable)
{
	task0_counter = 0;
   task1_counter = 0;
   task2_counter = 0;
   task3_counter = 0;
   task4_counter = 0;

   task0_enable = enable;
   task1_enable = enable;
   task2_enable = enable;
   task3_enable = enable;
   task4_enable = enable;

   return;
}

/*=============================================================================
 setup_multitask
 set the system to operate in time slice multitask
=============================================================================*/
void setup_multitask()
{
   //Initializing multitask variables
	init_multtask_var(TRUE);

   //Timer 1 programming - T1CON register programming
   //timer1_prog TMR1RESET_HIGH, TMR1RESET_LOW, 0
   setup_timer_1 ( T1_INTERNAL | T1_DIV_BY_1 );
   set_timer1(TMR1RESET);
   enable_interrupts(INT_CCP2);
   enable_interrupts(INT_TIMER1);
   enable_interrupts(GLOBAL);

   //print motor command stop
   //printf("STOP\n");

   //initialize IR_VAR variable
   ir_last_state = 1;
   ir_rx_on = 0;
   ir_rx_off = 0;

	return;

}


/*=============================================================================
  move_motor
  Set the motor speed
=============================================================================*/
void move_motor(BOOLEAN motor, BYTE vel)
{

   if (motor == MOTOR1)
   {
      pwm1 = vel;
   }
   else if (motor == MOTOR2)
   {
      pwm2 = vel;
   }

   return;
}

/*=============================================================================
 BLIND
Implement a blind behavior for the robot
 =============================================================================*/
void blind()
{
   //disable_interrupts(GLOBAL);
   /*Read analog channel 1 - light sensor */
   set_adc_channel(1);
   delay_us(20);
   light_sensor = read_adc();
   //printf("%u\n",light_sensor);

   if (light_sensor > 200)
   {
      blind_enable =  TRUE;
      blind_cmd = cmd_STOP;
      //printf("light_sensor = %u\n", light_sensor);
      //printf("%u\n",light_sensor);
      printf("blind_enable = %u\n", blind_enable);
   }
   else
   {
      blind_enable =  FALSE;
      //blind_cmd = cmd_FWD_PWM_ON;
   }

   //enable_interrupts(GLOBAL);

   return;
}


/*=============================================================================
 AVOID
Implement a avoid behavior for the robot
 =============================================================================*/
void avoid()
{

   if(!blind_enable)
   {
      if (obstacle_detected)
      {
         avoid_enable = TRUE;
#IF PRINT_MODES
         printf("avoid_enable = %u\n", avoid_enable);
#ENDIF

#IF INPUT_CHOPPING
         avoid_cmd = cmd_LEFT_ARC;
#ELSE
         avoid_cmd = cmd_LEFT_ARC_PWM_ON;
#ENDIF
      }
      else
      {
         avoid_enable = FALSE;
      }
   }

   return;
}
/*=============================================================================
 GET_BUMPER_VALUE
 Get the bumper analog value and set the appropriate directions
 =============================================================================*/
void get_bumper_value()
{
   unsigned BYTE bumper_sensor;

   //task1_enable = FALSE;

   enable_bumper = FALSE;

   /*Read analog channel 0 */
   set_adc_channel(0);
   delay_us(10);
   bumper_sensor = read_adc();
   //printf("bumper_sensor = %u\n", bumper_sensor);

   if (bumper_sensor > 74)                //> 74
   {
      if (bumper_sensor > 116)            // >116
      {
         if (bumper_sensor > 137)         // >137 enable_bumper = 0x07
         {
            bit_set(enable_bumper, ENABLE_BUMPER_BACK);
            bit_set(enable_bumper, ENABLE_BUMPER_LEFT);
            bit_set(enable_bumper, ENABLE_BUMPER_RIGHT);
         }
         else                            // 116 - 137 enable_bumper = 0x05
         {
            bit_set(enable_bumper, ENABLE_BUMPER_BACK);
            bit_set(enable_bumper, ENABLE_BUMPER_LEFT);
         }
      }
      else
      {
         if (bumper_sensor > 95)         // 95 - 116 enable_bumper = 0x06
         {
            bit_set(enable_bumper, ENABLE_BUMPER_BACK);
            bit_set(enable_bumper, ENABLE_BUMPER_RIGHT);
         }
         else
         {
            bit_set(enable_bumper, ENABLE_BUMPER_BACK);  // 74 - 95 enable_bumper = 0x04
         }
      }
   }
   else if (bumper_sensor > 31)          //31 - 74
   {
      if(bumper_sensor > 52)             //52 - 74 enable_bumper = 0x03
      {
         bit_set(enable_bumper, ENABLE_BUMPER_LEFT);
         bit_set(enable_bumper, ENABLE_BUMPER_RIGHT);
      }
      else
      {
         bit_set(enable_bumper, ENABLE_BUMPER_LEFT);     //31 - 52 enable_bumper = 0x01
      }
   }
   else if (bumper_sensor > 10)
   {
      bit_set(enable_bumper, ENABLE_BUMPER_RIGHT);       //10 - 31 enable_bumper = 0x02
   }
   else
   {
      enable_bumper = FALSE;
   }

   //printf("enable_bumper = %u\n", enable_bumper);
   return;
}

/*=============================================================================
 ESCAPE
Implement a escape behavior for the robot
=============================================================================*/
void escape()
{

   if(!avoid_enable && !blind_enable)
   {
      get_bumper_value();

      if (enable_bumper > 0)
      {
         escape_enable = TRUE;
         task4_counter = 0;
         timeout_1s = FALSE;
#IF PRINT_MODES
         printf("escape_enable = %u\n", escape_enable);
#ENDIF
         do
         {
            switch(enable_bumper)
            {
               case 0x01:  // left bumper activated
#IF INPUT_CHOPPING
                  escape_cmd = cmd_RIGHT_TURN;
#ELSE
                  escape_cmd = cmd_RIGHT_TURN_PWM_ON;
#ENDIF
               break;
               case 0x02:  // right bumper activated
#IF INPUT_CHOPPING
                  escape_cmd = cmd_LEFT_TURN;
#ELSE
                  escape_cmd = cmd_LEFT_TURN_PWM_ON;
#ENDIF
               break;
               case 0x04:  // back bumper activated
#IF INPUT_CHOPPING
                  escape_cmd = cmd_FWD;
#ELSE
                  escape_cmd = cmd_FWD_PWM_ON;
#ENDIF
               break;
               case 0x03:  //left and right bumper activated
#IF INPUT_CHOPPING
                  escape_cmd = cmd_BKW;
#ELSE
                  escape_cmd = cmd_BKW_PWM_ON;
#ENDIF
               break;
               case 0x05:  //back and left bumper activated
#IF INPUT_CHOPPING
                  escape_cmd = cmd_RIGHT_ARC_BKW;
#ELSE
                  escape_cmd = cmd_RIGHT_ARC_BKW_PWM_ON;
#ENDIF
               break;
               case 0x06:  //back and right bumper activated
#IF INPUT_CHOPPING
                  escape_cmd = cmd_LEFT_ARC_BKW;
#ELSE
                  escape_cmd = cmd_LEFT_ARC_BKW_PWM_ON;
#ENDIF
               break;
               case 0x07:  //back and right and left bumper activated
#IF INPUT_CHOPPING      //look at robot behaviour
                  escape_cmd = cmd_LEFT_ARC;
#ELSE
                  escape_cmd = cmd_LEFT_ARC_PWM_ON;
#ENDIF
               break;
            }
         }
         while(~timeout_1s);
      }
      else
      {
         escape_enable = FALSE;
      }
}

   return;

}

/*=============================================================================
CRUISE
Implement a cruise behavior for the robot
=============================================================================*/
void cruise()
{

   if(!escape_enable && !avoid_enable && !blind_enable)
   {
#IF INPUT_CHOPPING
      cruise_cmd = cmd_FWD;
#ELSE
      cruise_cmd = cmd_FWD_PWM_ON;
#ENDIF

      cruise_enable = TRUE;

     //printf("n_rev_2 = %lu \n", n_rev_2);
     //printf("T2 = %lu \n", t2);


#IF PRINT_MODES
      printf("CRUISE mode\n");
#ENDIF
   }

	return;
}

/*=============================================================================
 motor_init()
 Perform motor initializations
=============================================================================*/
void motor_init()
{

   //Initialize MOTOR_CMD and MOTOR_CMD_END Variable
   motor_cmd = cmd_STOP;
   motor_cmd_end = cmd_STOP;

   /* start pulses-PWM1 as 0 */
   n_pulses_PWM1 = 0;
   n_pulses_PWM2 = 0;

   tmp = input_b();
   tmp &= 0x03;        //strip the six most significant bits
   motor_cmd ^= tmp;   //set the motor cmd mask
   output_b(motor_cmd);

   //PWM initialization
	move_motor(MOTOR1, 95);
	move_motor(MOTOR2, 95);

   return;
}
/*=============================================================================
 prog_ports
 programming ports used in the microcontroler
=============================================================================*/
void prog_ports()
{

   //ADCON1 register programming (Analog/Digital ports)
   setup_adc_ports(A_ANALOG);

   //ADCON0 register programming (A/D conversion clock)
   setup_adc(ADC_CLOCK_DIV_32);    //ADCS1 = 0 ADCS0 = 1  select Fosc/32

   //programming input and output pins in PORTB
   SET_TRIS_B(0x01);    //RB0 = Input - RB1..RB7 = Output
   //#use fast_io (B)

   //programming input and output pins in PORTC
   //          76543210
   //movlf   b'10010100', TRISC     ; RC7=RX, RC1=PWM_OUTPUT RC2=input capture, RC4=IR_RX;
#IF USING_PWM
   SET_TRIS_C(0x94);
#ELSE
   //          76543210
   //movlf   b'10010110', TRISC     ; RC7=RX, RC1=RC2=input capture, RC4=IR_RX;
   SET_TRIS_C(0x96);
#ENDIF
   //#use fast_io (C)

   return;

}


/*=============================================================================
 initializations
 Perform some program initializations
=============================================================================*/
void initializations()
{

   // Programming ports
   prog_ports();

   // CCP1 register programming
   setup_ccp2 (CCP_CAPTURE_DIV_4);

	motor_init();

// setup initial multitask variables

	init_multtask_var(FALSE);

// Initialize LCDisplay and print startup message
// Comment out this routine to save debug time

	//lcd_init();
   printf("ROBOC MULTTASK \n");
   printf("Version 1 \n");

//reset behavior modes
   avoid_enable =  FALSE;
   escape_enable = FALSE;
   cruise_enable = FALSE;
   blind_enable = FALSE;

//init motor behaviour commands
   avoid_cmd = cmd_STOP;
   escape_cmd = cmd_STOP;
   cruise_cmd = cmd_STOP;
   avoid_cmd = cmd_STOP;

	ir_rx_state = FALSE;

//CCP module initializations
   t2 = 0;
   n_rev_2 = 0;
   t2_old = 0;

// setup multitask routine
   setup_multitask();

   return;

}

/******************************************************************************
Main code start here

******************************************************************************/
void main()
{

   setup_psp(PSP_DISABLED);
   setup_spi(FALSE);
   setup_counters(RTCC_INTERNAL,RTCC_DIV_1);
   setup_timer_2(T2_DISABLED,0,1);

   initializations();

   while(1)
   {
      
      printf("T2 = %lu \n", t2);
      blind();
      avoid();
      escape();
      cruise();

   }

}
