;**********************************************************************
;                                                                     *
;    Filename:	    multtsk4.asm                                      *
;    Date:          23/06/2002                                        *
;    File Version:  4.0                                               *
;                                                                     *
;    Author:        Marco carvalho                                    *
;    Company:       Robot hobby                                       *
;                                                                     * 
;                                                                     *
;**********************************************************************
;                                                                     *
;    Files required:    p16f877.inc                                   *
;                                                                     *
;                                                                     *
;                                                                     *
;**********************************************************************
;                                                                     *
;    Notes: This program perform a Multitask time slice system with 5 *
;           tasks running simultaneously. This program also implement *
;           a Subsumption behavior algoritm with Three behaviors.     *
;**********************************************************************


	list      p=16f877            ; list directive to define processor
	#include <p16f877.inc>        ; processor specific variable definitions
        ERRORLEVEL -302               ; skip warning 302 when compile   
	radix DEC

	
; Config --> Code Protect (CP) - OFF
;        --> Watch Dog Timer (WDT) - OFF
;        --> Brown Out Reset (BODEN)- ON
;        --> Power Up Timer (PWRTE) - ON
;        --> Oscillator (OSC) - High Speed (HS)
;        --> Flash Memory Write Enable (WRT) - ON
;        --> Low Voltage Programming (LVP) - OFF       
;        --> In Circuit Debug Mode (DEBUG) - OFF
;        --> Data EE Memory Code Protection (CPD) - OFF 

	__CONFIG _CP_OFF & _WDT_OFF & _BODEN_ON & _PWRTE_ON & _HS_OSC & _WRT_ENABLE_ON & _LVP_OFF & _DEBUG_OFF & _CPD_OFF 


;***** CONSTANTS USING IN PROGRAM

#DEFINE   Version                   1

;***** Motor constants

#DEFINE   n_turns     		    50
#DEFINE   LED_TASK_2                7
#DEFINE   ENC_1                     2
#DEFINE   ENC_2                     1
#DEFINE   ENC_RESOLUTION            16 
#DEFINE   CW                        1  
#DEFINE   CCW                       0   

#DEFINE   motor1_dir                3                 ; RB3 pin
#DEFINE   motor2_dir                6                 ; RB6 pin

;***** Motor Commands constants masks
;				      76543210        ; motor1  dir  motor2  dir	
#DEFINE   cmd_STOP                  b'01001000'       ;   off    X     off    X
#DEFINE   cmd_FWD                   b'01011010'       ;   on    FWD    on    FWD
#DEFINE   cmd_BKW                   b'01101100'       ;   on    BCW    on    BCW
#DEFINE   cmd_LEFT_TURN             b'01011100'       ;   on    FWD    on    BCW 
#DEFINE   cmd_RIGHT_TURN            b'01101010'       ;   on    BCW    on    FWD
#DEFINE   cmd_LEFT_ARC              b'01001010'       ;   on    FWD    off    X
#DEFINE   cmd_RIGHT_ARC             b'01011000'       ;   off    X     on    FWD
#DEFINE   cmd_LEFT_ARC_BKW          b'01001100'       ;   on    BCW    off    X
#DEFINE   cmd_RIGHT_ARC_BKW         b'01101000'       ;   off    X     on    BCW 

;***** constants for bumper sensor 

#DEFINE   BUMPER_TRUE_MASK          b'00000111'       ; Constants used in Bumper detection 
#DEFINE   BUMPER_BACK_MASK          b'00000001'
#DEFINE   BUMPER_RIGHT_MASK         b'00000010'
#DEFINE   BUMPER_LEFT_MASK          b'00000100'
#DEFINE   BUMPER_RIGHT_LEFT_MASK    b'00000110'

#DEFINE   BUMPER_BACK               b'00000001'       ; Constants used in Bumper_var variable
#DEFINE   BUMPER_RIGHT              b'00000010'
#DEFINE   BUMPER_LEFT               b'00000100'

;***** flag names for Robot behavior -> bits in BEHAVIOR_FLAG variable
#DEFINE   CRUISE_FLAG               0 
#DEFINE   ESCAPE_FLAG               1
#DEFINE   AVOID_FLAG                2 

;***** IR bits --> used in IR_VAR variable and PORTC bits
#DEFINE   IR_LAST_STATE             4
#DEFINE   IR_RX_bit                 4                 ; used to control PORTC bit 4
#DEFINE   IR_TX_bit                 5                 ; used to control PORTC bit 5

;***** TIME_VAR variable constants
#DEFINE   LCD_INIT_TIMER             0                 ; Controls the first LCD init
#DEFINE   FIRST_RUN                 1                 ; First time run program 
#DEFINE   timeout_1s                2                 ; Controls 1 seg timeout
#DEFINE   LCD_TIMER                 3                 ; Controls small time routines 
 
;***** Analog Channels constants 
#DEFINE   ANALOG_0                  0                 ; RA0 pin
#DEFINE   ANALOG_1                  0x08              ; RA1 pin
#DEFINE   ANALOG_2                  0x10              ; RA2 pin
#DEFINE   ANALOG_3                  0x18              ; RA3 pin
#DEFINE   ANALOG_4                  0x20              ; RA4 pin

;***** LCD constants

LCD_DATA	EQU	PORTD		              ; LCD data lines interface
LCD_DATA_TRIS	EQU	TRISD
LCD_CTRL	EQU	PORTE	                      ; LCD control lines interface
LCD_CTRL_TRIS   EQU     TRISE

; 4 x 16 characters LCD configuration

LCD_LINE0	EQU	0x00
LCD_LINE1	EQU	0x40
LCD_LINE2	EQU	0x10
LCD_LINE3	EQU	0x50

; PORTE bits
LCD_E		EQU	2		; LCD Enable control line
LCD_RW		EQU	1		; LCD Read/Write control line
LCD_RS		EQU	0		; LCD Register-Select control line

; PORTD bits
DB7		EQU	7		; LCD dataline 7 (MSB)
DB6		EQU	6		; LCD dataline 6
DB5		EQU	5		; LCD dataline 5
DB4		EQU	4		; LCD dataline 4
DB3		EQU	3		; LCD dataline 3
DB2		EQU	2		; LCD dataline 2
DB1		EQU	1		; LCD dataline 1
DB0		EQU	0		; LCD dataline 0 (LSB)

;***** Motor Port declaration
MOTOR_PORT      EQU     PORTB

;***** Constants used to perform time slice multitask system 

#DEFINE   TASK0_COUNTER_MAX         1    ; TASK0 @ 200us 
#DEFINE   TASK1_COUNTER_MAX         4    ; TASK1 @ 800us
#DEFINE   TASK2_COUNTER_MAX         100  ; TASK2 @ 20ms 
#DEFINE   TASK3_COUNTER_MAX         500  ; TASK3 @ 100ms 
#DEFINE   TASK4_COUNTER_MAX         5000 ; TASK4 @ 1s 


#DEFINE   TICKS_BETWEEN_INTERRUPTS  1000    ; interrupts @ 200us 
#DEfINE   INTERRUPT_OVERHEAD        19
#DEFINE   TMR1RESET                 (0xFFFF - (TICKS_BETWEEN_INTERRUPTS-INTERRUPT_OVERHEAD))
#DEFINE   TMR1RESET_HIGH            TMR1RESET >> 8
#DEFINE   TMR1RESET_LOW             (TMR1RESET & 0xFF)

FALSE         EQU     0
TRUE          EQU     1

;***** VARIABLE DEFINITIONS
w_temp                 EQU     0x30        ; variable used for context saving 
status_temp            EQU     0x31        ; variable used for context saving
;w_temp_table_read      EQU     0x52  

n_pulses_PWM1          EQU     0x32        
n_pulses_PWM2          EQU     0x44
PWM_1                  EQU     0x33        ; variable used to store Motor 1 PWM value  
PWM_2                  EQU     0x34        ; variable used to store motor 2 PWM value
n_rev_1                EQU     0x35        ; variable used to store Motor 1 revolutions 
n_rev_2                EQU     0x36        ; variable used to store Motor 2 revolutions 

reg_8                  EQU     0x70

task0_counter          EQU     reg_8 + 1
task1_counter          EQU     reg_8 + 4
task2_counter          EQU     reg_8 + 7
task3_counter          EQU     reg_8 + 10
task4_counter          EQU     reg_8 + 13
task5_counter          EQU     reg_8 + 16

task0_enable           EQU     reg_8 + 3
task1_enable           EQU     reg_8 + 6
task2_enable           EQU     reg_8 + 9
task3_enable           EQU     reg_8 + 12
task4_enable           EQU     reg_8 + 15
task5_enable  	       EQU     reg_8 + 18

;task2_go               EQU     0x43
;timeout_1s             EQU     0x41            ; used to task3 setting

bumper_hit             EQU     0x42

MOTOR_CMD              EQU     0x49
MOTOR_CMD_END          EQU     0x50

BEHAVIOR_FLAG          EQU     0x51

TASKX_COUNTER_MAX_P_L  EQU     0x37   ; variables to use in the pointer routine
TASKX_COUNTER_MAX_P_H  EQU     0x38
taskX_counter_P        EQU     0x39
taskX_counter_1_P      EQU     0x40

BUMPER_VAR             EQU     0x45  
LCD_TEMP               EQU     0x46
COUNT                  EQU     0x47
TIME_VAR               EQU     0x48

a_val                  EQU     0x53   ; variable used to store temporary analog values             

CRUISE_CMD             EQU     0x54
ESCAPE_CMD             EQU     0x55
AVOID_CMD              EQU     0x56

IR_VAR                 EQU     0x57 
IR_RX_ON               EQU     0x58
IR_RX_OFF              EQU     0x59
IR_RX_STATE            EQU     0x43 

;  Registers used in table
 CBLOCK 0x60
Tbllo, TblHI			;  Offsets into the Table
Temp
 ENDC

;**********************************************************************

;********************************************************************************************
;                        macro definitions
;********************************************************************************************
;=============================================================================
; movlf
; move the literal(l) to a register(f)
;============================================================================= 
movlf macro literal, register

	movlw literal
	movwf register

	endm

;=============================================================================
; set_motor_cmd
; Set the motor command - initial and end pwm values
;=============================================================================
set_motor_cmd macro COMMAND
	movlw   CRUISE_CMD	  ; 
	movwf   FSR		  ; start the pointer

	movlw   COMMAND           ; 
	addwf   FSR, F            ; Point to behavior motor command

	movf    INDF, w
	movwf   MOTOR_CMD
	call    chg_motor_cmd 

	endm
;=============================================================================
; move_motor1
; Set the velocity and direction to motor1
;=============================================================================
move_motor1 macro vel, dir
	movlw   vel
	movwf   PWM_1

        bcf     STATUS,RP0        ; bank = 0
	movlw   dir
	sublw   CW

	btfss   STATUS, Z         ; move == CW?  
        bcf     MOTOR_PORT,motor1_dir  ; No, motor_dir = CCW (pin RB3)

	btfsc   STATUS, Z         ; move == CCW?  
        bsf     MOTOR_PORT,motor1_dir  ; yes, motor_dir = CW (pin RB3)

	endm

;=============================================================================
; move_motor2
; Set the velocity and direction to motor2
;=============================================================================
move_motor2 macro vel, dir
	movlw   vel
	movwf   PWM_2

        bcf     STATUS,RP0        ; bank = 0
	movlw   dir
	sublw   CW

	btfss   STATUS, Z         ; move == CW?  
        bcf     MOTOR_PORT,motor2_dir  ; motor_dir = CCW (pin RB6)

	btfsc   STATUS, Z         ; move == CCW?  
        bsf     MOTOR_PORT,motor2_dir  ; yes, motor_dir = CW (pin RB6)

	endm

;=============================================================================
; timer0_prog
; program timer0 
;============================================================================= 
timer0_prog macro byte, pre_scale

;Timer 0  programming - Option_reg programming
	bsf     STATUS,RP0        ; bank = 1 
	movf    OPTION_REG, W
	andlw   0xC0              ; put 0 on the pre-scale bits(bit0..bit2)
	iorlw   pre_scale         ; set pre_scale
	movwf   OPTION_REG        ; OPTION_REG pre_scale programming

        bcf    STATUS,RP0         ; bank = 0
	movlw   byte
	movwf   TMR0              ; TMR0 = byte
	
;Reset TMR0/TMR2 interrupt flags
	bcf    INTCON, T0IF	  ; turn off TMR0 overflow flag 

;Enable the interrupts in the follow order TMR0-PEIE-GIE
	bsf    INTCON, T0IE       ; enable TMR0 interrupt		
	bsf    INTCON, PEIE       ; enable peripheral interrupts	
	bsf    INTCON, GIE        ; enable Global IRQs

	endm

;=============================================================================
; timer2_prog
; program timer2 
;============================================================================= 
timer2_prog macro byte, pre_scale 

;Timer 2 programming - T2CON register programming
	bcf     STATUS, RP0       ; bank = 0
	movf    T2CON, W
	andlw   0x07              ; set post scaler as 0 
	iorlw   pre_scale         ; set pre_scale 
	movwf   T2CON             ; T2CON programming

	bcf     T2CON, TMR2ON     ; turn TMR2 off

	movlw   byte
	movwf   TMR2              ; TMR2 = byte

	bsf     T2CON, TMR2ON     ; turn TMR2 on

;Reset TMR2 interrupt flags
        bcf    STATUS,RP0         ; bank = 0
	bcf    PIR1, TMR2IF	  ; turn off TMR2 overflow flag 

;Enable the TMR2 interrupt in the follow order TMR2-PEIE-GIE
        bsf    STATUS,RP0         ; bank = 1 
	bsf    PIE1, TMR2IE       ; enable TMR2 interrupt

        bcf    STATUS,RP0         ; bank = 0
	bsf    INTCON, PEIE       ; enable peripheral interrupts	
	bsf    INTCON, GIE        ; enable Global IRQs

	endm

;=============================================================================
; timer1_prog
; program timer1 timer
;============================================================================= 
timer1_prog macro byte_high, byte_low, pre_scale

;Timer 1 programming - T1CON register programming

	bcf     STATUS, RP0       ; bank = 0
	movlw   pre_scale         ; b'00000000' -->TMR1 OFF / Prescale 1:1
	andlw   0x30              ; pre_scale mask for T1CKPS1 and T1CKPS0
	movwf   T1CON             ; T1CON programming	

	movlw  byte_high          ; TMR1H=byte_high
	movwf  TMR1H
	
	movlw  byte_low           ; TMR1L=byte_low
	movwf  TMR1L	

	bsf    T1CON, TMR1ON      ; turn TMR1 on

        bcf    STATUS,RP0         ; bank = 0
	bcf    PIR1, TMR1IF	  ; turn off TMR1 overflow flag 
        bsf    STATUS,RP0         ; bank = 1 
	bsf    PIE1, TMR1IE       ; enable TMR1 interrupt

        bcf    STATUS,RP0         ; bank = 0
	bcf    PIR1, CCP1IF	  ; turn off CCP1 overflow flag 
        bsf    STATUS,RP0         ; bank = 1 
	bsf    PIE1, CCP1IE       ; enable CCP1 interrupt
	
	bsf    INTCON, PEIE       ; enable peripheral interrupts	
	bsf    INTCON, GIE        ; enable Global IRQs

	bcf     STATUS, RP0       ; bank = 0

	endm

;=============================================================================
; init_multtask_var
; Multtask variables setup 
;============================================================================= 
init_multtask_var macro bool

	clrf    task0_counter     
	clrf    task0_counter+1   ;task0_counter = 0	

	clrf    task1_counter     
	clrf    task1_counter+1   ;task1_counter = 0

	clrf    task2_counter     
	clrf    task2_counter+1   ;task2_counter = 0

	clrf    task3_counter     
	clrf    task3_counter+1   ;task3_counter = 0

	clrf    task4_counter     
	clrf    task4_counter+1   ;task4_counter = 0

	movlw   bool
	movwf   task0_enable      ;task0_enable = bool
	movwf   task1_enable      ;task1_enable = bool
	movwf   task2_enable      ;task2_enable = bool
	movwf   task3_enable      ;task3_enable = bool
	movwf   task4_enable      ;task4_enable = bool

	endm

;=============================================================================
; LCD_gotoxy
; Put LCD cursor in x,y position
;============================================================================= 
LCD_gotoxy macro x, y

	movlw	x + y
	call	LCDSDDA		; Position cursor

	endm

;=============================================================================
; execute_task_x
; Verify the conditions to task execution
; If the conditions are true return TRUE in W. Return FALSE in W if the
; conditions are false.
;=============================================================================
execute_task_x macro taskX_counter, TASKX_COUNTER_MAX

	movlw   reg_8		  ; reg_8[0] = &reg_8	
	movwf   FSR		  ; start the pointer

	movlw   taskX_counter+1   ; reg_8[X+1] = &taskX_counter+1
	addwf   FSR, F

	movf    FSR, w
	movwf   taskX_counter_1_P ; Pointer to taskX_counter+1

	movlf   High TASKX_COUNTER_MAX, TASKX_COUNTER_MAX_P_H   ; save TASKX_COUNTER_MAX_HIGH content

	movlf   TASKX_COUNTER_MAX & 0xFF, TASKX_COUNTER_MAX_P_L ; save TASKX_COUNTER_MAX_LOW content

	call    test_taskX

	endm

;=============================================================================
; print_message 
; Print MSG in LCD
;============================================================================= 
print_message macro LINE, COLUMM, MSG

	bcf     STATUS, RP0       ; bank = 0
	bcf     T1CON, TMR1ON     ; turn TMR1 off to prevent messages put in
                                  ; different lines  
	LCD_gotoxy LINE, COLUMM
	
	movlw   MSG               ; Display the MSGth message  
	call	GET_MSG_FROM_TABLE

	bsf     T1CON, TMR1ON     ; turn TMR1 on
	endm

;=============================================================================
; read_analog 
; Perform an analog conversion for a specified analog channel
; Analog value is put in a_val variable 
;============================================================================= 
read_analog macro analog_channel

	bcf     STATUS, RP0       ; bank = 0

	movf    ADCON0, W	  ; Select A/D input Channel
	andlw   0xC7              ; Mask for reset ADCON<CHS2:CHS0> bits
	iorlw   analog_channel
	movwf   ADCON0

	bsf     ADCON0, ADON      ; turn on A/D module

	call    WAIT_FOR_50us     ; Wait a sample time - decrease to 10us

	bsf     ADCON0, GO_DONE   ; start conversion

	call    POLL_GO_DONE_BIT  ; polling GO/DONE bit

	movf    ADRESH, w         ;  	
	movwf   a_val             ; a_val:=ADRESH  
 
	endm

;=============================================================================
; CCP1_prog
; Perform athe CCP1 register programming
;============================================================================= 
CCP1_prog macro pulses_per_irq

	movlf   pulses_per_irq, CCP1CON ; capture mode every 16th raising edge

	endm



;********************************************************************************
; Program starts here                                                           *
;********************************************************************************
START
	PAGE
	ORG     0x00                   ; processor reset vector
	clrf    PCLATH                 ; ensure page bits are cleared
	goto    main                   ; go to beginning of program

;**************************************************************************
;                      Interrupt Service Routines                         *
;**************************************************************************

	ORG     0x04                   ; interrupt vector location
	movwf   w_temp                 ; save off current W register contents
	movf	STATUS,w               ; move status register into W register
	movwf	status_temp            ; save off contents of STATUS register

IRQ_ISR
; Polling interrupts beggining with TMR1 IRQ

	btfss   PIR1, TMR1IF           ; TMR1 overflow?
          goto    Test_TMR0_overflow   ; No, Then goto Test_TMR0_overflow

;**************************************************************************
;			TMR1 overflow isr                                 *
;**************************************************************************

        bcf     STATUS,RP0             ; bank = 0
	
	bcf     PIR1, TMR1IF	       ; turn off TMR1 overflow flag 
	bcf     T1CON, TMR1ON          ; turn TMR1 off

	movlf   TMR1RESET_HIGH, TMR1H  ; TMR1H=TMR1RESET_HIGH 
	
	movlf   TMR1RESET_LOW, TMR1L   ; TMR1L=TMR1RESET_LOW

	bsf     T1CON, TMR1ON          ; turn TMR1 on

; Set LCD_COUNT to perform the LCD timing initialization

	btfss   TIME_VAR, FIRST_RUN    ; First program run? 
	  goto  TASK_SCHEDULER         ; No, then TASK_SCHEDULER

	bsf   TIME_VAR, LCD_INIT_TIMER ; LCD_INIT_TIMER:= 1
	bcf   TIME_VAR, FIRST_RUN      ; FIRST_RUN:= False
	goto  status_w                 ; goto TMR1 ISR end

TASK_SCHEDULER

TASK0

	incf    task0_counter, f  ; task0_counter++
	btfsc   STATUS, Z
	 incf   task0_counter+1, f

	execute_task_x 1, TASK0_COUNTER_MAX
	sublw   TRUE
	btfsc   STATUS, Z         ; task0_enable and number of ticks ==TRUE ? 
	 goto    TASK_0_code      ; yes, then goto TASK_0_code

TASK1

	incf    task1_counter, f  ; task1_counter++
	btfsc   STATUS, Z
	 incf   task1_counter+1, f

	execute_task_x 4, TASK1_COUNTER_MAX
	sublw   TRUE
	btfsc   STATUS, Z         ; task1_enable and number of ticks ==TRUE ? 
	 goto    TASK_1_code      ; yes, then goto TASK_1_code

TASK2

	incf    task2_counter, f  ; task2_counter++
	btfsc   STATUS, Z
	 incf   task2_counter+1, f

	execute_task_x 7, TASK2_COUNTER_MAX
	sublw   TRUE
	btfsc   STATUS, Z         ; task2_enable and number of ticks ==TRUE ? 
	 goto    TASK_2_code       ; yes, then goto TASK_2_code

TASK3

	incf    task3_counter, f  ; task3_counter++
	btfsc   STATUS, Z
	 incf   task3_counter+1, f

	execute_task_x 10, TASK3_COUNTER_MAX
	sublw   TRUE
	btfsc   STATUS, Z         ; task3_enable and number of ticks ==TRUE ? 
	 goto    TASK_3_code       ; yes, then goto TASK_3_code

TASK4

	incf    task4_counter, f  ; task4_counter++
	btfsc   STATUS, Z
	 incf   task4_counter+1, f

	execute_task_x 13, TASK4_COUNTER_MAX
	sublw   TRUE
	btfsc   STATUS, Z         ; task4_enable and number of ticks ==TRUE ? 
	 goto    TASK_4_code       ; yes, then goto TASK_4_code

	goto    status_w

;**************************************************************************
;			Code for task0 @ 200us                            *
;**************************************************************************
TASK_0_code
;this task perform the pwm duty cycle

	incf    n_pulses_PWM1, f       ; n_pulses_PWM++    

	movf    n_pulses_PWM1, w
	subwf   PWM_1, w     

	btfss   STATUS, Z              ; n_pulses_PWM == PWM1 ?
          goto    TASK1                ; no, then go to TASK1

	movf    PORTB, W
	andlw   b'10000001'            ; strip the middle bits
	iorwf   MOTOR_CMD, W           ; set the motor cmd mask
	movwf   PORTB

	clrf    n_pulses_PWM1          ; n_pulses_PWM1 = 0

	movlf   FALSE, task0_enable; task0_enable = FALSE 

	goto    TASK2     ;Changed for a while before TASK1 

;**************************************************************************
;			Code for task1 @ 800us                            *
;**************************************************************************
; this task perform the square wave for infra red sensor modulation
TASK_1_code

; when active task1 perform the output signal to IR circuit 

	btfss   IR_VAR, IR_LAST_STATE  ; IR_LAST_STATE == 1?
          goto    SET_IR_TX_BIT        ; No, Then SET_IR_BIT 

RESET_IR_TX_BIT                        ; Yes, then RESET_IR_BIT
	bsf     PORTC, IR_TX_bit       ; IR_TX_bit = 0 -> negative logic	    
	bcf     IR_VAR, IR_LAST_STATE  ; IR_LAST_STATE = 0	    
	goto    TASK1_end 
	
SET_IR_TX_BIT
	bcf     PORTC, IR_TX_bit       ; IR_TX_bit = 1	-> negative logic
	bsf     IR_VAR, IR_LAST_STATE  ; IR_LAST_STATE = 1 

TASK1_end	
	goto    TASK2

;**************************************************************************
;			Code for task2  @20ms                             *
;**************************************************************************
TASK_2_code
; This task perform the pwm frequency = 50Hz and Bumper sensor read

        bcf     STATUS,RP0             ; bank = 0

	movf    PORTB, W
	andlw   b'10000001'            ; strip the middle bits
	iorwf   MOTOR_CMD_END, W       ; set the motor cmd mask
	movwf   PORTB

	movlf   TRUE, task0_enable     ;task0_enable = TRUE

; Perform the bumper sensor read

;	read_analog 0                  ; read the PORTA<0> analog value

; Scheduler for the motor control. Put here the behavior in the highest priority
; order 
MOTOR_CONTROL_SCHEDULER

	btfss  BEHAVIOR_FLAG, AVOID_FLAG  ; AVOID behavior active?
	  goto Test_behavior_1

        set_motor_cmd 2	  ; set motor to AVOID_CMD
	goto   MOTOR_SCHDL_END	

Test_behavior_1

	btfss  BEHAVIOR_FLAG, ESCAPE_FLAG  ; ESCAPE behavior active?
	  goto Test_behavior_2

        set_motor_cmd 1	  ; set motor to ESCAPE_CMD
	goto   MOTOR_SCHDL_END	

Test_behavior_2   

	btfss  BEHAVIOR_FLAG, CRUISE_FLAG  ; CRUISE behavior active?
	  goto Test_behavior_3

        set_motor_cmd 0	  ; set motor to CRUISE_CMD	  
	goto   MOTOR_SCHDL_END

Test_behavior_3

MOTOR_SCHDL_END
	goto    TASK2
;**************************************************************************
;			Code for task3 @ 100ms                            *
;**************************************************************************
TASK_3_code
;	movlw   TRUE
;	movwf   task2_go
	goto    TASK3

;**************************************************************************
;			Code for task4 @ 1s                               *
;**************************************************************************
; this task only set the timer for 1 second to bumper sensor routine
TASK_4_code
	bsf    TIME_VAR, timeout_1s
	goto   status_w

;**************************************************************************
;			CCP1 isr                                          *
;**************************************************************************

Test_CCP1_IRQ
 
	btfss   PIR1, CCP1IF       ; CCP1 IRQ?
          goto    TASK1		   ; no, go back to task 1

; CCP1 isr

	bcf     PIR1, CCP1IF	   ; turn off CCP1 overflow flag 
	incf    n_rev_1, f         ; n_rev_1++
	goto    TASK1          

;**************************************************************************
;			TMR0 overflow isr                                 *
;**************************************************************************

Test_TMR0_overflow

	btfss   INTCON, T0IF           ; TMR0 overflow?
          goto    Test_TMR2_overflow   ; No, Then Test_TMR2_overflow


; TMR0 overflow isr

;LCD timer set

	bsf     TIME_VAR, LCD_TIMER

TMR0_end	
	bcf     INTCON, T0IF	       ; turn off TMR0 overflow flag 

	goto    status_w

;**************************************************************************
;			TMR2 overflow isr                                 *
;**************************************************************************

Test_TMR2_overflow

	btfss   PIR1, TMR2IF       ; TMR2 overflow?
          goto    status_w

; TMR2 overflow isr

;       bsf     STATUS,RP0        ; bank = 1 
;	bcf     PIE1, TMR2IE      ; Disable TMR2 interrupt
;       bcf     STATUS,RP0        ; bank = 0
	bcf     PIR1, TMR2IF	  ; turn off TMR2 overflow flag 
	goto    status_w


status_w
	movf    status_temp,w     ; retrieve copy of STATUS register
	movwf	STATUS            ; restore pre-isr STATUS register contents
	swapf   w_temp,f
	swapf   w_temp,w          ; restore pre-isr W register contents
	retfie                    ; return from interrupt


;*****************************************************************************
;                        Subroutines 
;*****************************************************************************

;=============================================================================
; POLL_GO_DONE_BIT
; Polling Go/DONE bit to verify the end convertion time for A/D port
;============================================================================= 
POLL_GO_DONE_BIT

WAIT_TIME
	btfsc   ADCON0, GO_DONE	  ; GO_DONE bit = 0? 
	  goto  WAIT_TIME         ; No, then WAIT_TIME

	return
   	
;=============================================================================
; chg_motor_cmd
; Set the MOTOR_CMD and MOTOR_CMD_END variables
;============================================================================= 
chg_motor_cmd

;test cmd_STOP

	movf    MOTOR_CMD, W
	sublw   cmd_STOP
	btfss   STATUS, Z         ; MOTOR_CMD = cmd_STOP?
	  goto  test_cmd_FWD 	  ; no, then go to test_cmd_FWD

	movf    MOTOR_CMD, W          
	movwf   MOTOR_CMD_END     ; MOTOR_CMD_END = MOTOR_CMD

	print_message LCD_LINE3, 1, 8 ; print STOP message
	goto    end_chg_motor_cmd

test_cmd_FWD

	movf    MOTOR_CMD, W
	sublw   cmd_FWD
	btfss   STATUS, Z              ; MOTOR_CMD = cmd_FWD?
	  goto  test_cmd_BKW 	       ; no, then go to test_cmd_BKW

	movlf   cmd_BKW, MOTOR_CMD_END ; MOTOR_CMD_END = cmd_BKW

;	print_message LCD_LINE3, 1, 2  ; print FWD message
	goto    end_chg_motor_cmd

test_cmd_BKW

	movf    MOTOR_CMD, W
	sublw   cmd_BKW
	btfss   STATUS, Z              ; MOTOR_CMD = cmd_BKW?
	  goto  test_cmd_LEFT_TURN     ; no, then go to test_cmd_LEFT_TURN

	movlf   cmd_FWD, MOTOR_CMD_END ; MOTOR_CMD_END = cmd_FWD 

	print_message LCD_LINE3, 1, 3  ; print BKW message
	goto    end_chg_motor_cmd

test_cmd_LEFT_TURN

	movf    MOTOR_CMD, W
	sublw   cmd_LEFT_TURN
	btfss   STATUS, Z              ; MOTOR_CMD = cmd_LEFT_TURN?
	  goto  test_cmd_RIGHT_TURN    ; no, then go to test_cmd_RIGHT_TURN

	movlf   cmd_RIGHT_TURN, MOTOR_CMD_END     ; MOTOR_CMD_END = cmd_RIGHT_TURN

	print_message LCD_LINE3, 1, 5  ; print TURN LEFT message
	goto    end_chg_motor_cmd

test_cmd_RIGHT_TURN

	movf    MOTOR_CMD, W
	sublw   cmd_RIGHT_TURN
	btfss   STATUS, Z              ; MOTOR_CMD = cmd_RIGHT_TURN?
	  goto  test_cmd_LEFT_ARC      ; no, then go to test_cmd_LEFT_ARC
   
	movlf   cmd_LEFT_TURN, MOTOR_CMD_END     ; MOTOR_CMD_END = cmd_LEFT_TURN

	print_message LCD_LINE3, 1, 4  ; print TURN RIGHT message
	goto    end_chg_motor_cmd

test_cmd_LEFT_ARC

	movf    MOTOR_CMD, W
	sublw   cmd_LEFT_ARC
	btfss   STATUS, Z         ; MOTOR_CMD = cmd_LEFT_ARC?
	  goto  test_cmd_RIGHT_ARC ; no, then go to test_cmd_RIGHT_ARC

	movlf   cmd_LEFT_ARC_BKW, MOTOR_CMD_END     ; MOTOR_CMD_END = cmd_LEFT_ARC_BKW
	goto    end_chg_motor_cmd

test_cmd_RIGHT_ARC

	movf    MOTOR_CMD, W
	sublw   cmd_RIGHT_ARC
	btfss   STATUS, Z         ; MOTOR_CMD = cmd_LEFT_ARC?
	  goto  end_chg_motor_cmd ; no, then go to test_cmd_LEFT_ARC

	movlf   cmd_RIGHT_ARC_BKW, MOTOR_CMD_END     ; MOTOR_CMD_END = cmd_LEFT_ARC_BKW
	goto    end_chg_motor_cmd

end_chg_motor_cmd
	return
	 
;=============================================================================
; test_taskX
; Complete execute_task_x macro
; Identify what task are ready to be executed
;=============================================================================
test_taskX

	movf    INDF, w
	subwf   TASKX_COUNTER_MAX_P_H, w     

	btfsc   STATUS, C         ; taskX_counter_hi > TASKX_COUNTER_MAX_hi?
          goto    TASKX_TEMP_HI   ; No, then go to TASKX_TEMP_HI
	goto    TASK_X_LO

TASKX_TEMP_HI
	btfss   STATUS, Z         ; taskX_counter_hi = TASKX_COUNTER_MAX_hi ?         
          goto    NEXT_TASK       ; No, then go to NEXT_TASK

TASK_X_LO

	movf    taskX_counter_1_P, w	  ; Restore the pointer to start of the array
	subwf   FSR, F	

	movlw   1
	subwf   taskX_counter_1_P, w      ; get the offset to reg_8[X]
	addwf   FSR, F

	movf    INDF, w
	subwf   TASKX_COUNTER_MAX_P_L, w      

	btfsc   STATUS, C         ; taskX_counter_lo > TASKX_COUNTER_MAX_lo?
 	  goto    TASKX_TEMP_LO   ; No, then go to TASKX_TEMP_LO
	goto    TASK_X

TASKX_TEMP_LO
	btfss   STATUS, Z         ; taskX_counter_lo = TASKX_COUNTER_MAX_lo ?        
	  goto    NEXT_TASK       ; No, then go to NEXT_TASK

TASK_X

	movlw   1
	subwf   taskX_counter_1_P, w   ; Restore the pointer to start of the array   
	subwf   FSR, F	

	movlw   1
	subwf   taskX_counter_1_P, w  
	addwf   FSR, F
	clrf    INDF     	      ; taskX_counter:=0

	subwf   FSR, F		      ; Restore the pointer to start of the array	

	movf    taskX_counter_1_P, w  ; reg_8[X+1] = &taskX_counter+1
	addwf   FSR, F
	clrf    INDF                  ; taskX_counter+1:=0

	subwf   FSR, F	              ; Restore the pointer to start of the array

	movf    taskX_counter_1_P, w
	addlw   1		      	
	addwf   FSR, F                ; get the offset to taskX_enable

	movf    INDF, w
	sublw   TRUE

	btfss   STATUS, Z              ; taskX_enable==TRUE ? 
          goto    NEXT_TASK            ; No, then go to NEXT_TASK

	movlw   TRUE                   ; yes, return TRUE in W  
	goto    END_TASKX	

NEXT_TASK
	movlw   FALSE                  ; return FALSE in W

END_TASKX
	return

;*****************************************************************************
; Useful timing routines
;*****************************************************************************

;=============================================================================
; WAIT_FOR_50us
; Wait for 50us using TMR0 timer for some commands used in LCD programming 
; like Entry mode Set, display ON/OFF control, Cursor or Display Shift, etc
;=============================================================================
WAIT_FOR_50us

	bcf     T1CON, TMR1ON          ; turn TMR1 off to prevent TMR1 interrupt	
	timer0_prog 0x83,0             ; 0x83->131 pulses@ 0.4us = 52.4us +50us = ovrflw   

	bcf     TIME_VAR, LCD_TIMER    ; Reset LCD_TIMER

WAIT_50us
	btfss   TIME_VAR, LCD_TIMER    ; LCD_TIMER == 1?
          goto    WAIT_50us	       ; No, Then WAIT_50us

	bsf    T1CON, TMR1ON           ; turn TMR1 on to activate TMR1 IRQ	
	return

;=============================================================================
; WAIT_FOR_1600us
; Wait for 1.6ms using TMR1 timer for some commands used in LCD programming
; like Clear display, Return Home etc
;=============================================================================
WAIT_FOR_1600us

	bcf     T1CON, TMR1ON          ; turn TMR1 off to prevent TMR1 interrupt	
	timer0_prog 0x06,4             ; prescale = 1:32, 0x06 = 0xFF-0xF9

	bcf     TIME_VAR, LCD_TIMER    ; Reset LCD_TIMER

WAIT_1600us
	btfss   TIME_VAR, LCD_TIMER    ; LCD_TIMER == 1?
          goto    WAIT_1600us          ; No, Then WAIT_1600us

	bsf    T1CON, TMR1ON      ; turn TMR1 on to activate TMR1 IRQ	
	return

;*****************************************************************************
; LCD Module Subroutines
;*****************************************************************************
;
;=============================================================================
; LCDINIT
; Initilize LC-Display Module
; Should be modified to your needs (i.e. display type, cursor on/off, etc.)
; 
;=============================================================================
LCDINIT

	bcf     STATUS,RP0	       ; bank = 0
				       ; Busy-flag is not yet valid
	clrf	LCD_CTRL	       ; ALL PORT output should output Low.

; power-up delay

	bsf     TIME_VAR, FIRST_RUN    ; 
	timer1_prog 0x55,0x19,0x20     ; reset TMR1, prescale = 1:4 , 5519 = FFFF-AA1E

	bcf     TIME_VAR, LCD_INIT_TIMER
  
WAIT_35ms
	btfss   TIME_VAR, LCD_INIT_TIMER
          goto    WAIT_35ms


; Busy Flag should be valid from here - setup interface, number of lines, etc

	movlw	0x38		; 8-bit-interface, 2-lines
	call	LCDPUTCMD

; wait for 50us 

	call    WAIT_FOR_50us

;Display ON/OFF Control

	movlw	0x0C		; disp.on, curs.off, no-blink
	call	LCDDMODE

; Clear LCD

	call	LCDCLEAR

; Entry mode set

	movlw	0x02		; auto-inc (shift-cursor)
	call	LCDEMODE

;*****************************************************************************
; Display Startup Message
;        01234567890123456
;	------------------
;	| ROBOC MULTTASK |
;	|   Version 1    |
;	|                |
;	|                |
;	------------------
;*****************************************************************************

	print_message LCD_LINE0, 1, 0 ; Display the 1th message - ROBOC MULTTASK

	print_message LCD_LINE1, 2, 8 ; Display the 9th message - Version


	LCD_gotoxy LCD_LINE1, 11
	movlw	"1"		; ASCII '1' 0x31 
	call	LCDPUTCHAR	; Display program version

	
	return

;=============================================================================
; LCDPUTCMD
; Sends command to LCD
; Required command must be in W
; OK
;=============================================================================
LCDPUTCMD
	movwf	LCD_TEMP	; Command to be sent is in W
	call	LCDBUSY		; Wait for LCD to be ready
	bcf	LCD_CTRL, LCD_RW; Set LCD in read mode
	bcf	LCD_CTRL, LCD_RS; Set LCD in command mode
	bsf	LCD_CTRL, LCD_E	; LCD E-line High
	movf	LCD_TEMP, W
	movwf	LCD_DATA	; Send data to LCD
	bcf	LCD_CTRL, LCD_E	; LCD E-line Low
	return

;=============================================================================
; LCDDMODE
; Sets display control.
; Required display mode must be set in W
;  b0	: 0 = cursor blink off	1 = cursor blink on
;  b1	: 0 = cursor off	1 = cursor on
;  b2	: 0 = display off	1 = display on (display data remains in DDRAM)
;  b3-7	: don't care
;=============================================================================
LCDDMODE
	andlw   0x07		; Strip upper bits
	iorlw	0x08		; Function set
	call	LCDPUTCMD
	call    WAIT_FOR_50us   ; wait for 50us
	return

;=============================================================================
; LCDEMODE
; Sets entry mode of display.
; Required entry mode must be set in W
;  b0	: 0 = no display shift	1 = display shift
;  b1	: 0 = auto-decrement	1 = auto-increment
;  b2-7	: don't care
;=============================================================================
LCDEMODE
	andlw	0x03		; Strip upper bits
	iorlw	0x04		; Function set
	call	LCDPUTCMD
	call    WAIT_FOR_50us   ; wait for 50us
	return

;=============================================================================
; LCDCLEAR
; Clears display and returns cursor to home position (upper-left corner).
;=============================================================================
LCDCLEAR
	movlw	0x01
	call	LCDPUTCMD
	call    WAIT_FOR_1600us  ; Wait for 1.6ms
	return

;=============================================================================
; LCDBUSY
; Returns when LCD busy-flag is inactive
;=============================================================================
LCDBUSY
	bsf	STATUS,RP0	; Select Register page 1
	movlw	0xFF		; Set PORTD for input
	movwf	LCD_DATA_TRIS
	bcf	STATUS, RP0	; Select Register page 0
	bcf	LCD_CTRL, LCD_RS   ; Set LCD for command mode
	bsf	LCD_CTRL, LCD_RW   ; Setup to read busy flag
	bsf	LCD_CTRL, LCD_E	; LCD E-line High
	movf	LCD_DATA, W	; Read busy flag + DDram address
	bcf	LCD_CTRL, LCD_E	; LCD E-line Low
	andlw	0x80		; Check Busy flag, High = Busy
	btfss	STATUS, Z
	goto	LCDBUSY

LCDNOTBUSY	
	bcf	LCD_CTRL, LCD_RW
	bsf	STATUS, RP0	; Select Register page 1
	movlw	0x00
	movwf	LCD_DATA_TRIS	; Set PORTD for output
	bcf	STATUS, RP0	; Select Register page 0
	return

;=============================================================================
; LCDHOME
; Returns cursor to home position.
; Returns display to original position (when shifted).
;=============================================================================
LCDHOME
	movlw	0x02
	call	LCDPUTCMD
	call    WAIT_FOR_1600us  ; Wait for 1.6ms
	return

;=============================================================================
; LCDSCGA
; Sets Character-Generator-RAM address. CGRAM is read/written after
;  this setting.
; Required CGRAM address must be set in W
;  b0-5	: required CGRAM address
;  b6-7	: don't care
;=============================================================================
LCDSCGA
	andlw	0x3F		; Strip upper bits
	iorlw	0x40		; Function set
	call	LCDPUTCMD
	call    WAIT_FOR_50us   ; wait for 50us
	return
;=============================================================================
; LCDSDDA
; Sets the Display-Data-RAM address. DDRAM data is read/written after
;  this setting.
; Required DDRAM address must be set in W
;  b0-6	: required DDRAM address
;  b7	: don't care
;=============================================================================
LCDSDDA
	iorlw	0x80		; Function set
	call	LCDPUTCMD
	call    WAIT_FOR_50us   ; wait for 50us	
	return
;=============================================================================
; LCDGADDR
; Returns address counter contents, used for both DDRAM and CGRAM.
; RAM address is returned in W
;
;=============================================================================
LCDGADDR
	bsf	STATUS,RP0	; Select Register page 1
	movlw	0xFF		; Set PORTB for input
	movlw	LCD_DATA_TRIS
	bcf	STATUS, RP0	; Select Register page 0
	bcf	LCD_CTRL, LCD_RS; Set LCD for command mode
	bsf	LCD_CTRL, LCD_RW; Setup to read busy flag
	bsf	LCD_CTRL, LCD_E	; LCD E-line High
	movf	LCD_DATA, W	; Read busy flag + RAM address
	bcf	LCD_CTRL, LCD_E	; LCD E-line Low
	andlw	0x7F		; Strip upper bit
	bcf	LCD_CTRL, LCD_RW
	bsf	STATUS, RP0	; Select Register page 1
	movlw	0x00
	movwf	LCD_DATA_TRIS	; Set PORTB for output
	bcf	STATUS, RP0	; Select Register page 0
	return
;=============================================================================
; LCDPUTCHAR
; Sends character to LCD
; Required character must be in W
; OK
;=============================================================================
LCDPUTCHAR
	movwf	LCD_TEMP	; Character to be sent is in W
	call	LCDBUSY		; Wait for LCD to be ready
	bcf	LCD_CTRL, LCD_RW; Set LCD in read mode
	bsf	LCD_CTRL, LCD_RS; Set LCD in data mode
	bsf	LCD_CTRL, LCD_E	; LCD E-line High
	movf	LCD_TEMP, W
	movwf	LCD_DATA	; Send data to LCD
	bcf	LCD_CTRL, LCD_E	; LCD E-line Low

	call    WAIT_FOR_50us
	return

;=============================================================================
;setup_multitask
;set the system to operate in time slice multitask
;=============================================================================
setup_multitask

;Initializing multitask variables

	init_multtask_var TRUE

;Timer 1 programming - T1CON register programming

	timer1_prog TMR1RESET_HIGH, TMR1RESET_LOW, 0  

; print motor command stop

;	print_message LCD_LINE3, 0, 8  ; print motor_stop message

; initialize IR_VAR variable

	clrf    IR_VAR                 ; Reset IR_pulses 
	bsf     IR_VAR, IR_LAST_STATE  ; IR_LAST_STATE = 1

	return

;=============================================================================
;flash_led_task_2
;flash_led for indicate the activation of task_2
;=============================================================================
flash_led_task_2

	btfsc   PORTB, LED_TASK_2; LED_TASK_2 == 0? 
	bsf     PORTB, LED_TASK_2; No, then set to 1

	btfss   PORTB, LED_TASK_2; LED_TASK_2 == 1? 	
	bcf     PORTB, LED_TASK_2	

	return

;=============================================================================
;prog_ports
;programming ports used in the microcontroler
;=============================================================================
prog_ports

; STATUS and TMR0 register reset

	movlf   0, STATUS

; ADCON1 register programming (Analog/Digital ports)

        bsf     STATUS,RP0            ; bank = 1 
		; 76543210	
        movlf   b'00000010', ADCON1   ; PORTE = Digital / PORTA = Analog

        bcf     STATUS,RP0            ; bank = 0 	

; ADCON0 register programming (A/D conversion clock)

	bcf     ADCON0, ADCS1
	bsf     ADCON0, ADCS0   ; select Fosc/8
	
; programming input and output pins in PORTB

        bsf     STATUS,RP0             ; bank = 1 

        movlf   b'00000001', TRISB     ; RB0 = Input - RB1..RB7 = Output

        bcf     STATUS,RP0             ; bank = 0

; programming input and output pins in PORTC

        bsf     STATUS,RP0             ; bank = 1
 
	;         76543210
        movlf   b'10010110', TRISC     ; RC7=RX, RC1=RC2=input capture, RC4=IR_RX; 

; programming LCD_DATA(PORTD) and LCD_CTRL(PORTE)

        bcf     STATUS,RP0             ; bank = 0		

	clrf	LCD_CTRL               ; ALL PORT output should output Low.
	clrf	LCD_DATA

	bsf	STATUS, RP0	       ; bank = 1
	movlf	0x00, LCD_CTRL_TRIS    ; RE2-0 outputs
	movlf	0x00, LCD_DATA_TRIS    ; RD7-0 outputs

	bcf	STATUS, RP0	  ; bank = 0
	
	return

;=============================================================================
;GET_MSG_FROM_TABLE
;Get messages from table. Message Number Must be in W.
;=============================================================================
GET_MSG_FROM_TABLE		;  Figure Out where the Specified
				;   Message is
	movwf  COUNT		;  Save the Message Count
	clrf   Tbllo	   	;  Reset the Table Values
	clrf   TblHI

SEARCH_MESSAGE

	movf   COUNT, F		;  If COUNT == 0, then Stop Search
	btfsc  STATUS, Z        ;  and print message
	 goto  PRINT_MSG

	call   MSGTable		;  get the Value in the Table

	iorlw  0		;  Are we at the End of a Message?
	btfsc  STATUS, Z
	  decf  COUNT, F	;  Yes - Decrement COUNT

	incf   Tbllo, F		;  Point to the Next Table Element
	btfsc  STATUS, Z
	  incf  TblHI, F	

	goto   SEARCH_MESSAGE

PRINT_MSG

	incf   Tbllo, F		;  Point to the Next Table Element
	btfsc  STATUS, Z
	  incf  TblHI, F	

	call   MSGTable	        ;  get the Value in the Table
	iorlw  0		;  Are we at the End of a Message?
	btfsc  STATUS, Z
	  goto MSG_END          ;  Yes, then finish PRINT_MSG

	call	LCDPUTCHAR	;  Display character

	goto   PRINT_MSG

MSG_END			
	return		

;=============================================================================
;AVOID 
;Implement a avoid behavior for the robot
;=============================================================================
AVOID

AVOID_STATE1
	btfss   IR_VAR, IR_LAST_STATE ; IR_LAST_STATE = 1? 
          goto  AVOID_STATE2          ; No, then goto AVOID_STATE2

    	movf    PORTC, W
	andlw   0x10                  ; Strip no valid data - mask for PORTC<4> pin   
	movwf   IR_RX_STATE           ; IR_RX_STATE := PORTC & 0x10
	goto    AVOID_STATE1	      ; No, then back to the AVOID_STATE1  

AVOID_STATE2
	movf    IR_RX_STATE, W
	movwf   IR_RX_ON              ; IR_RX_ON = PORTC, IR_RX pin input

AVOID_STATE3
	btfsc   IR_VAR, IR_LAST_STATE ; IR_LAST_STATE = 1? 
          goto  AVOID_STATE4          ; yes, then goto AVOID_STATE4

    	movf    PORTC, W
	andlw   0x10                  ; Strip no valid data  
	movwf   IR_RX_STATE           ; IR_RX_STATE := PORTC & 0x10 
	goto    AVOID_STATE3	      ; No, then goto AVOID_STATE3  

AVOID_STATE4
	movf    IR_RX_STATE, W
	movwf   IR_RX_OFF             ; IR_RX_OFF = PORTC, IR_RX pin input  

	comf    IR_RX_ON, W           ; W = /IR_RX_ON
	andwf   IR_RX_OFF, W          ; W = IR_RX_OFF & /IR_RX_ON 

	btfsc   STATUS, Z	      ; W = 0x10? 
	  goto  AVOID_STATE5
        
	bsf     BEHAVIOR_FLAG, AVOID_FLAG  ; yes, Set AVOID flag
	print_message LCD_LINE2, 1, 9      ; print AVOID behavior message

	movlw   cmd_LEFT_ARC
	movwf   AVOID_CMD           ; AVOID_CMD:=cmd_LEFT_ARC

	goto AVOID_END

AVOID_STATE5
        bcf     BEHAVIOR_FLAG, AVOID_FLAG  ; No, Reset AVOID flag

AVOID_END
	return

;=============================================================================
;GET_BUMPER_VALUE
;Get the bumper analog value and set the appropriate directions
;=============================================================================
GET_BUMPER_VALUE

	read_analog 0                  ; Read analog channel 0

	movlw 74
	subwf a_val, W                    
	btfsc STATUS, C                ; a_val >74
          goto BIGGER_THAN_74          ; Yes, then goto BIGGER_THAN_74

	movlw 31
	subwf a_val, W                    
	btfsc STATUS, C                ; a_val >31
          goto BIGGER_THAN_31          ; Yes, then goto BIGGER_THAN_31

	movlw 10
	subwf a_val, W                    
	btfsc STATUS, C                ; a_val >10
          goto RESET_BUMPER_VAR        ; Yes, then goto RESET_BUMPER_VAR	

	bsf BUMPER_VAR, BUMPER_RIGHT_MASK ; No, Set bumper_right flag
	goto BUMPER_VALUE_END

BIGGER_THAN_74
	movlw 116
	subwf a_val, W                    
	btfsc STATUS, C                ; a_val >116
          goto BIGGER_THAN_116         ; Yes, then goto BIGGER_THAN_116

	movlw 95
	subwf a_val , W                   
	btfsc STATUS, C                ; a_val >95
          goto SET_BACK_RIGHT          ; Yes, then SET_BACK_RIGHT direction

	bsf BUMPER_VAR, BUMPER_BACK_MASK ; No, Set bumper_back flag
	goto BUMPER_VALUE_END	

BIGGER_THAN_116
	movlw 137
	subwf a_val, W                    
	btfsc STATUS, C                ; a_val >137
          goto SET_BACK_LEFT_RIGHT     ; Yes, then goto SET_BACK_LEFT_RIGHT

	bsf BUMPER_VAR, BUMPER_LEFT_MASK ; No, Set bumper_left flag
	bsf BUMPER_VAR, BUMPER_BACK_MASK ; and bumper_back flag
	goto BUMPER_VALUE_END	

BIGGER_THAN_31
	movlw 52
	subwf a_val, W                    
	btfsc STATUS, C                ; a_val >31
          goto SET_LEFT_RIGHT          ; Yes, then goto SET_LEFT_RIGHT

	bsf BUMPER_VAR, BUMPER_LEFT_MASK ; No, Set bumper_left flag
	goto BUMPER_VALUE_END	

SET_BACK_RIGHT
	bsf BUMPER_VAR, BUMPER_BACK_MASK 
	bsf BUMPER_VAR, BUMPER_RIGHT_MASK
	goto BUMPER_VALUE_END

SET_BACK_LEFT_RIGHT
	bsf BUMPER_VAR, BUMPER_BACK_MASK 
	bsf BUMPER_VAR, BUMPER_RIGHT_MASK
	bsf BUMPER_VAR, BUMPER_LEFT_MASK
	goto BUMPER_VALUE_END

SET_LEFT_RIGHT
	bsf BUMPER_VAR, BUMPER_RIGHT_MASK
	bsf BUMPER_VAR, BUMPER_LEFT_MASK
	goto BUMPER_VALUE_END

RESET_BUMPER_VAR
	clrf BUMPER_VAR

BUMPER_VALUE_END
	return

;=============================================================================
;ESCAPE
;Implement a escape behavior for the robot
;=============================================================================
ESCAPE

	call GET_BUMPER_VALUE   ; get bumper direction
ESState_1

	movf  BUMPER_VAR, w
	andlw BUMPER_TRUE_MASK  ; check bumper activation

	btfss STATUS, Z		; Bumper activated?
	  goto ESCAPE_END       ; no, then go to ESCAPE_END

	bsf BEHAVIOR_FLAG, ESCAPE_FLAG ; yes then set ESCAPE flag

	print_message LCD_LINE2, 1, 6  ; print ESCAPE behavior message

	movf  BUMPER_VAR, w
	sublw BUMPER_RIGHT_LEFT_MASK
	btfsc STATUS, Z         ; BUMPER_RIGHT_LEFT activated?
	  goto ESState_2	; yes, then go to ESState_2

	movf BUMPER_VAR, w
	sublw BUMPER_RIGHT_MASK
	btfsc STATUS, Z         ; BUMPER_RIGHT activated?
	  goto ESState_4	; yes, then go to ESState_4

	movf BUMPER_VAR, w
	sublw BUMPER_BACK_MASK
	btfsc STATUS, Z         ; BUMPER_BACK activated?
	  goto ESState_5	; yes, then go to ESState_5

	goto ESState_3

ESState_2

	bcf   TIME_VAR, timeout_1s ; initializing flag timeout_1s = FALSE

ES_State_2
	btfsc  TIME_VAR, timeout_1s    ; timeout_1s = True?
	  goto ESState_1	       ; yes, then go to ESState_1

	movlf   cmd_BKW, ESCAPE_CMD    ; ESCAPE_CMD:=cmd_BKW

	goto ES_State_2

ESState_3

	bcf   TIME_VAR, timeout_1s ; initializing flag timeout_1s = FALSE

ES_State_3
	btfsc  TIME_VAR, timeout_1s    ; timeout_1s = True?
	  goto ESState_2	       ; yes, then go to ESState_2

	movlf   cmd_RIGHT_TURN, ESCAPE_CMD    ; ESCAPE_CMD:=cmd_RIGHT_TURN

	goto ES_State_3

ESState_4

	bcf   TIME_VAR, timeout_1s     ; initializing flag timeout_1s = FALSE

ES_State_4
	btfsc  TIME_VAR, timeout_1s    ; timeout_1s = True?
	  goto ESState_1	       ; yes, then go to ESState_1

	movlf   cmd_LEFT_TURN, ESCAPE_CMD      ; ESCAPE_CMD:=cmd_LEFT_TURN

	goto ES_State_4

ESState_5

	bcf   TIME_VAR, timeout_1s     ; initializing flag timeout_1s = FALSE

ES_State_5
	btfsc  TIME_VAR, timeout_1s    ; timeout_1s = True?
	  goto ESState_1               ; yes, then go to ESState_1

	movlf   cmd_FWD, ESCAPE_CMD    ; ESCAPE_CMD:=cmd_FWD

	goto ES_State_5

ESCAPE_END
	bcf BEHAVIOR_FLAG, ESCAPE_FLAG ; clear Escape_flag
	return

;=============================================================================
;CRUISE
;Implement a cruise behavior for the robot
;=============================================================================
CRUISE

	movlf   cmd_FWD, CRUISE_CMD         ; CRUISE_CMD:=cmd_FWD 

	bsf     BEHAVIOR_FLAG, CRUISE_FLAG  ; yes then set CRUISE flag

	print_message LCD_LINE2, 1, 5       ; print CRUISE behavior message	

	return

;=============================================================================
;MOTOR_INIT
;Perform motor initializations
;=============================================================================
MOTOR_INIT
;Initialize MOTOR_CMD and MOTOR_CMD_END Variable

	movlf   cmd_STOP, MOTOR_CMD
	movwf   MOTOR_CMD_END
	
	movf    PORTB, W
	andlw   b'10000001'        ; strip the middle bits
	iorwf   MOTOR_CMD, W       ; set the motor cmd mask
	movwf   PORTB

;PWM initialization
	move_motor1 95, CW
	move_motor2 95, CW

	return

;=============================================================================
;INITIALIZATIONS
;Perform some program initializations
;=============================================================================
INITIALIZATIONS

; Programming ports
	call    prog_ports

;CCP1 register programming

	CCP1_prog   b'0000111'        ; capture mode every 16th raising edge

	call    MOTOR_INIT

; setup initial multitask variables

	init_multtask_var FALSE

; Initialize LCDisplay and print startup message
; Comment out this routine to save debug time

	call	LCDINIT	

; setup multitask routine

	call    setup_multitask

	return

;=============================================================================
;read_temperature
;Read temperature from the thermistor in PORTA<0>
;=============================================================================
read_temperature

	read_analog  ANALOG_0

	movlw   128
	subwf   a_val, W         ; a_val >128 

	btfsc   STATUS, C
	  bsf     PORTB, LED_TASK_2; yes, then set to 1	

	btfss   STATUS, C	
          bcf     PORTB, LED_TASK_2; No, then set to 0

	return

;***********************************************************************************************

main

	call    INITIALIZATIONS

; main loop

while_1

	call    CRUISE
;	call    ESCAPE
;	call    AVOID

	goto    while_1          

;*****************************************************************************
; Program ends here
;*****************************************************************************
LOOP
		GOTO	LOOP		; Stay here forever

;=============================================================================
; Table message to display
;=============================================================================

MSGTable			;  Place all the Messages Here
  movlw  HIGH TableStart	;   Setup PCLATH for the Table
  addwf  TblHI, w
  movwf  PCLATH
  movlw  TableStart & 0x0FF	;  Figure out the Offset
  addwf  Tbllo, w
  btfsc  STATUS, C		;  If Necessary, Increment PCLATH
   incf  PCLATH, F			;   to get the Correct 256 Address Page
  movwf  PCL			;  Update the PC

TableStart			;  Table Data
Msg0
  dt	 " ROBOC MULTTASK", 0    ; 1st message
Msg1
  dt	 " Go FWD        ", 0            ; 2nd message
Msg2
  dt	 " Go BKW        ", 0            ; 3rd message
Msg3
  dt	 " Go TURN RIGHT ", 0     ; 4th message
Msg4
  dt	 " Go TURN LEFT  ", 0      ; 5th message
Msg5
  dt	 " CRUISE        ", 0      ; 6th message
Msg6
  dt	 " ESCAPE        ", 0      ; 7th message
Msg7
  dt	 " STOP          ", 0      ; 8th message  
Msg8
  dt	 " Version       ", 0      ; 9th message 
Msg9 
  dt	 " AVOID         ", 0      ; 10th message

	END                       ; directive 'end of program'

