/*
 * Title: Stepper Motor Code
 *
 *  Author: Pavan Yeddanapudi
 *
 *  Description: Decided to write my own stepper code rather than using AccelStepper library.
 */

#include <Arduino.h>

#ifndef _STEPPERS_H_
#define _STEPPERS_H_


/*
 * Steppers move at 3 types of speed depending on the need
 */
#define STEPPER1_HIGH_SPEED		300	// Microseconds delay for programming
#define STEPPER1_MEDIUM_SPEED	800	// Microseconds delay for programming
#define STEPPER1_SLOW_SPEED		1000	// Microseconds delay for programming

#define STEPPER2_HIGH_SPEED		150	// Microseconds delay for programming
#define STEPPER2_MEDIUM_SPEED	400	// Microseconds delay for programming
#define STEPPER2_SLOW_SPEED		600	// Microseconds delay for programming

#define STEPPER3_HIGH_SPEED		120		// Microseconds delay for programming
#define STEPPER3_MEDIUM_SPEED	300	// Microseconds delay for programming
#define STEPPER3_SLOW_SPEED		500	// Microseconds delay for programming


#define MINIMUM_PULSEWIDTH		20
#define STEPS_PER_REVOLUTION	6400


/*
 * Teensy 4.1 board pins used for right stepper motor direction and step.
 */
#define STEPPER1_DIR_PIN 	4
#define STEPPER1_STEP_PIN 	3

/*
 * Teensy 4.1 board pins used for base (center) stepper motor direction and step.
 */
#define STEPPER2_DIR_PIN 	7
#define STEPPER2_STEP_PIN 	6

/*
 * Teensy 4.1 board pins used for left stepper motor direction and step.
 */
#define STEPPER3_DIR_PIN 	11
#define STEPPER3_STEP_PIN 	10


/*
 * Resset or zero position switch of steppers
 */
#define STEPPER1_RESET_TOP_PIN		    35
#define STEPPER2_RESET_PIN            34
#define STEPPER3_RESET_TOP_PIN		    36
#define STEPPER3_RESET_BOTTOM_PIN	    40
#define STEPPER3_ENABLE_PIN			      37
#define STEPPER2_ENABLE_PIN			      38
#define STEPPER1_ENABLE_PIN			      39
#define STEPPER1_RESET_BOTTOM_PIN	    33


#define STEPPER_ENABLE				      HIGH
#define STEPPER_DISABLE				      LOW


/*
 * RESET and max positions
 */
#define STEPPER1_RESET_POSITION		  0
#define STEPPER2_RESET_POSITION		  0
#define STEPPER2_MAX_POSITION		    3500
#define STEPPER3_RESET_POSITION		  0


#define	STEPPER3_STEPPER1_SAFETY_GAP_STEPS	1000
#define STEPPER1_STABLE_POSITION            1000
#define STEPPER2_MID_POSITION               1500


/*
 * Direction of movement
 */
#define STEPPER1_MOVE_FORWARD	  LOW
#define STEPPER1_MOVE_BACKWARD	HIGH

#define STEPPER2_MOVE_RIGHT		HIGH
#define STEPPER2_MOVE_LEFT		LOW

#define STEPPER3_MOVE_FORWARD	  HIGH
#define STEPPER3_MOVE_BACKWARD	LOW

#define STEPPER1_SAFE_RESET_MOVE_CNT	1000
#define STEPPER2_SAFE_RESET_MOVE_CNT	1000
#define STEPPER3_SAFE_RESET_MOVE_CNT	500


/*
 * Direction of movement when facing the ARM
 */
#define ARM_MOVE_UP       STEPPER3_MOVE_BACKWARD
#define ARM_MOVE_DOWN     STEPPER3_MOVE_FORWARD
#define ARM_MOVE_LEFT     STEPPER2_MOVE_RIGHT
#define ARM_MOVE_RIGHT    STEPPER2_MOVE_LEFT


extern long	stepper1_position;
extern long	stepper2_position;
extern long	stepper3_position;

extern void delay_microseconds(unsigned long);

extern void setup_steppers();

extern long get_stepper1_position();
extern long get_stepper2_position();
extern long get_stepper3_position();

extern int get_stepper1_move_direction(long);
extern int get_stepper2_move_direction(long);
extern int get_stepper3_move_direction(long);

extern int get_stepper1_direction_step(int);
extern int get_stepper2_direction_step(int);
extern int get_stepper3_direction_step(int);

extern unsigned long slow_step_stepper1(int, bool);
extern unsigned long medium_step_stepper1(int, bool);
extern unsigned long fast_step_stepper1(int, bool);

extern unsigned long slow_step_stepper2(int, bool);
extern unsigned long medium_step_stepper2(int, bool);
extern unsigned long fast_step_stepper2(int, bool);

extern unsigned long slow_step_stepper3(int, bool);
extern unsigned long medium_step_stepper3(int, bool);
extern unsigned long fast_step_stepper3(int, bool);

extern unsigned long slow_step_all(int, int, int);
extern unsigned long medium_step_all(int, int, int);
extern unsigned long fast_step_all(int, int, int);

extern void reset_stepper1();
extern void reset_stepper2();
extern void reset_stepper3();

extern void slipping_check_steppers();
extern void test_all_movement();

#endif // _STEPPERS_H_
