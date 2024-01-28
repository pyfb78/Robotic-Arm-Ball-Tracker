/*
 * Title: Stepper Motor Code
 *
 *  Author: Pavan Yeddanapudi
 *
 *  Description: Decided to write my own stepper code rather than using AccelStepper library.
 *               As  there many specific issues which need to handled by my ARM with reset
 *               switches and hence decided to write a constant speed movement stepper functions.
 */

#include "steppers.hpp"

/*
 * Position of the stepper motors
 */
long	stepper1_position = -1;
long	stepper2_position = -1;
long	stepper3_position = -1;


/*
 * Teensy hardware reset pin assignments
 */
int	s1_reset_top 		= STEPPER1_RESET_TOP_PIN;
int	s1_reset_bottom 	= STEPPER1_RESET_BOTTOM_PIN;
int	s2_reset 			= STEPPER2_RESET_PIN;
int	s3_reset_top 		= STEPPER3_RESET_TOP_PIN;
int	s3_reset_bottom 	= STEPPER3_RESET_BOTTOM_PIN;


long cpu_wasting_time = 0;


/*
 * Microsecond delay function
 */
void
delay_microseconds(unsigned long ms)
{
	unsigned long t = micros();

	while (micros() < (t + ms)) {
		++cpu_wasting_time;
	}
}


/*
 * This function setsup the hardware configuration os steppers
 */
void
setup_steppers()
{
	pinMode(STEPPER1_DIR_PIN, OUTPUT);
	pinMode(STEPPER1_STEP_PIN, OUTPUT);
	pinMode(STEPPER1_ENABLE_PIN, OUTPUT);
	digitalWrite(STEPPER1_ENABLE_PIN, STEPPER_DISABLE);

	pinMode(STEPPER2_DIR_PIN, OUTPUT);
	pinMode(STEPPER2_STEP_PIN, OUTPUT);
	pinMode(STEPPER2_ENABLE_PIN, OUTPUT);
	digitalWrite(STEPPER2_ENABLE_PIN, STEPPER_DISABLE);

	pinMode(STEPPER3_DIR_PIN, OUTPUT);
	pinMode(STEPPER3_STEP_PIN, OUTPUT);
	pinMode(STEPPER3_ENABLE_PIN, OUTPUT);
	digitalWrite(STEPPER3_ENABLE_PIN, STEPPER_DISABLE);


	pinMode(STEPPER1_RESET_TOP_PIN, INPUT);
	pinMode(STEPPER1_RESET_BOTTOM_PIN, INPUT);

	pinMode(STEPPER2_RESET_PIN, INPUT);

	pinMode(STEPPER3_RESET_TOP_PIN, INPUT);
	pinMode(STEPPER3_RESET_BOTTOM_PIN, INPUT);

	delay(10);
}


/*
 * This function returns the position of stepper1
 */
long
get_stepper1_position()
{
	return stepper1_position;
}


/*
 * This function returns the position of stepper2
 */
long
get_stepper2_position()
{
	return stepper2_position;
}


/*
 * This function returns the position of stepper3
 */
long
get_stepper3_position()
{
	return stepper3_position;
}


/*
 * This function returns the move direction for stepper 1
 * based the delta or the move value.
 */
int
get_stepper1_move_direction(long delta)
{
	if (delta == 0) {
		return -1;
	}

	if (delta < 0) {
		return STEPPER1_MOVE_BACKWARD;
	}
	return STEPPER1_MOVE_FORWARD;
}


/*
 * This function returns the move direction for stepper 2
 * based the delta or the move value.
 */
int
get_stepper2_move_direction(long delta)
{
	if (delta == 0) {
		return -1;
	}

	if (delta < 0) {
		return STEPPER2_MOVE_LEFT;
	}
	return STEPPER2_MOVE_RIGHT;
}


/*
 * This function returns the move direction for stepper 3
 * based the delta or the move value.
 */
int
get_stepper3_move_direction(long delta)
{
	if (delta == 0) {
		return -1;
	}

	if (delta < 0) {
		return STEPPER3_MOVE_FORWARD;
	}
	return STEPPER3_MOVE_BACKWARD;
}


/*
 * This function returns the increment or decrement step based on direction for stepper 1
 */
int
get_stepper1_direction_step(int dir)
{
	if (dir == -1) {
		return 0;
	}
	if (dir == STEPPER1_MOVE_FORWARD) {
		return 1;
	}
	return -1;
}


/*
 * This function returns the increment or decrement step based on direction for stepper 2
 */
int
get_stepper2_direction_step(int dir)
{
	if (dir == -1) {
		return 0;
	}
	if (dir == STEPPER2_MOVE_RIGHT) {
		return 1;
	}
	return -1;
}


/*
 * This function returns the increment or decrement step based on direction for stepper 3
 */
int
get_stepper3_direction_step(int dir)
{
	if (dir == -1) {
		return 0;
	}
	if (dir == STEPPER3_MOVE_FORWARD) {
		return 1;
	}
	return -1;
}


/*
 * This function makes sure the stepper 2 motion is in the range for safety
 */
bool
is_stepper2_position_safe(long delta)
{
	long pos = stepper2_position + delta;

	if ((pos >= STEPPER2_RESET_POSITION) && (pos <= STEPPER2_MAX_POSITION)) {
		return true;
	}
	return false;
}


/*
 * This function makes sure the stepper 2 next postion is in the range for safety
 */
bool
is_absolute_stepper2_position_safe(long pos)
{
	if ((pos >= STEPPER2_RESET_POSITION) && (pos <= STEPPER2_MAX_POSITION)) {
		return true;
	}
	return false;
}


/*
 * Stepper1 prrogramming for moving the stepper by one step in the given
 * direction.
 */
void
just_step_stepper1(int dir)
{
	digitalWrite(STEPPER1_ENABLE_PIN, STEPPER_ENABLE);
	delay_microseconds(MINIMUM_PULSEWIDTH);
	digitalWrite(STEPPER1_DIR_PIN, dir);
	digitalWrite(STEPPER1_STEP_PIN, HIGH);
	delay_microseconds(MINIMUM_PULSEWIDTH);
	digitalWrite(STEPPER1_STEP_PIN, LOW);
	digitalWrite(STEPPER1_ENABLE_PIN, STEPPER_DISABLE);
}


/*
 * Slowly move the stepper3 to keep the safety gap with stepper1
 */
void
slow_limit_check_stepper3_for_stepper1()
{
	if (digitalRead(s3_reset_top)) {
		for (int i = 0; i < STEPPER3_STEPPER1_SAFETY_GAP_STEPS; i++) {
			slow_step_stepper3(STEPPER3_MOVE_BACKWARD, false);
		}
	}
}


/*
 * With medium speed move the stepper3 to keep the safety gap with stepper1
 */
void
medium_limit_check_stepper3_for_stepper1()
{
	if (digitalRead(s3_reset_top)) {
		for (int i = 0; i < STEPPER3_STEPPER1_SAFETY_GAP_STEPS; i++) {
			medium_step_stepper3(STEPPER3_MOVE_BACKWARD, false);
		}
	}
}


/*
 * With high speed move the stepper3 to keep the safety gap with stepper1
 */
void
fast_limit_check_stepper3_for_stepper1()
{
	if (digitalRead(s3_reset_top)) {
		for (int i = 0; i < STEPPER3_STEPPER1_SAFETY_GAP_STEPS; i++) {
			fast_step_stepper3(STEPPER3_MOVE_BACKWARD, false);
		}
	}
}


/*
 * With slow speed move the stepper1 by one step
 * dflag can be used to delay till the step is complete.
 */
unsigned long
slow_step_stepper1(int dir, bool dflag)
{
	unsigned long d = 0;

	if (digitalRead(s1_reset_bottom)) {
		stepper1_position = STEPPER1_RESET_POSITION;
		if (dir == STEPPER1_MOVE_BACKWARD) {
			return 0;
		}
	}

	if (digitalRead(s1_reset_top) && (dir == STEPPER1_MOVE_FORWARD)) {
		return 0;
	}
	slow_limit_check_stepper3_for_stepper1();

	just_step_stepper1(dir);

	if (dflag) {
		d = STEPPER1_SLOW_SPEED;
	} else {
		delay_microseconds(STEPPER1_SLOW_SPEED);
	}

	if (dir == STEPPER1_MOVE_FORWARD) {
		++stepper1_position;
	} else {
		--stepper1_position;
	}
	return d;
}


/*
 * With medium speed move the stepper1 by one step
 * dflag can be used to delay till the step is complete.
 */
unsigned long
medium_step_stepper1(int dir, bool dflag)
{
	unsigned long d = 0;

	if (digitalRead(s1_reset_bottom)) {
		stepper1_position = STEPPER1_RESET_POSITION;
		if (dir == STEPPER1_MOVE_BACKWARD) {
			return 0;
		}
	}

	if (digitalRead(s1_reset_top) && (dir == STEPPER1_MOVE_FORWARD)) {
		return 0;
	}
	medium_limit_check_stepper3_for_stepper1();

	just_step_stepper1(dir);

	if (dflag) {
		d = STEPPER1_MEDIUM_SPEED;
	} else {
		delay_microseconds(STEPPER1_MEDIUM_SPEED);
	}

	if (dir == STEPPER1_MOVE_FORWARD) {
		++stepper1_position;
	} else {
		--stepper1_position;
	}
	return d;
}


/*
 * With high speed move the stepper1 by one step
 * dflag can be used to delay till the step is complete.
 */
unsigned long
fast_step_stepper1(int dir, bool dflag)
{
	unsigned long d = 0;

	if (digitalRead(s1_reset_bottom)) {
		stepper1_position = STEPPER1_RESET_POSITION;
		if (dir == STEPPER1_MOVE_BACKWARD) {
			return 0;
		}
	}

	if (digitalRead(s1_reset_top) && (dir == STEPPER1_MOVE_FORWARD)) {
		return 0;
	}
	fast_limit_check_stepper3_for_stepper1();

	just_step_stepper1(dir);

	if (dflag) {
		d = STEPPER1_HIGH_SPEED;
	} else {
		delay_microseconds(STEPPER1_HIGH_SPEED);
	}

	if (dir == STEPPER1_MOVE_FORWARD) {
		++stepper1_position;
	} else {
		--stepper1_position;
	}
	return d;
}



/*
 * Stepper2 prrogramming for moving the stepper by one step in the given
 * direction.
 */
void
just_step_stepper2(int dir)
{
	digitalWrite(STEPPER2_ENABLE_PIN, STEPPER_ENABLE);
	delay_microseconds(MINIMUM_PULSEWIDTH);
	digitalWrite(STEPPER2_DIR_PIN, dir);
	digitalWrite(STEPPER2_STEP_PIN, HIGH);
	delay_microseconds(MINIMUM_PULSEWIDTH);
	digitalWrite(STEPPER2_STEP_PIN, LOW);
	digitalWrite(STEPPER2_ENABLE_PIN, STEPPER_DISABLE);
}


/*
 * With slow speed move the stepper2 by one step
 * dflag can be used to delay till the step is complete.
 */
unsigned long
slow_step_stepper2(int dir, bool dflag)
{
	unsigned long d = 0;

	if (digitalRead(s2_reset)) {
		stepper2_position = STEPPER2_RESET_POSITION;
		if (dir == STEPPER2_MOVE_LEFT) {
			return 0;
		}
	}

  if (dflag) {
	  if (!is_stepper2_position_safe(get_stepper2_direction_step(dir))) {
		  return 0;
	  }
  }
  
	just_step_stepper2(dir);

	if (dflag) {
		d = STEPPER2_SLOW_SPEED;
	} else {
		delay_microseconds(STEPPER2_SLOW_SPEED);
	}

	if (dir == STEPPER2_MOVE_RIGHT) {
		++stepper2_position;
	} else {
		--stepper2_position;
	}
	return d;
}


/*
 * With medium speed move the stepper1 by one step
 * dflag can be used to delay till the step is complete.
 */
unsigned long
medium_step_stepper2(int dir, bool dflag)
{
	unsigned long d = 0;

	if (digitalRead(s2_reset)) {
		stepper2_position = STEPPER2_RESET_POSITION;
		if (dir == STEPPER2_MOVE_LEFT) {
			return 0;
		}
	}

	if (dflag) {
    if (!is_stepper2_position_safe(get_stepper2_direction_step(dir))) {
      return 0;
    }
  }

	just_step_stepper2(dir);

	if (dflag) {
		d = STEPPER2_MEDIUM_SPEED;
	} else {
		delay_microseconds(STEPPER2_MEDIUM_SPEED);
	}

	if (dir == STEPPER2_MOVE_RIGHT) {
		++stepper2_position;
	} else {
		--stepper2_position;
	}
	return d;
}


/*
 * With high speed move the stepper1 by one step
 * dflag can be used to delay till the step is complete.
 */
unsigned long
fast_step_stepper2(int dir, bool dflag)
{
	unsigned long d = 0;

	if (digitalRead(s2_reset)) {
		stepper2_position = STEPPER2_RESET_POSITION;
		if (dir == STEPPER2_MOVE_LEFT) {
			return 0;
		}
	}

	if (dflag) {
    if (!is_stepper2_position_safe(get_stepper2_direction_step(dir))) {
      return 0;
    }
  }

	just_step_stepper2(dir);

	if (dflag) {
		d = STEPPER2_HIGH_SPEED;
	} else {
		delay_microseconds(STEPPER2_HIGH_SPEED);
	}

	if (dir == STEPPER2_MOVE_RIGHT) {
		++stepper2_position;
	} else {
		--stepper2_position;
	}
	return d;
}


/*
 * Stepper3 prrogramming for moving the stepper by one step in the given
 * direction.
 */
void
just_step_stepper3(int dir)
{
	digitalWrite(STEPPER3_ENABLE_PIN, STEPPER_ENABLE);
	delay_microseconds(MINIMUM_PULSEWIDTH);
	digitalWrite(STEPPER3_DIR_PIN, dir);
	digitalWrite(STEPPER3_STEP_PIN, HIGH);
	delay_microseconds(MINIMUM_PULSEWIDTH);
	digitalWrite(STEPPER3_STEP_PIN, LOW);
	digitalWrite(STEPPER3_ENABLE_PIN, STEPPER_DISABLE);
}


/*
 * With slow speed move the stepper3 by one step
 * dflag can be used to delay till the step is complete.
 */
unsigned long
slow_step_stepper3(int dir, bool dflag)
{
	unsigned long d = 0;

  if (digitalRead(s3_reset_top)) {
	  if (dir == STEPPER3_MOVE_FORWARD) {
			return 0;
		}
	}
	if (digitalRead(s3_reset_bottom)) {
    stepper3_position = STEPPER3_RESET_POSITION;
   	if (dir == STEPPER3_MOVE_BACKWARD) {
		  return 0;
		}
	}

	just_step_stepper3(dir);

	if (dflag) {
		d = STEPPER3_SLOW_SPEED;
	} else {
		delay_microseconds(STEPPER3_SLOW_SPEED);
	}

	if (dir == STEPPER3_MOVE_FORWARD) {
		++stepper3_position;
	} else {
		--stepper3_position;
	}
	return d;
}


/*
 * With medium speed move the stepper3 by one step
 * dflag can be used to delay till the step is complete.
 */
unsigned long
medium_step_stepper3(int dir, bool dflag)
{
	unsigned long d = 0;

  if (digitalRead(s3_reset_top)) {
	  if (dir == STEPPER3_MOVE_FORWARD) {
      return 0;
    }
  }
  if (digitalRead(s3_reset_bottom)) {
    stepper3_position = STEPPER3_RESET_POSITION;
    if (dir == STEPPER3_MOVE_BACKWARD) {
      return 0;
    }
  }

	just_step_stepper3(dir);

	if (dflag) {
		d = STEPPER3_MEDIUM_SPEED;
	} else {
		delay_microseconds(STEPPER3_MEDIUM_SPEED);
	}

	if (dir == STEPPER3_MOVE_FORWARD) {
		++stepper3_position;
	} else {
		--stepper3_position;
	}
	return d;
}


/*
 * With high speed move the stepper3 by one step
 * dflag can be used to delay till the step is complete.
 */
unsigned long
fast_step_stepper3(int dir, bool dflag)
{
	unsigned long d = 0;

  if (digitalRead(s3_reset_top)) {
	  if (dir == STEPPER3_MOVE_FORWARD) {
      return 0;
    }
  }
  if (digitalRead(s3_reset_bottom)) {
    stepper3_position = STEPPER3_RESET_POSITION;
    if (dir == STEPPER3_MOVE_BACKWARD) {
      return 0;
    }
  }

	just_step_stepper3(dir);

	if (dflag) {
		d = STEPPER3_HIGH_SPEED;
	} else {
		delay_microseconds(STEPPER3_HIGH_SPEED);
	}

	if (dir == STEPPER3_MOVE_FORWARD) {
		++stepper3_position;
	} else {
		--stepper3_position;
	}
	return d;
}


/*
 * Moves all 3 steppers slowly by a step. If dir is -1 it will not move that stepper.
 * Returns the microseconds delay needed to complete the operation
 */
unsigned long
slow_step_all(int dir1, int dir2, int dir3)
{
	unsigned long d = 0;
	unsigned long dtemp;

	if (dir1 != -1) {
		d = slow_step_stepper1(dir1, true);
	}

	if (dir2 != -1) {
		dtemp = slow_step_stepper2(dir2, true);
		if (dtemp > d) {
			d = dtemp;
		}
	}

	if (dir3 != -1) {
		dtemp = slow_step_stepper3(dir3, true);
		if (dtemp > d) {
			d = dtemp;
		}
	}

	return d;
}


/*
 * Moves all 3 steppers with medium speed by a step. If dir is -1 it will not move that stepper.
 * Returns the microseconds delay needed to complete the operation
 */
unsigned long
medium_step_all(int dir1, int dir2, int dir3)
{
	unsigned long d = 0;
	unsigned long dtemp;

	if (dir1 != -1) {
		d = medium_step_stepper1(dir1, true);
	}

	if (dir2 != -1) {
		dtemp = medium_step_stepper2(dir2, true);
		if (dtemp > d) {
			d = dtemp;
		}
	}

	if (dir3 != -1) {
		dtemp = medium_step_stepper3(dir3, true);
		if (dtemp > d) {
			d = dtemp;
		}
	}

	return d;
}


/*
 * Moves all 3 steppers with high speed by a step. If dir is -1 it will not move that stepper.
 * Returns the microseconds delay needed to complete the operation
 */
unsigned long
fast_step_all(int dir1, int dir2, int dir3)
{
  if (dir1 != -1) {
    fast_step_stepper1(dir1, false);
  }

  if (dir2 != -1) {
    fast_step_stepper2(dir2, false);
  }

  if (dir3 != -1) {
    fast_step_stepper3(dir3, false);
  }

  return 0;
}


/*
 * This function moves all the 3 steppers to reset position
 */
void
slipping_check_steppers()
{
	while (stepper1_position < (STEPPER1_RESET_POSITION - 1)) {
		slow_step_stepper1(STEPPER1_MOVE_BACKWARD, false);
	}
	while (stepper2_position < (STEPPER2_RESET_POSITION - 1)) {
		slow_step_stepper2(STEPPER2_MOVE_LEFT, false);
	}
	while (stepper3_position < (STEPPER3_RESET_POSITION - 1)) {
		slow_step_stepper3(STEPPER3_MOVE_BACKWARD, false);
	}
}


/*
 * Test code to test steppers movement
 */
void
test_all_movement()
{
  int i;
  int nsteps1 = 3000;
  int nsteps2 = 2500;

  Serial.println("Moving Right");
  for (i = 0; i < (nsteps1/2); i++) {
      delay_microseconds(fast_step_all(-1, ARM_MOVE_RIGHT, -1));
  }
  delay(5000);

  Serial.println("Moving Up");
  for (i = 0; i < (nsteps2/4); i++) {
      delay_microseconds(fast_step_all(-1, -1, ARM_MOVE_UP));
  }
  delay(5000);
  
  while (1) {
    Serial.println("Moving Up");
    for (i = 0; i < nsteps2; i++) {
      delay_microseconds(fast_step_all(-1, -1, ARM_MOVE_UP));
    }
    delay(5000);

    Serial.println("Moving Down");
    for (i = 0; i < nsteps2; i++) {
      delay_microseconds(fast_step_all(-1, -1, ARM_MOVE_DOWN));
    }
    delay(5000);

    Serial.println("Moving Left");
    for (i = 0; i < nsteps1; i++) {
      delay_microseconds(fast_step_all(-1, ARM_MOVE_LEFT, -1));
    }
    delay(5000);

    Serial.println("Moving Right");
    for (i = 0; i < nsteps1; i++) {
      delay_microseconds(fast_step_all(-1, ARM_MOVE_RIGHT, -1));
    }
    delay(5000);
  }
}


/*
 * Test code to test reset switches
 */
void
test_reset_switches()
{
	while (1) {
		if (digitalRead(s1_reset_top)) {
			Serial.println("Switch 1 TOP ON");
		} else {
			Serial.println("Switch 1 TOP OFF");
		}
		if (digitalRead(s1_reset_bottom)) {
			Serial.println("Switch 1 BOTTOM ON");
		} else {
			Serial.println("Switch 1 BOTTOM OFF");
		}
		if (digitalRead(s2_reset)) {
			Serial.println("Switch 2 ON");
		} else {
			Serial.println("Switch 2 OFF");
		}
		if (digitalRead(s3_reset_top)) {
			Serial.println("Switch 3 TOP ON");
		} else {
			Serial.println("Switch 3 TOP OFF");
		}
		if (digitalRead(s3_reset_bottom)) {
			Serial.println("Switch 3 BOTTOM ON");
		} else {
			Serial.println("Switch 3 BOTTOM OFF");
		}
		delay(10);
	}
}


/*
 * Test code to test stepper1
 */
void
test_stepper1_movement()
{
	int dir = STEPPER1_MOVE_BACKWARD;

	while (1) {

		if (dir == STEPPER1_MOVE_BACKWARD) {
			if (digitalRead(s1_reset_top)) {
				dir = STEPPER1_MOVE_FORWARD;
				Serial.println("Steppers 1 Forward");
			}
		}

		if (dir == STEPPER1_MOVE_FORWARD) {
			if (digitalRead(s1_reset_bottom)) {
				dir = STEPPER1_MOVE_BACKWARD;
				Serial.println("Steppers 1 Backward");
			} 
		}
		slow_step_stepper1(dir, false);
	}
}


/*
 * Test code to test stepper3
 */
void
test_stepper3_movement()
{
	int dir = STEPPER3_MOVE_BACKWARD;

	while (1) {

		if (dir == STEPPER3_MOVE_BACKWARD) {
			if (digitalRead(s3_reset_bottom)) {
				dir = STEPPER3_MOVE_FORWARD;
				Serial.println("Steppers 3 Forward");
			}
		}

		if (dir == STEPPER3_MOVE_FORWARD) {
			if (digitalRead(s3_reset_top)) {
				dir = STEPPER3_MOVE_BACKWARD;
				Serial.println("Steppers 3 Backward");
			} 
		}
		if (digitalRead(s3_reset_top) && digitalRead(s3_reset_bottom)) {
			dir = STEPPER3_MOVE_FORWARD;
			Serial.println("Steppers 3 Forward 2");
		}

		slow_step_stepper3(dir, false);
	}
}


/*
 * Stepper1 reset code
 * Moves the stepper to reset position with respect to the reset switches
 */
void
reset_stepper1()
{
	int dir = STEPPER1_MOVE_BACKWARD;

  Serial.println("Resetting Stepper 1");

	while (1) {
		if (dir == STEPPER1_MOVE_BACKWARD) {
			if (digitalRead(s1_reset_bottom)) {
				dir = STEPPER1_MOVE_FORWARD;
				Serial.println("Steppers 1 Forward");
			}
		}

		if (dir == STEPPER1_MOVE_FORWARD) {
			if (digitalRead(s1_reset_top)) {
				Serial.println("Steppers 1 Setup Done.");
				break;
			} 
		}
		fast_step_stepper1(dir, false);

	}

  for (int i = 0; i < STEPPER1_STABLE_POSITION; i++) {
    slow_step_stepper1(STEPPER1_MOVE_BACKWARD, false);
  }
	Serial.println("Resetting Stepper 1 Complete");
}


/*
 * Stepper3 reset code
 * Moves the stepper to reset position with respect to the reset switches
 */
void
reset_stepper3()
{
	int dir = STEPPER3_MOVE_BACKWARD;

 Serial.println("Resetting Stepper 3");

	while (1) {
		if (dir == STEPPER3_MOVE_BACKWARD) {
			if (digitalRead(s3_reset_bottom)) {
				dir = STEPPER3_MOVE_FORWARD;
				Serial.println("Steppers 3 Forward");
			}
		}

		if (dir == STEPPER3_MOVE_FORWARD) {
			if (digitalRead(s3_reset_top)) {
				Serial.println("Steppers 1 Setup Done.");
				break;
			} 
		}
		slow_step_stepper3(dir, false);

	}

	Serial.println("Resetting Stepper 3 Complete");
}


/*
 * Stepper2 reset code
 * Moves the stepper to reset position with respect to the reset switches
 */
void
reset_stepper2()
{
	int dir = STEPPER2_MOVE_LEFT;
	int	scnt = 0;

 Serial.println("Resetting Stepper 2");

	while (1) {
		if (dir == STEPPER2_MOVE_LEFT) {
			if (digitalRead(s2_reset)) {
				dir = STEPPER2_MOVE_RIGHT;
				Serial.println("Steppers 2 Right");
			}
		}

		medium_step_stepper2(dir, false);

		if (dir == STEPPER2_MOVE_RIGHT) {
			scnt++;
			if (scnt >= STEPPER2_MID_POSITION) {
				break;
			}
		}

	}

	Serial.println("Resetting Stepper 2 Complete");
}
