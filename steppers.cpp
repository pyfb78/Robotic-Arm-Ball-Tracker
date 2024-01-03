/*
 * Title: Stepper Motor Code
 *
 *  Author: Pavan Yeddanapudi
 *
 *  Description: Decided to write my own stepper code rather than use the AccelStepper library.
 */

#include "steppers.hpp"

long	stepper1_position = -1;
long	stepper2_position = -1;
long	stepper3_position = -1;

int	s1_reset_top 		= STEPPER1_RESET_TOP_PIN;
int	s1_reset_bottom 	= STEPPER1_RESET_BOTTOM_PIN;
int	s2_reset 			= STEPPER2_RESET_PIN;
int	s3_reset_top 		= STEPPER3_RESET_TOP_PIN;
int	s3_reset_bottom 	= STEPPER3_RESET_BOTTOM_PIN;


long cpu_wasting_time = 0;


void
delay_microseconds(unsigned long ms)
{
	unsigned long t = micros();

	while (micros() < (t + ms)) {
		++cpu_wasting_time;
	}
}


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


long
get_stepper1_position()
{
	return stepper1_position;
}


long
get_stepper2_position()
{
	return stepper2_position;
}


long
get_stepper3_position()
{
	return stepper3_position;
}


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


bool
is_stepper2_position_safe(long delta)
{
	long pos = stepper2_position + delta;

	if ((pos >= STEPPER2_RESET_POSITION) && (pos <= STEPPER2_MAX_POSITION)) {
		return true;
	}
	return false;
}


bool
is_absolute_stepper2_position_safe(long pos)
{
	if ((pos >= STEPPER2_RESET_POSITION) && (pos <= STEPPER2_MAX_POSITION)) {
		return true;
	}
	return false;
}


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


void
slow_limit_check_stepper3_for_stepper1()
{
	if (digitalRead(s3_reset_top)) {
		for (int i = 0; i < STEPPER3_STEPPER1_SAFETY_GAP_STEPS; i++) {
			slow_step_stepper3(STEPPER3_MOVE_BACKWARD, false);
		}
	}
}


void
medium_limit_check_stepper3_for_stepper1()
{
	if (digitalRead(s3_reset_top)) {
		for (int i = 0; i < STEPPER3_STEPPER1_SAFETY_GAP_STEPS; i++) {
			medium_step_stepper3(STEPPER3_MOVE_BACKWARD, false);
		}
	}
}


void
fast_limit_check_stepper3_for_stepper1()
{
	if (digitalRead(s3_reset_top)) {
		for (int i = 0; i < STEPPER3_STEPPER1_SAFETY_GAP_STEPS; i++) {
			fast_step_stepper3(STEPPER3_MOVE_BACKWARD, false);
		}
	}
}


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
 * Returns the microseconds delay needed to complete the operation
 */
unsigned long
fast_step_all_old(int dir1, int dir2, int dir3)
{
	unsigned long d = 0;
	unsigned long dtemp;

	if (dir1 != -1) {
		d = fast_step_stepper1(dir1, true);
	}

	if (dir2 != -1) {
		dtemp = fast_step_stepper2(dir2, true);
		if (dtemp > d) {
			d = dtemp;
		}
	}

	if (dir3 != -1) {
		dtemp = fast_step_stepper3(dir3, true);
		if (dtemp > d) {
			d = dtemp;
		}
	}

	return d;
}


/*
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

