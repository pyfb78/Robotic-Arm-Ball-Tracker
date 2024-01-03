/*
 * Title: uStepper ARM
 *
 * Author : Pavan Yeddanapudi
 *
 * Hardware:
 * The hardware used to test this code includes
 * 3 X Nema 17 Stepper Motor 2A 59Ncm from STEPPERONLINE
 * 3 X DM542T Nema 17 Stepper Motor Driver
 * Teensy 4.1 Microcontroller
 * Robotic arm mechanicals
 * Wires, breadboard and power supply
 * PWMServo connected but not tested here.
 *
 * Description:
 * This code gets the serial packet and performs the uStepper ARM movement.
 */
#include <Arduino.h>
#include <PWMServo.h>
#include "steppers.hpp"
/*
 * This is serial port communication baudrate
 */
#define SERIAL_BAUDRATE 115200
#define SERIAL_PACKET_SIZE 28

/*
 * Serial packet flag bits
 */
#define ARM_RESET_POSITION        0x80  // Reset to starting position
#define ARM_MAX_HEIGHT_POSITION   0x40  // Move steper 1 full height
#define ARM_FULL_FORWARD_POSITION 0x20  // Move stepper 1 full forward
#define ARM_OPEN_PWMSERVO         0x10  // Open the PWMServo completely
#define ARM_CLOSE_PWMSERVO        0x08  // Close the PWMServo
#define ARM_ONLY_FLAG_VALID       0x04  // Use only flag value of Serial Packet
#define ARM_NOCHECK_MOVE          0x02  // Move without hardware checks
#define ARM_MOVE_ABSOLUTE         0x01  // Move absolute (default relative)


/*
 * Teensy 4.1 board pin used for PWMServo.
 */
#define servoPin 1


unsigned long   serial_packet_number;


/*
 * PWMServo initialization.
 */
PWMServo handControl = PWMServo();

/*
 * This function initalizes the default serial port on Teensy 4.1
 */
void
sd_init(void)
{
  Serial.begin(SERIAL_BAUDRATE);
}



bool
move_arm_absolute_position(long pos1, long pos2, long pos3, long x1, long y1, long z1)
{
  /*
   * Here we have to do co-ordinate transformation
   */
   long x2, y2, z2;
   long delta1, delta2, delta3;
   int  dir1, dir2, dir3;
   long p, q, r;
   //unsigned long dly;

   p = get_stepper1_position();
   q = get_stepper2_position();
   r = get_stepper3_position();

   x2 = x1 + (p - pos1);
   y2 = y1 + (q - pos2);
   z2 = z1 + (r - pos3);
   
   delta1 = x2 - p;
   delta2 = y2 - q;
   delta3 = z2 - r; 
   
    dir1 = get_stepper1_move_direction(delta1);
    dir2 = get_stepper2_move_direction(delta2);
    dir3 = get_stepper3_move_direction(-delta3);

    while ((y2 != q) || (z2 != r) || (x2 != p)) {
      fast_step_all(dir1, dir2, dir3);

      if (x2 > p) {
        x2 -= 1;
      } else if (x2 < p) {
        x2 += 1;
      }
      
      if (y2 > q) {
        y2 -= 1;
      } else if (y2 < q) {
        y2 += 1;
      }

      if (z2 > r) {
        z2 -= 1;
      } else if (z2 < r) {
        z2 += 1;
      }
      //delay_microseconds(dly);
    }
    return true;
}


/*
 * This function reads 4 serial bytes and returns a 4 byte integer.
 */
long
serial_read_number_as4bytes()
{
  unsigned char byte0, byte1, byte2, byte3;
  long  n;

  byte0 = Serial.read();
  byte1 = Serial.read();
  byte2 = Serial.read();
  byte3 = Serial.read();

  //Serial.printf("Byte0=0x%x, Byte1=0x%x, Byte2=0x%x, Byte3=0x%x\n", byte0, byte1, byte2, byte3);

  n = (byte3 << 24) | (byte2 << 16) | (byte1 << 8) | byte0;
  return n;
}


/*
 * This function reads 2 serial bytes and returns a 4 byte integer.
 */
long
serial_read_number_as2bytes()
{
  unsigned char byte0, byte1;
  long  n;

  byte0 = Serial.read();
  byte1 = Serial.read();

  n = (byte1 << 8) | byte0;
  return n;
}


/*
 * This fuction reads a serial packet with 3 numbers
 */
void
serial_read_packet(short *flag, long *n0, long *n1, long *n2, long *n3, long *n4, long *n5)
{
  while(Serial.read() != '#') {
    delay_microseconds(100);
  }
  *flag = serial_read_number_as2bytes();
  *n0 = serial_read_number_as4bytes();
  *n1 = serial_read_number_as4bytes();
  *n2 = serial_read_number_as4bytes();
  *n3 = serial_read_number_as4bytes();
  *n4 = serial_read_number_as4bytes();
  *n5 = serial_read_number_as4bytes();
}


void
pwmservo_open_jaws()
{
}


void
pwmservo_close_jaws()
{
}


int   packet_num = 0;


void
print_received_packet(short flag, long s1, long s2, long s3)
{
  packet_num++;
  Serial.print("<< New Packet ");
  Serial.print(packet_num, DEC);
  Serial.print(" [ ");
  Serial.print("flag=0x");
  Serial.print(flag, HEX);
  Serial.print(", right=");
  Serial.print(s1, DEC);
  Serial.print(", base=");
  Serial.print(s2, DEC);
  Serial.print(", left=");
  Serial.print(s3, DEC);
  Serial.println("] >>");
}


/*
 * This function takes the latest serial packet and performs the relative move on
 * the 3 steppers.
 */
void
serial_packet_steppers_processing()
{
  int cnt;
  long s1 = 0, s2 = 0, s3 = 0, s4 = 0, s5 = 0, s6 = 0;
  short flag;

  cnt = Serial.available();
  if (cnt == 0) {
    return;
  }
  while (cnt < SERIAL_PACKET_SIZE) {
    delay(1);
    cnt = Serial.available();
  }
  serial_read_packet(&flag, &s1, &s2, &s3, &s4, &s5, &s6);

  ++serial_packet_number;
  //Serial.print("Pkt = ");
  //Serial.println(serial_packet_number, DEC);

  if (flag & ARM_RESET_POSITION) {
    reset_stepper1();
    reset_stepper3();
    reset_stepper2();
    Serial.println("Reset ARM");
    print_received_packet(flag, s1, s2, s3);
    return;
  }


  if (flag & ARM_OPEN_PWMSERVO) {
    pwmservo_open_jaws();
    Serial.println("PWMServo Open");
    print_received_packet(flag, s1, s2, s3);
    return;
  }

  if (flag & ARM_CLOSE_PWMSERVO) {
    pwmservo_close_jaws();
    Serial.println("PWMServo Close");
    print_received_packet(flag, s1, s2, s3);
    return;
  }

  if (flag & ARM_ONLY_FLAG_VALID) {
    Serial.println("Only Flag Valid");
    return;
  }

  if (flag & ARM_MOVE_ABSOLUTE) {
    if (!move_arm_absolute_position(s1, s2, s3, s4, s5, s6)) {
      Serial.println("WARNING: ARM absolute move failed!");
    }
  }
}


/*
 * This setup() function is called by the Aurdino platform code
 * during initialization.
 */
void setup() {

  /*
   * Initialize the serial port
   */
  sd_init();

  setup_steppers();

  /*
   * Reset steppers
   */
  reset_stepper1();
  reset_stepper3();
  reset_stepper2();

  delay(1000); // Just add some delay can be decreased.

  /*
   * Attach PWMServo. Note it is not tested in this code.
   */
  handControl.attach(servoPin);

  serial_packet_number = 0;

  Serial.println("ARM Setup Done");

  Serial.flush();
}


/*
 * This loop() function is called by the Aurdino platform code
 * during runtime in the mainloop.
 */
void loop() {
  
  /*
   * Now just call serial packet processing
   */
  serial_packet_steppers_processing();
  //Serial.println("$$");
  Serial.flush();
}

