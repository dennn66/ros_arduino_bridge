/* Define single-letter commands that will be sent by the PC over the
   serial link.
*/

#ifndef COMMANDS_H
#define COMMANDS_H

#define ANALOG_READ      'a'
#define GET_BAUDRATE     'b'
#define PIN_MODE         'c'
#define DIGITAL_READ     'd'
#define READ_ENCODERS    'e'
#define ALL_SERVO_READ   'f'
#define MAGNETO_READ     'g'
#define SERVO_ATTACH     'h'
#define ALL_SERVO_ATTACH 'i'
#define ALL_SERVO_STATE  'j'
#define ALL_SERVO_WRITE  'k'
#define MOTOR_SPEEDS     'm'
#define SEND_PWM         'n'
#define PING             'p'
#define RESET_ENCODERS   'r'
#define SERVO_WRITE      's'
#define SERVO_READ       't'
#define UPDATE_PID       'u'
#define DIGITAL_WRITE    'w'
#define ANALOG_WRITE     'x'
#define LEFT              0
#define RIGHT             1

#endif


