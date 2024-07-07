#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <stdio.h>

// Define pins for ultrasonic sensors
#define TRIG_PIN_LEFT PB0
#define ECHO_PIN_LEFT PB1
#define TRIG_PIN_MIDDLE PB2
#define ECHO_PIN_MIDDLE PB3
#define TRIG_PIN_RIGHT PB4
#define ECHO_PIN_RIGHT PB5

// Define motor control pins
#define MOTOR_LEFT_PWM PD6
#define MOTOR_LEFT_EN PD7
#define MOTOR_RIGHT_PWM PD5
#define MOTOR_RIGHT_EN PD4

// Define encoder pins
#define ENCODER_LEFT_A PC0
#define ENCODER_LEFT_B PC1
#define ENCODER_RIGHT_A PC2
#define ENCODER_RIGHT_B PC3

// Define I2C addresses for current sensors
#define CURRENT_SENSOR_1_ADDR 0x40  // Replace with the actual I2C address of current sensor 1
#define CURRENT_SENSOR_2_ADDR 0x41  // Replace with the actual I2C address of current sensor 2

// Define constants
#define STOP_DISTANCE 15      // Distance in cm to stop the robot
#define DETOUR_DISTANCE 20    // Distance in cm to start detour
#define DESIRED_SPEED 100     // Desired speed (adjust as needed)

// PID controller constants
#define KP 1.0
#define KI 0.1
#define KD 0.01

// Global variables
volatile int32_t encoder_count_left = 0;
volatile int32_t encoder_count_right = 0;
volatile float heading = 0;

// PID controller state
int16_t integral_left = 0;
int16_t integral_right = 0;
int16_t prev_error_left = 0;
int16_t prev_error_right = 0;

// Function prototypes
void init_ultrasonic();
void init_timer();
void init_motor_controller();
void init_encoders();
void init_pid();
void init_i2c();
void i2c_start();
void i2c_stop();
void i2c_write(uint8_t data);
uint8_t i2c_read_ack();
uint8_t i2c_read_nack();
uint16_t read_distance(uint8_t trig_pin, uint8_t echo_pin);
void stop_robot();
void move_forward();
void turn_left();
void turn_right();
void detour();
void set_motor_speed(uint8_t motor_pwm, uint8_t motor_en, uint8_t speed);
void accelerate(uint8_t motor_pwm, uint8_t motor_en, uint8_t target_speed, uint16_t delay_ms);
void decelerate(uint8_t motor_pwm, uint8_t motor_en, uint8_t target_speed, uint16_t delay_ms);
int16_t read_encoder(uint8_t encoder_a, uint8_t encoder_b);
int16_t compute_pid(int16_t setpoint, int16_t measured_value, int16_t *integral, int16_t *prev_error, uint16_t current_feedback);
void update_heading();
uint16_t read_current_sensor(uint8_t sensor_addr);

// Interrupt Service Routines
ISR(PCINT1_vect);

int main(void) {
	// Initialize peripherals
	init_ultrasonic();
	init_timer();
	init_motor_controller();
	init_encoders();
	init_i2c();
	init_pid();
	
	sei();  // Enable global interrupts
	
	while (1) {
		// Read distances from the ultrasonic sensors
		uint16_t distance_left = read_distance(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
		uint16_t distance_middle = read_distance(TRIG_PIN_MIDDLE, ECHO_PIN_MIDDLE);
		uint16_t distance_right = read_distance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);

		// Check if any distance is below the stop threshold
		if (distance_middle < STOP_DISTANCE) {
			detour();
			} else {
			move_forward();
		}

		// Update heading
		update_heading();

		// PID control for motor speeds
		int16_t speed_left = read_encoder(ENCODER_LEFT_A, ENCODER_LEFT_B);
		int16_t speed_right = read_encoder(ENCODER_RIGHT_A, ENCODER_RIGHT_B);

		// Read current sensor feedback
		uint16_t current_left = read_current_sensor(CURRENT_SENSOR_1_ADDR);
		uint16_t current_right = read_current_sensor(CURRENT_SENSOR_2_ADDR);

		int16_t pwm_left = compute_pid(DESIRED_SPEED, speed_left, &integral_left, &prev_error_left, current_left);
		int16_t pwm_right = compute_pid(DESIRED_SPEED, speed_right, &integral_right, &prev_error_right, current_right);

		set_motor_speed(MOTOR_LEFT_PWM, MOTOR_LEFT_EN, pwm_left);
		set_motor_speed(MOTOR_RIGHT_PWM, MOTOR_RIGHT_EN, pwm_right);
		
		_delay_ms(10);  // Adjust delay as needed for PID control loop frequency
	}
	
	return 0;
}

// Initialize ultrasonic sensor pins
void init_ultrasonic() {
	// Set trigger pins as output
	DDRB |= (1 << TRIG_PIN_LEFT) | (1 << TRIG_PIN_MIDDLE) | (1 << TRIG_PIN_RIGHT);
	// Set echo pins as input
	DDRB &= ~((1 << ECHO_PIN_LEFT) | (1 << ECHO_PIN_MIDDLE) | (1 << ECHO_PIN_RIGHT));
}

// Initialize timer for measuring pulse width
void init_timer() {
	TCCR1B |= (1 << CS11);  // Set prescaler to 8
}

// Initialize motor controller pins
void init_motor_controller() {
	// Set motor control pins as output
	DDRD |= (1 << MOTOR_LEFT_PWM) | (1 << MOTOR_LEFT_EN) | (1 << MOTOR_RIGHT_PWM) | (1 << MOTOR_RIGHT_EN);
	
	// Set PWM mode for the motor control pins
	TCCR0A |= (1 << WGM00) | (1 << WGM01);  // Fast PWM mode
	TCCR0A |= (1 << COM0A1) | (1 << COM0B1);  // Clear OC0A/OC0B on Compare Match, set OC0A/OC0B at BOTTOM
	TCCR0B |= (1 << CS00);  // No prescaling
}

// Initialize encoder pins and interrupts
void init_encoders() {
	// Set encoder pins as input
	DDRC &= ~((1 << ENCODER_LEFT_A) | (1 << ENCODER_LEFT_B) | (1 << ENCODER_RIGHT_A) | (1 << ENCODER_RIGHT_B));
	
	// Enable pin change interrupts
	PCICR |= (1 << PCIE1);
	PCMSK1 |= (1 << PCINT8) | (1 << PCINT9) | (1 << PCINT10) | (1 << PCINT11);
}

// Initialize PID controller variables
void init_pid() {
	integral_left = 0;
	integral_right = 0;
	prev_error_left = 0;
	prev_error_right = 0;
}

// Initialize I2C communication
void init_i2c() {
	TWSR = 0x00;
	TWBR = ((F_CPU / 100000UL) - 16) / 2;  // Set SCL to 100kHz
	TWCR = (1 << TWEN);  // Enable TWI
}

// Start I2C communication
void i2c_start() {
	TWCR = (1 << TWSTA) | (1 << TWEN) | (1 << TWINT);
	while (!(TWCR & (1 << TWINT)));
}

// Stop I2C communication
void i2c_stop() {
	TWCR = (1 << TWSTO) | (1 << TWEN) | (1 << TWINT);
	while (TWCR & (1 << TWSTO));
}

// Write data to I2C
void i2c_write(uint8_t data) {
	TWDR = data;
	TWCR = (1 << TWEN) | (1 << TWINT);
	while (!(TWCR & (1 << TWINT)));
}

// Read data from I2C with ACK
uint8_t i2c_read_ack() {
	TWCR = (1 << TWEN) | (1 << TWINT) | (1 << TWEA);
	while (!(TWCR & (1 << TWINT)));
	return TWDR;
}

// Read data from I2C without ACK
uint8_t i2c_read_nack() {
	TWCR = (1 << TWEN) | (1 << TWINT);
	while (!(TWCR & (1 << TWINT)));
	return TWDR;
}

// Read distance from ultrasonic sensor
uint16_t read_distance(uint8_t trig_pin, uint8_t echo_pin) {
	// Send trigger pulse
	PORTB |= (1 << trig_pin);
	_delay_us(10);
	PORTB &= ~(1 << trig_pin);
	
	// Wait for echo pulse
	while (!(PINB & (1 << echo_pin)));
	TCNT1 = 0;  // Clear timer counter
	while (PINB & (1 << echo_pin));
	
	// Calculate distance in cm
	uint16_t pulse_width = TCNT1;
	uint16_t distance = pulse_width / 58;
	
	return distance;
}

// Stop the robot
void stop_robot() {
	decelerate(MOTOR_LEFT_PWM, MOTOR_LEFT_EN, 0, 10);
	decelerate(MOTOR_RIGHT_PWM, MOTOR_RIGHT_EN, 0, 10);
}

// Move the robot forward
void move_forward() {
	accelerate(MOTOR_LEFT_PWM, MOTOR_LEFT_EN, DESIRED_SPEED, 10);
	accelerate(MOTOR_RIGHT_PWM, MOTOR_RIGHT_EN, DESIRED_SPEED, 10);
}

// Turn the robot left
void turn_left() {
	accelerate(MOTOR_LEFT_PWM, MOTOR_LEFT_EN, 64, 10);  // Adjust as needed
	accelerate(MOTOR_RIGHT_PWM, MOTOR_RIGHT_EN, 128, 10);  // Adjust as needed
}

// Turn the robot right
void turn_right() {
	accelerate(MOTOR_LEFT_PWM, MOTOR_LEFT_EN, 128, 10);  // Adjust as needed
	accelerate(MOTOR_RIGHT_PWM, MOTOR_RIGHT_EN, 64, 10);  // Adjust as needed
}

// Perform a detour maneuver
void detour() {
	turn_right();
	_delay_ms(1000);  // Adjust this delay as needed
	
	while (1) {
		uint16_t distance_left = read_distance(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
		uint16_t distance_right = read_distance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);
		
		if (distance_left < DETOUR_DISTANCE) {
			turn_right();
			} else if (distance_right < DETOUR_DISTANCE) {
			turn_left();
			} else {
			move_forward();
		}
		
		uint16_t distance_middle = read_distance(TRIG_PIN_MIDDLE, ECHO_PIN_MIDDLE);
		if (distance_middle > STOP_DISTANCE) {
			break;
		}
	}
	
	turn_left();
	_delay_ms(1000);  // Adjust this delay as needed
	move_forward();
}

// Set motor speed with PWM
void set_motor_speed(uint8_t motor_pwm, uint8_t motor_en, uint8_t speed) {
	if (motor_pwm == MOTOR_LEFT_PWM) {
		OCR0A = speed;  // Set PWM duty cycle for left motor
		} else if (motor_pwm == MOTOR_RIGHT_PWM) {
		OCR0B = speed;  // Set PWM duty cycle for right motor
	}
	PORTD |= (1 << motor_en);  // Enable the motor
}

// Accelerate motor to target speed
void accelerate(uint8_t motor_pwm, uint8_t motor_en, uint8_t target_speed, uint16_t delay_ms) {
	uint8_t current_speed = 0;
	while (current_speed < target_speed) {
		set_motor_speed(motor_pwm, motor_en, current_speed);
		current_speed++;
		_delay_ms(delay_ms);
	}
	set_motor_speed(motor_pwm, motor_en, target_speed);
}

// Decelerate motor to target speed
void decelerate(uint8_t motor_pwm, uint8_t motor_en, uint8_t target_speed, uint16_t delay_ms) {
	uint8_t current_speed = (motor_pwm == MOTOR_LEFT_PWM) ? OCR0A : OCR0B;
	while (current_speed > target_speed) {
		set_motor_speed(motor_pwm, motor_en, current_speed);
		current_speed--;
		_delay_ms(delay_ms);
	}
	set_motor_speed(motor_pwm, motor_en, target_speed);
}

// Read encoder value
int16_t read_encoder(uint8_t encoder_a, uint8_t encoder_b) {
	static uint8_t last_state_a = 0;
	static uint8_t last_state_b = 0;
	uint8_t state_a = PINC & (1 << encoder_a);
	uint8_t state_b = PINC & (1 << encoder_b);

	int16_t count = 0;
	if (state_a != last_state_a || state_b != last_state_b) {
		if (state_a != state_b) {
			count++;
			} else {
			count--;
		}
	}
	last_state_a = state_a;
	last_state_b = state_b;

	return count;
}

// Compute PID control output
int16_t compute_pid(int16_t setpoint, int16_t measured_value, int16_t *integral, int16_t *prev_error, uint16_t current_feedback) {
	int16_t error = setpoint - measured_value;
	*integral += error;
	int16_t derivative = error - *prev_error;
	*prev_error = error;

	int16_t output = (KP * error) + (KI * (*integral)) + (KD * derivative);
	return output;
}

// Update robot heading based on encoders
void update_heading() {
	int16_t delta_left = read_encoder(ENCODER_LEFT_A, ENCODER_LEFT_B);
	int16_t delta_right = read_encoder(ENCODER_RIGHT_A, ENCODER_RIGHT_B);

	float distance_per_tick = 0.1;  // Distance per encoder tick, adjust as needed
	float wheel_base = 10.0;  // Distance between wheels, adjust as needed

	float distance_left = delta_left * distance_per_tick;
	float distance_right = delta_right * distance_per_tick;

	heading += (distance_right - distance_left) / wheel_base;
}

// Read current sensor value
uint16_t read_current_sensor(uint8_t sensor_addr) {
	i2c_start();
	i2c_write((sensor_addr << 1) | 0);
	i2c_write(0x00);  // Assuming 0x00 is the register to read current
	i2c_start();
	i2c_write((sensor_addr << 1) | 1);
	uint16_t current = (i2c_read_ack() << 8) | i2c_read_nack();
	i2c_stop();
	return current;
}

// ISR for pin change interrupt (encoder reading)
ISR(PCINT1_vect) {
	// Read encoders
	if (PINC & (1 << ENCODER_LEFT_A)) {
		if (PINC & (1 << ENCODER_LEFT_B)) {
			encoder_count_left++;
			} else {
			encoder_count_left--;
		}
		} else {
		if (PINC & (1 << ENCODER_LEFT_B)) {
			encoder_count_left--;
			} else {
			encoder_count_left++;
		}
	}

	if (PINC & (1 << ENCODER_RIGHT_A)) {
		if (PINC & (1 << ENCODER_RIGHT_B)) {
			encoder_count_right++;
			} else {
			encoder_count_right--;
		}
		} else {
		if (PINC & (1 << ENCODER_RIGHT_B)) {
			encoder_count_right--;
			} else {
			encoder_count_right++;
		}
	}
}
