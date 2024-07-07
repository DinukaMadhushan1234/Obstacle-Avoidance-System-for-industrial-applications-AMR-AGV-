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
#define CURRENT_SENSOR_1_ADDR 0x40  // I2C address of current sensor 1
#define CURRENT_SENSOR_2_ADDR 0x41  // I2C address of current sensor 2

// Define constants
#define STOP_DISTANCE 15      // Distance in cm to stop the robot
#define DETOUR_DISTANCE 20    // Distance in cm to start detour
#define DESIRED_SPEED 100     // Desired speed (adjust as needed)

#define CPR 360  // Counts per revolution provided in the encoder's datasheet
#define WHEEL_DIAMETER 0.1  // Diameter in meters
#define TIME_INTERVAL 0.1  // Time interval in seconds
#define WHEELBASE 0.5 // Wheelbase 

// Define acceleration and deceleration constants in m/s²
#define ACCELERATION 0.5 // Example value
#define DECELERATION 0.5 // Example value

// PID controller constants
#define KP 1.0
#define KI 0.1
#define KD 0.01

// Global variables
volatile int32_t encoder_count_left = 0;
volatile int32_t encoder_count_right = 0;

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
void turn_left_90();
void turn_right_90();
void detour();
void set_motor_speed(uint8_t motor_pwm, uint8_t motor_en, uint8_t speed);
void accelerate(uint8_t motor_pwm, uint8_t motor_en, float target_speed, float initial_speed);
void decelerate(uint8_t motor_pwm, uint8_t motor_en, float target_speed);
int16_t read_encoder(uint8_t encoder_a, uint8_t encoder_b);
float update_heading();
int16_t compute_pid(int16_t setpoint, int16_t measured_value, int16_t *integral, int16_t *prev_error, uint16_t current_feedback);
uint16_t read_current_sensor(uint8_t sensor_addr);
float calculate_speed(int32_t encoder_counts, float time_interval);
float map_speed_to_pwm(float speed);



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
			stop_robot();
			detour();
			} else {
			move_forward();
		}

		// Update heading
		update_heading();

		// PID control for motor speeds
		int16_t left_encoder_counts = read_encoder(ENCODER_LEFT_A, ENCODER_LEFT_B);
		int16_t right_encoder_counts = read_encoder(ENCODER_RIGHT_A, ENCODER_RIGHT_B);

		// Convert encoder counts to speed in m/s
		float speed_left = calculate_speed(left_encoder_counts, TIME_INTERVAL);
		float speed_right = calculate_speed(right_encoder_counts, TIME_INTERVAL);

		// Read current sensor feedback
		uint16_t current_left = read_current_sensor(CURRENT_SENSOR_1_ADDR);
		uint16_t current_right = read_current_sensor(CURRENT_SENSOR_2_ADDR);

		// PID control for motor speeds
        int16_t pid_output_left = compute_pid(DESIRED_SPEED, speed_left, &integral_left, &prev_error_left, current_left);
        int16_t pid_output_right = compute_pid(DESIRED_SPEED, speed_right, &integral_right, &prev_error_right, current_right);

        // Map PID output to PWM duty cycle
        uint8_t pwm_left = map_speed_to_pwm(pid_output_left);
		uint8_t pwm_right = map_speed_to_pwm(pid_output_right);

		// Set motor speeds
		set_motor_speed(MOTOR_LEFT_PWM, MOTOR_LEFT_EN, pwm_left);
		set_motor_speed(MOTOR_RIGHT_PWM, MOTOR_RIGHT_EN, pwm_right);

		_delay_ms(100);  // Adjust delay as needed for PID control loop frequency
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
    // Decelerate both motors to 0 speed
    decelerate(MOTOR_LEFT_PWM, MOTOR_LEFT_EN, 0);
    decelerate(MOTOR_RIGHT_PWM, MOTOR_RIGHT_EN, 0);
}

// Move the robot forward
void move_forward() {
    // Accelerate both motors to the desired speed
    accelerate(MOTOR_LEFT_PWM, MOTOR_LEFT_EN, DESIRED_SPEED, 0);
    accelerate(MOTOR_RIGHT_PWM, MOTOR_RIGHT_EN, DESIRED_SPEED, 0);
}

// Turn the robot left
void turn_left_90() {
    int32_t initial_left_count = encoder_count_left;
    int32_t initial_right_count = encoder_count_right;
    
    // Calculate encoder counts for approximately 90 degrees turn
    int32_t target_count_left = initial_left_count + (int32_t)(WHEELBASE * 3.14159 / 4.0 / (3.14159 * WHEEL_DIAMETER) * CPR);
    int32_t target_count_right = initial_right_count - (int32_t)(WHEELBASE * 3.14159 / 4.0 / (3.14159 * WHEEL_DIAMETER) * CPR);
    
    // Turn left until the target counts are reached
    while (encoder_count_left < target_count_left && encoder_count_right > target_count_right) {
        set_motor_speed(MOTOR_LEFT_PWM, MOTOR_LEFT_EN, map_speed_to_pwm(DESIRED_SPEED * 0.25));  
        set_motor_speed(MOTOR_RIGHT_PWM, MOTOR_RIGHT_EN, map_speed_to_pwm(DESIRED_SPEED * 0.75));
    }
    
    stop_robot();
}


// Turn the robot right
void turn_right_90() {
    int32_t initial_left_count = encoder_count_left;
    int32_t initial_right_count = encoder_count_right;
    
    // Calculate encoder counts for approximately 90 degrees turn
    int32_t target_count_left = initial_left_count + (int32_t)(WHEELBASE * 3.14159 / 4.0 / (3.14159 * WHEEL_DIAMETER) * CPR);
    int32_t target_count_right = initial_right_count - (int32_t)(WHEELBASE * 3.14159 / 4.0 / (3.14159 * WHEEL_DIAMETER) * CPR);
    
    // Turn left until the target counts are reached
    while (encoder_count_left < target_count_left && encoder_count_right > target_count_right) {
        set_motor_speed(MOTOR_LEFT_PWM, MOTOR_LEFT_EN, map_speed_to_pwm(DESIRED_SPEED * 0.75));  
        set_motor_speed(MOTOR_RIGHT_PWM, MOTOR_RIGHT_EN, map_speed_to_pwm(DESIRED_SPEED * 0.25));
    }
    
    stop_robot();
}

// Perform a detour maneuver
void detour() {
    float total_distance = 0.0;
    float back_distance = 0.0;

    while (1) {
        uint16_t distance_left = read_distance(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
        uint16_t distance_right = read_distance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);

        if (distance_left <= distance_right) {
            if (distance_right > DETOUR_DISTANCE) {
                turn_right_90();
                while (distance_left < STOP_DISTANCE) {
                    distance_left = read_distance(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
                    move_forward();
                    float distance_moved = update_heading();
                    total_distance += distance_moved;
                }
                stop_robot();
                turn_left_90();
                while (distance_left < STOP_DISTANCE) {
                    distance_left = read_distance(TRIG_PIN_LEFT, ECHO_PIN_LEFT);
                    move_forward();
                }
                stop_robot();
                turn_left_90();
                while (back_distance <= total_distance) {
                    move_forward();
                    float distance_moved = update_heading();
                    back_distance += distance_moved;
                    _delay_ms(100);  // Adjust delay as needed
                }
                break;
            } else {
                stop_robot();
            }
        } else {
            if (distance_left > DETOUR_DISTANCE) {
                turn_left_90();
                while (distance_right < STOP_DISTANCE) {
                    distance_right = read_distance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);
                    move_forward();
                    float distance_moved = update_heading();
                    total_distance += distance_moved;
                }
                stop_robot();
                turn_right_90();
                while (distance_right < STOP_DISTANCE) {
                    distance_right = read_distance(TRIG_PIN_RIGHT, ECHO_PIN_RIGHT);
                    move_forward();
                }
                stop_robot();
                turn_right_90();
                while (back_distance <= total_distance) {
                    move_forward();
                    float distance_moved = update_heading();
                    back_distance += distance_moved;
                    _delay_ms(100);  // Adjust delay as needed
                }
                break;
            } else {
                stop_robot();
            }
        }
    }
}



//calculate_speed in meter/seconds
float calculate_speed(int32_t encoder_counts, float time_interval) {
	float wheel_circumference = 3.14159 * WHEEL_DIAMETER;
	float distance_per_count = wheel_circumference / CPR;
	float distance_traveled = encoder_counts * distance_per_count;
	float speed = distance_traveled / time_interval;
	return speed;
}

float map_speed_to_pwm(float speed) {
    if (speed > DESIRED_SPEED) speed = DESIRED_SPEED;
    if (speed < 0) speed = 0;
	uint8_t duty_cycle = (uint8_t)((speed / DESIRED_SPEED) * 255);
    return duty_cycle;
}
// Set motor speed with PWM
void set_motor_speed(uint8_t motor_pwm, uint8_t motor_en, uint8_t duty_cycle) {
	if (motor_pwm == MOTOR_LEFT_PWM) {
		OCR0A = duty_cycle;  // Set PWM duty cycle for left motor
		} else if (motor_pwm == MOTOR_RIGHT_PWM) {
		OCR0B = duty_cycle;  // Set PWM duty cycle for right motor
	}
	PORTD |= (1 << motor_en);  // Enable the motor
}

// Accelerate motor to target speed
void accelerate(uint8_t motor_pwm, uint8_t motor_en, float target_speed, float initial_speed) {
    float current_speed = initial_speed;
    float time_interval = 0.1; // Time interval in seconds 

    while (current_speed < target_speed) {
        set_motor_speed(motor_pwm, motor_en, map_speed_to_pwm(current_speed));
        current_speed += ACCELERATION * time_interval;  //(v= u + at)
        _delay_ms(time_interval * 1000); // Convert seconds to milliseconds
    }
    set_motor_speed(motor_pwm, motor_en, map_speed_to_pwm(target_speed));
}
// Decelerate motor to target speed
void decelerate(uint8_t motor_pwm, uint8_t motor_en, float target_speed) {
    float current_speed = (motor_pwm == MOTOR_LEFT_PWM) ? OCR0A / 255.0 * DESIRED_SPEED : OCR0B / 255.0 * DESIRED_SPEED;
    float time_interval = 0.1; // Time interval in seconds (e.g., 100 ms)

    while (current_speed > target_speed) {
        set_motor_speed(motor_pwm, motor_en, map_speed_to_pwm(current_speed));
        current_speed -= DECELERATION * time_interval;  //(v = u - at)
        _delay_ms(time_interval * 1000); // Convert seconds to milliseconds
    }
    set_motor_speed(motor_pwm, motor_en, map_speed_to_pwm(target_speed));
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



// Update robot heading based on encoders
float update_heading() {
	int16_t encoder_counts = (read_encoder(ENCODER_LEFT_A, ENCODER_LEFT_B) +  read_encoder(ENCODER_RIGHT_A, ENCODER_RIGHT_B))/2; //to get avg count
    float wheel_circumference = 3.14159 * WHEEL_DIAMETER; // Calculate wheel circumference
    float distance_per_count = wheel_circumference / CPR; // Distance per encoder count
    return encoder_counts * distance_per_count;
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