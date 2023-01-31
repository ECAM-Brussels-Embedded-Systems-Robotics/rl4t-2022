/**
* @example rc_balance_team2_LS4
*
* Reference solution for balancing EduMiP
**/

#include <stdio.h>
#include <unistd.h> // for isatty()
#include <stdlib.h> // for strtof()
#include <math.h> // for M_PI
#include <robotcontrol.h>

#include "light_tracker6_defs.h"

/**
 * NOVICE: Drive rate and turn rate are limited to make driving easier.
 * ADVANCED: Faster drive and turn rate for more fun.
 */
typedef enum drive_mode_t{
	NOVICE,
	ADVANCED
}drive_mode_t;

/**
 * ARMED or DISARMED to indicate if the controller is running
 */
typedef enum arm_state_t{
	ARMED,
	DISARMED
}arm_state_t;

/**
 * Feedback controller setpoint written to by setpoint_manager and read by the
 * controller.
 */
typedef struct setpoint_t{
	arm_state_t arm_state;	///< see arm_state_t declaration
	drive_mode_t drive_mode;///< NOVICE or ADVANCED
	double theta;			///< body lean angle (rad)
	double phi;				///< wheel position (rad)
	double phi_dot;			///< rate at which phi reference updates (rad/s)
	double gamma;			///< body turn angle (rad)
	double gamma_dot;		///< rate at which gamma setpoint updates (rad/s)
	double light;		 	///< Angle at which we want to see the light come from (deg)
}setpoint_t;

/**
 * This is the system state written to by the balance controller.
 */
typedef struct core_state_t{
	double wheelAngleR;	///< wheel rotation relative to body
	double wheelAngleL;
	double theta;		///< body angle radians
	double phi;			///< average wheel angle in global frame
	double gamma;		///< body turn (yaw) angle radians
	double vBatt;		///< battery voltage
	double d1_u;		///< output of balance controller D1 to motors
	double d3_u;		///< output of steering controller D3 to motors
	double mot_drive;	///< u compensated for battery voltage
	double phi_dot;		///< wheel speed (rad/s)
	double theta_dot;	///< robot angle (rad)
	double phi_dot_mem; ///< Wheel angle
	double light; 		///< Angle at which the light comes from (deg)
} core_state_t;

// possible modes, user selected with command line arguments
typedef enum m_input_mode_t{
	NONE,
	DSM,
	STDIN
} m_input_mode_t;


static void __print_usage(void);
static void __balance_controller(void);		///< mpu interrupt routine
static void* __setpoint_manager(void* ptr);	///< background thread
static void* __battery_checker(void* ptr);	///< background thread
static void* __printf_loop(void* ptr);		///< background thread
static int __zero_out_controller(void);
static int __disarm_controller(void);
static int __arm_controller(void);
static int __wait_for_starting_condition(void);
static void __on_pause_press(void);
static void __on_mode_release(void);


// global variables
core_state_t cstate;
setpoint_t setpoint;
rc_filter_t D3 = RC_FILTER_INITIALIZER;
rc_mpu_data_t mpu_data;
m_input_mode_t m_input_mode = DSM;

/*
 * printed if some invalid argument was given
 */
static void __print_usage(void)
{
	printf("\n");
	printf("-i {dsm|stdin|none}     specify input\n");
	printf("-h                      print this help message\n");
	printf("\n");
}

/**
 * Initialize the filters, mpu, threads, & wait until shut down
 *
 * @return     0 on success, -1 on failure
 */
int main(int argc, char *argv[])
{
	int c;
	pthread_t setpoint_thread = 0;
	pthread_t battery_thread = 0;
	pthread_t printf_thread = 0;

	// parse arguments
	opterr = 0;
	while ((c = getopt(argc, argv, "i:")) != -1){
		switch (c){
		case 'i': // input option
			if(!strcmp("dsm", optarg)) {
				m_input_mode = DSM;
			} else if(!strcmp("stdin", optarg)) {
				m_input_mode = STDIN;
			} else if(!strcmp("none", optarg)){
				m_input_mode = NONE;
			} else {
				__print_usage();
				return -1;
			}
			break;
		case 'h':
			__print_usage();
			return -1;
			break;
		default:
			__print_usage();
			return -1;
			break;
		}
	}

	// make sure another instance isn't running
	// if return value is -3 then a background process is running with
	// higher privaledges and we couldn't kill it, in which case we should
	// not continue or there may be hardware conflicts. If it returned -4
	// then there was an invalid argument that needs to be fixed.
	if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
	if(rc_enable_signal_handler()==-1){
		fprintf(stderr,"ERROR: failed to start signal handler\n");
		return -1;
	}

	// initialize buttons
	if(rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH,
						RC_BTN_DEBOUNCE_DEFAULT_US)){
		fprintf(stderr,"ERROR: failed to initialize pause button\n");
		return -1;
	}
	if(rc_button_init(RC_BTN_PIN_MODE, RC_BTN_POLARITY_NORM_HIGH,
						RC_BTN_DEBOUNCE_DEFAULT_US)){
		fprintf(stderr,"ERROR: failed to initialize mode button\n");
		return -1;
	}

	// Assign functions to be called when button events occur
	rc_button_set_callbacks(RC_BTN_PIN_PAUSE,__on_pause_press,NULL);
	rc_button_set_callbacks(RC_BTN_PIN_MODE,NULL,__on_mode_release);

	// initialize enocders
	if(rc_encoder_eqep_init()==-1){
		fprintf(stderr,"ERROR: failed to initialize eqep encoders\n");
		return -1;
	}

	// initialize motors
	if(rc_motor_init()==-1){
		fprintf(stderr,"ERROR: failed to initialize motors\n");
		return -1;
	}
	rc_motor_standby(1); // start with motors in standby

	// start dsm listener
	if(m_input_mode == DSM){
		if(rc_dsm_init()==-1){
			fprintf(stderr,"failed to start initialize DSM\n");
			return -1;
		}
	}

	// initialize adc
	if(rc_adc_init()==-1){
		fprintf(stderr, "failed to initialize adc\n");
	}

	// make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();

	printf("\nPress and release MODE button to toggle DSM drive mode\n");
	printf("Press and release PAUSE button to pause/start the motors\n");
	printf("hold pause button down for 2 seconds to exit\n");

	if(rc_led_set(RC_LED_GREEN, 0)==-1){
		fprintf(stderr, "ERROR in rc_balance, failed to set RC_LED_GREEN\n");
		return -1;
	}
	if(rc_led_set(RC_LED_RED, 1)==-1){
		fprintf(stderr, "ERROR in rc_balance, failed to set RC_LED_RED\n");
		return -1;
	}

	// set up mpu configuration
	rc_mpu_config_t mpu_config = rc_mpu_default_config();
	mpu_config.dmp_sample_rate = SAMPLE_RATE_HZ;
	mpu_config.orient = ORIENTATION_Y_UP;

	// if gyro isn't calibrated, run the calibration routine
	if(!rc_mpu_is_gyro_calibrated()){
		printf("Gyro not calibrated, automatically starting calibration routine\n");
		printf("Let your MiP sit still on a firm surface\n");
		rc_mpu_calibrate_gyro_routine(mpu_config);
	}

	// make sure setpoint starts at normal values
	setpoint.arm_state = DISARMED;
	setpoint.drive_mode = NOVICE;

	// set up D3 gamma (steering) controller
	if(rc_filter_pid(&D3, D3_KP, D3_KI, D3_KD, 4*DT, DT)){
		fprintf(stderr,"ERROR in rc_balance, failed to make steering controller\n");
		return -1;
	}
	rc_filter_enable_saturation(&D3, -STEERING_INPUT_MAX, STEERING_INPUT_MAX);

	// start a thread to slowly sample battery
	if(rc_pthread_create(&battery_thread, __battery_checker, (void*) NULL, SCHED_OTHER, 0)){
		fprintf(stderr, "failed to start battery thread\n");
		return -1;
	}
	// wait for the battery thread to make the first read
	while(cstate.vBatt<1.0 && rc_get_state()!=EXITING) rc_usleep(10000);

	// start printf_thread if running from a terminal
	// if it was started as a background process then don't bother
	if(isatty(fileno(stdout))){
		if(rc_pthread_create(&printf_thread, __printf_loop, (void*) NULL, SCHED_OTHER, 0)){
			fprintf(stderr, "failed to start battery thread\n");
			return -1;
		}
	}

	// start mpu
	if(rc_mpu_initialize_dmp(&mpu_data, mpu_config)){
		fprintf(stderr,"ERROR: can't talk to IMU, all hope is lost\n");
		rc_led_blink(RC_LED_RED, 5, 5);
		return -1;
	}

	// start balance stack to control setpoints
	if(rc_pthread_create(&setpoint_thread, __setpoint_manager, (void*) NULL, SCHED_OTHER, 0)){
		fprintf(stderr, "failed to start battery thread\n");
		return -1;
	}

	// this should be the last step in initialization
	// to make sure other setup functions don't interfere
	rc_mpu_set_dmp_callback(&__balance_controller);

	// start in the RUNNING state, pressing the pause button will swap to
	// the PAUSED state then back again.
	printf("\nHold your MIP upright to begin balancing\n");
	rc_set_state(RUNNING);

	// chill until something exits the program
	rc_set_state(RUNNING);
	while(rc_get_state()!=EXITING){
		rc_usleep(200000);
	}

	// join threads
	rc_pthread_timed_join(setpoint_thread, NULL, 1.5);
	rc_pthread_timed_join(battery_thread, NULL, 1.5);
	rc_pthread_timed_join(printf_thread, NULL, 1.5);

	// cleanup

	rc_filter_free(&D3);
	rc_mpu_power_off();
	rc_led_set(RC_LED_GREEN, 0);
	rc_led_set(RC_LED_RED, 0);
	rc_led_cleanup();
	rc_encoder_eqep_cleanup();
	rc_button_cleanup();	// stop button handlers
	rc_remove_pid_file();	// remove pid file LAST
	return 0;
}

/**
 * This thread is in charge of adjusting the controller setpoint based on user
 * inputs from dsm radio control. Also detects pickup to control arming the
 * controller.
 *
 * @param      ptr   The pointer
 *
 * @return     { description_of_the_return_value }
 */
void* __setpoint_manager(__attribute__ ((unused)) void* ptr)
{
	double drive_stick, turn_stick; // input sticks
	int i, ch, chan, stdin_timeout = 0; // for stdin input
	char in_str[11];

	// wait for mpu to settle
	__disarm_controller();
	rc_usleep(2500000);
	rc_set_state(RUNNING);
	rc_led_set(RC_LED_RED,0);
	rc_led_set(RC_LED_GREEN,1);

	while(rc_get_state()!=EXITING){
		// clear out input of old data before waiting for new data
		if(m_input_mode == STDIN) fseek(stdin,0,SEEK_END);

		// sleep at beginning of loop so we can use the 'continue' statement
		rc_usleep(1000000/SETPOINT_MANAGER_HZ);

		// nothing to do if paused, go back to beginning of loop
		if(rc_get_state() != RUNNING || m_input_mode == NONE) continue;

		// if we got here the state is RUNNING, but controller is not
		// necessarily armed. If DISARMED, wait for the user to pick MIP up
		// which will we detected by wait_for_starting_condition()
		if(setpoint.arm_state == DISARMED){
			if(__wait_for_starting_condition()==0){
				__zero_out_controller();
				__arm_controller();
			}
			else continue;
		}

		// if dsm is active, update the setpoint rates
		switch(m_input_mode){
		case NONE:
			continue;
		case DSM:
			if(rc_dsm_is_new_data()){
				// Read normalized (+-1) inputs from RC radio stick and multiply by
				// polarity setting so positive stick means positive setpoint
				turn_stick  = rc_dsm_ch_normalized(DSM_TURN_CH) * DSM_TURN_POL;
				drive_stick = rc_dsm_ch_normalized(DSM_DRIVE_CH)* DSM_DRIVE_POL;

				// saturate the inputs to avoid possible erratic behavior
				rc_saturate_double(&drive_stick,-1,1);
				rc_saturate_double(&turn_stick,-1,1);
				// use a small deadzone to prevent slow drifts in position
				if(fabs(drive_stick)<DSM_DEAD_ZONE) drive_stick = 0.0;
				if(fabs(turn_stick)<DSM_DEAD_ZONE)  turn_stick  = 0.0;

				// translate normalized user input to real setpoint values
				switch(setpoint.drive_mode){
				case NOVICE:
					setpoint.phi_dot   = DRIVE_RATE_NOVICE * drive_stick;
					setpoint.gamma_dot =  TURN_RATE_NOVICE * turn_stick;
					break;
				case ADVANCED:
					setpoint.phi_dot   = DRIVE_RATE_ADVANCED * drive_stick;
					setpoint.gamma_dot = TURN_RATE_ADVANCED  * turn_stick;
					break;
				default: break;
				}
			}
			// if dsm had timed out, put setpoint rates back to 0
			else if(rc_dsm_is_connection_active()==0){
				setpoint.theta = 0;
				setpoint.phi_dot = 0;
				setpoint.gamma_dot = 0;
				continue;
			}
			break;
		case STDIN:
			i = 0;

			while ((ch = getchar()) != EOF && i < 10){
				stdin_timeout = 0;
				if(ch == 'n' || ch == '\n'){
					if(i > 2){
						if(chan == DSM_TURN_CH){
							turn_stick = strtof(in_str, NULL)* DSM_TURN_POL;
							setpoint.phi_dot = drive_stick;
						}
						else if(chan == DSM_TURN_CH){
							drive_stick = strtof(in_str, NULL)* DSM_DRIVE_POL;
							setpoint.gamma_dot = turn_stick;
						}
					}
					if(ch == 'n') i = 1;
					else i = 0;
				}
				else if(i == 1){
					chan = ch - 0x30;
					i = 2;
				}
				else{
					in_str[i-2] = ch;
				}
			}

			// if it has been more than 1 second since getting data
			if(stdin_timeout >= SETPOINT_MANAGER_HZ){
				setpoint.theta = 0.0;
				setpoint.phi_dot = 0.0;
				setpoint.gamma_dot = 0.0;
			}
			else{
				stdin_timeout++;
			}
			continue;
			break;
		default:
			fprintf(stderr,"ERROR in setpoint manager, invalid input mode\n");
			break;
		}
	}

	// if state becomes EXITING the above loop exists and we disarm here
	__disarm_controller();
	return NULL;
}

/**
 * discrete-time balance controller operated off mpu interrupt Called at
 * SAMPLE_RATE_HZ
 */
static void __balance_controller(void)
{
	static int inner_saturation_counter = 0;
	double dutyL, dutyR;
	double theta_a;
	double phi_dot_sp_N;
	double states_K;
	double L_array[]= L;
	double A21_LA11_array[]=A21_LA11;
	double K_array[]=K_co;
	
	
	/******************************************************************
	* STATE_ESTIMATION
	* read sensors and compute the state when either ARMED or DISARMED
	******************************************************************/
	
	// Theta_dot measurement
	rc_mpu_read_gyro(&mpu_data);
	cstate.theta_dot=mpu_data.gyro[0]*PI/180;

	
	// Theta with complementary algorythm (positive in the direction of forward tip around X axis)
    rc_mpu_read_accel(&mpu_data);
	theta_a = -atan2(mpu_data.accel[2],mpu_data.accel[1])+BOARD_MOUNT_ANGLE ;
	cstate.theta = ((1-DT/SENSOR_FUSION_T)*(cstate.theta + DT*cstate.theta_dot) + (DT/SENSOR_FUSION_T*theta_a)) ;

	// Phi_dot observation
    cstate.phi_dot = 
		(L_array[0]*cstate.theta+L_array[1]*cstate.theta_dot+L_array[2]*cstate.phi) 
		+ cstate.phi_dot_mem;

	// collect encoder positions, right wheel is reversed
	cstate.wheelAngleR = (rc_encoder_eqep_read(ENCODER_CHANNEL_R) * 2.0 * M_PI) \
				/(ENCODER_POLARITY_R * GEARBOX * ENCODER_RES);
	cstate.wheelAngleL = (rc_encoder_eqep_read(ENCODER_CHANNEL_L) * 2.0 * M_PI) \
				/(ENCODER_POLARITY_L * GEARBOX * ENCODER_RES);

	// Phi is average wheel rotation also add theta body angle to get absolute
	// wheel position in global frame since encoders are attached to the body
	cstate.phi = ((cstate.wheelAngleL+cstate.wheelAngleR)/2) + cstate.theta;

	// steering angle gamma estimate
	cstate.gamma = (cstate.wheelAngleR-cstate.wheelAngleL) \
					* (WHEEL_RADIUS_M/TRACK_WIDTH_M);

	int nSensors=4;
	const int xdir[] = {  1, -1,  0,  0}; // x-direction sensor weights
	const int ydir[] = {  0,  0,  1, -1}; // y-direction sensor weights
	double x=0;
	double y=0;
	for (int i=0; i < nSensors; ++i) {
	
		x += xdir[i] * rc_adc_read_volt(i);
		y += ydir[i] * rc_adc_read_volt(i);
	}
	cstate.light=atan2(y,x)*180.0/3.14;
	
	if (cstate.light>=0) setpoint.light=90;
	else setpoint.light=-90;
	setpoint.gamma_dot=(cstate.light-setpoint.light)/8.0;

	if (abs(setpoint.light-cstate.light)<30) 
		if (setpoint.light>0) setpoint.phi_dot=-15;
		else setpoint.phi_dot=18;
	else setpoint.phi_dot=0;
	
	/*************************************************************
	* check for various exit conditions AFTER state estimate
	***************************************************************/
	if(rc_get_state()==EXITING){
		rc_motor_set(0,0.0);
		return;
	}
	// if controller is still ARMED while state is PAUSED, disarm it
	if(rc_get_state()!=RUNNING && setpoint.arm_state==ARMED){
		__disarm_controller();
		return;
	}
	// exit if the controller is disarmed
	if(setpoint.arm_state==DISARMED){
		return;
	}

	// check for a tipover
	if(fabs(cstate.theta) > TIP_ANGLE){
		__disarm_controller();
		printf("tip detected \n");
		return;
	}


	/************************************************************
	* State feedback Velocity controller
	*************************************************************/
	phi_dot_sp_N = setpoint.phi_dot * N_co * V_NOMINAL/cstate.vBatt ;
	states_K = K_array[0]*cstate.theta + K_array[1]*cstate.theta_dot + K_array[2]*cstate.phi_dot ;
	cstate.d1_u = (phi_dot_sp_N - states_K)/2;
	//cstate.d1_u=0;

	/*************************************************************
	* Check if the inner loop saturated. If it saturates for over
	* a second disarm the controller to prevent stalling motors.
	*************************************************************/
	if(fabs(cstate.d1_u)>0.95) inner_saturation_counter++;
	else inner_saturation_counter = 0;
	// if saturate for a second, disarm for safety
	if(inner_saturation_counter > (SAMPLE_RATE_HZ*D1_SATURATION_TIMEOUT)){
		printf("inner loop controller saturated\n");
		__disarm_controller();
		inner_saturation_counter = 0;
		return;
	}

	/**********************************************************
	* gama (steering) controller D3
	* move the setpoint gamma based on user input like phi
	***********************************************************/
	if(fabs(setpoint.gamma_dot)>0.0001) setpoint.gamma += setpoint.gamma_dot * DT;
	cstate.d3_u = rc_filter_march(&D3,setpoint.gamma - cstate.gamma);

	/**********************************************************
	* Send signal to motors
	* add D1 balance control u and D3 steering control also
	* multiply by polarity to make sure direction is correct.
	***********************************************************/
	dutyL = cstate.d1_u - cstate.d3_u;
	dutyR = cstate.d1_u + cstate.d3_u;
	rc_motor_set(MOTOR_CHANNEL_L, MOTOR_POLARITY_L * dutyL);
	rc_motor_set(MOTOR_CHANNEL_R, MOTOR_POLARITY_R * dutyR);

	// Phi_dot observation using motors inputs
	cstate.phi_dot_mem =
		(A22_LA12)*cstate.phi_dot
		+ (A21_LA11_array[0]*cstate.theta+A21_LA11_array[1]*cstate.theta_dot+A21_LA11_array[2]*cstate.phi) 
		+ (B2_LB1)*(dutyL+dutyR);

	return;
}

/**
 * Clear the controller's memory and zero out setpoints.
 *
 * @return     { description_of_the_return_value }
 */
static int __zero_out_controller(void)
{

	rc_filter_reset(&D3);
	cstate.phi_dot_mem=0;
	cstate.phi_dot=0;
	cstate.theta=0;
	setpoint.theta = 0.0;
	setpoint.phi   = 0.0;
	setpoint.gamma = 0.0;
	rc_motor_set(0,0.0);
	return 0;
}

/**
 * disable motors & set the setpoint.core_mode to DISARMED
 *
 * @return     { description_of_the_return_value }
 */
static int __disarm_controller(void)
{
	rc_motor_standby(1);
	rc_motor_free_spin(0);
	setpoint.arm_state = DISARMED;
	return 0;
}

/**
 * zero out the controller & encoders. Enable motors & arm the controller.
 *
 * @return     0 on success, -1 on failure
 */
static int __arm_controller(void)
{
	__zero_out_controller();
	rc_encoder_eqep_write(ENCODER_CHANNEL_L,0);
	rc_encoder_eqep_write(ENCODER_CHANNEL_R,0);
	// prefill_filter_inputs(&D1,cstate.theta);
	rc_motor_standby(0);
	setpoint.arm_state = ARMED;
	return 0;
}

/**
 * Wait for MiP to be held upright long enough to begin. Returns
 *
 * @return     0 if successful, -1 if the wait process was interrupted by pause
 *             button or shutdown signal.
 */
static int __wait_for_starting_condition(void)
{
	int checks = 0;
	const int check_hz = 20;	// check 20 times per second
	int checks_needed = round(START_DELAY*check_hz);
	int wait_us = 1000000/check_hz;

	// wait for MiP to be tipped back or forward first
	// exit if state becomes paused or exiting
	while(rc_get_state()==RUNNING){
		// if within range, start counting
		if(fabs(cstate.theta) > START_ANGLE) checks++;
		// fell out of range, restart counter
		else checks = 0;
		// waited long enough, return
		if(checks >= checks_needed) break;
		rc_usleep(wait_us);
	}
	// now wait for MiP to be upright
	checks = 0;
	// exit if state becomes paused or exiting
	while(rc_get_state()==RUNNING){
		// if within range, start counting
		if(fabs(cstate.theta) < START_ANGLE) checks++;
		// fell out of range, restart counter
		else checks = 0;
		// waited long enough, return
		if(checks >= checks_needed) return 0;
		rc_usleep(wait_us);
	}
	return -1;
}

/**
 * Slow loop checking battery voltage. Also changes the D1 saturation limit
 * since that is dependent on the battery voltage.
 *
 * @return     nothing, NULL poitner
 */
static void* __battery_checker(__attribute__ ((unused)) void* ptr)
{
	double new_v;
	while(rc_get_state()!=EXITING){
		new_v = rc_adc_batt();
		// if the value doesn't make sense, use nominal voltage
		if (new_v>9.0 || new_v<5.0) new_v = V_NOMINAL;
		cstate.vBatt = new_v;
		rc_usleep(1000000 / BATTERY_CHECK_HZ);
	}
	return NULL;
}

/**
 * prints diagnostics to console this only gets started if executing from
 * terminal
 *
 * @return     nothing, NULL pointer
 */
static void* __printf_loop(__attribute__ ((unused)) void* ptr)
{
	rc_state_t last_rc_state, new_rc_state; // keep track of last state
	last_rc_state = rc_get_state();
	while(rc_get_state()!=EXITING){
		new_rc_state = rc_get_state();
		// check if this is the first time since being paused
		if(new_rc_state==RUNNING && last_rc_state!=RUNNING){
			printf("\nRUNNING: Hold upright to balance.\n");
			printf("    θ    |");
			printf("  θ_ref  |");
			printf("    φ    |");
			printf("  φ_ref  |");
			printf("    γ    |");
			printf("  D1_u   |");
			printf("  D3_u   |");
			printf("  vBatt  |");
			printf("arm_state|");
			printf("\n");
		}
		else if(new_rc_state==PAUSED && last_rc_state!=PAUSED){
			printf("\nPAUSED: press pause again to start.\n");
		}
		last_rc_state = new_rc_state;

		// decide what to print or exit
		if(new_rc_state == RUNNING){
			printf("\r");
			printf("Light angle: %7.3f | ", cstate.light);
			printf("Gamma_dot: %7.3f | ", setpoint.gamma_dot);
			printf("Gamma SP: %7.3f | ", setpoint.gamma);
			printf("Gamma: %7.3f | ", cstate.gamma);
			printf("Phi_dot SP: %7.3f | ", setpoint.phi_dot);
			printf("Phi_dot: %7.3f | ", cstate.phi_dot);
			if(setpoint.arm_state == ARMED) printf("  ARMED  |");
			else printf("DISARMED |");
			fflush(stdout);
		}
		rc_usleep(1000000 / PRINTF_HZ);
	}
	return NULL;
}

/**
 * Disarm the controller and set system state to paused. If the user holds the
 * pause button for 2 seconds, exit cleanly
 */
static void __on_pause_press(void)
{
	int i=0;
	const int samples = 100;	// check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds

	switch(rc_get_state()){
	// pause if running
	case EXITING:
		return;
	case RUNNING:
		rc_set_state(PAUSED);
		__disarm_controller();
		rc_led_set(RC_LED_RED,1);
		rc_led_set(RC_LED_GREEN,0);
		break;
	case PAUSED:
		rc_set_state(RUNNING);
		__disarm_controller();
		rc_led_set(RC_LED_GREEN,1);
		rc_led_set(RC_LED_RED,0);
		break;
	default:
		break;
	}

	// now wait to see if the user want to shut down the program
	while(i<samples){
		rc_usleep(us_wait/samples);
		if(rc_button_get_state(RC_BTN_PIN_PAUSE)==RC_BTN_STATE_RELEASED){
			return; //user let go before time-out
		}
		i++;
	}
	printf("long press detected, shutting down\n");
	//user held the button down long enough, blink and exit cleanly
	rc_led_blink(RC_LED_RED,5,2);
	rc_set_state(EXITING);
	return;
}

/**
 * toggle between position and angle modes if MiP is paused
 */
static void __on_mode_release(void)
{
	// toggle between position and angle modes
	if(setpoint.drive_mode == NOVICE){
		setpoint.drive_mode = ADVANCED;
		printf("using drive_mode = ADVANCED\n");
	}
	else {
		setpoint.drive_mode = NOVICE;
		printf("using drive_mode = NOVICE\n");
	}

	rc_led_blink(RC_LED_GREEN,5,1);
	return;
}
