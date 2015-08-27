#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

// For manage task and threads
#include <px4_config.h>
#include <nuttx/sched.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

// For the publisher and subscrivers
#include <nuttx/config.h>
#include <poll.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_combined.h>

// Includes for motors
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/actuator_controls.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/actuator_controls_0.h>
#include <uORB/topics/actuator_controls_1.h>
#include <uORB/topics/actuator_controls_2.h>
#include <uORB/topics/actuator_controls_3.h>



// GLOBAL variables for the thread managment 
static bool thread_should_exit = false;		
static bool thread_running = false;		
static int daemon_task;				
 

// GLOBAL variables for the management of sensors
int sensor_sub_fd;
struct pollfd fds[1];
int missingDataCounter;

// GLOBAL variables for motors
struct actuator_controls_s actuators;
orb_advert_t actuator_pub_fd;
orb_advert_t arm_pub_fd;
struct actuator_armed_s arm;
int arm_sub_fd;
hrt_abstime stime;

/**
 * daemon management function.
 */
__EXPORT int TestMotor_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int motors_thread_main(int argc, char *argv[]);

/**
 * Print the correct usage.
 */
static void usage(const char *reason);

static void
usage(const char *reason){
  if (reason) {
    warnx("%s\n", reason);
  }
  
  warnx("usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}
/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */

// --------------------------------------- System initialization ---------------------------//

int systeminitialization(void);

int systeminitialization(void){
  
  //---------- sensor initialization ---------//
  // subscribe to sensor_combined topic
  sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
  // set up the discretization time
  orb_set_interval(sensor_sub_fd, 30);
  // set up the poll structure
  fds[0].fd= sensor_sub_fd;
  fds[0].events= POLLIN;
  // initialize the counter
  missingDataCounter= 0;
  
  
  //---------- motor initialization ---------//
  warnx("DO NOT FORGET TO STOP THE COMMANDER APP!");
  warnx("(run <commander stop> to do so)");
  memset(&actuators, 0, sizeof(actuators));
  actuator_pub_fd = orb_advertise(ORB_ID(actuator_controls_0), &actuators);
  
  memset(&arm, 0 , sizeof(arm));
  
  arm.timestamp = hrt_absolute_time();
  arm.ready_to_arm = true;
  arm.armed = true;
  arm_pub_fd = orb_advertise(ORB_ID(actuator_armed), &arm);
  orb_publish(ORB_ID(actuator_armed), arm_pub_fd, &arm);
  
  /* read back values to validate */
  arm_sub_fd = orb_subscribe(ORB_ID(actuator_armed));
  orb_copy(ORB_ID(actuator_armed), arm_sub_fd, &arm);
  
  if (arm.ready_to_arm && arm.armed) {
    warnx("Actuator armed");
    
  } else {
    errx(1, "Arming actuators failed");
  }


  
  /*
  float outESC= 1.0f;
  int count= 1;
  while(count < 100){
    actuators.control[0] = outESC;
    actuators.control[1] = outESC;
    actuators.control[2] = outESC;
    actuators.control[3] = outESC;
    actuators.timestamp = hrt_absolute_time();
    orb_publish(ORB_ID(actuator_controls_0), actuator_pub_fd, &actuators);
    usleep(10);
  }

  */



  
  return 0;
}

// --------------------------------------- MAIN ---------------------------------//
int TestMotor_main(int argc, char *argv[]){
  if (argc < 2) {
    usage("missing command");
    return 1;
  }
  
  if (!strcmp(argv[1], "start")) {
    
    if (thread_running) {
      warnx("motor test already running\n");
      /* this is not an error */
      return 0;
    }
    
    thread_should_exit = false;
    
    daemon_task = px4_task_spawn_cmd("TestMotorD",
				     SCHED_DEFAULT,
				     SCHED_PRIORITY_DEFAULT,
				     2000,
				     motors_thread_main,
				     (argv) ? (char * const *)&argv[2] : (char * const *)NULL);
    return 0;
  }
 
  if (!strcmp(argv[1], "stop")) {
    thread_should_exit = true;
    return 0;
  }
 
  if (!strcmp(argv[1], "status")) {
    if (thread_running) {
      warnx("\trunning\n");
      
    } else {
      warnx("\tnot started\n");
    }
    
    return 0;
  }
  
  usage("unrecognized command");
  return 1;
}


// --------------------------------------- Read sensors --------------------------------//
int readSensors(void);

int readSensors(void){
  /* wait for sensor update of 1 file descriptor for 1000 ms (1 second) */
  int poll_ret = poll(fds, 1, 1000);
  
  /* handle the poll result */
  if (poll_ret == 0) {
    /* this means none of our providers is giving us data */
    printf("[px4_simple_app] Got no data within a second\n");
  } else if (poll_ret < 0) {
    /* this is seriously bad - should be an emergency */
    if (missingDataCounter < 10 || missingDataCounter % 50 == 0) {
      /* use a counter to prevent flooding (and slowing us down) */
      printf("[px4_simple_app] ERROR return value from poll(): %d\n"
	     , poll_ret);
    }
    missingDataCounter++;
  } else {
    
    if (fds[0].revents & POLLIN) {
      /* obtained data for the first file descriptor */
      struct sensor_combined_s raw;
      /* copy sensors raw data into local buffer */
      orb_copy(ORB_ID(sensor_combined), sensor_sub_fd, &raw);
      warnx("[px4_simple_app] Accelerometer:\t%8.4f\t%8.4f\t%8.4f\n",
	    (double)raw.accelerometer_m_s2[0],
	    (double)raw.accelerometer_m_s2[1],
	    (double)raw.accelerometer_m_s2[2]);
    }
    
  }
  
  return 0;
}

// --------------------------------------- Set ESC ------------------------------------//
int setESC(void);

int setESC(void){
  
  static int count1= 0;
  static int count2= 0;
  static int increm= 1;
  float outESC= -1.0f;
  
  if (count1 <= 5) {
    outESC = -1.0f;
 
  } else if (count1 <= 10) {
    outESC = -0.7f;
    
  } else if (count1 <= 15) {
    outESC = -0.5f;
    
  } else if (count1 <= 20) {
    outESC = -0.3f;
    
  } else if (count1 <= 25) {
    outESC= 0.0f;
    
  } else if (count1 <= 30) {
    outESC = 0.3f;
    
  } else {
    outESC = 0.5f;
  }
  
  count2+= 1;
  if(count2 > 30){
    count2= 0;
    count1+= increm;
  }
  
  if(count1 > 30)
    increm= -1;
  
  if(count1 == 0)
    increm= 1;


  warnx("outESC= %8.4f \n", (double)outESC);
  actuators.control[0] = outESC;
  actuators.control[1] = outESC;
  actuators.control[2] = outESC;
  actuators.control[3] = outESC;
  actuators.timestamp = hrt_absolute_time();
  orb_publish(ORB_ID(actuator_controls_0), actuator_pub_fd, &actuators);
  //usleep(10000);

  
  return 0;
}


// --------------------------------------- motors_thread_main -----------------------------------//
int motors_thread_main(int argc, char *argv[]){
  systeminitialization();

  warnx("[motors test] starting\n");
  

  sleep(2);
  
  thread_running = true;
  
  while (!thread_should_exit) {
    //warnx("Hello daemon!\n");
    
    readSensors();
    
    setESC();
  }
  
  warnx("[motors test] exiting.\n");
  
  thread_running = false;
  
  return 0;
}
