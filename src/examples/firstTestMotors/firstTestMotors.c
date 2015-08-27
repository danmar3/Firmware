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


// GLOBAL variables for the thread managment 
static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */
 

// GLOBAL variables for the management of sensors
int sensor_sub_fd;
struct pollfd fds[1];
int missingDataCounter;

/**
 * daemon management function.
 */
__EXPORT int firstTestMotors_main(int argc, char *argv[]);
 
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
int systeminitialization(void);

int systeminitialization(void){

  //---------- sensor initialization ---------//
  // subscribe to sensor_combined topic
  sensor_sub_fd = orb_subscribe(ORB_ID(sensor_combined));
  // set up the discretization time
  orb_set_interval(sensor_sub_fd, 1000);
  // set up the poll structure
  fds[0].fd= sensor_sub_fd;
  fds[0].events= POLLIN;
  // initialize the counter
  missingDataCounter= 0;

  
  return 0;
}

int firstTestMotors_main(int argc, char *argv[]){
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
    
    daemon_task = px4_task_spawn_cmd("testMotorsDaniel",
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
 
int motors_thread_main(int argc, char *argv[]){
  systeminitialization();

  warnx("[motors test] starting\n");
  

  sleep(20);
  
  thread_running = true;
  
  while (!thread_should_exit) {
    //warnx("Hello daemon!\n");
    
    readSensors();
    
  }
  
  warnx("[motors test] exiting.\n");
  
  thread_running = false;
  
  return 0;
}
