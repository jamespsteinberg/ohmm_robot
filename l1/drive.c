/**
*
*  FILE:  drive.c
*  This file contains the following commands for driving the robot,
*  as specified by lab 1.
*
*  dgvw  : Drive get v, w
*  dsvw  : Drive set v, w
*  dsvl  : Drive set v, l
*  dgp   : Drive get pose
*  drp   : Drive reset pose
*  df    : Drive forward
*  dgs   : Drive get status
*  dst   : Drive stop
*  drs   : Drive resume
*  dri   : Drive reinit
*  dt    : Drive turn
**/

#include <drive.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "pololu/orangutan.h"
#include "../../ohmm-sw/llp/monitor/ohmm/mot.h"
#include "../../ohmm-sw/llp/monitor/ohmm/hlp.h"
#include "../../ohmm-sw/llp/monitor/ohmm/cmd.h"



// Wheel radius:  40 mm
#define RADIUS 40.0

// Baseline (Distance between wheels):  192 mm
#define BASELINE 192.0

//Starting Command id for drive commands
//(There are 111 predefined commands, so we will start with 112)
#define DRIVE_ID 112

//Frequency at which to calculate odometry;  (20 times a second) 
#define ODOMETRY_PERIOD MS_TO_TICKS(50)

//Maximum number of DF commands to be queued
#define MAX_COMMANDS 32

//Value to account odometer for counting rotations short. (Maybe due to friction?)
//Note:  0.938 works well on lab tile floor
//       0.87 works well on carpeted floor (rm 212)
#define FRICTION_ERROR 0.87

// Maximum speed of wheels
// The max speed of each wheel is: 0.8 rotations/sec = ~200 mm/s = 32 radians/sec
#define MAX_ROTATION_SPEED 0.8

////////////////////////
//  static variables  //
////////////////////////

//Odometry
static volatile float x_w = 0; //Robot x-coordinate in world frame (mm)
static volatile float y_w = 0; //Robot y-coordinate in world frame (mm)
static volatile float theta_w = 0; //Robot orientation, in world frame (Radians)

static volatile float prev_r = 0; //Position of left wheel at last check, in rotations
static volatile float prev_l = 0; //Position of right wheel at last check, in rotations

//Command Queue

static struct Drive_Cmd cmd_queue[MAX_COMMANDS]; // queue of all driving commands
static short cmd_push_index = 0; // queue position for push
static short cmd_pop_index = 0; // queue position for pop
static short cmd_in_progress = 0; //0 = no command in progress. 1 = command in progress
static volatile short cmd_count = 0; // Number of commands in queue

//Drive forward
static volatile float df_speed_fast = 100; // 100 mm/sec
static volatile float df_speed_slow = 20; // 20 mm/sec
static volatile float df_dist_left = 0; // distance left to go, in mm
static volatile float df_start_theta = 0; 

// Drive Turn
static volatile float dt_speed = M_PI / 2; // angular speed
static volatile float dt_rad_left = 0.0; // rotation left to go, in radians
//Parameters for resume
float resume_params_vw[2] = {0, 0};// [1] = v, [2] = w


//////////////////////
//  INITIALIZATION  //
//////////////////////
void initState()
{
   //Set all values in command queue to empty
   struct Drive_Cmd empty_cmd;
   enum CMD_TYPE_ENUM empty_type = empty; 
   empty_cmd.cmd_type = empty_type;
   empty_cmd.cmd_val = 0;
   for(int i = 0; i < MAX_COMMANDS; i++)
   {
     cmd_queue[i] = empty_cmd;
   }
   drp_cmd_handler(); //Start the odometer
}




///////////////////////////////////
//  FORWARD VELOCITY KINEMATICS  //
///////////////////////////////////

/**
 *  dgvw
 *  "Drive get v, w"
 *  Monitor command that takes no arguments, and returns two
 *  floating point numbers.
 *  The first is the current robot forward velocity v, in mm/sec.
 *  The second is the current CCS angular velocity w, in rad/sec.
**/
float* dgvw()
{
    float l; // l is the left wheel velocity in rotations per sec
    float r; // r is the right wheel velocity in rotations per sec
    motGetVel(&l, &r);
    float vl = l * 2 * M_PI * RADIUS; // get vl in mm/sec
    float vr = r * 2 * M_PI * RADIUS; // get vl in mm/sec
    float result[2];
    // v as calculated
    result[0] = (vl + vr) / 2.0;
    // w as calculated
    result[1] = (vr - vl) / BASELINE;
    hlpPrintf_P(PSTR("[v]: %f.   [w]: %f\n"), result[0], result[1]);       
    
    return result;
}


///////////////////////////////////
//  INVERSE VELOCITY KINEMATICS  //
///////////////////////////////////
/**
 * dsvw
 * "Drive set v, w"
 * Monitor command that takes two arguments, v and w.
 * v:  Forward Velocity, in mm/sec.
 * w:  Angular Velocity, in radians/sec.
 *
 * The command makes a corresponding call to motSetVel() after applying
 * velocity IK transformation.
**/
void dsvw(float v, float w)
{ 
   //v: mm/s 
   HLP_DBG("dsvw: v: %f,  w: %f", v, w);

   float vl = (v - (BASELINE * w / 2.0)) / (2 * M_PI * RADIUS); //Left motor vel in rotations per sec
   float vr = (v + (BASELINE * w / 2.0)) / (2 * M_PI * RADIUS); //Right motor vel in rotations per sec

   if (vl > MAX_ROTATION_SPEED)
   {
      HLP_DBG("Warning: Left wheel (vl) can only go 0.8 rotations per second. vl set to %f\n", vl);
   }
   if (vr > MAX_ROTATION_SPEED)
   {
      HLP_DBG("Warning: Right wheel (vr) can only go 0.8 rotations per second. vr set to %f\n", vr);
   }
   motSetVelCmd(vl, vr);
}

/**
 * dsvl
 * "Drive set v, l"
 * Monitor command that takes two arguments, v and l.
 * v:  Forward velocity, in mm/sec.
 * l:  Desired turning radius, in mm.
**/
void dsvl(float v, float l)
{
    //Arc radius (l) = velocity / angular velocity
    // l = v / w --> w = v / l

    if(l != 0)
    {
      float w = v / l;
      dsvw(v, w);
    }
    else
    {
      dsvw(0, v);
    }
}               


///////////////////////////////////
//  FORWARD POSITION KINEMATICS  //
///////////////////////////////////
/** dodomTask() 
 * "Drive odometry task"
 * Monitor foreground task that implements incremental odometry.
 * Task runs at 20Hz.  Robot pose starts at (x, y, theta) = (0, 0, 0).
 * 
**/
void dodom_task()
{
   float l, r;
   motGetPos(&l, &r);
   float l_dif = l - prev_l;
   float r_dif = r - prev_r;

   //Use point-and-shoot approximation to calculate change in motion
   // 2 * pi converts rotations to radians
   float theta_dif = ((RADIUS / BASELINE) * (r_dif - l_dif)) * 2 * M_PI * FRICTION_ERROR; 
   float x_dif = (RADIUS / 2) * ((r_dif + l_dif) * 2 * M_PI) * cos(theta_dif + theta_w);
   float y_dif = (RADIUS / 2) * ((r_dif + l_dif) * 2 * M_PI) * sin(theta_dif + theta_w);


   x_w = x_w + x_dif;
   y_w = y_w + y_dif;
   theta_w = theta_w + theta_dif;

   prev_l = l;
   prev_r = r;
    
     //Command isn't currently running, but there are more in the queue
    if(cmd_in_progress == 0 && cmd_count > 0) 
    {
        //////////Pop command from queue and execute it /////////////
        struct Drive_Cmd cmd = pop_cmd(); //Get command

        if(cmd.cmd_type == df_cmd)
        {
           df(cmd.cmd_val);
	   cmd_in_progress = 1; //There is now a command in progress   
        }
        else if(cmd.cmd_type == dt_cmd)
        {
            dt(cmd.cmd_val);
	    cmd_in_progress = 2; //There is now a command in progress
        }
    }   

 

    // df command
    if(cmd_in_progress == 1)
    {
        double dist_travelled = sqrt((x_dif * x_dif) + (y_dif * y_dif));

        if(df_dist_left > 0 )
        {
            df_dist_left = df_dist_left - dist_travelled; //Forwards
        }
        else if(df_dist_left < 0)
        {
            df_dist_left = df_dist_left + dist_travelled; //Backwards
        }
        if(fabs(df_dist_left) < 10)  //10 mm tolerance
        {
            dsvw(0, 0); //Stop.
            cmd_in_progress = 0;
        }
    }
   // dt command
   else  if(cmd_in_progress == 2)
    {
        if(dt_rad_left > 0 )
        {
            dt_rad_left = dt_rad_left - fabs(theta_dif); //Forwards

            if(dt_rad_left < 0.01){
               dsvw(0, 0); //Stop.
	       cmd_in_progress = 0;
            }
	 }
        else if(dt_rad_left < 0)
        {
            dt_rad_left = dt_rad_left + fabs(theta_dif); //Backwards

            if(dt_rad_left > -0.01){
                dsvw(0, 0); //Stop.
		cmd_in_progress = 0;
	    }
        }
        else if(dt_rad_left == 0)
        {
            dsvw(0, 0); //Stop.
	    cmd_in_progress = 0;
        }
    }
}

/**
 * dgp
 * "Drive get pose"
 * Monitor command that takes no arguments, and returns three floating
 * point numbers.
 * Return value:
 *   [0]: x     - x displacement of robot, in world frame
 *   [1]: y     - y displacement of robot, in world frame
 *   [2]: theta - orientation of robot, in world frame
**/
float* dgp()
{
    float result[3] = {x_w, y_w, theta_w};
    
    HLP_DBG("X: %f   Y: %f   THETA: %f", x_w, y_w, theta_w);
    HLP_DBG("Dist left: %f", df_dist_left);
    return result;
}

/**
 * dgpr
 * "Drive get pose repeated"
 * Repeatedly call the dgp function
 **/

void dgpr_task()
{
    HLP_DBG("X: %f   Y: %f   THETA: %f", x_w, y_w, theta_w);
}


/**
 * drp
 * "Drive reset pose"
 * Monitor command that takes no arguments, and resets the robot pose.
 * x, y, and theta are all reset to 0.
**/
void drp()
{
    taskCancel(dodom_task);
    x_w = 0;
    y_w = 0;
    theta_w = 0;

    taskRegister(dodom_task, ODOMETRY_PERIOD, PSTR("dodom"));
}

///////////////////
// COMMAND QUEUE //
///////////////////

//Pushes a command onto the command queue
void push_cmd(struct Drive_Cmd cmd)
{
  enum CMD_TYPE_ENUM cmd_type = cmd.cmd_type;
  if(cmd_queue[cmd_push_index].cmd_type == empty) //Push index points to an empty 'slot'
  {
     cmd_queue[cmd_push_index] = cmd;
     //HLP_DBG("Pushing!  Index: %d  Command: %d  Value: %f", cmd_push_index, cmd_queue[cmd_push_index].cmd_type, cmd_queue[cmd_push_index].cmd_val);

     cmd_push_index = cmd_push_index + 1;
     if(cmd_push_index >= MAX_COMMANDS) //Go back to zero after reaching end of array
     {
       cmd_push_index = cmd_push_index - MAX_COMMANDS;
     }
     cmd_count = cmd_count + 1; //Keep track of queue size


  }
  else
  { //Slot is occupied!
    HLP_DBG("Queue is full!  Index: %d   Value: %f", cmd_push_index, cmd_queue[cmd_push_index]);
  }
}

//Pops command from command queue, returning the value, and removing from queue
struct Drive_Cmd pop_cmd()
{
   //HLP_DBG("Popping!  Index: %d  Command: %d  Value: %f", cmd_pop_index, cmd_queue[cmd_pop_index].cmd_type, cmd_queue[cmd_pop_index].cmd_val);
   struct Drive_Cmd cmd_val = cmd_queue[cmd_pop_index];
   //Clear command from queue
   cmd_queue[cmd_pop_index].cmd_type = empty; 
   cmd_queue[cmd_pop_index].cmd_val = 0;
 
   cmd_pop_index = cmd_pop_index + 1;
   if(cmd_pop_index >= MAX_COMMANDS) //Go back to zero after reaching end of array
   {
     cmd_pop_index = cmd_pop_index - MAX_COMMANDS;
   }

   if(cmd_count > 0) //Just to be safe, in case we call pop when queue is empty.
   {
      cmd_count = cmd_count - 1; //Keep track of queue size
   }
   else{
     HLP_DBG("Warning:  Tried to pop from empty command queue!");
   }
   return cmd_val;
}


///////////////////////////////////
//  POSITION INVERSE KINEMATICS  //
///////////////////////////////////


/**
 * df
 * "Drive Forward"
 * Drives forward a total of d mm
 **/
void df(float d)
{
    df_dist_left = d;
    //DRIVE SPEEDS:
    if(d > 20)
    {    
        dsvw(df_speed_fast, 0); //Set velocity to 100 mm/s.
    }
    else if(d < 20 && d > 0)
    {
        dsvw(df_speed_slow, 0); //Go slower (20 mm/s) for small distances < 20 mm
    }
    else if(d < 0 && d > -20)
    {
        dsvw(-df_speed_slow, 0); //Go slow backwards (20 mm/s) for small distances < 20 mm   
    }
    else //d < -20
    {
        dsvw(-df_speed_fast, 0);
    }
}

/**
 * dt
 * "Drive Turn"
 * Turns in a place a total of an angle of a radians
 **/
void dt(float a)
{
    dt_rad_left = a;
    if(a < 0) // turn left
    {
        dsvw(0, -dt_speed); // set angular velocity
    }
    else // turn right
    {
	dsvw(0, dt_speed); // set angular velocity
    }
}


/**
 * dgs
 * "Drive Get Status"
 * Returns the number of commands in the current command queue.
 * *(Includes command currently executing, if any)
 */
int dgs()
{
  HLP_DBG("Current commands in queue: %d", cmd_count + cmd_in_progress);
  int result = cmd_count;
  if(cmd_in_progress > 0)
  {
    result = result + 1;
  }
  return result;
}

/**
 * dst
 * "Drive stop"
 * Pauses any drive cmd.
 */
void dst()
{
	//save current drive params
	float * params = dgvw();
	float v = *params;
	params++;
	float w = *params;
	
	if (!((v == 0) && (w == 0)))
	{
	    resume_params_vw[0] = v;
	    resume_params_vw[1] = w;

	    //Pause current drive
	    dsvw(0,0);
	}
}

/**
 * drs
 * "Drive resume"
 * Resumes any previously paused drive command.
 */
void drs()
{
	//resume motion before drive was stopped
    if ((resume_params_vw[0] != 0) || (resume_params_vw[1] != 0))
    {
	dsvw(resume_params_vw[0], resume_params_vw[1]);
	resume_params_vw[0] = 0;
	resume_params_vw[1] = 0;
    }
    hlpPrintf_P(PSTR("[v]: %f.   [w]: %f\n"), resume_params_vw[0], resume_params_vw[1]);       
}

/**
 * dri
 * "Drive stop and reinitialize"
 * Stops current drive command and empties queue of commands.
 */
void dri()
{
	//stop motion
	dsvw(0, 0);
	cmd_in_progress = 0;
	int i;
	int old_cmd_count = cmd_count;
	cmd_count = 0;
	//clear queue
	for (i = 0; i < old_cmd_count; i++)
	{
		//This must be done like this due to the way pop_cmd works
		cmd_count = old_cmd_count - i;
		pop_cmd();
		cmd_count = 0;
	}
	//clear queue variables
	resume_params_vw[0] = 0;
	resume_params_vw[1] = 0;

}

//REGISTER COMMANDS
void driveRegisterCmds() {
    
    //dgvw
    cmdRegister_P(DRIVE_ID, PSTR("dgvw"), PSTR("Drive get v w"),
                  dgvw_cmd_handler, 0, 0);
    
    //dsvw
    cmdRegister_P(DRIVE_ID + 1, PSTR("dsvw"), PSTR("Drive set v w"),
                  dsvw_cmd_handler, 2, 2);

    //dsvl
    cmdRegister_P(DRIVE_ID + 2, PSTR("dsvl"), PSTR("Drive set v l"),
                  dsvl_cmd_handler, 2, 2);
    
    //dgp
    cmdRegister_P(DRIVE_ID + 3, PSTR("dgp"), PSTR("Drive get pose"),
                  dgp_cmd_handler, 0, 0);

    //dgpr
    cmdRegister_P(DRIVE_ID + 4, PSTR("dgpr"), PSTR("Drive get pose repeat"),
                  dgpr_cmd_handler, 0, 0);
    
    //dgpst
    cmdRegister_P(DRIVE_ID + 5, PSTR("dgpst"), PSTR("Drive get pose stop repeat"),
                  dgpst_cmd_handler, 0, 0);
    
    //drp
    cmdRegister_P(DRIVE_ID + 6, PSTR("drp"), PSTR("Drive reset pose"),
                  drp_cmd_handler, 0, 0);

    //df
    cmdRegister_P(DRIVE_ID + 7, PSTR("df"), PSTR("Drive Forward"),
                  df_cmd_handler, 1, 1);

    //dt
    cmdRegister_P(DRIVE_ID + 8, PSTR("dt"), PSTR("Drive Turn"),
                  dt_cmd_handler, 1, 1);


    //dgs
    cmdRegister_P(DRIVE_ID + 9, PSTR("dgs"), PSTR("Drive get status"),
                  dgs_cmd_handler, 0, 0);

    //dst
    cmdRegister_P(DRIVE_ID + 10, PSTR("dst"), PSTR("Drive stop "),
                  dst_cmd_handler, 0, 0);

    //drs
    cmdRegister_P(DRIVE_ID + 11, PSTR("drs"), PSTR("Drive resume"),
                  drs_cmd_handler, 0, 0);

    //dri
    cmdRegister_P(DRIVE_ID + 12, PSTR("dri"), PSTR("Drive stop and reinit"),
                  dri_cmd_handler, 0, 0);

    //test square_square
    cmdRegister_P(DRIVE_ID + 13, PSTR("dtsq"), PSTR("Drive test square"),
                  drive_square, 1, 1);

    //test boomerang
    cmdRegister_P(DRIVE_ID + 14, PSTR("dtb"), PSTR("Drive test boomerang"),
                  boomerang, 1, 1);

}

//COMMAND HANDLERS
void dgvw_cmd_handler() { //dgvw
    dgvw();
}

void dsvw_cmd_handler() { //dsvw
    float v, w;
    int8_t v_parse = cmdParseFloat(&v);
    int8_t w_parse = cmdParseFloat(&w);

    //Safety check for parsing errors
    if(!(v_parse < 0) && !(w_parse < 0))
    {
      dsvw(v, w);
    }
    else
    {
      hlpPrintf_P(PSTR("dsvw parse failed."));
    }
}
                
 
void dsvl_cmd_handler() { //dsvl
   float v, l;
   int8_t v_parse = cmdParseFloat(&v);
   int8_t l_parse = cmdParseFloat(&l);

   //Safety check for parsing errors
   if(!(v_parse < 0) && !(l_parse < 0))
   {
     dsvl(v, l);
   }
   else
   {
     hlpPrintf_P(PSTR("dsvl parse failed."));
   }
}
 
void dgp_cmd_handler() { //dgp
   dgp();
}
 
void drp_cmd_handler() { //drp
   drp();
}

//FOR CONTINUOUS OUTPUT
void dgpr_cmd_handler() { //dgpr
    taskRegister(dgpr_task, MS_TO_TICKS(2000), PSTR("dgpr"));
}

void dgpst_cmd_handler() { //dgpst
    taskCancel(dgpr_task);
}

void df_cmd_handler() //df
{
   float d;
   int8_t d_parse = cmdParseFloat(&d);
   HLP_DBG("d: %f and d_parse: %d", d, d_parse);
   //Safety check for parsing errors
   if(!(d_parse < 0))
   {
     struct Drive_Cmd cmd;
     enum CMD_TYPE_ENUM cmd_type = df_cmd;
     cmd.cmd_type = df_cmd;
     cmd.cmd_val = d;
     push_cmd(cmd);
   }
   else
   {
     hlpPrintf_P(PSTR("df parse failed."));
   }
} 

void dt_cmd_handler() //dt
{
   float d;
   int8_t d_parse = cmdParseFloat(&d);
   HLP_DBG("d: %f and d_parse: %d", d, d_parse);
   //Safety check for parsing errors
   if(!(d_parse < 0))
   {
     struct Drive_Cmd cmd;
     enum CMD_TYPE_ENUM cmd_type = dt_cmd;
     cmd.cmd_type = dt_cmd;
     cmd.cmd_val = d;
     push_cmd(cmd);
   }
   else
   {
     hlpPrintf_P(PSTR("dt parse failed."));
   }
}


void dgs_cmd_handler() //dgs
{
  dgs();
}

void dst_cmd_handler() //dst
{
  dst();
}

void drs_cmd_handler() //drs
{
  drs();
}

void dri_cmd_handler() //dri
{
  dri();
}

//Tests
/**
 * drive_square
 * Drives robot in a square with side length specified by input
 */
void drive_square()
{
    float side;
    int8_t side_parse = cmdParseFloat(&side);
    float angle;
    if (side > 0)
	angle = M_PI/2;
    else
	angle = -M_PI/2;
    
    int i;
    struct Drive_Cmd cmd;
    enum CMD_TYPE_ENUM cmd_type;
    for (i = 0; i < 4; i++)
    {
	HLP_DBG("d: %f and d_parse: %d", side, side_parse);
	//Safety check for parsing errors
	if(!(side_parse < 0))
	{
	    cmd_type = df_cmd;
	    cmd.cmd_type = df_cmd;
	    cmd.cmd_val = side;
	    push_cmd(cmd);
   	}
   	else
   	{
     	    hlpPrintf_P(PSTR("df parse failed."));
   	}


	HLP_DBG("angle: %f radians", angle);
	//Safety check for parsing errors
	if(!(side_parse < 0))
	{
	    cmd_type = dt_cmd;
	    cmd.cmd_type = dt_cmd;
	    cmd.cmd_val = angle;
	    push_cmd(cmd);
   	}
   	else
   	{
     	    hlpPrintf_P(PSTR("dt parse failed."));
   	}
    }

}

/**
 * boomerang
 * Drives robot forward the input distance, turns 90 degrees then drives back to starting position and turns to original orientation
 */
void boomerang()
{
    float d;
    int8_t d_parse = cmdParseFloat(&d);
    float angle = M_PI;
    
    int i;
    struct Drive_Cmd cmd;
    enum CMD_TYPE_ENUM cmd_type;
    for (i = 0; i < 2; i++)
    {
	HLP_DBG("d: %f and d_parse: %d", d, d_parse);
	//Safety check for parsing errors
	if(!(d_parse < 0))
	{
	    cmd_type = df_cmd;
	    cmd.cmd_type = df_cmd;
	    cmd.cmd_val = d;
	    push_cmd(cmd);
   	}
   	else
   	{
     	    hlpPrintf_P(PSTR("df parse failed."));
   	}


	HLP_DBG("angle: %f radians", angle);
	//Safety check for parsing errors
	if(!(d_parse < 0))
	{
	    cmd_type = dt_cmd;
	    cmd.cmd_type = dt_cmd;
	    cmd.cmd_val = angle;
	    push_cmd(cmd);
   	}
   	else
   	{
     	    hlpPrintf_P(PSTR("dt parse failed."));
   	}
    }
}
