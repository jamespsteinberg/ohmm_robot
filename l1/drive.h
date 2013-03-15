/**
 *
 *  FILE:  drive.h
 *  This file contains the headers for the 
 *  following commands for driving the robot,
 *  as specified by lab 1.
 *
 *  dgvw : Drive get v, w
 *  dsvw : Drive set v, w
 *  dsvl : Drive set v, l
 *  dgp  : Drive get pose
 *  drp  : Drive reset pose
 *  df   : Drive forward
 *  dgs  : Drive get status
 *  dst  : Drive stop
 *  drs  : Drive resume
 *  dri  : Drive reinit
 *  dt   : Drive turn
 **/

/////////////////////////////////////////////
//  Helper enums and structs //
/////////////////////////////////////////////
enum CMD_TYPE_ENUM
{
  empty = 0,
  df_cmd = 1,
  dt_cmd = 2
} ;

struct Drive_Cmd
{
  enum CMD_TYPE_ENUM cmd_type;
  float cmd_val;
};

/////////////////////////////////////////////
//  COMMAND HANDLERS (Static definitions)  //
/////////////////////////////////////////////

//Forward Velocity Kinematics
static void dgvw_cmd_handler();

//Inverse Velocity Kinematics 
static void dsvw_cmd_handler();
static void dsvl_cmd_handler();

//Forward Position Kinematics
static void dodom_task(); //Task for odometry

static void dgp_cmd_handler();
static void dgpr_cmd_handler();
static void dgpr_task(); //Task for repeatedly calling get pose
static void df_task(); //Task for calling df
static void dgpst_cmd_handler(); //Stops repeated output
static void drp_cmd_handler();

//Inverse Position Kinematics
static void df_cmd_handler();
static void dt_cmd_handler();
static void dgs_cmd_handler();
static void drs_cmd_handler();
static void dri_cmd_handler();

//Stop-Resume
static void dst_cmd_handler();
static void drs_cmd_handler();

//Tests
void drive_square();
void boomerang();

/////////////////////////////////////////////
//  COMMANDS //
/////////////////////////////////////////////

float* dgvw();// (drive get v, w)
void dsvw(float v, float w); // (drive set v, w)
void dsvl(float v, float l); // (drive set v, l)
void dodom_task(); // (Start odometer)
float* dgp(); // (drive get pose)
void drp();   // (drive reset pose)
void df(float d);  // (drive forward)
void dt(float a); // (drive turn)
int dgs(); //(drive get status) 
void dst(); //(Drive stop)
void drs(); // (drive resume stop)
void dri(); // (drive stop and reinitialize)

/////////////////////////////////////////////
//  HELPER FUNCTIONS //
/////////////////////////////////////////////

void initState();
void dgpr_task();// (repeat dgpr)
void push_cmd(struct Drive_Cmd cmd); // (pushes command to queue)
struct Drive_Cmd pop_cmd(); // (Pops command from queue)
void driveRegisterCmds(); // Registers commands for minicom

/////////////////////////////////////////////
//  TEST FUNCTIONS //
/////////////////////////////////////////////
void drive_square();
void boomerang();

