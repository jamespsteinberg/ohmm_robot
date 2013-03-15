/**
 * <p>OHMMShell Demo.</p>
 *
 * <p>This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the Free
 * Software Foundation; either version 2 of the License, or (at your option)
 * any later version.</p>
 *
 * <p>This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.</p>
 *
 * <p>You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc., 51
 * Franklin St, Fifth Floor, Boston, MA 02110-1301 USA</p>
 *
 * <p>Copyright (c) 2011 Marsette Vona</p>
 **/

//package l2;

import ohmm.*;
import static ohmm.OHMM.*;
import ohmm.OHMM.AnalogChannel;
import static ohmm.OHMM.AnalogChannel.*;
import ohmm.OHMM.DigitalPin;
import static ohmm.OHMM.DigitalPin.*;

import java.io.*;

/**
 * <p>Group 8 Lab 2 - Local and Global Navigation</p>
 *
 *
 * 
 **/
public class OhmmNavigation{
    
    private static final String svnid =
    "$Id: OHMMShellDemo.java 360 2013-02-06 19:28:55Z vona $";
    
    /**
     * <p>Entry point, see {@link ohmm.OHMM#USAGE}.</p>
     **/
    
    private OhmmMapServer server;
    private ServerRunner runner;
    private OHMMDrive ohmm;
	
    ////////////////////////////////////////////////
	// constants
    ////////////////////////////////////////////////
    // normal robot speed ==  mm/sec
	private static final float ROBOT_SPEED = (float)50; 
	
	// for robot checking for corners -- mm/sec
	private static final float ROBOT_SPEED_SLOW = (float)20;
	
	// from the goal -- 5 cm
	private static final double ERROR_AMT = 0.05;
	
	// moves robot a little bit, space from the obstacles
	private static final float PARTIAL_FORWARD = (float)175; 
	
	// adjusts the robot so it faces obstacle by this amount -- in radians
	private static final float PARTIAL_TURN = (float)(Math.PI / 20); 
	
	 // robot should stay x mm from wall
	private static final float WALL_DISTANCE = 185;
	
	// amount IR sensors can be off by
	private static final float IR_ERROR = 15; // 25 mm
	
	// distance from wall above which is large enough to make a turn
	// below which is little enough that the robot will turn and try 
	// to follow obstacle still
	private static final float TURN_LIMIT = 300; // 300 mm
	
	private static enum BUG2_STATE{ // different states of bug2 algorithm
	        DRIVE_FORWARD,
	        OBSTACLE_ENCOUNTERED,
	        BACKING_UP,
	        READ_IRS,
	        CORNER_DETECTED,
	        LEAVE_POINT_REACHED,
	        GOAL_REACHED
	        
	    }
	
	private static enum BUMPER{ // all possibilities of bumpers being hit
		LEFT,
		RIGHT,
		BOTH
	}
	

    ////////////////////////////////////////////////
	// global variables
    ////////////////////////////////////////////////
	
	private double robotX = 0.0; // X position of robot
	private double robotY = 0.0; // Y position of robot
	private double robotT = 0.0; // Theta of robot
	private double goalX = 0; // X position of goal
	private double goalY = 0;  // Y position of goal 
    private double leaveLineSlope = 0; //m, the slope of the leave line
    private double leaveLineYIntersect = 0; // b, the y intersect of leave line
	
    // what bumper was hit first when hitting an obstacle
	private BUMPER bumpedFirst;
	
	// bump Mode allows the robot to know when to move along an obstacle 
	//in the rare case both bumpers can not be hit at once
    private boolean bumpMode = false; 
    
    // whether the bot is currently following a wall
    private boolean followWall = false;
    
    //////////////////////MAIN//////////////////////////////////
    public static void main(String[] argv) throws IOException {
        
        //to use argv for your own purposes, one option is to replace the above
        //call with a hardcoded version like this:
        
        
        //to use this file as a template:
        //
        //1 - rename the file to YourClass.java
        //
        //2 - change the class name above from OHMMShellDemo to YourClass
        //
        //3 - change the author info and other documentation above
        //
        //4 - comment out the next two lines
        //
        //5 - write whatever code you want here that makes use of the ohmm object
        
        //OHMMShell shell = new OHMMShell(ohmm);
        //shell.readEvalPrintLoop();
        if(argv.length < 1)
        {
        	printUsage();
        }
        else
        {
            OHMMDrive ohmm = (OHMMDrive)OHMM.makeOHMM(new String[]{"-r", "/dev/ttyACM1"});
            
            if (ohmm == null) System.exit(0);
            
	        double dist = Double.parseDouble(argv[0]);
	        OhmmNavigation ohmmNav = new OhmmNavigation(dist, ohmm);
	        ohmmNav.runNavLoop();  
        }
        //ohmm.close();
    }
    
    //CONSTRUCTOR
    public OhmmNavigation(double distanceFromGoal, OHMMDrive ohmm)
    {
        goalX = distanceFromGoal;
        this.ohmm = ohmm;
    }
    
    public void runNavLoop()
    {
    	this.runner = new ServerRunner(this.ohmm, this.goalX);
    	Thread serverThread = new Thread(runner);
    	serverThread.start();
    	
    	System.out.println("Proceeding in new thread.");
    	bug2Nav();
    }
    
    //PRINT USAGE
    private static void printUsage()
    {
    	System.out.println("Usage: OhmmNavigation <goal-x-coor (meters)>");
    }
    //////////////////////////////////////////////////////
    //                    BUG 2                         //
    //////////////////////////////////////////////////////
    
    //////////////////////////////////////////////////////
    //Implement the bug2 local navigation algorithm
    private void bug2Nav()
    {
    	
    	ohmm.driveResetPose();
    	ohmm.senseConfigDigital(IO_A0, true, false);
    	ohmm.senseConfigDigital(IO_A1, true, false);
    	BUG2_STATE state = BUG2_STATE.DRIVE_FORWARD;
    	
    	while(state != BUG2_STATE.GOAL_REACHED) //Continue loop until goal is reached
    	{
            if(state == BUG2_STATE.DRIVE_FORWARD) {
            	state = bug2DriveForward();
            }
            else if(state == BUG2_STATE.OBSTACLE_ENCOUNTERED) {
            	state = bug2ObstacleEncountered();
            }
            else if(state == BUG2_STATE.BACKING_UP) {
            	state = bug2BackingUp();
            }
            else if(state == BUG2_STATE.READ_IRS) {
            	state = bug2ReadIrs();
            }
            else if(state == BUG2_STATE.CORNER_DETECTED) {
            	state = bug2CornerDetected();
            }
            else if(state == BUG2_STATE.LEAVE_POINT_REACHED) {
            	state = bug2LeavePointReached();
            }
    	}
    	
    }

    ////////////////////////////////////////////////////// 
    //Handle the algorithm while driving forward
    private BUG2_STATE bug2DriveForward()
    {
    	//1.  Set robot speed to some constant value
    	//2.  If goal has been reached, return BUG2_STATE.GOAL_REACHED
    	//3.  If a bump sensor goes off, return BUG2_STATE.OBSTACLE_ENCOUNTERED
    	if (getGoalDist() < ERROR_AMT) {
    		ohmm.driveSetVW(new Float(0), new Float(0));
			return BUG2_STATE.GOAL_REACHED;
		}
    	else if (bumpedLeft() && bumpedRight()){
    		bumpedFirst = BUMPER.BOTH;
    		return BUG2_STATE.OBSTACLE_ENCOUNTERED;
    	}
		else if (bumpedLeft() || bumpedRight()){
			return BUG2_STATE.OBSTACLE_ENCOUNTERED;
		}
		else {
			ohmm.driveSetVW(ROBOT_SPEED, new Float(0));
			return BUG2_STATE.DRIVE_FORWARD;
		}
    }	
	
	//////////////////////////////////////////////////////
	//prepares the actions if left bumper is activated
	private Boolean bumpedLeft() 
	{
		if (ohmm.senseReadDigital(IO_A0)) {
			if (!bumpMode){
				if (getRightBump()) {
					bumpedFirst = BUMPER.BOTH;
				}
				else {
					bumpedFirst = BUMPER.LEFT;
				}
			}
			return getLeftBump();
		}
		return false;
	}

	//////////////////////////////////////////////////////
	//Returns true if left bump switch is bumped
	private Boolean getLeftBump(){
		return ohmm.senseReadDigital(IO_A0); //left bump
	}
	
	//////////////////////////////////////////////////////
//	prepares the actions if right bumper is activated
	private Boolean bumpedRight()
	{
		if (ohmm.senseReadDigital(IO_A1)) {
			if (!bumpMode){
				if (getLeftBump()) {
					bumpedFirst = BUMPER.BOTH;
				}
				else {
					bumpedFirst = BUMPER.RIGHT;
				}
			}
			return getRightBump();
		}
		return false;
	}
	
	//////////////////////////////////////////////////////
	//Returns true if left bump switch is bumped
	private Boolean getRightBump(){
		return ohmm.senseReadDigital(IO_A1); //right bump
	}
	
	
	//////////////////////////////////////////////////////
	//Get the robot pose
	private void getPose()
	{
		float[] pose = ohmm.driveGetPose();
        robotX = (double)pose[0] / 1000;
        robotY = (double)pose[1] / 1000;
        robotT = (double)pose[2];
	}
	
	//////////////////////////////////////////////////////
	//Return the distance in meters from the by calculating a goal line
	private Double getGoalDist()
	{
        getPose();
		return Math.sqrt((robotX-goalX)*(robotX-goalX) + (robotY-goalY)*(robotY-goalY));
	}
	
    //////////////////////////////////////////////////////
    //Handle the algorithm when an obstacle is encountered
    private BUG2_STATE bug2ObstacleEncountered()
    {
        if(this.runner != null)
        {
            runner.mapAddObstaclePoint(robotX, robotY);   
        }
        System.out.println(bumpedFirst);
		if (bumpedFirst == BUMPER.BOTH || (bumpedFirst == BUMPER.RIGHT && bumpedLeft()) || (bumpedFirst == BUMPER.LEFT && bumpedRight())) {
			//Next we will back up a fixed amount
			System.out.println("bumper is: " + bumpedFirst);
			System.out.println("starting backing up");
			bumpMode = false;
			return BUG2_STATE.BACKING_UP;
		}
		getPose();
    	//1.  "Line up" to obstacle until both obstacles are triggered
		if (bumpedFirst == BUMPER.LEFT) {
			System.out.println("waiting for right bumper hit");
				faceObstacle(PARTIAL_TURN);
				bumpMode = true;
		}
		else if (bumpedFirst == BUMPER.RIGHT) {
			System.out.println("waiting for left bumper hit");
				faceObstacle(-PARTIAL_TURN);
				bumpMode = true;
		}
		if (followWall){
			return BUG2_STATE.READ_IRS;
		}
		else {
			return BUG2_STATE.DRIVE_FORWARD;
		}
    }
    
    
    //////////////////////////////////////////////////////
    //turns bit by bit until both sensors are flat on the obstacle
    private void faceObstacle(float turnAmt){
    	ohmm.driveStraight(-PARTIAL_FORWARD);
    	ohmm.driveTurn(turnAmt);
    	while (ohmm.driveGetQueue() != 0){
			;
    	}
    }
    
    
    //////////////////////////////////////////////////////
    //Handle the algorithm when the OHMM is backing up from an obstacle
    private BUG2_STATE bug2BackingUp()
    {
	    //1.  Back up a fixed amount (25 cm)
		ohmm.driveStraight(-PARTIAL_FORWARD);
	    //2.  Turn left π/2 radians
		ohmm.driveTurn(new Float(Math.PI / 2));
		while (ohmm.driveGetQueue() != 0){
				;
		}
        return BUG2_STATE.READ_IRS;
    }
    //////////////////////////////////////////////////////
    //Handle the algorithm when the OHMM starts reading IRs
    private BUG2_STATE bug2ReadIrs()
    {
    	
    	//1.  Start driving forwards slowly, while trying to maintain distance and parallelism to the wall 
    	ohmm.driveSetVW(ROBOT_SPEED_SLOW, new Float(0));
    	//2.  Monitor IR readings for obstacle corners
    	//3.  If corner is detected, return BUG2_STATE.CORNER_DETECTED
    	
    	if (isLeaveLine()){
    		return BUG2_STATE.LEAVE_POINT_REACHED;
    	}
    	else if ((getFrontSensor() >= (TURN_LIMIT + IR_ERROR)) && (getBackSensor() >= (TURN_LIMIT + IR_ERROR))) {
    		return BUG2_STATE.CORNER_DETECTED;
    	}
    	else if (getFrontSensor() >= (WALL_DISTANCE + IR_ERROR)) {
    		System.out.println("Shifting right");
    		ohmm.driveTurn(-PARTIAL_TURN);
    		while (ohmm.driveGetQueue() != 0){
    			;
    		}
    		System.out.println("done shifting");

    	}
    	else if (getFrontSensor() <= (WALL_DISTANCE - IR_ERROR)) {
    		System.out.println("shifting left");
    		ohmm.driveTurn(PARTIAL_TURN);
    		while (ohmm.driveGetQueue() != 0){
    			;
    		}
    		System.out.println("done shifting");
    	}
    	
    	
    	if (bumpedLeft() && bumpedRight()){
    		bumpedFirst = BUMPER.BOTH;
    		followWall = true;
    		return BUG2_STATE.OBSTACLE_ENCOUNTERED;
    	}
		else if (bumpedLeft() || bumpedRight()){
			followWall = true;
			return BUG2_STATE.OBSTACLE_ENCOUNTERED;
		}
	
		System.out.println("Front: " + getFrontSensor());
		System.out.println("Back: " + getBackSensor());
    	return BUG2_STATE.READ_IRS;
    }
    
	//////////////////////////////////////////////////////
	//sets the line between the robot and the goal
	private void setLeaveLine() {
		getPose();
		leaveLineSlope = (robotX - goalX) / (robotY - goalY);
		leaveLineYIntersect = goalY - (leaveLineSlope * goalX);
	}
	
	//////////////////////////////////////////////////////
	//determines whether the robot is currently on the leave line
	private boolean isLeaveLine() {
		getPose();
		return (robotY <= ((leaveLineSlope * robotX) + leaveLineYIntersect + ERROR_AMT)) && (robotY >= ((leaveLineSlope * robotX) + leaveLineYIntersect - ERROR_AMT));
	}
	    
    //////////////////////////////////////////////////////
    //returns output of front IR sensor
    private float getBackSensor() {
		ohmm.senseConfigAnalogIR(CH_2, 1);
		return ohmm.senseReadAnalog(CH_2);
    }
    
	//////////////////////////////////////////////////////
	//returns output of back IR sensor
	private float getFrontSensor() {
		ohmm.senseConfigAnalogIR(CH_3, 1);
		return ohmm.senseReadAnalog(CH_3);
	}
    
    //////////////////////////////////////////////////////
    //Handle the algorithm when the OHMM has detected a corner
    private BUG2_STATE bug2CornerDetected()
    {
    	//1.  Drive forward a fixed amount, and turn -π/2 so that the robot will be the same
    	//distance from the wall
		ohmm.driveStraight(PARTIAL_FORWARD);
		ohmm.driveTurn(new Float(-Math.PI / 2));
		while (ohmm.driveGetQueue() != 0){
			;
		}
    	return BUG2_STATE.READ_IRS;
    }
    //////////////////////////////////////////////////////
    //Handle the algorithm when the OHMM has reached leave point
    private BUG2_STATE bug2LeavePointReached()
    {	
    	getPose();
		double deltaX = goalX - robotX;
		double deltaY = goalY - robotY;
		double goalT = Math.atan2(deltaY, deltaX);
		if (robotT - goalT > ERROR_AMT) {
			ohmm.driveTurn(new Float((2 * Math.PI) - Math.abs(goalT - robotT)));
		}
		else if (goalT - robotT > ERROR_AMT) {
			ohmm.driveTurn(new Float(goalT - robotT));
		}
		followWall = false;
    	return BUG2_STATE.DRIVE_FORWARD;
    }
    
    
}
