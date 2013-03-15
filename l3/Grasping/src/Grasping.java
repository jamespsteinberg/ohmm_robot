
import ohmm.*;
import static ohmm.OHMM.*;
import ohmm.OHMM.AnalogChannel;
import static ohmm.OHMM.AnalogChannel.*;
import ohmm.OHMM.DigitalPin;
import static ohmm.OHMM.DigitalPin.*;

import ohmm.ConsoleNonblocking;
import java.io.*;

public class Grasping {
    boolean DEBUG = true;

    //The step distance we will be using for movement, in millimeters
    public double stepDist;
    
    private float[] gripW = {(float)0.0, (float)0.0, (float)0.0};  //Vector representing the grip pose, in world frame    
    private static final float SHOULDER_X = (float)38.0; //Distance between robot origin and shoulder joint in x-direction
    private static final float SHOULDER_Z = (float)91.0; //Distance between ground and shoulder joint, in mm
    
    private static final float L0 = (float)99.0; //Length of joint 0 (mm)
    private static final float L1 = (float)74.0; //Length of joint 1 (mm)
    
    private static final float WX = (float)74.0;
    private static final float WZ = (float)19.0;
    //Used to receive console input without blocking program
    private ConsoleNonblocking cnb;
    private OHMMDrive ohmm;
    private double goalX; //Goal x coordinate (mm) in world frame
    private double goalY;
    
    //0.938 works well on tile floor, 0.87 works well on carpet floor
    private static final double FRICTION_DRIVE_MULTIPLIER = 1.0; //When driving, compensate for friction so that desired distance is still driven
    private static final double FRICTION_TURN_MULTIPLIER = 1.15; //When turning, compensate for friction
    private static final double GRABBING_DISTANCE = 250; // object within grabbing distance

    
    //Constructors
    public Grasping(OHMM ohmm)
    {
        System.out.println("Starting Grasping Program");
        init(ohmm);
    }
   
    //Constructor that takes in goal coordinates
    public Grasping(OHMM ohmm, Double goalX, Double goalY)
    {
        System.out.println("Starting 3D Grasping Program");
	this.goalX = goalX;
	this.goalY = goalY;
        init(ohmm);
    }

    //Initialize stuff in here
    private void init(OHMM ohmm){
        this.cnb = new ConsoleNonblocking();
        this.stepDist = 5.0; //Initialize step distance to 5
        this.ohmm = (OHMMDrive)ohmm;
        
        this.ohmm.armEnable(true);  //ENABLE ARM
        
        //Reset drive pose
        this.ohmm.driveResetPose();        
        this.ohmm.armHome(); //Go to arm home pose
        while(ohmm.armActive()) {} //Wait for gripper to finish
        //Tell the gripper that it's in the home pose
        this.gripW[0] = 220; //I measured this approximately.  May need to be more exact
        this.gripW[1] = 0;
        this.gripW[2] = 140; //This may need to be more exact, also
        
    }
    
    
    /////////////////////
    //       RUN       //
    /////////////////////
    public void run() {
	// turn towards the object
	 float[] pose = ohmm.driveGetPose();
	System.out.println("pose[0]: " + pose[0] + " pose[1]: " + pose[1] + "pose[2]: " + pose[2]);
	 double homeToGoalAngle = Math.atan2((goalY - (double)pose[1]), (goalX - (double)pose[0]));
	 if (Double.isNaN(homeToGoalAngle))
                homeToGoalAngle = 0;
	System.out.println("hometogoalangle: " + homeToGoalAngle);
         ohmm.driveTurn((float)homeToGoalAngle);
         

        double distTravel =  Math.sqrt(Math.pow(goalX - pose[0], 2) + (Math.pow(goalY - pose[1], 2))) - GRABBING_DISTANCE;
        
	System.out.println("distTravel: " + distTravel);
	ohmm.driveStraight((float)distTravel);
	while(ohmm.driveGetQueue() != 0) { }	

	grabSequence();
	while(ohmm.armActive()) {}

	// turn back towards world frame (0, 0, 0)
	ohmm.driveTurn((float)(Math.PI));

	// drive forward to it
	ohmm.driveStraight((float)(distTravel * FRICTION_DRIVE_MULTIPLIER));

	// face forward again
	ohmm.driveTurn((float)((2 * Math.PI) - homeToGoalAngle - Math.PI));
    }


    public void grabSequence()
    {
        this.ohmm.armSetGripper((float)1.5); //Open the gripper
        while(ohmm.armActive()) {} //Wait for gripper to finish

        //Lower the arm with a z interpolation to within 2cm of ground
        moveGripper(0, 0, -80);
        while(ohmm.armActive()) {} //Wait for gripper to finish
        
       //Move arm forward with a robot-frame x 
        moveGripper(100, 0, 0);
        while(ohmm.armActive()) {} //Wait for gripper to finish
        
        this.ohmm.armSetGripper((float)0.0); //Close gripper
        while(ohmm.armActive()) {} //Wait for gripper to finish
  
    }

    public void endSequence() {
	//Return to home pose
        this.ohmm.armHome(); //Go to arm calibration pose
        while(ohmm.armActive()) {} //Wait for gripper to finish
        //Tell the gripper that it's in the home pose
        //TODO:  THESE NEED TO INCORPORATE THE NEW ROBOT POSE
        this.gripW[0] = 220;
        this.gripW[1] = 0;
        this.gripW[2] = 140;

        while(true){
            try{
                char c = (char)(cnb.getChar());
                if((int)c == ConsoleNonblocking.ESC) break;

            }catch(Exception e)
            {
                System.out.println("Encountered Exception");
                System.out.println(e.getMessage());
            }

        }

        System.out.println("Done.");
        this.ohmm.armEnable(false);
        System.exit(0);

    }

    
    
    
////////////////////////////////////////////////////////////////////////
    ///////////////////////////// MOVE GRIPPER /////////////////////////
    private void moveGripper(double x, double y, double z)
    {
        System.out.println("Moving gripper!");
        System.out.println("x: " + x + "   y: " + y + "   z: " + z); 
        
        
        gripW[0] += (float)x; gripW[1] += (float)y; gripW[2] += (float)z;
        float[] robotPose = ohmm.driveGetPose();
        
        gripW[0] -= robotPose[0]; gripW[1] -= robotPose[1];  //14 - Account for robot not being at origin
        
        double thetar = Math.atan2(gripW[1], gripW[0]); // 15
        
        //Calculate gripper pose in robot frame
        float gx = (float)(gripW[0] * Math.cos(thetar) + gripW[1] * Math.sin(thetar));
        float gy = (float)(-gripW[0] * Math.sin(thetar) + gripW[1] * Math.cos(thetar));
        float gz = gripW[2];
        float[] gripR = {gx, gy, gz};
        
        
        
        //The farthest the gripper can possibly reach is when both joints are extended fully
        //if(gripR[0] > SHOULDER_X + L0 + L1)
        //    gripR[0] = SHOULDER_X + L0 + L1;
        
        
        //Reach vector, in world frame       ((6) in lecture notes)
        float[] reachVector = {gripR[0] - WX - SHOULDER_X, 0, gripR[2] - WZ - SHOULDER_Z};
        double reachLength = Math.sqrt(Math.pow(reachVector[0], 2) + Math.pow(reachVector[2], 2));
        
        double c1 = (Math.pow(reachLength, 2) - Math.pow(L0, 2) - Math.pow(L1, 2)) / (2 * L0 * L1); //(8)
        
        double s1 = Math.sqrt(1 - Math.pow(c1, 2)); //(9)
        
        //Use inverse kinematics to calculate rotations of different joints
        double theta1 = Math.atan2(s1, c1); //Elbow joint rotation (10)
        if (Double.isNaN(theta1))
            theta1 = 0;
        
        double alpha = Math.atan2(-reachVector[2], reachVector[0]); //11
        if (Double.isNaN(alpha))
            alpha = 0;
        
        double cb = (Math.pow(L1, 2) - Math.pow(L0, 2) - Math.pow(reachLength, 2)) / (-2.0 * L0 * reachLength);
        double sb = Math.sqrt(1 - Math.pow(cb, 2));
        double beta = Math.atan2(sb, cb); //13
        if (Double.isNaN(beta))
            beta = 0;
        
        double theta0 = alpha - beta;//12
        
        double theta2 = -(theta0 + theta1); // 1
        
        
        if(DEBUG)
        {
            System.out.println("Setting shoulder joint to: " + theta0 + " radians.");
            System.out.println("Setting elbow joint to: " + theta1 + " radians.");
            System.out.println("Setting wrist joint to: " + theta2 + " radians.");
            System.out.println("Setting turn to: " + thetar + " radians.");
            
        }
        
        if(withinRotationBounds(theta0, theta1, theta2))
        {
            rotateSafely(0, theta0);
            rotateSafely(1, theta1);
            rotateSafely(2, theta2);
            
            while(ohmm.armActive()) {}
           // ohmm.driveOrientationServo((float)thetar);
        }
        else
        {
            System.out.println("Can't rotate that far!");
            gripR[0] -= (float)x; gripR[1] -= (float)y; gripR[2] -= (float)(z); //Arm didn't rotate, so put the gripper pose back
        }
        
        if(DEBUG)
        {
            System.out.println("--------------------------------------------");
            System.out.println("reachVector: [" + reachVector[0] + ", " + reachVector[1] + ", " + reachVector[2] + "]");
            System.out.println("gripR: [" + gripR[0] + ", " + gripR[1] + ", " + gripR[2] + "]");
            System.out.println("reachLength: " + reachLength);
            System.out.println("c1: " + c1 + "     s1: " + s1);
            System.out.println("cb: " + cb + "     sb: " + sb);
            System.out.println("alpha: " + alpha + "     beta: " + beta);
            System.out.println("theta0: " + theta0);
            System.out.println("theta1: " + theta1);
            System.out.println("theta2: " + theta2);
            System.out.println("thetar: " + thetar);
            System.out.println("--------------------------------------------");
            
        }
        
         gripW[0] += robotPose[0]; gripW[1] += robotPose[1];  //14 - Put back
    }
    
    //Sets rotation, without trying to go past range of motion
    private void rotateSafely(int joint, double rotation)
    {
       if(joint == 0)
       {
           if(rotation < -Math.PI / 2)
           {
               rotation = -Math.PI / 2;
           }
           else if(rotation > Math.PI / 4) rotation = Math.PI / 4;
       }
       else if(joint == 1)
       {
           if(rotation < -Math.PI * (5.0 / 9.0)) rotation = -Math.PI * (5.0 / 9.0); 
           else if(rotation > Math.PI * (5.0 / 6.0)) rotation = Math.PI * (5.0 / 6.0);
       }
       else if(joint == 2)
       {
           if(rotation < -Math.PI / 2) rotation = -Math.PI / 2;
           else if(rotation > Math.PI / 2) rotation = Math.PI / 2;
       }
        
        System.out.println("Setting joint " + joint + " to " + rotation + " radians.");
        ohmm.armSetJointRad(joint, (float)rotation);
    }
       
       
    private boolean withinRotationBounds(double theta0, double theta1, double theta2)
    {
        return theta0 >= -Math.PI / 2 && theta0 <= Math.PI / 4 &&
        theta1 >= -Math.PI * (5.0 / 9.0) && theta1 <= Math.PI * (5.0 / 6.0); //&&
       // theta2 >= -Math.PI /2 && theta2 <= Math.PI / 2;        
    }
   
    //////////////////////////////////////
    //               MAIN               //
    //////////////////////////////////////
    public static void main(String[] argv) throws IOException
    {
        //argv will either have length 1, (port given), or length 3.  (x, y, and port given)
        //If no x, y are given, then the program will revert to keyboard-controlled Inverse Kinematics
        OHMM ohmm = null;
        if(argv.length == 1) ohmm = OHMM.makeOHMM(new String[]{"-r", argv[0]});
        else if(argv.length == 3) ohmm = OHMM.makeOHMM(new String[]{"-r", argv[2]});
        else{
            System.out.println("Invalid arguments.  Expected [port], or [x, y, port]");
            System.exit(0);
        }
        
        //Make sure ohmm connected properly
        if (ohmm == null) {
            System.out.println("Could not connect to OHMM");
            System.exit(0);
        }

	if (argv.length == 1) {
	        Grasping grasping = new Grasping(ohmm);
	        grasping.grabSequence();
		grasping.endSequence();
	}
	else {
		Grasping grasping = new Grasping(ohmm, Double.parseDouble(argv[0]), Double.parseDouble(argv[1]));
		grasping.run();
		grasping.endSequence();
	}
    }
}
