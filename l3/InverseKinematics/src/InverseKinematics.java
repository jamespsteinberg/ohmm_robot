//This class is used to run inverse kinematics with the OHMM bot

import ohmm.*;
import static ohmm.OHMM.*;
import ohmm.OHMM.AnalogChannel;
import static ohmm.OHMM.AnalogChannel.*;
import ohmm.OHMM.DigitalPin;
import static ohmm.OHMM.DigitalPin.*;

import ohmm.ConsoleNonblocking;
import java.io.*;

public class InverseKinematics
{
    boolean DEBUG = true;

    //The step distance we will be using for movement, in millimeters
    public double stepDist;

    private double STEP_MAX = 20.0;
    private double STEP_MIN = 5.0;
    
    private float[] gripW = {(float)0.0, (float)0.0, (float)0.0};  //Vector representing the grip pose, in world frame
    private float[] robotPose = {(float)0.0, (float)0.0, (float)0.0}; //Robot pose, in world frame
    private float gripperState = (float)0.0; //0.0 = closed, 2.0 = fully open
    private double GRIPPER_STEP = 0.25;
    
    private static final float SHOULDER_X = (float)38.0; //Distance between robot origin and shoulder joint in x-direction
    private static final float SHOULDER_Z = (float)91.0; //Distance between ground and shoulder joint, in mm
    
    private static final float L0 = (float)99.0; //Length of joint 0 (mm)
    private static final float L1 = (float)74.0; //Length of joint 1 (mm)
    
    //Used to receive console input without blocking program
    private ConsoleNonblocking cnb;
    private OHMMDrive ohmm;
    //Constructor
    public InverseKinematics(OHMM ohmm)
    {
        System.out.println("Starting Inverse Kinematics Program");
        init(ohmm);
    }
    
    //Initialize stuff in here
    private void init(OHMM ohmm){
        this.cnb = new ConsoleNonblocking();
        this.stepDist = 5.0; //Initialize step distance to 5
        this.ohmm = (OHMMDrive)ohmm;
        
        this.ohmm.armEnable(true);  //ENABLE ARM
        this.ohmm.armCal(); //Go to arm home pose
        
        //Reset drive pose
        this.ohmm.driveResetPose();
        this.robotPose[0] = (float)0.0; this.robotPose[1] = (float)0.0; this.robotPose[2] = (float)0.0;
        
        //Tell the gripper that it's in the calibration pose
        this.gripW[0] = SHOULDER_X + L0 + L1;
        this.gripW[1] = 0;
        this.gripW[2] = SHOULDER_Z;
    }
    
    
    /////////////////////
    //       RUN       //
    /////////////////////
    public void run()
    {
        
        
        System.out.println("Waiting for input.  Press ESC to exit");
        while(true){
           try{                char c = (char)(cnb.getChar());
               
            
               //Move gripper if input is received
               if(c == 'w')  moveGripper(stepDist, 0, 0);       // + stepDist mm in x      
               else if(c == 's') moveGripper(-stepDist, 0, 0);  // - stepDist mm in x
               else if(c == 'a') moveGripper(0, stepDist, 0);  // + stepDist mm in y
               else if(c == 'd') moveGripper(0, -stepDist, 0);  // - stepDist mm in y
               else if(c == 'r') moveGripper(0, 0, stepDist);   // + stepDist mm in z
               else if(c == 'f') moveGripper(0, 0, -stepDist);   // - stepDist mm in z
               else if(c == 'q') {
	       		stepDist = Math.min(stepDist + 5.0, STEP_MAX); //Increase step distance
			System.out.println("New step distance: " + stepDist);
		    }
               else if(c == 'z') {
			stepDist = Math.max(stepDist -5.0, STEP_MIN);   //Decrease step distance
               		System.out.println("New step distance: " + stepDist);
		    }
               
               else if(c == 'g'){
                   gripperState = (float)(Math.max(0.0, gripperState - GRIPPER_STEP));
                   ohmm.armSetGripper((float)gripperState); //Open gripper
               }
               else if(c == 'h')
               {
                   gripperState = (float)(Math.min(2.0, gripperState + GRIPPER_STEP));
                   ohmm.armSetGripper((float)gripperState); //Close gripper
               }
               
               
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

    ///////////////////////////// MOVE GRIPPER /////////////////////////
    private void moveGripper(double x, double y, double z)
    {
        System.out.println("Moving gripper!");
        System.out.println("x: " + x + "   y: " + y + "   z: " + z); 
        
        
        gripW[0] += (float)x; gripW[1] += (float)y; gripW[2] += (float)z;
        
        gripW[0] -= robotPose[0]; gripW[1] -= robotPose[1];  //14 - Account for robot not being at origin

        double thetar = Math.atan2(gripW[1], gripW[0]); // 15
        
        //Calculate gripper pose in robot frame
        float gx = (float)(gripW[0] * Math.cos(thetar) + gripW[1] * Math.sin(thetar));
        float gy = (float)(-gripW[0] * Math.sin(thetar) + gripW[1] * Math.cos(thetar));
        float gz = gripW[2];
        float[] gripR = {gx, gy, gz};


        
        //The farthest the gripper can possibly reach is when both joints are extended fully
        if(gripR[0] > SHOULDER_X + L0 + L1)
            gripR[0] = SHOULDER_X + L0 + L1;
        
        
        //Reach vector, in world frame       ((6) in lecture notes)
        float[] reachVector = {gripR[0] - robotPose[0] - SHOULDER_X, 0, gripR[2] - robotPose[2] - SHOULDER_Z};
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

        ohmm.driveOrientationServo((float)thetar);
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
         //  if(rotation < -Math.PI / 2) rotation = -Math.PI / 2;
           //else if(rotation > Math.PI / 2) rotation = Math.PI / 2;
       }
        
        System.out.println("Setting joint " + joint + " to " + rotation + " radians.");
        ohmm.armSetJointRad(joint, (float)rotation);
    }
       
       
    private boolean withinRotationBounds(double theta0, double theta1, double theta2)
    {
        return theta0 >= -Math.PI / 2 && theta0 <= Math.PI / 4 &&
        theta1 >= -Math.PI * (5.0 / 9.0) && theta1 <= Math.PI * (5.0 / 6.0);// &&
       // theta2 >= -Math.PI /2 && theta2 <= Math.PI / 2;
        
    }
    
    
    
    

    //////////////////////////////////////
    //               MAIN               //
    //////////////////////////////////////
    public static void main(String[] argv) throws IOException
    {
        //OHMM ohmm = OHMM.makeOHMM(argv);
        OHMM ohmm = OHMM.makeOHMM(new String[]{"-r", argv[0]});
        if (ohmm == null) {
            System.out.println("Could not connect to OHMM");
            System.exit(0);
        }
        InverseKinematics IK = new InverseKinematics(ohmm);
        IK.run();
    }
}
