/******************************************************************************
*                                                                              *
*                              ObjectDetect.java                               *
*                                                                              *
*******************************************************************************/
//This is the main class for Object Detection.
//This class extends CVBase, and processes incoming frames from the OHMM camera.
//CVBase allows these frames to be made available on an ImageServer, which can
//be viewed in a web browser by putting in [OHMM IP Address]:8080 as the URL.


import com.googlecode.javacpp.*;
import com.googlecode.javacv.*;
import com.googlecode.javacv.cpp.*;
import static com.googlecode.javacv.cpp.opencv_calib3d.*;
import static com.googlecode.javacv.cpp.opencv_contrib.*;
import static com.googlecode.javacv.cpp.opencv_core.*;
import static com.googlecode.javacv.cpp.opencv_features2d.*;
import static com.googlecode.javacv.cpp.opencv_flann.*;
import static com.googlecode.javacv.cpp.opencv_highgui.*;
import static com.googlecode.javacv.cpp.opencv_imgproc.*;
import static com.googlecode.javacv.cpp.opencv_legacy.*;
import static com.googlecode.javacv.cpp.opencv_ml.*;
import static com.googlecode.javacv.cpp.opencv_objdetect.*;
import static com.googlecode.javacv.cpp.opencv_video.*;

import ohmm.*;
import static ohmm.OHMM.*;
import ohmm.OHMM.AnalogChannel;
import static ohmm.OHMM.AnalogChannel.*;
import ohmm.OHMM.DigitalPin;
import static ohmm.OHMM.DigitalPin.*;

import java.lang.Exception;

public class ObjectDetect extends CvBase {

	private static final float L0 = (float)99.0; //Length of joint 0 (mm)
	private static final float L1 = (float)74.0; //Length of joint 1 (mm)
    
    private CvScalar hsvMin = cvScalar(0, 30, 80, 0); //TODO:  set this to work for pink color
    private CvScalar hsvMax = cvScalar(20, 150, 255, 255);
   
    
    //Set to true to go grasp object, or search for it if it isn't visible.
    //This will be set when user hits 'g' key, and should be done after
    //calibration is finished.
    private boolean GO = false;

    private boolean verifying = false; //Set to true when bot is verifying if grab was successful




    //Used to control when frames are processed and whatnot


    //For the visual servo stuff
    private OHMMDrive ohmm; 
    //Pixel coordinates clicked by the user.  (-1 if nothing has been clicked)
    private int[] clickedXY = new int[]{-1, -1};
    
    // Pixel coordinates of the blob center
    private int[] targetXY = new int[]{-1, -1};

    //The pixel xy coordinates of a successful grab
    private int[] successXY = new int[]{-1, -1};
    private boolean foundTarget = false; //Has the OHMM found the target?
    //Images
    private IplImage procImg;
    private IplImage blobImg;
    private IplImage combinedImg;
    private IplImage tempImg; //Image to be used for temporary purposes
   
    //VISUAL SERVO
    private VisualServo servo;
    //ROBOT POSE AND GRIPPER STUFF
    //private Grasp grasp;
    /** sets options for the demo **/
    public ObjectDetect(OHMM ohmm, VisualServo servo) {
        super("ObjectDetect");
        useWindow = false;
        useCanvasFrame = !java.awt.GraphicsEnvironment.isHeadless();
        useConsole = true;
        useServer = true;
        brightness = 0.3;
        maxFPS = 10;


        this.servo = servo;

        //INITIALIZE OHMM
        try{
           this.ohmm = (OHMMDrive)ohmm;
       /*    this.grasp = new Grasp(this.ohmm);
           this.grasp.init(); //Initialize grasp and go to home pose
           this.ohmm.armSetGripper(0); //Close gripper
           this.ohmm.driveResetPose(); */
        }
        catch(Exception e){
           System.out.println("Error initializing OHMM.");
           System.out.println(e.getMessage());
           System.exit(0);
        }
    }
    
    /** shows how to do special config **/
    public int initExt(int argc, String argv[], int ate) {
        //    v4l2DisableAuto();
        return ate;
    }


   //OVERRIDE PROCESS
   protected IplImage process(IplImage frame) {
           

       if(procImg == null)
       {
          procImg = IplImage.create(frame.width(), frame.height(), IPL_DEPTH_8U, 3);
       }
       if(blobImg == null)
       {
          blobImg = IplImage.create(frame.width(), frame.height(), IPL_DEPTH_8U, 1);
       }
       
       //Use handleMouse for click coordinates
       int x = -1, y = -1;
       synchronized(clickedXY)
       { x = clickedXY[0]; y = clickedXY[1]; }
       


       //COLOR CONVERSION
       cvCvtColor(frame, procImg, CV_BGR2HSV_FULL);//Convert to HSV so it's easier to determine hue to track
       
       //Set hsvMin and hsvMax if user has clicked on a point       
       if(frame != null && x >= 0 && y >= 0)
       {
           System.out.println("Clicked pixel: " + x + ", " + y);
           CvRect oldROI = cvGetImageROI(procImg);
           
           //Average 3x3 pixel square centered at click point
           cvSetImageROI(procImg, cvRect(x - 1, y-1, 3, 3));
           CvScalar avg = cvAvg(procImg, null);
           
           hsvMin.setVal(0, avg.getVal(0) - 20); //For now, we only differentiate based on hue
           hsvMin.setVal(1, 0);
           hsvMin.setVal(2, 0);
           
           hsvMax.setVal(0, avg.getVal(0) + 20);
           hsvMax.setVal(1, 255);
           hsvMax.setVal(2, 255);
           
           //Reset old roi
           cvSetImageROI(procImg, oldROI);
       }
       
       
       
       //SMOOTHING
       cvSmooth(procImg, procImg, CV_GAUSSIAN, 3, 3, 0, 0);   //Smooth blob image to reduce noise
       
       //THRESHOLDING
       
       //If hsvMin > hsvMax, then the range "crosses the boundary" from 360 to 0, and must be treated as the 
       //union of two ranges. TODO
      // if(hsvMin <= hsvMax)
      // {
       cvInRangeS(procImg, hsvMin, hsvMax, blobImg); //Binary blob image
       
       //Reduce noise
       cvErode(blobImg, blobImg, null, 1);
       cvDilate(blobImg, blobImg, null, 1);
       
       //Find contours
       CvMemStorage storage = CvMemStorage.create();
       CvSeq contour = new CvSeq();

       cvFindContours(blobImg, storage, contour, Loader.sizeof(CvContour.class), 
                      CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
       
       if(contour != null && !contour.isNull())
       {
	   // largest rectangle of the blobs in the list of contours
	   CvRect largest = null;
           for (CvSeq c = contour; c != null && !c.isNull(); c = c.h_next()) {  
		if (c.elem_size() > 0) {
		     // if largest is not set yet, set it to current contour
                     if (largest == null) {
			largest = cvBoundingRect(c, 0);			
		     }
		     // if largest is already set, compare it to current contour and set to largest
		     else {
			CvRect other = cvBoundingRect(c, 0);
			if ((largest.width() * largest.height()) < (other.width() * other.height())) {
				largest = other;
			}
		     }
		}
	    }
       
	// Set center of target Blob
	   targetXY[0] = largest.x() + largest.width() / 2;
	   targetXY[1]  = largest.y() + largest.height() / 2;
	   CvPoint center  = new CvPoint(targetXY[0], targetXY[1]);
    
           if((clickedXY[0] >= 0 && clickedXY[1] >= 0) || (successXY[0] >= 0 && successXY[1] >= 0))
           {
               foundTarget = true; //User has selected the blob to track, and the camera sees target
             
               if(clickedXY[0] >= 0 && clickedXY[1] >=0) //User has specified success position
               {
                  successXY[0] = targetXY[0];
                  successXY[1] = targetXY[1];
               }
           } 
           //Draw "+" sign on blob center
           CvPoint pt1 = new CvPoint(targetXY[0] - 10, targetXY[1]);
           CvPoint pt2 = new CvPoint(targetXY[0] + 10, targetXY[1]);
           CvPoint pt3 = new CvPoint(targetXY[0], targetXY[1] - 10);
           CvPoint pt4 = new CvPoint(targetXY[0], targetXY[1] + 10);
        /*  System.out.println("Bounding Box: (" + pt1.x() + ", " + pt1.y() + ") - (" + pt2.x() + ", " + pt2.y() + ")");  
        cvDrawRect(procImg, pt1, pt2, CvScalar.BLUE, 2, 8, 0);  */
           cvDrawLine(blobImg, pt1, pt2, CvScalar.WHITE, 1, 8, 0);
           cvDrawLine(blobImg, pt3, pt4, CvScalar.WHITE, 1, 8, 0);      
      }      
      else{  //The object isn't visible!
             foundTarget = false;
      }
      cvClearMemStorage(storage); 
      synchronized(clickedXY){  clickedXY[0] = -1; clickedXY[1] = -1;}
    
     
///////////////////////// OHMM SERVO /////////////////////////
     //Is servo currently verifying?
     if(servo.getState() == VisualServo.STATE.VERIFY){
        verifying = true;
     }
    
      if(GO && !verifying  && servo.getState() != VisualServo.STATE.RESET_ARM
          && servo.getState() != VisualServo.STATE.RETURN_HOME)
       {
        if(foundTarget)
        {
          double turnAmt = 0.0; //How fast we should turn, in radians
                 
          double xDif = Math.abs(targetXY[0] - successXY[0]);
          if(xDif > 50) //We're pretty far away, pixel-wise
          {
             turnAmt = 0.05;
          }
          else if(xDif > 25) //Closer
          {
             turnAmt = 0.02;
          }
          else{
             turnAmt = 0.01; //Reall close!  Rotate very slowly.
          }
 
          float theta = ohmm.driveGetPose()[2];
          if(targetXY[0] > successXY[0] + 25)
          {
            //  grasp.orientAndWait((float)(theta -  turnAmt));
            synchronized(this){
                servo.turn(-turnAmt);
            }
          }
          else if(targetXY[0] < successXY[0]- 25)
          {
             
            //  grasp.orientAndWait((float)(theta + turnAmt));
            
            synchronized(this){
                servo.turn(turnAmt);
            }
          }
          else
          {
            synchronized(this){
                
                servo.stopTurn(); //Stop the turn.
                
                //xDif is within a good range!  Drive forward a little bit
                double yDif = Math.abs(targetXY[1] - successXY[1]);
         
                float driveDist = (float)0.0;

                if(yDif > 100) //Trials indicate that target is more than a meter away in this case
                {
                    driveDist = (float)600.0;  //Drive 600 mm
                }
                else if(yDif > 50.0) //At least half a meter away from target
                {
                    driveDist = (float)200.0;
                }
                else if(yDif > 30) //About 300 mm from target
                {
                   driveDist = (float)100.0;
                }
                else if(yDif > 10){      //We're close.  Drive 5 cm
                   driveDist = (float)50.0;
                }
                else{
                   driveDist = (float)10.0; //1 cm at a time when we're this close
                }
                //Move forward
                if(targetXY[1] < successXY[1] - 4.0) //within 4 px of desired
                {
                   servo.driveForward(driveDist); //Drive forward
                }
                else if(targetXY[1] > successXY[1] + 4.0){
                  //Too close!  Back away slowly...
                  servo.driveForward(-10.0); //(1 cm)
                }
                else  //We should be within grabbing distance.  Grab it!
                {
                   servo.grabSequence();
                   
                }
            }
          }
        }
        else{  //SEARCH FOR TARGET
            System.out.println("Searching for target");
            servo.search();
        }
      } //End if GO && verifying
     else if(GO && verifying) 
     {
        //Wait for verification pose to finish
        if(servo.getState() == VisualServo.STATE.WAIT){ //Verification pose complete
           
           System.out.println("VERIFICATION");
           System.out.println("targetXY[1]: " + targetXY[1]);
           System.out.println("successXY[1]: " + successXY[1]);
           if(targetXY[1] < successXY[1] - 10) //ball was lifted
           {
               System.out.println("Successful Grab!");
               verifying = false;
               servo.returnHome();
           }
           else{
               System.out.println("Grab failed.  Trying again!");
               verifying = false;
               servo.resetArm();
           }

        }       
     }


 //cvInRangeS(blobImage, hsvMin, hsvMax, blobImage);
      IplImage[] imgs = {frame, procImg, blobImg};
      IplImage combined = combine(imgs);
/*    if(foundTarget)
       return blobImg;
    else
       return frame; 
  */   return combine(imgs);
    // return procImg;  
 }

    //OVERRIDE MOUSE HANDLING
    protected void handleMouse(int event, int x, int y, int flags) {
        
        if(event == CV_EVENT_LBUTTONDOWN)
            synchronized (clickedXY) { clickedXY[0] = x; clickedXY[1] = y; }
        super.handleMouse(event, x, y, flags);
    }
   
    //OVERRIDE KEY HANDLING
    protected boolean handleKey(int code)
    {
      switch (code) {

         case 'r' : case 'R': {  //Start the motor stuff
           GO = true;
           System.out.println("Go!");
           return true;
         }
         case 'h' : case 'H': {  //Stop the motor stuff
           GO = false;
           System.out.println("Stop!");
           return true;
          }
         case 'q' : case 'Q': {
           boolean result = super.handleKey(code);

           //End servo thread
           servo.finish();

           //System.exit(0); //quit
           return false;
         }
         case 'o' : case 'O': {
             GO = false;
             System.out.println("Open gripper!");
             servo.openGripper();
             return true;
         }
         case 'p' : case 'P': { 
             GO = false;
             System.out.println("Close gripper!");
             servo.closeGripper();
             return true;
         }
       }
     
      return super.handleKey(code);
    }

   //Combine IplImages into a single image, vertically stacked
   //This method kills framerate, but is good for debugging.
   private IplImage combine(IplImage[] imgs){
       
       
       int height = 0;
       int width = 0;
       
       for(int i = 0; i < imgs.length; i++)
       {
           height = height + imgs[i].height();
           width = Math.max(width, imgs[i].width());
       }
       
       if(combinedImg == null)
       {
         //Initialize combined image
         combinedImg = IplImage.create(width, height, IPL_DEPTH_8U, 3);
       }
       
       int offset = 0;
      
       cvZero(combinedImg); 
       for(IplImage img : imgs)
       {
           cvSetImageROI(combinedImg, cvRect(0, offset, img.width(), img.height()));
           if(img.nChannels() == 3)
           {
              cvCopy(img, combinedImg);
           }
           else if(img.nChannels() == 1)  //Grayscale
           { 
              if(tempImg == null)
                 tempImg = IplImage.create(img.width(), img.height(), IPL_DEPTH_8U, 3);
              
              cvCvtColor(img, tempImg, CV_GRAY2BGR);
              cvCopy(tempImg, combinedImg);
           }   
           cvResetImageROI(combinedImg);
           offset = offset + img.height();
       }
       
       return combinedImg;
  } 
    
    /** Program entry point. **/
    public static void main(String argv[]) {
        try{
           OHMM ohmm = OHMM.makeOHMM(new String[]{"-r", "/dev/ttyACM1"});
          
           //Run servo in new thread
           VisualServo servo = new VisualServo(ohmm);
           Thread servoThread = new Thread(servo);
           servoThread.start();
 
           ObjectDetect d = new ObjectDetect(ohmm, servo);
           System.out.println("ate "+d.init(argv.length, argv)+" args");
           d.mainLoop();
           d.release();
        }
        catch(Exception e){
           System.out.println("Error initializing OHMM.");
           System.out.println(e.getMessage());
           System.exit(0);
        }

       // CvSeq contour = new CvSeq(null);
       // cvFindContours(grayImage, storage, contour, Loader.sizeof(CvContour.class),
       //     CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
       System.out.println("Done.");
       System.exit(0);
    }
}
