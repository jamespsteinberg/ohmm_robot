import javax.swing.JPanel;
import java.awt.geom.AffineTransform;
import java.awt.image.BufferedImage;
import java.awt.Font;
import java.awt.Graphics2D;
import java.awt.Image;
import java.awt.Color;
import java.util.ArrayList;

//This class is responsible for drawing an OhmmMap to the screen
public class MapPanel extends JPanel{
    
    private Image screenBuffer; //Use double-buffering to render graphics
	private Graphics2D bufferG; //Graphics for the screen buffer
    
	//JPanel dimensions, in pixels
	private int panelWidth; private int panelHeight;
	
	//World boundaries, in meters, in global frame
	private double worldXMin; private double worldYMin;
	private double worldXMax; private double worldYMax;
	
	private double worldDistX; private double worldDistY; //Total distance, in meters, represented by map
	
	private int originPxX; //How many pixels from the left of screen origin should be drawn
	private int originPxY; //How many pixels from top of screen origin should be drawn
	
	//BufferedImages
    private BufferedImage robotImage;
    private BufferedImage goalImage;
    
	//Robot pose
	private double robotX = 0.0; //(meters)
	private double robotY = 0.0; //(meters)
	private double robotTheta = 0.0; //(radians)
    
    //Goal pose
    private double goalX = 0.0; //(meters)
    private double goalY = 0.0; //(meters)
    private double goalTheta = 0.0; //(radians)
    private int goalPxX = 0; //Pixel coordinates, x
    private int goalPxY = 0; //Pixel coordinates, y
	
	//Obstacle data points, each given by an x and y pixel coordinate
	private ArrayList<int[]> obstacles;
	
	//FONTS
	private Font tinyFont = new Font("Arial", 8, 8);
	private Font smallFont = new Font("Arial", 10, 10);
	private Font axesFont = new Font("Arial", 12, 12);
	
	//COLORS
	private Color bgColor = Color.getHSBColor(0, 0, (float)0.62);
	private Color xAxisColor = Color.getHSBColor(0, (float)(0.56), 1);
	private Color yAxisColor = Color.getHSBColor((float)(125.0 / 360.0), (float)(0.56), 1);
	private Color gridColor = Color.getHSBColor(0, 0, (float)(0.56));
	private Color gridLabelColor = Color.getHSBColor(0, 0, (float)(0.37));
	private Color obstacleColor = Color.black;
	private Color robotColor = Color.getHSBColor(0, 0, (float)(0.28));
	private Color goalColor = Color.getHSBColor((float)(121.0 / 360.0), (float)(0.71), (float)0.68);
	
	//User gets one System.out.println warning if they give units in millimeters
	private boolean warnedConversion = false; 
	
	public MapPanel(int pxWidth, int pxHeight, double worldXMin, double worldYMin,
                    double worldXMax, double worldYMax)
	{
        //Set panel dimensions
        this.panelWidth = pxWidth;
        this.panelHeight = pxHeight;
        
        this.worldXMin = worldXMin;
        this.worldYMin = worldYMin;
        this.worldXMax = worldXMax;
        this.worldYMax = worldYMax;
        
        this.worldDistX = worldXMax - worldXMin;
        this.worldDistY = worldYMax - worldYMin;
        
        //Initialize obstacle Arraylist.
        //Each obstacle is represented as a pair of integers, representing the x and y
        //pixel coordinates on the screen
        this.obstacles = new ArrayList<int[]>();
        
        
        originPxX = (int)((Math.abs((worldDistX - worldXMax)) / worldDistX) * panelWidth);
        originPxY = panelHeight / 2;
        System.out.println("originPxX: " + originPxX);
        //originPxX = panelWidth / 2;
        
        
        initRobotImage();
        initGoalImage();  
        
	}
	
	//Initialize BufferedImage robotImage 
	private void initRobotImage()
	{
        this.robotImage = new BufferedImage(50, 50, BufferedImage.TYPE_INT_ARGB);
        Graphics2D g2 = (Graphics2D)robotImage.getGraphics();
        g2.setColor(robotColor);
        g2.fillRect(14, 14, 22, 22);
        
        //Wheels
        g2.setColor(Color.black);
        g2.fillRect(28, 14, 8, 4);
        g2.fillRect(28, 32, 8, 4);
        
        //Axes
        g2.setColor(Color.red); 
        g2.drawLine(30, 25, 50, 25);  //x-axis
        
        g2.setColor(Color.green);
        g2.drawLine(30, 25, 30, 5); //y-axis
	}
	
	//Initialize BufferedImage goalImage
	private void initGoalImage()
	{
		this.goalImage = new BufferedImage(50, 50, BufferedImage.TYPE_INT_ARGB);
		Graphics2D g2 = (Graphics2D)goalImage.getGraphics();
		g2.setColor(goalColor);
		g2.fillRect(10, 10, 30, 30);
		
		//Orientation axes
		g2.setColor(Color.red);
		g2.drawLine(30, 25, 50, 25);
		g2.setColor(Color.green);
		g2.drawLine(30, 25, 30, 5);
	}
	
    
    //Initialize the viewer
    public void init()
    {
        this.setSize(panelWidth, panelHeight + 150);
        this.setVisible(true); //Display the map frame   
        
        this.screenBuffer = this.createImage(panelWidth, panelHeight);
        this.bufferG = (Graphics2D)screenBuffer.getGraphics();
        
        clearBuffer();
        
    }
    
    //Returns the pixel that corresponds to the given x coordinate in world frame
    private int worldToPxlX(double x)
    {
    	return (int)((x / worldDistX) * panelWidth) + originPxX;
    }
    
    //Returns the pixel that corresponds to the given y coordinate in world frame
    private int worldToPxlY(double y)
    {
    	return + originPxY - (int)((y / worldDistY) * panelHeight);
    }
    
    
    ///////////////////////////////// UPDATE ////////////////////////////////
    public void updateRobot(double x, double y, double theta)
    {
        if(x > 20.0 || y > 20.0)
        {
	       	//x and y were probably given in millimeters, so convert into meters.
	       	if(!warnedConversion)
	       	{
	       		System.out.println("drawRobot: Warning: received x or y value > 20.0.  Assuming value to be given in mm, not meters.");
	       		warnedConversion = true;
	       	}
            robotX = x / 1000.0;
            robotY = y / 1000.0;
            robotTheta = theta;
        }
        else
        {
            robotX = x;
            robotY = y;
            robotTheta = theta;
        }   
    }
    
    
    //Register a new obstacle
    public void addObstacle(double x, double y)
    {
    	int pxX = worldToPxlX(x);
    	int pxY = worldToPxlY(y);
    	
    	int[] pos = {pxX, pxY};
    	this.obstacles.add(pos);
    }
	
    //Register goal pose
    public void setGoal(double x, double y, double theta)
    {
    	this.goalX = x;
    	this.goalY = y;
    	this.goalTheta = theta;
    	this.goalPxX = worldToPxlX(goalX) - robotImage.getWidth() / 2;
    	this.goalPxY = worldToPxlY(goalY) - robotImage.getHeight() / 2;
    }
    
    
    /////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////
    //                             DRAWING                                 // 
    /////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////
    
    //Draw scene
    public void drawScene()
    { 
        clearBuffer(); //Clear the buffer to prepare the new frame
        drawGoal(); //Re-draw the goal
        drawRobot();
        drawOverlays(); //Re-draw the overlays
        drawObstacles(); //Re-draw obstacles
        showBuffer();
    }
    //Display the buffer
    private void showBuffer()
    {
        this.getGraphics().drawImage(screenBuffer, 0, 0, null);
    }
    
    //Clear the buffer, to just the background
    private void clearBuffer()
    {
    	bufferG.setColor(bgColor);
    	bufferG.fillRect(0, 0, panelWidth, panelHeight);
    }
    
    /////////////////////////////////////////////////////////////////////////
    //////////////////////
    //    DRAW ROBOT    //
    //////////////////////
    //Draw the robot in the frame.
    //Note:  x, y are given in METERS
    private void drawRobot()
    {        
        
        int ulX = worldToPxlX(robotX);
        int ulY = worldToPxlY(robotY);
        
        //We have to subtract half the robot image width / height from the world->pixel coordinates,
        //so the the robot will be drawn with it's origin at the proper location, instead of
        //the upper-left-hand corner
        int drawX = ulX - robotImage.getWidth() / 2 - 5;  //Add 5, because the origin is drawn 5 px right of center
        int drawY = ulY - robotImage.getHeight() / 2;
        //Rotate the image of the robot by theta
    	AffineTransform trans = AffineTransform.getRotateInstance(-robotTheta, drawX + robotImage.getWidth() / 2 + 5, drawY + robotImage.getHeight() / 2);
    	trans.translate(drawX, drawY);
    	
    	bufferG.drawRenderedImage(robotImage, trans);
    	//bufferG.drawRenderedImage(op.filter(robotImage, null), drawX, drawY, null);
        
    }
    
    /////////////////////////////////////////////////////////////////////////  
    
    /////////////////////////
    // DRAW GRAPH OVERLAYS //
    /////////////////////////
    //Draws constant graph elements, such as grid lines and the world frame axes
    public void drawOverlays()
    {
        int xSpacing = panelWidth / 11;
        int ySpacing = panelHeight / 10;
        double mmPerSpacingX = worldDistX / 11.0;
        double mmPerSpacingY = worldDistY / 10.0;
        
        
        //vertical grid lines
        bufferG.setFont(tinyFont);
        for(int x = 0; x <= 11; x++)
        {
            bufferG.setColor(gridColor);
            bufferG.drawLine(x * xSpacing, 0, x * xSpacing, panelHeight);
            
            bufferG.setColor(gridLabelColor);
            double xVal = (worldXMin + (mmPerSpacingX * x)); //Display grid in meters
            if(x > 0 && x < 11)
            {
                bufferG.drawString(new Double(xVal).toString(), x * xSpacing + 5, originPxY - 3);
            }
            else if(x == 11) //Draw on left of line so label doesn't get cut off screen
            {
                bufferG.drawString(new Double(xVal).toString(), x * xSpacing - 12, originPxY - 3);
            }
            else //Include unit in output
            {
                bufferG.drawString(new Double(xVal).toString() + "  m", x * xSpacing + 3, originPxY - 3);
            }
        }
        
        //horizontal grid lines
        bufferG.setFont(tinyFont);
        for(int y = 0; y <= 10; y++)
        {
            bufferG.setColor(gridColor);
            bufferG.drawLine(0, y * ySpacing, panelWidth, y * ySpacing);
            
            bufferG.setColor(gridLabelColor);
            double yVal = (worldYMax - (mmPerSpacingY * y)); //Display grid in meters
            if(y < 10)
            {
                bufferG.drawString(new Double(yVal).toString(), originPxX - 15, y * ySpacing + 12);
            }
            else //Include unit in output
            {
                bufferG.drawString(new Double(yVal).toString() + "  m", originPxX - 25, y * ySpacing - 3);
            }
        }
        
        //Draw x, y axes
    	bufferG.setFont(axesFont);
        bufferG.setColor(xAxisColor);
        bufferG.drawLine(0, originPxY, panelWidth, originPxY); //x-axis
        bufferG.setColor(Color.red);
        bufferG.drawChars("x".toCharArray(), 0, 1, panelWidth - 20, originPxY + 12); //Label
        
        bufferG.setColor(yAxisColor);
        bufferG.drawLine(originPxX, 0, originPxX, panelHeight); //y-axis
        bufferG.setColor(Color.green);
        bufferG.drawChars("y".toCharArray(), 0, 1, originPxX + 3, 20); //Label
    }
    
    ////////////////////////
    //   DRAW OBSTACLES   //
    ////////////////////////
    
    private void drawObstacles()
    {
    	bufferG.setColor(obstacleColor);
    	for(int i = 0; i < obstacles.size(); i++)
    	{
    		int[] coors = obstacles.get(i);
    	    bufferG.fillRect(coors[0], coors[1], 2, 2);
    	}
    }
    
    ///////////////////////
    //     DRAW GOAL     //
    ///////////////////////
    private void drawGoal()
    {
        bufferG.setColor(goalColor);
        
    	AffineTransform trans = AffineTransform.getRotateInstance(-goalTheta, goalPxX + goalImage.getWidth() / 2 + 5, goalPxY + goalImage.getHeight() / 2);
    	trans.translate(goalPxX, goalPxY);
    	
    	bufferG.drawRenderedImage(goalImage, trans);
        
    }
    /////////////////////////////////////////////////////////////////////////  
    
    
    
}
