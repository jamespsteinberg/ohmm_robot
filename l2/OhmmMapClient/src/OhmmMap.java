import java.util.Vector;
import javax.swing.JFrame;
//This class represents the Ohmm Map.
//The map is constructed as an array of boxes, which can either be "empty", if there is no obstacle,
//or "filled", if there is an obstacle present.
//The size of each box, in millemeters, is given by the BOX_SIZE attribute.
//The resolution of the map can be computed by taking MAP_SIXE / BOX_SIZE.
public class OhmmMap {
    
    //Robot coordinates, in world frame
    private double robotX = 0; //x-coordinate, in meters.
    private double robotY = 0; //y-coordinate, in meters.
    private double robotTheta = 0.0; //Theta-rotation, in radians.
    
	//MAP SIZE
	public static double MAP_MIN_X = -0.5; //Min x coordinate of map, in meters, of global frame
	public static double MAP_MAX_X = 5.0; //Max x                                        
	public static double MAP_MIN_Y = -2.5;//Min y
	public static double MAP_MAX_Y = 2.5; //Max y
	
	//FRAME SIZE
	public static int MAP_PIXEL_HEIGHT = 500; //Height of map panel, in pixels
	public static int MAP_PIXEL_WIDTH = 550; //Width of map panel, in pixels
	
	
	//For Drawing
    private JFrame mapFrame;
	private MapPanel mapPanel;
	
	
	//Constructor
	public OhmmMap()
	{
        //Initialize the frame
        this.mapFrame = new JFrame();
        //Add 22 pixels to account for pixels taken up by title bar
        mapFrame.setSize(MAP_PIXEL_WIDTH, MAP_PIXEL_HEIGHT + 22);
        mapFrame.setResizable(false);
        mapFrame.setVisible(true);
        
        
        mapPanel = new MapPanel(MAP_PIXEL_WIDTH, MAP_PIXEL_HEIGHT, MAP_MIN_X, MAP_MIN_Y,
                                MAP_MAX_X, MAP_MAX_Y);
        
        //Add panel to the frame
        mapFrame.add(mapPanel);
        
        mapPanel.init();
        
	}
    
    public void updateRobotPose(double x, double y, double theta)
    {
        this.robotX = x;
        this.robotY = y;
        this.robotTheta = theta;
        mapPanel.updateRobot(x, y, theta);
        mapPanel.drawScene();
    }
    
    //Adds an obstacle point to the map
    public void addObstacle(double x, double y)
    {
    	mapPanel.addObstacle(x, y); //Don't update the 
    	
    }
    
    public void setGoal(double x, double y, double theta)
    {
        mapPanel.setGoal(x, y, theta);   
    }
    
}
