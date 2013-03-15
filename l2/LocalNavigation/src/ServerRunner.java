

import ohmm.*;
import static ohmm.OHMM.*;
import ohmm.OHMM.AnalogChannel;
import static ohmm.OHMM.AnalogChannel.*;
import ohmm.OHMM.DigitalPin;
import static ohmm.OHMM.DigitalPin.*;

//This class runs the server in a new thread
public class ServerRunner implements Runnable{
    
	
	private OhmmMapServer server;
	private OHMMDrive ohmm;
    private double goalX;
	
    
	
	//Constructor
	public ServerRunner(OHMMDrive ohmm, double goalX)
	{
		this.ohmm = ohmm;
        this.goalX = goalX;
	}
	
	public void run() {
        server = new OhmmMapServer(ohmm, goalX);
        server.waitForConnection();     
    }
	
    //ADD OBSTACLE POINT
    public void mapAddObstaclePoint(double x, double y)
    {
    	server.mapAddObstaclePoint(x, y);
    }
}
