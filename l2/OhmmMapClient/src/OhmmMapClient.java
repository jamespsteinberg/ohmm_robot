import java.io.BufferedReader;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.IOException;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;


//This project will run on the computer (client), and receive messages from the OHMM (Server)
//This program will draw a visual map, 2.5 x 2.5 meters, of the robot's environment

public class OhmmMapClient {
	//////////////////////MAIN/////////////////////
	public static void main(String[] args)
	{
		if(args.length == 2)
		{
            
            String ipStr = args[0];
            int port = Integer.parseInt(args[1]);
            
            OhmmMapClient client = new OhmmMapClient(ipStr, port);
            //client.tryConnect(ipStr, port);
            client.debug();
		}
		else
		{
			printUsage();
		}
	}
	
	public static void printUsage()
	{
		System.out.println("Usage:  OhmmMapClient " + '"' + "[ipAddress]" + '"' + " [port]");
	}
	///////////////////////////////////////////////
	private Socket server = null;
    private OhmmMap ohmmMap;
    
    private String serverIP;
    private int port;
    
    
    
	//Constructor
	public OhmmMapClient(String ipAddress, int port)
	{
		this.serverIP = ipAddress;
		this.port = port;
		this.ohmmMap = new OhmmMap();
        tryConnect(ipAddress, port);
	}
    
	//Attempt to connect to the server (Ohmm)
	public void tryConnect(String ip, int port)
    {
		InputStream sockInput = null;
		OutputStream sockOutput = null;
		
        System.out.println("Opening connection to: " + ip + "  on port: " + port);
		try{
			//This method will wait forever until a connection is made
			server = new Socket(ip, port);
			sockInput = server.getInputStream();
			sockOutput = server.getOutputStream();
		}
		catch(IOException e)
		{
			e.printStackTrace(System.err);
		}
        
		//handleConnection will loop infinitely, for now
		handleConnection(sockInput, sockOutput);
		
		//Close the socket
		try{
			System.out.println("Closing socket.");
			server.close();
		}
		catch(IOException e)
		{
			System.err.println("Exception caught while closing socket.");
			e.printStackTrace(System.err);
		}
		
        
		
    }
	
	
	//HERE IS WHERE WE WILL TAKE THE CONNECTION, AND LINK IT TO THE MAP DRAWING SOMEHOW
	private void handleConnection(InputStream input, OutputStream output)
	{
		System.out.println("Connected to OHMM server!");
        BufferedReader inReader = new BufferedReader(new InputStreamReader(input));
        while(true)
        {
        	System.out.println("Listening");
            try{
                String inStr = null;
                
                while((inStr = inReader.readLine()) != null)
                {
                   processMessage(inStr); //Update the map with the new data
                    
                }
            }
            catch(IOException e)
            {
                System.err.println("Error while reading from InputStream");
                e.printStackTrace(System.err);
            }
        }
	}	
    
    
    //PROCESS MESSAGE RECEIVED FROM OHMM SERVER
    //Message is given as a single string, with arguments separated by spaces
    private void processMessage(String msg)
    {
        
        //FOR DEBUGGING
        //debug();
        
        
        String[] args = msg.split(" ");
        //The first argument will be identify the type of message we have been given.
        String msgId = args[0].trim();
        if(msgId.equals("UPDATE_ROBOT")) //Update robot orientation
        {
            processUpdateRobot(args);
        }
        else if(msgId.equals("ADD_OBSTACLE")) //Update grid data (Obstacle locations, etc)
        {
            processAddObstacle(args);   
        }
        else if(msgId.equals("SET_GOAL"))
        {
        	processSetGoal(args);
        }
        else
        {
            System.out.println("UNKOWN COMMAND: " + msg);   
        }
        
    }
    
    
    
    //Update the robot orientation
    private void processUpdateRobot(String[] args)
    {
    	double x = Double.parseDouble(args[1]);
    	double y = Double.parseDouble(args[2]);
    	double theta = Double.parseDouble(args[3]);
    	ohmmMap.updateRobotPose(x, y, theta);
        
    }
    
    //Add an obstacle
    private void processAddObstacle(String[] args)
    {
    	double x = Double.parseDouble(args[1]);
    	double y = Double.parseDouble(args[2]);
    	ohmmMap.addObstacle(x, y);
    }
    
    //Set the goal
    private void processSetGoal(String[] args)
    {
    	double x = Double.parseDouble(args[1]);
        double y = Double.parseDouble(args[2]);
        double theta = Double.parseDouble(args[3]);
        ohmmMap.setGoal(x, y, theta);
    }
    
    
    
    //DEBUGGING ANIMATION
    private void debug()
    {
    	//Full rotation
    	for(int i = 0; i <= 100; i++)
    	{
    		ohmmMap.updateRobotPose(0, 0, (Math.PI / 50) * i);
    	    Long curTime = System.currentTimeMillis();
    	    
    	    //Loop to slow down update
    	    while(System.currentTimeMillis() - curTime < 50)
    	    {
    	    }
    	    
    	}
    	
    	//Drive x
    	for(int i = 0; i <= 100; i++)
    	{
    		ohmmMap.updateRobotPose((double)i * 0.03, 0, 0);
    	    Long curTime = System.currentTimeMillis();
    	    
    	    //Loop to slow down update
    	    while(System.currentTimeMillis() - curTime < 50)
    	    {
    	    }
    	    
    	}
    	//Rotate pi / 3
    	for(int i = 0; i <= 30; i++)
    	{
    		ohmmMap.updateRobotPose(3, 0, (Math.PI / 90) * i);
    	    Long curTime = System.currentTimeMillis();
    	    
    	    //Loop to slow down update
    	    while(System.currentTimeMillis() - curTime < 50)
    	    {
    	    }
    	    
    	}
    	
     	//Drive forward
    	for(int i = 0; i <= 50; i++)
    	{
    		double newX = 3 + (Math.cos(Math.PI / 3) * i * 0.03);
    		double newY = (Math.sin(Math.PI / 3) * i * -0.03);
    		ohmmMap.updateRobotPose(newX, newY, Math.PI / 3);
    	    Long curTime = System.currentTimeMillis();
    	    
    	    //Loop to slow down update
    	    while(System.currentTimeMillis() - curTime < 50)
    	    {
    	    }
    	    
    	}
    	
        
        System.out.println("Debugging");
    }
	
	
}
