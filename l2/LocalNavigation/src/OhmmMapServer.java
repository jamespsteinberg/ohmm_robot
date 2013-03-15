
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.IOException;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.net.ServerSocket;
import java.net.Socket;



//This program will run on the server (OHMM), and send messages
//to the client (Computer), about orientation and obstacle coordinates.

import ohmm.*;
import static ohmm.OHMM.*;
import ohmm.OHMM.AnalogChannel;
import static ohmm.OHMM.AnalogChannel.*;
import ohmm.OHMM.DigitalPin;
import static ohmm.OHMM.DigitalPin.*;


public class OhmmMapServer {
	//////////////////////MAIN/////////////////////
    
	///////////////////////////////////////////////
	public static int SERVER_PORT = 4321;
	private ServerSocket serverSock = null;
    private Socket client = null;
    private InputStream sockInput = null;
    private OutputStream clientOut = null;
    private PrintWriter outWriter = null;
    
    public boolean isConnected = false;
    
    
    
	private int mapUpdateRateMillis = 200; //Number of milliseconds between map 
	private OHMMDrive ohmm;
    private double goalX;
	//Constructor
	public OhmmMapServer(OHMMDrive ohmm, double goalX)
	{		   
        try{
        	this.ohmm = ohmm;
        	this.goalX = goalX;
            serverSock = new ServerSocket(SERVER_PORT); //Listen on port PORT   
            System.out.println("Server listening on port: " + SERVER_PORT);
            
        }catch(IOException e)
        {
            e.printStackTrace(System.err); //Print the error
        }
        
	}
    
	//Wait for connection from client
	public void waitForConnection()
    {
		System.out.println("Waiting for client to connect.");
		try{
            //This method will wait until a connection is made
            client = serverSock.accept();
            System.out.println("Client connected!");
            clientOut = client.getOutputStream();
            outWriter = new PrintWriter(new OutputStreamWriter(clientOut));
            
            //Handle the connection
            handleConnection();
            
            
        }
        catch(IOException e)
        {
        	handleException(e);          
        }
        
        
        //Close the socket
        try{
            System.out.println("Closing socket.");
            client.close();
        }
        catch(IOException e)
        {
            System.err.println("Exception caught while closing socket.");
            e.printStackTrace(System.err);
        }
        
        System.out.println("Finished with socket.");
		
    }
    
    
    //Send message to client (Computer)    
    private void sendMessage(String message)
    {
    	if(outWriter != null) //Don't try and write to client if there isn't one!
	    {
	    	try{
	    		outWriter.println(message);
	            outWriter.flush();
	    	} catch(Exception e)
	    	{
	    		System.err.println("Exception encountered in sending message.");
	    		e.printStackTrace(System.err);
	    	}
    	}
    }
    
    
    //Handle the connection with the client
    private void handleConnection()
    {
        
        sendMessage("You have connected to OHMM Server at port: " + SERVER_PORT);
        this.isConnected = true;
        mapSetGoal(goalX, 0);
        
        while(true)
        {
            
            //If client has disconnected, wait for a connection again
            if(client.isClosed() || outWriter.checkError())
            {
                System.out.println("Client disconnected.");
                waitForConnection();
            }
            else
            {
                try {
                    float[] pose = ohmm.driveGetPose();
                    mapSetRobot(pose[0] / 1000.0, pose[1] / 1000.0, pose[2]); // divide by 1000 : mm -> m
                    Thread.sleep(mapUpdateRateMillis);
                } catch (Exception e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                    
                }
                
                
            }
        }
        
    }
    //s 
    //Handle a connection error
    private void handleException(IOException e)
    {
    	System.out.println("Client did not connect to OHMM Server.");
        e.printStackTrace(System.err);
    }
    
    
    //SET THE ROBOT POSE
    //Send information to client on the new robot pose
    //x, y, theta:  Robot coordinates in world frame
    public void mapSetRobot(double x, double y, double theta)
    {
    	sendMessage("UPDATE_ROBOT " + x + " " + y + " " + theta);
    	
    }
	
    //ADD OBSTACLE POINT
    public void mapAddObstaclePoint(double x, double y)
    {
    	sendMessage("ADD_OBSTACLE " + x + " " + y);
    }
    
    //SET THE GOAL
    public void mapSetGoal(double x, double y)
    {
    	sendMessage("SET_GOAL " + x + " " + y + " 0"); //Theta = 0 in simplified version
    }
    
    
	
}
