package l2;

import ohmm.*;
import static ohmm.OHMM.*;
import ohmm.OHMM.AnalogChannel.*;
import static ohmm.OHMM.AnalogChannel.*;
import ohmm.OHMM.DigitalPin.*;
import static ohmm.OHMM.DigitalPin.*;

import java.io.*;
import java.util.*;
import java.util.Arrays;
import java.lang.Math.*;

public class GNav
{
	private float l1x1, l1y1, l1x2, l1y2, l2x1, l2y1, l2x2, l2y2;
	private float m1 = 0; private float c1 = 0; private float m2 = 0; private float c2 = 0;
	private float intersection_X = 0; private float intersection_Y = 0;

public int IsPointInBoundingBox(float x1, float y1, float x2, float y2, float px, float py)
{
    float left, top, right, bottom; // Bounding Box For Line Segment
    // For Bounding Box
    if(x1 < x2)
    {
        left = x1;
        right = x2;
    }
    else
    {
        left = x2;
        right = x1;
    }
    if(y1 < y2)
    {
        top = y1;
        bottom = y2;
    }
    else
    {
        top = y1;
        bottom = y2;
    }
 
    if( (px+0.01) >= left && (px-0.01) <= right && 
            (py+0.01) >= top && (py-0.01) <= bottom )
    {
        return 1;
    }
    else
        return 0;
}
 
public int LineIntersection(float l1x1, float l1y1, float l1x2, float l1y2, float l2x1, float l2y1, float l2x2, float l2y2, float m1, float c1, float m2, float c2, float intersection_X, float intersection_Y)
{
    float dx, dy;
 
    dx = l1x2 - l1x1;
    dy = l1y2 - l1y1;
 
    m1 = dy / dx;
    // y = mx + c
    // intercept c = y - mx
    c1 = l1y1 - m1 * l1x1; // which is same as y2 - slope * x2
 
    dx = l2x2 - l2x1;
    dy = l2y2 - l2y1;
 
    m2 = dy / dx;
    // y = mx + c
    // intercept c = y - mx
    c2 = l2y1 - m2 * l2x1; // which is same as y2 - slope * x2
 
    if( (m1 - m2) == 0)
        return 0;
    else
    {
        intersection_X = (c2 - c1) / (m1 - m2);
        intersection_Y = m1 * intersection_X + c1;
    }
	return 1;
}
 
public int LineSegmentIntersection(float l1x1, float l1y1, float l1x2, float l1y2, float l2x1, float l2y1, float l2x2, float l2y2, float m1, float c1, float m2, float c2, float intersection_X, float intersection_Y)
{
    float dx, dy;
 
    dx = l1x2 - l1x1;
    dy = l1y2 - l1y1;
 
    m1 = dy / dx;
    // y = mx + c
    // intercept c = y - mx
    c1 = l1y1 - m1 * l1x1; // which is same as y2 - slope * x2
 
    dx = l2x2 - l2x1;
    dy = l2y2 - l2y1;
 
    m2 = dy / dx;
    // y = mx + c
    // intercept c = y - mx
    c2 = l2y1 - m2 * l2x1; // which is same as y2 - slope * x2
 
    if( (m1 - m2) == 0)
        return 0;
    else
    {
        intersection_X = (c2 - c1) / (m1 - m2);
        intersection_Y = m1 * intersection_X + c1;
    }
    if(IsPointInBoundingBox(l1x1, l1y1, l1x2, l1y2, intersection_X, intersection_Y) == 1 &&
        IsPointInBoundingBox(l2x1, l2y1, l2x2, l2y2, intersection_X, intersection_Y) == 1)
    {
        return 1;
    }
    else
        return 0;
}

 
	public void nav(String args[]) throws IOException
	{	
		int node_count = 0;
		int count = 0;
		// Indicate that you are planning to opena file
		File map_file = new File(args[0]);
        	// Prepare a Scanner that will "scan" the document
	        Scanner s = new Scanner(map_file);
		s.useLocale(Locale.US);

		// Read each line in the file
	        while( s.hasNext() ) 
		{
			s.next();
			count++;
			if (count > 6)
				node_count++;

		}
		node_count += 2;//We always have a source and goal at least
		s.close();

		//Initialize Nodes for visibility graph
		float[][] nodes = new float[2][((node_count - 2) * 4) + 2];//First node is source, last node is goal

		System.out.println("Scanned " + node_count + " nodes from  " + args[0]);
		map_file = new File(args[0]);
		s = new Scanner(map_file);
        	// Prepare a Scanner that will "scan" the document

		// Create map, goal node and obstacle nodes
		count = 0;
		float[] source = new float[2];//coordinates for source
		source[0] = 0; source[1] = 0;
		float[] goal = new float[2];//coordinates for goal
		float[] map_pts = new float[4];//coordinates for map area
		float temp_xmin, temp_xmax, temp_ymin, temp_ymax;
	        while( count < node_count) 
		{
			if (count == 0)
			{
				goal[0] = s.nextFloat();
				goal[1] = s.nextFloat();

				nodes[0][0] = source[0];
				nodes[1][0] = source[1];
				System.out.println("at " + (count) + ": (" + nodes[0][0] + "," + nodes[1][0] + ")");
				count++;

				nodes[0][node_count - 1] = goal[0];
				nodes[1][node_count - 1] = goal[1];
				System.out.println("at " + (node_count - 1) + ": (" + nodes[0][node_count - 1] + "," + nodes[1][node_count - 1] + ")");
				count++;
			}
			else if (count == 2)
			{
				map_pts[0] = s.nextFloat();
				map_pts[1] = s.nextFloat();
				map_pts[2] = s.nextFloat();
				map_pts[3] = s.nextFloat();
				
				count++;//Just to move to the next stage
			}
			else
			{
				if (count == 3)
					count--;//Decrement one since this is not a node
				temp_xmin = s.nextFloat();
				temp_xmax = s.nextFloat();
				temp_ymin = s.nextFloat();
				temp_ymax = s.nextFloat();

				nodes[0][count - 1] = temp_xmin;//top left
				nodes[1][count - 1] = temp_ymin;
				System.out.println("at " + (count - 1) + ": (" + nodes[0][count - 1] + "," + nodes[1][count - 1] + ")");
				count++;

				nodes[0][count - 1] = temp_xmax;//top right
				nodes[1][count - 1] = temp_ymin;
				System.out.println("at " + (count - 1) + ": (" + nodes[0][count - 1] + "," + nodes[1][count - 1] + ")");
				count++;

				nodes[0][count - 1] = temp_xmin;//bottom left
				nodes[1][count - 1] = temp_ymax;
				System.out.println("at " + (count - 1) + ": (" + nodes[0][count - 1] + "," + nodes[1][count - 1] + ")");
				count++;

				nodes[0][count - 1] = temp_xmax;//bottom right
				nodes[1][count - 1] = temp_ymax;
				System.out.println("at " + (count - 1) + ": (" + nodes[0][count - 1] + "," + nodes[1][count - 1] + ")");
				count++;
			}
		}
		s.close();

		//Create graph (matrix of edge weights)
		float[][] graph = new float[node_count][node_count]; //A graph (matrix) which gives the length(cost) of an edge between [x][y]
		int nRet;

		//Compares each edge in tje graph against each of the obstacles to see if the line intersects the obstacle (therefore making it an invalid edge)
		for (int i = 0; i < node_count; i++)
		{
			for (int k = 0; k < node_count; k++)
			{
				//This loop is for each of the obstacles
				for (int m = 1; m < node_count; m++)
				{ 
					nRet = 0;
					
					//Compares the line from i to k, with the top, left, right and bottom edges of each obstacle to check for intersection
					l1x1 = nodes[0][i];l1y1 = nodes[1][i];l1x2 = nodes[0][k];l1y2 = nodes[1][k];
					//For bottom
					l2x1 = nodes[0][m];l2y1 = nodes[1][m];l2x2 = nodes[0][m + 1];l2y2 = nodes[1][m + 1];
					nRet += LineSegmentIntersection(l1x1, l1y1, l1x2, l1y2, l2x1, l2y1, l2x2, l2y2, m1, c1, m2, c2, intersection_X, intersection_Y);
					
					//For left
					l2x1 = nodes[0][m];l2y1 = nodes[1][m];l2x2 = nodes[0][m + 2];l2y2 = nodes[1][m + 2];
					nRet += LineSegmentIntersection(l1x1, l1y1, l1x2, l1y2, l2x1, l2y1, l2x2, l2y2, m1, c1, m2, c2, intersection_X, intersection_Y);
					//For right
					l2x1 = nodes[0][m + 1];l2y1 = nodes[1][m + 1];l2x2 = nodes[0][m + 3];l2y2 = nodes[1][m + 3];
					nRet += LineSegmentIntersection(l1x1, l1y1, l1x2, l1y2, l2x1, l2y1, l2x2, l2y2, m1, c1, m2, c2, intersection_X, intersection_Y);
					//For top
					l2x1 = nodes[0][m + 2];l2y1 = nodes[1][m + 2];l2x2 = nodes[0][m + 3];l2y2 = nodes[1][m + 3];
					nRet += LineSegmentIntersection(l1x1, l1y1, l1x2, l1y2, l2x1, l2y1, l2x2, l2y2, m1, c1, m2, c2, intersection_X, intersection_Y);
 
					//If the edge between i and k intersect an obstacle twice, then it is an invalid edge (if only intersects once then it just touches a vertex)
					if (nRet < 2)
						graph[i][k] = (float)Math.sqrt((float)Math.pow((nodes[0][i] - nodes[0][k]),2) + (float)Math.pow((nodes[1][i] - nodes[1][k]),2));//Signifies an edge of this cost for these two points
					else
						graph[i][k] = -1;//Signifies no edge for these two points

					m += 3; //Moves increment to next obstacle instead of next node
				}
			}
		}

		//Dijkstra's Algorithm
		//1) Set initial distances from source node to each node, and optimal previous node for each
		float[] dist = new float[node_count];//Distance to each node from source
		int[] prev = new int[node_count];//previous node in optimal path to node
		int[] removed_nodes = new int[node_count];//using array which gives which nodes have been "removed" from queue instead of using queue
		for (int i = 0; i < node_count; i++)
		{
			dist[i] = -1;//-1 represents infinity
			prev[i] = -1;//-1 represents undefined node

			removed_nodes[i] = 0;//Says that none of the nodes have been "removed" from the queue
		}
		dist[0] = 0;
		//2)Main loop: finds minimum distance 
		int min_dist_node = 0;
		float min_dist = -1;//-1 represents infinity
		float alt = 0;
		for (int i = 0; i < node_count; i++)
		{
			min_dist = -1;
System.out.println(i);
			for (int k = 0; k < node_count; k++)
			{
				if (min_dist == -1)
				{
					if ((dist[k] != -1) && (removed_nodes[k] != 1))
					{
						min_dist_node = k;
						min_dist = dist[k];
					}
				}
				else
				{
					if ((dist[k] < min_dist) && (dist[k] != -1) && (removed_nodes[k] != 1))
					{
						min_dist_node = k;
						min_dist = dist[k];
					}
				}
			}
			removed_nodes[min_dist_node] = 1;//"Remove" node from queue
			if (min_dist == -1) //Checks if minimum dist == infinity, which signifies no further routes can be made
				break;

			for (int k = 0; k < node_count; k++)
			{
				if (k != i)
				{
				if (graph[min_dist_node][k] != -1)
				{
					alt = graph[min_dist_node][k];
					alt += dist[min_dist_node];
				}
				else if (graph[k][min_dist_node] != -1)
				{
					alt = graph[k][min_dist_node];
					alt += dist[min_dist_node];
				}
				else
					alt = -1;

				if (((alt < dist[k]) || (dist[k] == -1)) && (alt != -1))
				{
					dist[k] = alt;
					prev[k] = min_dist_node;
				}
				}
			}
		}
		//End of Dijkstra's
		
		//Store minimum path to goal from source as path
		System.out.println("=======Solution...=======");
		System.out.println("GRAPH:___________________");
		System.out.println(Arrays.deepToString(graph));
		System.out.println("_________________________");
		if (dist[node_count - 1] == -1)
		{
			System.out.println("No path to node!");
		}
		else
		{
			int new_node = node_count - 1;
			int path_count = 1;//set to 1 initially to include goal node
			System.out.println("Traceback of path (goal to source)...");
			for (int l = 0; l < node_count; l++)
			{
				System.out.println("From node " + new_node + " to " + prev[new_node]); 
				new_node = prev[new_node];
				path_count++;

				if (new_node == 0)//reached source
					break;
			}

			//Create drive path
			int[] path = new int[path_count];
			path[path_count - 1] = node_count - 1;//Set last node in path to final node (we previously defined the goal as the final node)
			for (int i = path_count - 2; i >= 0; i--)
			{
				path[i] = prev[path[i + 1]];
			}

			//Perform the drive
			//NOTE: For now treating robot as point not sqaure
			float robot_side = (float).192;//Measured side of the robot
			float ep = (float).5;//Epsilon (buffer space)
			float r = (float)(.5 * (float)Math.sqrt(2 * (float)Math.pow(robot_side, 2)));//radius of the encompasing circle
			float side_length = 2 * (r + ep);//side length of square virtual robot
			float dy, dx;
			int prev_node = 0;
			float angle = 0;
            		OHMMDrive robot = (OHMMDrive)OHMM.makeOHMM(new String[]{"-r", "/dev/ttyACM1"});

			for (int i = 1; i < path_count; i++)
			{
				dx = Math.abs(nodes[0][path[i]] - nodes[0][prev_node]);
				dy = Math.abs(nodes[1][path[i]] - nodes[1][prev_node]);
				angle = (float)Math.atan(dx/dy);
			
				robot.driveTurn(angle);
				robot.driveStraight(graph[path[i]][prev_node]);
				robot.driveTurn(-angle);
	
				if (i < path_count - 1)
					prev_node = path[i];		
			}		
		}
		System.out.println("=========================");
		System.out.println("(0:3) = " + graph[0][3] + "(3:0) = " + graph[3][0]); 
	}

	public static void main(String args[])
	{
		GNav gnav = new GNav();
		try
		{
			gnav.nav(args);
		}
		catch (IOException ex)
		{
			System.out.println("IO Exception caught");
		}
	}

}
