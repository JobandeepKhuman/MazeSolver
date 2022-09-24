/*
*GrandFinale Preamble
*I implemented my own approach instead of following a suggested approach, because
*I believe that my appraoch saves memory space and results in more efficent code.
*In my appraoch I use the first run through a maze to create a map, this map holds
*the absolute direction (N,E,S,W) that the robot should face after each move in
*order to reach the target without hitting any deadEnds, this results in any runs
*after the first run being executed extremley efficently. Only 1 if statement is
*processed, then an elemnet is read directly from the map, without any searching
*required because the map is ordered (firstElement=firstMove, secondElemnet=secondMove)
*etc, and then finally pollRun is incremented. The map is created using an arrayList
*,because on the first run of a maze the map needs to be treated as a stack and on
*the second run of the maze the map needs to be treated as a queue, an arrayList
*can be treated as both if 2 seperate pointers are used. To create the map I added
*the direction the robot faced on each move to the map arrayList if the robot was
*exploring. If the robot is backtracking the last direction to be added to the
*map arrayList is removed. This results in any directions that lead to deadEnds
*being removed from the map.
*
*In addition to the map I added a heuristic to the junctionorCrossroads method.
*Whenever, a robot is at a junction it chooses an unexplored direction that leads
*the robot closer to the target, if no such direction exists a random unexplored
*direction is chosen. This results in the robot taking less moves to explore the
*maze in the first run on average. Additionally, on loopy mazes this heuristic
*allows the robot to on average learn shorter routes than if the heuristic was
*not implemented. To implement this I created a method to check if the target is
*North of the robot and another method to check if the target is East of the robot.
*Additionally, in the junctionorCrossroads method each absolute direction had to
*be converted to a relative direction using a mathematical formula, because the
*robot can only look in relative directions.
*
*My robot is able to solve loopy mazes, because in the initial run of a maze the robot
*traverses the maze using my solution to exercise 3, which turns loopy mazes
*into non-loopy mazes. It can also deal with repeated runs of the same maze
*,because after the first run the robot just reads of the map and the contents
*of the map are not altered after the first run, and pollRun ,which is
*used to increment through the map, is reset after each run. The robot can deal
*with new mazes, because on the first run of a new maze the map is reset and
*then the robot creates a new map.
*/
//Importing libraries
import uk.ac.warwick.dcs.maze.logic.IRobot;
import java.util.*;

public class GrandFinale {
  private int pollRun = 0; // Incremented after each pass
  private int explorerMode=1; //1=explore, 0=backtrack
  //Stack will hold the entryDirections of each junction that is encountered
  private Stack<Integer> entryDirections = new Stack<Integer>();
  //map ArrayList will hold the direction that the robot needs to take on each move
  //in order to get to the target, without hitting any dead ends
  private ArrayList<Integer> map = new ArrayList<Integer>();
  private int mapPointer = -1;//Pointer to point to the next empty spot in the map arrayList

  //Method to decide what direction the robot should face
  public void controlRobot(IRobot robot) {
  //Only running through the explore and backtrack algorithms if it is the first run of the maze
    if(robot.getRuns()==0){
      //If it is the first move of a new maze clear the map arrayList and reset mapPointer
      if(pollRun==0){
        map.clear();
        mapPointer=-1;
      }
      //Calling the explore or backtrack method based on the value of explorerMode
      if(explorerMode==1){
        exploreControl(robot);
      }
      else{
        backtrackControl(robot);
      }
    }
  //If it is not the first run of the maze the robot reads the directions of the map
    else{
  //Using the pollRun variable to increment through the map, then setting the robots heading to the direction specified by the map
      robot.setHeading(map.get(pollRun));
    }
    //Increment pollRun so that the data is not reset each time the robot moves on a new maxe,
    //and so the robot can read the correct direction off the map if not in a new maze
    pollRun++;
  }

  //Method that controls what direction the robot chooses to face when explorerMode=1
  //Also decides when the robot should start backtracking
  public void exploreControl(IRobot robot){
  //Initialisng variables
  int direction=0;
    //Calling the appropriate method to respond to being in a deadEnd, corridor or junction/crossroads
    //based on the number of walls surrounding the robot
    switch(nonwallExits(robot)){
      case(1)://1 nonwall exit implies the robot is at a deadEnd
        robot.face(deadEnd(robot));
        //Robot will only switch to backtrack at deadEnd if it is NOT the first move
        if (pollRun!=0){
          explorerMode=0;
        }
        break;
      case(2)://2 nonwall exit implies the robot is in a corridor
        robot.face(corridor(robot));
        break;
      case(3)://3 or 4 nonwall exit implies the robot is at a junction or crossroads
      case(4):
        //Adding the juction information to entryDirections if it is the first time encountering it
        //If the robot starts of at a junction beenbeforeExits=0, if it it encounters it for the first
        //time and it is not the initial move beenbeforeExits=1
        if(beenbeforeExits(robot)<=1){
          entryDirections.push(robot.getHeading());
          robot.face(junctionorCrossroads(robot));
        }
        //If the robot has visited a junction before, the robot turns around and
        //switches to backtrack mode
        else{
          explorerMode=0;
          robot.face(IRobot.BEHIND);
        }
      }
      //Only adding directions to the map if the robot does not switch to backtrack mode
      if(explorerMode==1){
        map.add(robot.getHeading());
        mapPointer++;
      }
    }

    //Method that decides what direction the robot should face when explorerMode=0
    //Also decides when the robot should start exploring again
    public void backtrackControl(IRobot robot){
      //Calling the appropriate method to respond to being in a deadEnd, corridor or junction/crossroads
      //based on the number of walls surrounding the robot
      if(nonwallExits(robot)==2){//2 non wall exits implies the robot is at a corridor
          robot.face(corridor(robot));
      }
      else{//Not 2 non wall exits implies the robot is at a junction or crossroads
          //Instructing the robot to start exploring on the next move, if there are
          //unexplored exits at the current junction and facing the robot towards
          //an unexplored exit
        if(beenbeforeExits(robot)!=nonwallExits(robot)){
          explorerMode=1;
          //Facing the robot in the correct direction
          robot.face(junctionorCrossroads(robot));
          //Replacing the old direction the robot took at the current junction with the new direction in the map arrayList
          map.remove(mapPointer);
          map.add(robot.getHeading());
        }
        else{
          //assigning the heading to the opposite of the direction that the robot entered the junction
          //Simultaneously, removing the direction the robot entered the junction from the Stack
          robot.setHeading(IRobot.NORTH+((((entryDirections.pop() - IRobot.NORTH) +2)%4)+4)%4);
        }
    }
    //Removing the most recent direction from the map arrayList, if the robot is still backtracking
    if(explorerMode==0){
      map.remove(mapPointer);
      mapPointer--;}
  }

  //Method that calculates the number of non wall exits surrounding the robot
  private int nonwallExits (IRobot robot){
    //Initialisng variables
    int notWalls=0;
    //Checking the 4 directions around the robot
    for(int count=0; count<4; count++){
    //If the direction being checked is not a wall, the notWalls variable is incremented
      if(robot.look(IRobot.AHEAD+count)!=IRobot.WALL){
        notWalls++;
        }
      }
    return notWalls;
    }

  //Method that decides what the robot should do when it meets a deadEnd
  //Not as simple as just turning around as the robot could start at a deadEnd
  //but be facing the exit direction
  private int deadEnd (IRobot robot){
  //Initialisng variables
  int direction=0;
  //Looping through each direction
    for(int count=0; count<4; count++){
      //Checking if the currcent direction is not a wall
      if(robot.look(IRobot.AHEAD+count)!=IRobot.WALL){
        //setting the direction to the current direction
        direction=IRobot.AHEAD+count;}
    }
      return direction;
  }

  //Method that decides what the robot should do at a corridor
  private int corridor (IRobot robot){
    //Move forward if there is no wall ahead
    if(robot.look(IRobot.AHEAD)!=IRobot.WALL){
      return IRobot.AHEAD;}
    //Turn right if the corridor bends right
    else{
      if(robot.look(IRobot.RIGHT)!=IRobot.WALL){
        return IRobot.RIGHT;}
      //Turn left if the corridor bends left
      else{
        return IRobot.LEFT;}}
  }

  //Method that decides what the robot should do at a junction or crossroads.
  private int junctionorCrossroads(IRobot robot){
    //Initialisng variables
    int randno=0;
    int direction=0;
    //Initialisng an array to hold any unexplored directions around the robot
    ArrayList<Integer> unexploredDirections = new ArrayList<Integer>();
    //finding the absolute directions relative to the robot
    int relativeNorth=(robot.AHEAD+(((IRobot.NORTH - robot.getHeading())%4)+4)%4);
    int relativeEast=(robot.AHEAD+(((IRobot.EAST - robot.getHeading())%4)+4)%4);
    int relativeSouth=(robot.AHEAD+(((IRobot.SOUTH - robot.getHeading())%4)+4)%4);
    int relativeWest=(robot.AHEAD+(((IRobot.WEST - robot.getHeading())%4)+4)%4);
    //Checking each direction to see if it is an unexplored exit and it leads to the target
    //Check for passage exit north of robot
    if(robot.look(relativeNorth)==IRobot.PASSAGE & isTargetNorth(robot)==1){
      direction=relativeNorth;//setting the direction to North relative to the robot
    }
    //checking for passage exit South of robot
    else if(robot.look(relativeSouth)==IRobot.PASSAGE & isTargetNorth(robot)==-1){
      direction=relativeSouth;//setting the direction to South relative to the robot
    }
    //checking for passage exit East of robot
    else if(robot.look(relativeEast)==IRobot.PASSAGE & isTargetEast(robot)==1){
      direction=relativeEast;//setting the direction to East relative to the robot
    }
    //checking for passage exit West of robot
    else if(robot.look(relativeWest)==IRobot.PASSAGE & isTargetEast(robot)==-1){
      direction=relativeWest;//setting the direction to West relative to the robot
    }
    //If none of the unexplored directions lead the robot closer to the target choose
    //random unexplored direction
    if(direction==0){
      //Checking all directions around the robot
      for(int count=0; count<4; count++){
        //Adding all unexplored directions to unexploredDirections arrayList
        if(robot.look(IRobot.AHEAD+count)==IRobot.PASSAGE){
          unexploredDirections.add(IRobot.AHEAD+count);}
    }
    //generating random integer between 0 and length of unexplored directions-1
    randno= (int) Math.ceil(Math.random()*unexploredDirections.size())-1;
    //Setting direction to face a random unexplored exit
    direction=unexploredDirections.get(randno);}
    return direction;
  }

  private int beenbeforeExits (IRobot robot){
    //Initialisng variables
    int beenBeforeSquares=0;
    //Checking the 4 directions around the robot
    for(int count=0; count<4; count++){
    //If the direction being checked is a square that has been visited, the
    //the beenBeforeSquares variable is incremented
      if(robot.look(IRobot.AHEAD+count)==IRobot.BEENBEFORE){
        beenBeforeSquares++;
        }
      }
    return beenBeforeSquares;

    }

  //Resetting variables when the reset button is pressed
  public void reset() {
      explorerMode=1;
      pollRun=0;
      entryDirections.clear();
  }

  //This method detects if the target is North, South or the same latitude as the
  //robot
    private byte isTargetNorth(IRobot robot) {
      byte result;
    //returning 1 if target is North of robot, -1 if South and 0 if same latitude
      if((robot.getLocation().y-robot.getTargetLocation().y)>0)
        result=1;
      else if ((robot.getLocation().y-robot.getTargetLocation().y)<0)
        result= -1;
      else
        result=0;
      return result;
    }

    //This method detects if the target is East, West or the same longitude as the
    //robot
    private byte isTargetEast(IRobot robot) {
      byte result;
    //returning 1 if target is East of robot, -1 if West and 0 if same longitude
      if((robot.getLocation().x-robot.getTargetLocation().x)<0)
        result=1;
      else if ((robot.getLocation().x-robot.getTargetLocation().x)>0)
        result= -1;
      else
        result=0;
      return result;
    }

}
