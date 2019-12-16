/**
 * make-plan.cc
 * 
 * Sample code for a robot that has two front bumpers and a laser, and
 * which is provided with localization data.
 *
 * The code also allows the controller to read and write a "plan", a sequence
 * of location that the robot should move to and to read in a "map", a matrix
 * of 1 and 0 values that can be used as an occupancy grid.
 *
 * Written by: Simon Parsons
 * Date:       4th December 2011
 *  
 **/


#include <iostream>
#include <fstream>
#include "customgraph.h"
#include <math.h>
#include <libplayerc++/playerc++.h>
using namespace PlayerCc;  

/**
 * Global constants
 *
 **/

const int SIZE = 32; // The number of squares per side of the occupancy grid
                     // (which we assume to be square)

/**
 * Function headers
 *
 **/

// void fillMap(int maptext[][32]);
Node* findPath(int path[256][2], int map[32][32], int init[], int goal[]);
double manhattanDistance(int x1, int y1, int x2, int y2);
void mapOut(int maptext[][32]);
void pathOut(int path[][2], int pathlength);
void createPath(int map[][32], Node *curr, int path[][2], int &pathlength);
void swapPathNode(int path[][2], int i, int j);
void truncatePath(int path[][2], int &pathlength);

player_pose2d_t readPosition(LocalizeProxy& lp);
void printRobotData(BumperProxy& bp, player_pose2d_t pose);
void printLaserData(LaserProxy& sp);

void readMap(int[SIZE][SIZE]);
void writeMap(int [SIZE][SIZE]);
void printMap(int [SIZE][SIZE]);
int  readPlanLength(void);
void readPlan(double *, int);
void printPlan(double *,int);  
void writePlan(double *, int);

/**
 * main()
 *
 **/

int main(int argc, char *argv[])
{  
  // Variables
  int counter = 0;
  double speed;            // How fast do we want the robot to go forwards?
  double turnrate;         // How fast do we want the robot to turn?
  player_pose2d_t  pose;   // For handling localization data

  // The occupancy grid

  int oGrid[SIZE][SIZE];
  int path[256][2];
  int pathlength;

  // The set of coordinates that makes up the plan

  int pLength;
  double *plan;

  // Set up proxies. These are the names we will use to connect to 
  // the interface to the robot.
  PlayerClient    robot("localhost");  
  BumperProxy     bp(&robot,0);  
  Position2dProxy pp(&robot,0);
  LocalizeProxy   lp (&robot, 0);
  LaserProxy      sp (&robot, 0);
  
  // Allow the program to take charge of the motors (take care now)
  pp.SetMotorEnable(true);

  // Map handling
  //
  // The occupancy grid is a square array of integers, each side of
  // which is SIZE elements, in which each element is either 1 or 0. A
  // 1 indicates the square is occupied, an 0 indicates that it is
  // free space.
  readMap(oGrid);   // Read a map in from the file map.txt
  printMap(oGrid);  // Print the map on the screen
  // writeMap(oGrid);  // Write a map out to the file map-out.txt

  int start[] = {-12, -12};
  int end[] = {13, 13};
  Node *node = findPath(path, oGrid, start, end);
  createPath(oGrid, node, path, pathlength);
  mapOut(oGrid);
  truncatePath(path, pathlength);
  pathOut(path, pathlength);


  // Plan handling
  // 
  // A plan is an integer, n, followed by n doubles (n has to be
  // even). The first and second doubles are the initial x and y
  // (respectively) coordinates of the robot, the third and fourth
  // doubles give the first location that the robot should move to, and
  // so on. The last pair of doubles give the point at which the robot
  // should stop.
  pLength = readPlanLength(); // Find out how long the plan is from plan.txt
  plan = new double[pLength]; // Create enough space to store the plan
  readPlan(plan, pLength);    // Read the plan from the file plan.txt.
  printPlan(plan,pLength);    // Print the plan on the screen
  writePlan(plan, pLength);   // Write the plan to the file plan-out.txt

  
  // Main control loop
  while(true) 
    {    
      // Update information from the robot.
      robot.Read();
      // Read new information about position
      pose = readPosition(lp);
      // Print data on the robot to the terminal
      printRobotData(bp, pose);
      // Print information about the laser. Check the counter first to stop
      // problems on startup
      if(counter > 2){
	printLaserData(sp);
      }

      // Print data on the robot to the terminal --- turned off for now.
      // printRobotData(bp, pose);
      
      // If either bumper is pressed, stop. Otherwise just go forwards

      if(bp[0] || bp[1]){
	speed= 0;
	turnrate= 0;
      } 
      else {
	speed=.1;
        turnrate = 0;
      }     

      // What are we doing?
      std::cout << "Speed: " << speed << std::endl;      
      std::cout << "Turn rate: " << turnrate << std::endl << std::endl;

      // Send the commands to the robot
      pp.SetSpeed(speed, turnrate);  
      // Count how many times we do this
      counter++;
    }
  
} // end of main()

/**
 * readMap
 *
 * Reads in the contents of the file map.txt into the array map
 * in such a way that the first element of the last row of the
 * file map.txt is in element [0][0].
 *
 * This means that whatever is in the file looks like the occupancy
 * grid would if you drew it on paper.
 *
 **/

void readMap(int map[SIZE][SIZE])
{
  std::ifstream mapFile;
  mapFile.open("map.txt");

  for(int i = SIZE - 1; i >= 0; i--){
    for(int j = 0; j < SIZE; j++)
      {
	mapFile >> map[i][j];
      }
  }

  mapFile.close();

} // End of readMap()

/**
 * printMap
 *
 * Print map[][] out on the screen. The first element to be printed
 * is [SIZE][0] so that the result looks the same as the contents of
 * map.txt
 *
 **/

void printMap(int map[SIZE][SIZE])
{
  for(int i = SIZE -1; i >= 0; i--){
    for(int j = 0; j < SIZE; j++)
      {
	std::cout << map[i][j] << " ";
      }
    std::cout << std::endl;
  }

} // End of printMap()

/**
 * writeMap
 *
 * Write a map into map-out.txt in such a way that the [0][0] element
 * ends up in the bottom left corner of the file (so that the contents
 * of the file look like the relevant occupancy grid.
 *
 **/

void writeMap(int map[SIZE][SIZE])
{
  std::ofstream mapFile;
  mapFile.open("map-out.txt");

  for(int i = SIZE - 1; i >= 0; i--){
    for(int j = 0; j < SIZE; j++)
      {
	mapFile << map[i][j];
      }
    mapFile << std::endl;
  }

  mapFile.close();
}

/**
 * readPosition()
 *
 * Read the position of the robot from the localization proxy. 
 *
 * The localization proxy gives us a hypothesis, and from that we extract
 * the mean, which is a pose. 
 *
 **/

player_pose2d_t readPosition(LocalizeProxy& lp)
{

  player_localize_hypoth_t hypothesis;
  player_pose2d_t          pose;
  uint32_t                 hCount;

  // Need some messing around to avoid a crash when the proxy is
  // starting up.

  hCount = lp.GetHypothCount();

  if(hCount > 0){
    hypothesis = lp.GetHypoth(0);
    pose       = hypothesis.mean;
  }

  return pose;
} // End of readPosition()


void printLaserData(LaserProxy& sp)
{

  double maxRange, minLeft, minRight, range, bearing;
  int points;

  maxRange  = sp.GetMaxRange();
  minLeft   = sp.MinLeft();
  minRight  = sp.MinRight();
  range     = sp.GetRange(5);
  bearing   = sp.GetBearing(5);
  points    = sp.GetCount();

  //Uncomment this to print out useful laser data
  //std::cout << "Laser says..." << std::endl;
  //std::cout << "Maximum distance I can see: " << maxRange << std::endl;
  //std::cout << "Number of readings I return: " << points << std::endl;
  //std::cout << "Closest thing on left: " << minLeft << std::endl;
  //std::cout << "Closest thing on right: " << minRight << std::endl;
  //std::cout << "Range of a single point: " << range << std::endl;
  //std::cout << "Bearing of a single point: " << bearing << std::endl;

  return;
} // End of printLaserData()

/**
 *  printRobotData
 *
 * Print out data on the state of the bumpers and the current location
 * of the robot.
 *
 **/

void printRobotData(BumperProxy& bp, player_pose2d_t pose)
{

  // Print out what the bumpers tell us:
  std::cout << "Left  bumper: " << bp[0] << std::endl;
  std::cout << "Right bumper: " << bp[1] << std::endl;
  // Can also print the bumpers with:
  //std::cout << bp << std::endl;

  // Print out where we are
  std::cout << "We are at" << std::endl;
  std::cout << "X: " << pose.px << std::endl;
  std::cout << "Y: " << pose.py << std::endl;
  std::cout << "A: " << pose.pa << std::endl;

  
} // End of printRobotData()

/**
 * readPlanLength
 *
 * Open the file plan.txt and read the first element, which should be
 * an even integer, and return it.
 *
 **/

int readPlanLength(void)
{
  int length;

  std::ifstream planFile;
  planFile.open("plan.txt");

  planFile >> length;
  planFile.close();

  // Some minimal error checking
  if((length % 2) != 0){
    std::cout << "The plan has mismatched x and y coordinates" << std::endl;
    exit(1);
  }

  return length;

} // End of readPlanLength

/**
 * readPlan
 *
 * Given the number of coordinates, read them in from plan.txt and put
 * them in the array plan.
 *
 **/

void readPlan(double *plan, int length)
{
  int skip;

  std::ifstream planFile;
  planFile.open("plan.txt");

  planFile >> skip;
  for(int i = 0; i < length; i++){
    planFile >> plan[i];
  }

  planFile.close();

} // End of readPlan

/**
 * printPlan
 *
 * Print the plan on the screen, two coordinates to a line, x then y
 * with a header to remind us which is which.
 *
 **/

void printPlan(double *plan , int length)
{
  std::cout << std::endl;
  std::cout << "   x     y" << std::endl;
  for(int i = 0; i < length; i++){
    std::cout.width(5);
    std::cout << plan[i] << " ";
    if((i > 0) && ((i % 2) != 0)){
      std::cout << std::endl;
    }
  }
  std::cout << std::endl;

} // End of printPlan


/**
 * writePlan
 * 
 * Send the plan to the file plan-out.txt, preceeded by the information
 * about how long it is.
 *
 **/

void writePlan(double *plan , int length)
{
  std::ofstream planFile;
  planFile.open("plan-out.txt");

  planFile << length << " ";
  for(int i = 0; i < length; i++){
    planFile << plan[i] << " ";
  }

  planFile.close();
}

// Creates a proper plan

void pathOut(int path[][2], int pathlength) {
  std::ofstream os;
  os.open("plan-out.txt");
  os << pathlength << " ";
  for (int i = 0; i < pathlength; i++) {
    os << (double)path[i][0]/2 << " " << (double)path[i][1]/2 << (i == pathlength-1 ? "" : " ");
  }
}

// Truncates the plan reduces waypoints if the slope is common between a set of points.

void truncatePath(int path[][2], int &pathlength) {
  int lt = 0, rt = 1;
  int dx = path[rt][0]-path[rt-1][0];
  int dy = path[rt][1]-path[rt-1][1];
  int ddx, ddy;
  while (rt < pathlength) {
    ddx = path[rt][0]-path[rt-1][0];
    ddy = path[rt][1]-path[rt-1][1];
    if (!(ddx == dx && ddy == dy)) {
      lt++;
      path[lt][0] = path[rt-1][0];
      path[lt][1] = path[rt-1][1];
      dx = path[rt][0]-path[rt-1][0];
      dy = path[rt][1]-path[rt-1][1];
    }
    rt++;
  }
  path[lt][0] = path[rt-1][0];
  path[lt][1] = path[rt-1][1];
  pathlength = lt+1;
}

// Helper function for plan truncation

void swapPathNode(int path[][2], int i, int j) {
  int tmpx = path[i][0]; int tmpy = path[i][1];
  path[i][0] = path[j][0]; path[i][1] = path[j][1];
  path[j][0] = tmpx; path[j][1] = tmpy;
}

// Outputs map file with path indication

void mapOut(int map[][32]) {
  std::ofstream ofs;
  ofs.open("map-out.txt");
  for (int i = 0; i < 32; i++) {
    for (int j = 0; j < 32; j++) {
        ofs << map[i][j] << (j == 31 ? "" : " ");
    }
    if (i < 31) ofs << std::endl;
  }
}

// Creates a path by backtracking through nodes

void createPath(int map[][32], Node *curr, int path[][2], int &pathlength) {
  int pl = 0;
  while (curr != NULL) {
    path[pl][0] = curr->x;
    path[pl][1] = curr->y;
    map[16-curr->y][16+curr->x] = 2;
    curr = curr->prev;
    pl++;
  }
  for (int i = 0; i < (int)pl/2; i++) {
    swapPathNode(path, i, pl-i-1);
  }
  pathlength = pl;
}

// Finds the optimal path produced by A* search

Node* findPath(int path[256][2], int map[32][32], int start[], int end[]) {
  int d[8][2] = {{-1,-1},{0,-1},{1,-1},{0,-1},{0,1},{1,-1},{1,0},{1,1}};
  int newx, newy;
  double cost;
  Graph frontier = Graph();
  Graph explored = Graph();
  frontier.push(start[0], start[1], 0, NULL);
  while (!frontier.isEmpty()) {
    Node *node = frontier.pop();
    if (node->x == end[0] && node->y == end[1]) {
      return node;
    }
    if (explored.contains(*node)) continue;
    explored.push(node);
    for (int i = 0; i < 8; i++) {
      newx = node->x+d[i][0];
      newy = node->y+d[i][1];
      if (newx+16 < 0 || newx+16 >= 32) continue;
      if (16-newy < 0 || 16-newy >= 32) continue;
      if (map[16-newy][16+newx] == 1) continue;
      cost = (node->cost) + 1 + manhattanDistance(newx, newy, end[0], end[1]);
      frontier.push(newx, newy, cost, node);
    }
    frontier.sort();
  }
  return NULL;
}

// Heuristic used for A* search

double manhattanDistance(int x1, int y1, int x2, int y2) {
  double dx, dy;
  dx = abs(x2 - x1);
  dy = abs(y2 - y1);
  return dx+dy;
}


