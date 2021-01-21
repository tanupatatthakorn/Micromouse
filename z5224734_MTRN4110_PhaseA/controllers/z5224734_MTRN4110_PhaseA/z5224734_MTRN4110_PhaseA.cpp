// File:          z5224734_MTRN4110_PhaseA.cpp
// Date:          20/06/2020
// Description:   Predetermined path planned locomotion
// Author:        Tanupat Atthakorn
// Modifications: Please do not adjust the timestep of the main loop or 
//                the sensors (64).

#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/DistanceSensor.hpp>
#include <webots/PositionSensor.hpp>
#include <webots/Compass.hpp>
#include <iostream>
#include <fstream>
#include <string>
#include <math.h>
#include <vector>
#include <queue>
#include <list>

#define FILE_PATH_IN "../../PathPlan.txt"
#define TIME_STEP 64
#define LIN_STEP 165.0/20.0 
#define ROT_STEP pi/2*28.3/20.0
#define MAX_SPEED 6.28
const double pi = 3.14159265358979323846;
#define FILE_PATH "../../testMap.txt"
#define FILE_OUT_PATH "../../../PathPlanFound.txt"
const int ROW = 5;
const int COL = 9;
const int TARGET_ROW = 2;
const int TARGET_COL = 4;
const int DEFAULT_START_ROW = 0;
const int DEFAULT_START_COL = 0;
//struct for storing coordinates and headings
struct Coor{
    int r;
    int c;
    int heading;
};

// All the webots classes are defined in the "webots" namespace
using namespace webots;

using namespace std;


//--------MAIN----------
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();
  
  //init dist sensors
  DistanceSensor *ds[4]; // 0=front, 1=right, 2=back, 3=left
  char dsNames[4][10] = {
    "dsF", "dsR", "dsB", "dsL"
  };
  for (int i = 0; i < 4; i++) {
    ds[i] = robot->getDistanceSensor(dsNames[i]);
    ds[i]->enable(TIME_STEP);
  }
  
  //init compass
  Compass *compass = robot->getCompass("mycompass");
  compass->enable(TIME_STEP);
  
  ///init motors
  Motor *leftmotor = robot->getMotor("left wheel motor");
  Motor *rightmotor = robot->getMotor("right wheel motor");
  
  //init position sensors
  PositionSensor *leftpos = robot->getPositionSensor("left wheel sensor");
  PositionSensor *rightpos = robot->getPositionSensor("right wheel sensor");
  leftpos->enable(TIME_STEP);
  rightpos->enable(TIME_STEP);
  
  cout << "Start - Read path plan from " << FILE_PATH << endl;
  
  //read string from txt
  ifstream myfile(FILE_PATH);
  string path;
  myfile >> path;
  cout << path << endl;
  
  cout << "Done - Path plan read!" << endl; 
  
  //DECLARE VARIABLES
  //string command starts at index 3.
  int i = 3;
  int count = 0;
  
  //locomotion and encoder
  // STATE 0 => read str and move / 1 => read sensors and wait / 2=> IDLE
  int STATE = 0; 
  bool fFlag = false;
  bool lFlag = false;
  bool rFlag = false;
  double lCount = 0;
  double rCount = 0;
  double leftposition = 0;
  double rightposition = 0;
  
  //pose and heading
  double compassVal[3];
  char row = path[0];
  char col = path[1]; 
  char heading = path[2];
  
  //distance sensors
  char wallFlag[4]; //F,R,B,L
  
  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();
  
  cout << "Start - Execute path plan!" << endl;
  
  
  // -------------Main loop:--------------
  while (robot->step(64) != -1) {
   
   // reading distance sensor values
   double dsValues[4];
   for (int i = 0; i < 4 ; i++){
   dsValues[i] = ds[i]->getValue();
   }
   
   // reading compass for heading
   for (int i = 0; i < 3; i++){
     compassVal[i] = *(compass->getValues() + i);
   }
   
   // set wall flags
   for(int i = 0; i<4; i++){
     if(dsValues[i] < 650){
       wallFlag[i] = 'Y';
     } else {
       wallFlag[i] = 'N';
     }
   }
   
   // set heading from compass
   if(abs(compassVal[0]-1) < 0.01){
     heading = 'N';
   } else if(abs(compassVal[0]+1) < 0.01){
     heading = 'S';
   } else if(abs(compassVal[2]-1) < 0.01){
     heading = 'W';
   } else if(abs(compassVal[2]+1) < 0.01){
     heading = 'E';
   }
   
   
    //DO AND WAIT FSM
    switch (STATE){

        case (0) : //DO AND PRINT
           
          //PRINTING VALUES
          cout << "Step: "; 
          if(count < 10)
            cout << 0;
          cout << count++ << ", Row: " << row
          << ", Column: " << col << ", Heading: " << heading << "," <<
          " Left Wall: " << wallFlag[3] << ", Front Wall: " << 
          wallFlag[0] << ", Right Wall " << wallFlag[1] << endl;
          
          if(i == path.length()){
            cout << "Done - Path plan executed!" << endl;
            STATE = 2;
          }
              
          switch (path[i]){
            case('F') : //forward
              leftmotor->setVelocity(MAX_SPEED*1);
              rightmotor->setVelocity(MAX_SPEED*1);
              leftposition += LIN_STEP;
              rightposition += LIN_STEP;
              fFlag = true;
              i++;
                  
              //determine pose based on heading
              switch(heading){
                case('N'): row--; break;
                case('E'): col++; break;
                case('S'): row++; break;
                case('W'): col--;
              }
                  
            STATE = 1;
            break;
            case('L') : //turn left
              leftmotor->setVelocity(MAX_SPEED*0.4);
              rightmotor->setVelocity(MAX_SPEED*0.4);
              leftposition -= ROT_STEP;
              rightposition += ROT_STEP;
              lFlag = true;
              i++;
              STATE = 1;
            break;
            case('R') : //turn right
              leftmotor->setVelocity(MAX_SPEED*0.4);
              rightmotor->setVelocity(MAX_SPEED*0.4);
              leftposition += ROT_STEP;
              rightposition -= ROT_STEP;
              rFlag = true;
              i++;
              STATE = 1;
          }
                     
          break;

        case (1) : //encoder reading / WAITING
        
          //position values
          double lEn = leftpos->getValue(); 
          double rEn = rightpos->getValue(); 
          // encoder diff
          double ldiff = lEn - lCount;
          double rdiff = rEn - rCount;
            
          //cout << "diff: " << ldiff << " " << rdiff << " En: " <<
          //rEn << " Count " << rCount << endl;
            
          if (fFlag && abs(ldiff-8.25 ) < 0.15){
                
            fFlag = false;
            lCount = lEn;
            rCount = rEn;
            //reset(ldiff, rdiff, lCount, rCount, lEn, rEn);
            STATE = 0;
              
          } else if(lFlag && abs(rdiff-2.2227) <= 0.1){
          
            lFlag = false;
            lCount = lEn;
            rCount = rEn;
            STATE = 0;
              
          } else if(rFlag && abs(ldiff-2.2227) <= 0.1 ){
              
            rFlag = false;
            lCount = lEn;
            rCount = rEn;
            STATE = 0;
          }
            
          //set MOTOR
          leftmotor->setPosition(leftposition);
          rightmotor->setPosition(rightposition);        

    }
    
    

  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}

void PathPlan(){
  //------------INIT/DECLARE VARS---------------
    ifstream myfile(FILE_PATH);
    string map;
    int startrow = DEFAULT_START_ROW;
    int startcol = DEFAULT_START_COL;
    int startheading = 0; //(N,E,S,W)
    int hrow = 0;
    int hcol = 0;
    int line = 0;
    int vWall[5][10] = {0};
    int hWall[6][9] = {0};
    int cellVal[5][9];
    int vrow = 0;
    int vcol = 0;
  
//---------------READ MAP-------------------
    cout << "--- Task 1 ---" << endl;
  while(getline(myfile,map)){
    //read and display map
    cout << map << endl;
    if(line%2==0){
      for(int i = 2; i<map.length();i+=4){
        if(map[i]=='-') {
          hWall[hrow][hcol] = 1;
        } else {
          hWall[hrow][hcol] = 0;
        }
        hcol++;
      }
      hcol = 0;
      hrow++;
    } else {
      for(int i = 0; i<map.length();i+=4){
        if(map[i]=='|') {
          vWall[vrow][vcol] = 1;
        } else {
          vWall[vrow][vcol] = 0;
        }
        vcol++;
      }
      vrow++;
      vcol = 0;
    }

    if(line%2){
      for(int i = 0; i < 10; i++){
        switch(map[4*i+2]){
          case('^'): startheading = 0; startrow = line/2; startcol = i; break;
          case('>'): startheading = 1; startrow = line/2; startcol = i; break;
          case('v'): startheading = 2; startrow = line/2; startcol = i; break;
          case('<'): startheading = 3; startrow = line/2; startcol = i;
        }
      }
    }
    line++;//check for even or odd lines of string
  } //end of while
    

//------------FLOODFILL------------------
    //initilize
    for(int i =0;i<5;i++){
      for(int j = 0; j < 9; j++)
        cellVal[i][j] = ROW*COL;
    }
    
    cellVal[TARGET_ROW][TARGET_COL] = 0;
    
    int N = ROW*COL;
    int explored = 0;
    bool changed = true;
    
    while(changed != 0){
      changed = false;
      for(int row = 0; row < 5; row++){
        for(int col = 0; col < 9; col++){
          if(cellVal[row][col]==explored){
            //check N wall
            if(!hWall[row][col]){
              if(cellVal[row-1][col] == N){
                cellVal[row-1][col] = cellVal[row][col] + 1;
                changed = true;
              }
            }
            //check E wall
            if(!vWall[row][col+1]){
              if(cellVal[row][col+1] == N){
                cellVal[row][col+1] = cellVal[row][col] + 1;
                changed = true;
              }
            }
            //check S wall
            if(!hWall[row+1][col]){
              if(cellVal[row+1][col] == N){
                cellVal[row+1][col] = cellVal[row][col] + 1;
                changed = true;
              }
            }
            //check W wall
            if(!vWall[row][col]){
              if(cellVal[row][col-1] == N){
                cellVal[row][col-1] = cellVal[row][col] + 1;
                changed = true;
              }
            }
          }
        }
      }
      explored++;
    }
    
 
    /*for(int i = 0; i<5; i++){
        for(int j = 0; j<9;j++) {
            cout << cellVal[i][j];
            if (cellVal[i][j] < 10)
                cout << " ";
            cout << " | ";
        }
        cout << "\n";
    } */
    
    //------------------PATH PLAN-------------------
    cout << "--- Task 2 ---" << endl;
    //direction arrays
    int dr[] = {-1, 0 , 1, 0};
    int dc[] = {0, 1, 0, -1};
    int cr, cc, nr, nc, head;

    //initializing containers
    vector<Coor> path;
    vector<Coor> copy_path;
    queue< vector<Coor> > q;
    list< vector<Coor> > completedPaths;
    //initializing Coor
    Coor current_coor;
    current_coor.r = startrow;
    current_coor.c = startcol;
    current_coor.heading = startheading;
    path.push_back(current_coor);
    q.push(path);
    
    int total_paths = 0;

    while (!q.empty()){
        //cout << "q size: " << q.size() << endl;
        path = q.front();
        q.pop();
        //current row,col,heading
        cr = path.back().r;
        cc = path.back().c;
        head = path.back().heading;
        int currentVal = cellVal[path.back().r][path.back().c];
        int pathFound = 0;
        
        for (int i = 0; i < 4; i++){
            Coor next_coor;
            nr = cr + dr[i];
            nc = cc + dc[i];
            next_coor.r = nr;
            next_coor.c = nc;
            next_coor.heading = i;
            //check boundaries
            if (next_coor.r < 0 || next_coor.r >= ROW || next_coor.c < 0 || next_coor.c >= COL)
                continue;
            
            //check for walls
            if (i == 0 && hWall[cr][cc] == 1){
                //cout << "N wall " << i << endl;
                continue;
            } else if (i == 1 && vWall[cr][cc+1] == 1){
                //cout << "E wall " << i << endl;
                continue;
            } else if (i == 2 && hWall[cr+1][cc] == 1){
                //cout << "S wall " << i << endl;
                continue;
            } else if (i == 3 && vWall[cr][cc] == 1){
                //cout << "W wall " << i << endl;
                continue;
            }
            
            //check if complete
            if (cellVal[nr][nc] == 0){
                path.push_back(next_coor);
                completedPaths.push_back(path);
                total_paths++;
            }
            //check if path found
            if (cellVal[nr][nc] == currentVal-1){
                //if more than on path found then branch by copy and push onto queue
                if(pathFound == 0){
                    path.push_back(next_coor);
                    q.push(path);
                    path.pop_back();
                } else {
                    copy_path = path;
                    copy_path.push_back(next_coor);
                    q.push(copy_path);
                }
                //increment paths found
                pathFound++;
                
            } else{
                //do nothing: dead branch, removed in next iteration
            }
        }
    }

    //----------------PATH PRINTING--------------------
    //masks and turns initialized
    bool mask[5][9] = {0};
    bool bestMask[5][9] = {0};
    int turns = 0;
    int least_turns = 0;;
    int heading = startheading;
    int n = 1;
    //motion sequence initialized
    string seq = "";
    string best_seq = "";
    string motion_seq = to_string(startrow) + to_string(startcol);
    switch (startheading){
        case 0: motion_seq += "N"; break;
        case 1: motion_seq += "E"; break;
        case 2: motion_seq += "S"; break;
        case 3: motion_seq += "W";
    }
    
    //loop through list of shortest paths
    //go through all vectors in list
    for (auto list_it = completedPaths.begin(); list_it != completedPaths.end(); list_it++){
        
        //set initial Coor
        heading = startheading;
        turns = 0;
        seq = "";
        
        //go through all objects in vector
        for (auto vec_it = list_it->begin(); vec_it != (*list_it).end(); vec_it++){
            mask[vec_it->r][vec_it->c] = 1;
            //printf("(%d,%d,%d)", vec_it->r, vec_it->c, vec_it->heading);
            
            if (vec_it->r == startrow && vec_it->c == startcol)
                continue;
            
            //iterate through heading of Coor and find condition for change in heading and get turns
            if (vec_it->heading != heading){
                //check heading for motion sequence
                if (abs(vec_it->heading - heading) == 2){
                    seq+= "LL";
                    turns++;
                } else if (vec_it->heading == 3 && heading == 0){
                    seq += "L";
                } else if (vec_it->heading == 0 && heading == 3){
                    seq += "R";
                } else if (vec_it->heading < heading){
                    seq += "L";
                } else if (vec_it->heading > heading){
                    seq += "R";
                }
                heading = vec_it->heading;
                turns++;
            }
            seq += "F";
        }
        //for first path
        if (least_turns == 0){
            least_turns = turns;
            best_seq = seq;
            for (int i = 0; i < ROW; i++){
                for (int j = 0; j < COL; j++)
                    bestMask[i][j] = mask[i][j];
            }
        }
        //if replace mask if new least turns found
        if (turns < least_turns){
            least_turns = turns;
            best_seq = seq;
            for (int i = 0; i < ROW; i++){
                for (int j = 0; j < COL; j++)
                    bestMask[i][j] = mask[i][j];
            }
        }
        //-------------Print Paths-----------------
        printf("--- Path %d ---\n", n++);
        printmap(mask, cellVal, vWall, hWall, startrow, startcol, startheading);
        //clear mask
        for (auto vec_it = list_it->begin(); vec_it != (*list_it).end(); vec_it++){
            mask[vec_it->r][vec_it->c] = 0;
        }
        cout << endl;
    }
    //cout << "Done - " << total_paths << " shortest paths found!" << endl;
    
    //----------------Find shortest path with least turns--------------------
    cout << "--- Task 3 ---" << endl;
    printmap(bestMask, cellVal, vWall, hWall, startrow, startcol, startheading);
    
    motion_seq += best_seq;
    printf("Steps: %d\nPath: ", int(best_seq.size()));
    for (int i = 0; i < motion_seq.size(); i++){
        cout << motion_seq[i];
    }
    cout << endl;
    //cout << "Done - Shortest path with least turns found!" << endl;
    
    //----------------Write file to PathPlanFound.txt--------------------
    cout << "--- Task 4 ---" << endl;
    cout << "File: " << FILE_OUT_PATH << endl;
    //write shortest motion sequence to txt file
    ofstream outfile(FILE_OUT_PATH);
    outfile << motion_seq << endl;
    //read shortest motion sequence from txt file
    string input_path;
    ifstream infile(FILE_OUT_PATH);
    getline(infile, input_path);
    cout << "Path: " << input_path << endl;
    
}