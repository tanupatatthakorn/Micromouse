//
//  main.cpp
//  pathplanning
//
//  Created by Tanupat Atthakorn on 2/7/2563 BE.
//  Copyright © 2563 Tanupat Atthakorn. All rights reserved.
//

#include <string>
#include <iostream>
#include <fstream>
#include <math.h>
#include <vector>
#include <queue>
#include <list>

#define FILE_PATH "../../testMap.txt"
#define FILE_OUT_PATH "../../../PathPlanFound.txt"
const int ROW = 5;
const int COL = 9;
const int TARGET_ROW = 2;
const int TARGET_COL = 4;
const int DEFAULT_START_ROW = 0;
const int DEFAULT_START_COL = 0;

using namespace std;

//struct for storing coordinates and headings
struct Coor{
    int r;
    int c;
    int heading;
};

void printmap(bool mask[][9], int cellVal[][9], int vWall[][10], int hWall[][9], int startrow, int startcol, int startheading);

int main(int argc, char **argv) {
  
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
    
  return 0;
}
//-----------------END OF MAIN()-------------------


//--------------FUNCTIONS-------------------
void printmap(bool mask[][9], int cellVal[][9], int vWall[][10], int hWall[][9], int startrow, int startcol, int startheading){
    //-------------PRINT MAP----------------
      for (int i=0;i<5*2+1;i++){
      if (i%2){//odd
        for (int j=0;j<(9*2)+1;j++){
          if (j%2){//odd
              if (startrow == i/2 && startcol == j/2){
              switch(startheading){
                case 0: std::cout << "^ ";break;
                case 1: std::cout << "> ";break;
                case 2: std::cout << "v ";break;
                case 3: std::cout << "< ";break;
              }
              }
            else{
              if (mask[i/2][j/2]){
                cout << cellVal[i/2][j/2];
                if (cellVal[i/2][j/2]<10 )
                  cout << " ";
              }
              else
                cout << "  ";
            }
          }
          else{//even
            if (vWall[i/2][j/2])
              cout << "| ";
            else
              cout << "  ";
          }
        }
        cout << "\n";
      }
      else{//even(horizontal wall
        cout << " ";
        for (int j=0;j<9;j++){
          if(hWall[i/2][j])
            std::cout << "---";
          else
            std::cout << "   ";
          std::cout << " ";
        }
        std::cout << "\n";
      }
    }
    
}
