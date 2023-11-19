#include "Micromouse.h"
#include "Maze.h"
#include "Node.h"

int prev_y; //these are global variables used in memberfunctions of the micromouse class. 
int new_y; 
int prev_x; 
int new_x;
double limit = 15;

Micromouse::Micromouse(){}; //Default constructor.

Micromouse::Micromouse(double l_s_val, double r_s_val, double f_s_val, int position, std::shared_ptr<Node> a, 
  std::shared_ptr<Maze> b): //Constructor with parameters to set membervariables.
  m_left_sensor_val(l_s_val), m_right_sensor_val(r_s_val), m_front_sensor_val(f_s_val), m_position(position),
  m_active_cell(a), m_maze_obj(b){}

//Standard getter and setter functions below:
double Micromouse::getLeftSensorVal(){
    return m_left_sensor_val;
  }

double Micromouse::getRightSensorVal(){
    return m_right_sensor_val;
  }
double Micromouse::getFrontSensorVal(){
    return m_front_sensor_val;
  }
  int Micromouse::getPosition(){
    return m_position;
  }
  std::shared_ptr<Node> Micromouse::getActiveCell(){
    return m_active_cell;
  }
  std::shared_ptr<Maze> Micromouse::getMazeObj(){
    return m_maze_obj;
  }
  void Micromouse::setLeftSensorVal(double value){
    m_left_sensor_val = value;
  }
  void Micromouse::setRightSensorVal(double value){
    m_right_sensor_val = value;
  }
  void Micromouse::setFrontSensorVal(double value){
    m_front_sensor_val = value;
  }

  void Micromouse::setPosititon(int value){
    if (m_position + value == 360 || m_position + value == -360){
      m_position = 0;
    }
    m_position += value;
  }

  void Micromouse::setActiveCell(std::shared_ptr<Node> n){
    m_active_cell = n;
  }
  void Micromouse::setMazeObj(std::shared_ptr<Maze> n){
    m_maze_obj = n;
  }
  bool Micromouse::isGoal(){ //Function to check if the micromouse has entered one of the four goal/middle cells:
      if (
      (m_active_cell->getXidx() == (m_maze_obj->getNumCol()/2)-1 && m_active_cell->getYidx() == (m_maze_obj->getNumRows()/2)-1) ||
      (m_active_cell->getXidx() == (m_maze_obj->getNumCol()/2)-1 && m_active_cell->getYidx() == (m_maze_obj->getNumRows()/2)) || 
      (m_active_cell->getXidx() == (m_maze_obj->getNumCol()/2) && m_active_cell->getYidx() == (m_maze_obj->getNumRows()/2)-1) || 
      (m_active_cell->getXidx() == (m_maze_obj->getNumCol()/2) && m_active_cell->getYidx() == (m_maze_obj->getNumRows()/2))){
        return true;
      }
      return false;     
  }
  //Functions to connect the current cell/node the micromouse is in, with its neighbouring left, right or forward nodes based on position. Explanation behind the logic of this function 
  //can be seen in my blog contribution from week 11.   
  void Micromouse::connectNodesLeft(std::shared_ptr<Node> current, std::shared_ptr<Node> next, int pos){
    if (pos == 0){
      current->setWestptr(next);
      next->setEastptr(current);
    }
    if (pos == 90 || pos == -270){
      current->setNorthptr(next);
      next->setSouthptr(current);
    }
    if (pos == 180 || pos == -180){
      current->setEastptr(next);
      next->setWestptr(current);
    }
    if (pos == -90 || pos == 270){
      current->setSouthptr(next);
      next->setNorthptr(current);
    }
  }
  void Micromouse::connectNodesRight(std::shared_ptr<Node> current, std::shared_ptr<Node> next, int pos){
    if (pos == 0){
      current->setEastptr(next);
      next->setWestptr(current);
    }
    if (pos == 90 || pos == -270){
      current->setSouthptr(next);
      next->setNorthptr(current);
    }
    if (pos == 180 || pos == -180){
      current->setWestptr(next);
      next->setEastptr(current);
    }
    if (pos == -90 || pos == 270){
      current->setNorthptr(next);
      next->setSouthptr(current);
    }
  }

  void Micromouse::connectNodesForw(std::shared_ptr<Node> current, std::shared_ptr<Node> next, int pos){
    if (pos == 0){
      current->setNorthptr(next);
      next->setSouthptr(current);
    }
    if (pos == 90 || pos == -270){
      current->setEastptr(next);
      next->setWestptr(current);
    }
    if (pos == 180 || pos == -180){
      current->setSouthptr(next);
      next->setNorthptr(current);
    }
    if (pos == -90 || pos == 270){
      current->setWestptr(next);
      next->setEastptr(current);
    }
  }
 //The createAndConnect functions below creates a new neighbouring node and connects it to the active cell/node the micromouse is currently in, in the right, forward and left directions (seperately) based on position.
  void Micromouse::createAndConnectForwNode(int act_x, int act_y, int pos, Vector<Vector<std::shared_ptr<Node>>>& grid){
      if (pos == 0){
        prev_y = act_y;
        new_y = act_y++;
        if (grid[act_x][new_y] == nullptr){ //If there is a nullptr node on the next index you want to create cells/squares: create and connect the new node on that index to 
        //the cell the micromouse is currently in.
          grid[act_x][new_y] = std::make_shared<Node>(act_x, new_y, false);
          connectNodesForw(grid[act_x][prev_y], grid[act_x][new_y], pos);
        } else if (grid[act_x][new_y]->getIsVisited() == true){ //If there is no nullptr node on the next index and the node has already been visited, 
        //we dont want to create and connect a new node on this index. So, just returning is the appropriate action here.
          return;
        } 
        else {
          connectNodesForw(grid[act_x][prev_y], grid[act_x][new_y], pos); //If a node on the next index is not a nullptr, but it hasn't been visited,
          //we want to only connect it to the active cell the micromouse is in.
        }
      }
      //The comments above is valid for the if-tests below (of this function and the right- and left-createAndConnect functions below) as well, but the position and thereby the creation and connection direction is of course different.  
      if (pos == 90 || pos == -270){
        prev_x = act_x;
        new_x = act_x++;
        if (grid[new_x][act_y] == nullptr){
          grid[new_x][act_y] = std::make_shared<Node>(new_x, act_y, false);
          connectNodesForw(grid[prev_x][act_y], grid[new_x][act_y], pos);
        } else if (grid[new_x][act_y]->getIsVisited() == true){
          return;
        }
        else {
          connectNodesForw(grid[prev_x][act_y], grid[new_x][act_y], pos);
        }
      }
      if (pos == 180 || pos == -180){
        prev_y = act_y;
        new_y = act_y--;
        if (grid[act_x][new_y] == nullptr){
          grid[act_x][new_y] = std::make_shared<Node>(act_x, new_y, false);
          connectNodesForw(grid[act_x][prev_y], grid[act_x][new_y], pos);
        } else if (grid[act_x][new_y]->getIsVisited() == true){
          return;
        } else {
          connectNodesForw(grid[act_x][prev_y], grid[act_x][new_y], pos);
        }
      }
      if (pos == -90 || pos == 270){
        prev_x = act_x;
        new_x = act_x--;
        if (grid[new_x][act_y] == nullptr){
          grid[new_x][act_y] = std::make_shared<Node>(new_x, act_y, false);
          connectNodesForw(grid[prev_x][act_y], grid[new_x][act_y], pos);
        } else if (grid[new_x][act_y]->getIsVisited() == true){
          return;
        } 
        else {
          connectNodesForw(grid[prev_x][act_y], grid[new_x][act_y], pos);
        }
      }
      }
    
    void Micromouse::createAndConnectRightNode(int act_x, int act_y, int pos, Vector<Vector<std::shared_ptr<Node>>>& grid){
      if (pos == 0){
        prev_x = act_x;
        new_x = act_x++;
        if (grid[new_x][act_y] == nullptr){
          grid[new_x][act_y] = std::make_shared<Node>(new_x, act_y, false);
          connectNodesRight(grid[prev_x][act_y], grid[new_x][act_y], pos);
        } else if (grid[new_x][act_y]->getIsVisited() == true){
          return;
        } 
        else {
          connectNodesRight(grid[prev_x][act_y], grid[new_x][act_y], pos);
        }
      }
      if (pos == 90 || pos == -270){
        prev_y = act_y;
        new_y = act_y--;
        if (grid[act_x][new_y] == nullptr){
          grid[act_x][new_y] = std::make_shared<Node>(act_x, new_y, false);
          connectNodesRight(grid[act_x][prev_y], grid[act_x][new_y], pos);
        } else if (grid[act_x][new_y]->getIsVisited() == true){
          return;
        } 
        else {
          connectNodesRight(grid[act_x][prev_y], grid[act_x][new_y], pos);
        }
      }
      if (pos == 180 || pos == -180){
        prev_x = act_x;
        new_x = act_x--;
        if (grid[new_x][act_y] == nullptr){
          grid[new_x][act_y] = std::make_shared<Node>(new_x, act_y, false);
          connectNodesRight(grid[prev_x][act_y], grid[new_x][act_y], pos);
        } else if (grid[new_x][act_y]->getIsVisited() == true){
          return;
        } 
        else {
          connectNodesRight(grid[prev_x][act_y], grid[new_x][act_y], pos);
        }
      }
      if (pos == -90 || pos == 270){
        prev_y = act_y;
        new_y = act_y--;
        if (grid[act_x][new_y] == nullptr){
          grid[act_x][new_y] = std::make_shared<Node>(act_x, new_y, false);
          connectNodesRight(grid[act_x][prev_y], grid[act_x][new_y], pos);
        } else if (grid[act_x][new_y]->getIsVisited() == true){
          return;
        } 
        else {
          connectNodesRight(grid[act_x][prev_y], grid[act_x][new_y], pos);
        }
      }

      }
      void Micromouse::createAndConnectLeftNode( int act_x, int act_y, int pos, Vector<Vector<std::shared_ptr<Node>>>& grid){
      if (pos == 0){
        prev_x = act_x;
        new_x = act_x--;
        if (grid[new_x][act_y] == nullptr){
          grid[new_x][act_y] = std::make_shared<Node>(new_x, act_y, false);
          connectNodesLeft(grid[prev_x][act_y], grid[new_x][act_y], pos);
        } else if (grid[new_x][act_y]->getIsVisited() == true){
          return;
        } 
        else {
          connectNodesLeft(grid[prev_x][act_y], grid[new_x][act_y], pos);
        }
      }
      if (pos == 90 || pos == -270){
        prev_y = act_y;
        new_y = act_y++;
        if (grid[act_x][new_y] == nullptr){
          grid[act_x][new_y] = std::make_shared<Node>(act_x, new_y, false);
          connectNodesLeft(grid[act_x][prev_y], grid[act_x][new_y], pos);
        } else if (grid[act_x][new_y]->getIsVisited() == true){
          return;
        } 
        else {
          connectNodesLeft(grid[act_x][prev_y], grid[act_x][new_y], pos);
        }
      }
      if (pos == 180 || pos == -180){
        prev_x = act_x;
        new_x = act_x++;
        if (grid[new_x][act_y] == nullptr){
          grid[new_x][act_y] = std::make_shared<Node>(new_x, act_y, false);
          connectNodesLeft(grid[prev_x][act_y], grid[new_x][act_y], pos);
        } else if (grid[new_x][act_y]->getIsVisited() == true){
          return;
        } 
        else {
          connectNodesLeft(grid[prev_x][act_y], grid[new_x][act_y], pos);
        }
      }
      if (pos == -90 || pos == 270){
        prev_y = act_y;
        new_y = act_y--;
        if (grid[act_x][new_y] == nullptr){
          grid[act_x][new_y] = std::make_shared<Node>(act_x, new_y, false);
          connectNodesLeft(grid[act_x][prev_y], grid[act_x][new_y], pos);
        } else if (grid[act_x][new_y]->getIsVisited() == true){
          return;
        } 
        else {
          connectNodesLeft(grid[act_x][prev_y], grid[act_x][new_y], pos);
        }
      }
      }
      
  void Micromouse::createAllNodes(){ //This is a function that calls the createAndConnect functions, if the front, left and right sensor value is bigger than a set limit, to reduce 
  //data redundancy and making this function more readable for the viewer, compared to the previous implementations where this function got terribly long 
  //because the many if-tests of the createAndConnect functions were hardcoded into it.   
    
    int act_x_idx = m_active_cell->getXidx(); //This is the active x and y indexes of the cell the micromouse is currently inside. 
    int act_y_idx = m_active_cell->getYidx();

    Vector<Vector<std::shared_ptr<Node>>> grid_vector = m_maze_obj->getGrid(); //Making a local variable to store the temporary maze grid-vector.
    
    if (m_front_sensor_val > limit){ 
      createAndConnectForwNode(act_x_idx, act_y_idx, m_position, grid_vector);
    }
    if (m_right_sensor_val > limit){
      createAndConnectRightNode(act_x_idx, act_y_idx, m_position, grid_vector);
    }
    if (m_left_sensor_val > limit){
      createAndConnectLeftNode(act_x_idx, act_y_idx, m_position, grid_vector);
    }
    
    m_active_cell->setIsVisited(true); //setting the active cell the micromouse is in to true.
    m_maze_obj->setNewMaze(grid_vector); //setting the new updated maze to be the grid vector I made changes to.
}

void Micromouse::changeActiveCell(){ //This function changes the active cell for the micromouse based on forward movement in different positions.
  std::shared_ptr<Node> prev_cell = m_active_cell; //temporary local variable to store the prev active cell. 

  if (m_position == 0){
    setActiveCell(m_maze_obj->getGrid()[prev_cell->getXidx()][prev_cell->getYidx()+1]); //for instance: If position = 0, the new active cell will be the node at an 
    //index where only the y has increased by one.  
  }
  else if (m_position == 90 || m_position == -270){
    setActiveCell(m_maze_obj->getGrid()[prev_cell->getXidx()+1][prev_cell->getYidx()]);
  }
  else if (m_position == 180 || m_position == -180){
    setActiveCell(m_maze_obj->getGrid()[prev_cell->getXidx()][prev_cell->getYidx()-1]);
  }
  else if (m_position == -90 || m_position == 270){
    setActiveCell(m_maze_obj->getGrid()[prev_cell->getXidx()-1][prev_cell->getYidx()]);
  }
}