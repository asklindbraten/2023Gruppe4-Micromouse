#include "Node.h"
#include <Maze.h>


int y_start = 0;
int x_start = 0;

void Maze::setStartCell(){ //setting the start cell in the maze. The micromouse's first active cell will be the node at this position in the grid vector.
    m_grid[x_start][y_start] = new Node(x_start,y_start, false);
  }

  void Maze::setGoalCells(){ //setting the four goal cells based on the number of columns and rows given when constructing the maze object.
    int x_low_idx = m_num_columns/2 - 1;
    int y_low_idx = m_num_rows/2 - 1;
    int x_high_idx = m_num_columns/2;
    int y_high_idx = m_num_rows/2;

    for (int i{x_low_idx}; i <= x_high_idx; i++){
        for (int j{y_low_idx}; j <= y_high_idx; j++){
          m_grid[i][j] = new Node(i, j, false);
        }
    }
  }

Maze::Maze(int num_rows, int num_columns){ //constructor with parameters for the maze object. Number of columns represent the total amount of squares on 
//the x axis and the number of rows represent the total amount of squares on the y axis. These multiplied with eachother gives the total amount of nodes/cells in the entire maze.
    m_num_columns = num_columns;
    m_num_rows = num_rows;

    m_grid = (Node***)malloc(sizeof(Node**) * m_num_rows);
    
    for (int i = 0; i < m_num_rows; i++){
      m_grid[i] = (Node**)malloc(sizeof(Node*)*m_num_columns);
    }
    for (int i = 0; i < m_num_rows; i++){
      for (int j = 0; j < m_num_columns; j++){
        m_grid[i][j] = nullptr;
      }
    }
    setGoalCells(); //Create goal and start cell immediatly when creating the maze 
    setStartCell();
  }
    //Standard setter and getter functions.
  int Maze::getNumRows(){
    return m_num_rows;
  }

  int Maze::getNumCol(){
    return m_num_columns;
  }

  Node*** Maze::getGrid(){
    return m_grid;
  }

  void Maze::setNewMaze(Node* **a){
    m_grid = a;
  } 
  //This function prints the x- and y index of a specific cell, aswell as the neighbouring nodes its connected to. 
  void Maze::printPointerInfo(Node* cell){
    Serial.print("(");
    Serial.print(cell->getXidx());
    Serial.print(", ");
    Serial.print(cell->getYidx());
    Serial.print("): -------> ");
    Serial.print("N: ");
    if (cell->getNorthptr() != nullptr){
      Serial.print("(");
      Serial.print(cell->getNorthptr()->getXidx());
      Serial.print(", ");
      Serial.print(cell->getNorthptr()->getYidx());
      Serial.print(")");
    } else {
      Serial.print("0");
    }
    Serial.print("E: ");
    if (cell->getEastptr() != nullptr){
      Serial.print("(");
      Serial.print(cell->getEastptr()->getXidx());
      Serial.print(", ");
      Serial.print(cell->getEastptr()->getYidx());
      Serial.print(")");
    } else {
      Serial.print("0");
    }
    Serial.print("S: ");
    if (cell->getSouthptr() != nullptr){
      Serial.print("(");
      Serial.print(cell->getSouthptr()->getXidx());
      Serial.print(", ");
      Serial.print(cell->getSouthptr()->getYidx());
      Serial.print(")");
    } else {
      Serial.print("0");
    }
    Serial.print("W: ");
    if (cell->getWestptr() != nullptr){
      Serial.print("(");
      Serial.print(cell->getWestptr()->getXidx());
      Serial.print(", ");
      Serial.print(cell->getWestptr()->getYidx());
      Serial.print(")");
    } else {
      Serial.print("0");
    }
  }

  void Maze::printNetwork(){ //This function uses a double for-loop to print the entire node-network or graph, by checking if the node at the indexes 
  //is not a nullptr and then passing it as a parameter to the function above.
    for (int i{0}; i < m_num_rows; i++){
      for (int j{0}; j < m_num_columns; j++){
        if (m_grid[i][j] != nullptr){
        printPointerInfo(m_grid[i][j]);
        Serial.println();
        }
      }
    }
  }
