#include <Vector.h>

//Developer of the entire class (both header file and cpp file), with membervariables and functions: Ask. 

class Maze;
class Node;
//To prevent redefinitions of the variables below I used extern.
extern int prev_y;
extern int new_y; 
extern int prev_x; 
extern int new_x;
extern double limit;

class Micromouse
{
private: 
  double m_left_sensor_val;
  double m_right_sensor_val;
  double m_front_sensor_val;
  int m_position = 0;
  Node* m_active_cell;
  Maze* m_maze_obj;
public: 
  Micromouse();
  Micromouse(double l_s_val, double r_s_val, double f_s_val, int position, Node *a, 
  Maze *b);
  ~Micromouse();
  double getLeftSensorVal();
  double getRightSensorVal();
  double getFrontSensorVal();
  int getPosition();
  Node* getActiveCell();
  Maze* getMazeObj();
  void setLeftSensorVal(double value);
  void setRightSensorVal(double value);
  void setFrontSensorVal(double value);
  void setPosititon(int value);
  void setActiveCell(Node* n);
  void setMazeObj(Maze* n);
  bool isGoal();
  void connectNodesLeft(Node* current, Node* next, int pos);
  void connectNodesRight(Node* current, Node* next, int pos);
  void connectNodesForw(Node* current, Node* next, int pos);
  void createAndConnectForwNode( int act_x, int act_y, int pos, Node*** grid);  
  void createAndConnectRightNode(int act_x, int act_y, int pos, Node*** grid);
  void createAndConnectLeftNode( int act_x, int act_y, int pos, Node*** grid);    
  void createAllNodes();
  void changeActiveCell();
};