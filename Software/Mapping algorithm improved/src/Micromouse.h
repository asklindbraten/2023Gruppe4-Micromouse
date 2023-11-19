#include <ArxSmartPtr.h>
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
  std::shared_ptr<Node> m_active_cell;
  std::shared_ptr<Maze> m_maze_obj;
public: 
  Micromouse();
  Micromouse(double l_s_val, double r_s_val, double f_s_val, int position, std::shared_ptr<Node> a, 
  std::shared_ptr<Maze> b);

  double getLeftSensorVal();
  double getRightSensorVal();
  double getFrontSensorVal();
  int getPosition();
  std::shared_ptr<Node> getActiveCell();
  std::shared_ptr<Maze> getMazeObj();
  void setLeftSensorVal(double value);
  void setRightSensorVal(double value);
  void setFrontSensorVal(double value);
  void setPosititon(int value);
  void setActiveCell(std::shared_ptr<Node> n);
  void setMazeObj(std::shared_ptr<Maze> n);
  bool isGoal();
  void connectNodesLeft(std::shared_ptr<Node> current, std::shared_ptr<Node> next, int pos);
  void connectNodesRight(std::shared_ptr<Node> current, std::shared_ptr<Node> next, int pos);
  void connectNodesForw(std::shared_ptr<Node> current, std::shared_ptr<Node> next, int pos);
  void createAndConnectForwNode( int act_x, int act_y, int pos, Vector<Vector<std::shared_ptr<Node>>>& grid);  
  void createAndConnectRightNode(int act_x, int act_y, int pos, Vector<Vector<std::shared_ptr<Node>>>& grid);
  void createAndConnectLeftNode( int act_x, int act_y, int pos, Vector<Vector<std::shared_ptr<Node>>>& grid);    
  void createAllNodes();
  void changeActiveCell();
};