
class Node;

//Developer of the entire class (both header file and cpp file), with membervariables and functions: Ask. 
extern int x_start;
extern int y_start;
class Maze
{
private: 
  Vector<Vector<std::shared_ptr<Node>>> m_grid;
  int m_num_columns;
  int m_num_rows;
 
public: 
  Maze(int num_rows, int num_columns);
  int getNumRows();
  int getNumCol();
  Vector<Vector<std::shared_ptr<Node>>> getGrid();
  void setStartCell();
  void setGoalCells();
  void setNewMaze(Vector<Vector<std::shared_ptr<Node>>>);
  void printPointerInfo(std::shared_ptr<Node> cell);
  void printNetwork();
};