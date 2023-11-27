#include <Vector.h>

//Developer of the entire class (both header file and cpp file), with membervariables and functions: Ask. 

class Node 
{
private: 
	Node* m_north_ptr = nullptr;
	Node* m_east_ptr = nullptr;
	Node* m_south_ptr = nullptr;
    Node* m_west_ptr = nullptr;
	bool m_isVisited;
	int m_xidx;
	int m_yidx;
public: 
	Node(){};
	Node(int x_idx, int y_idx, bool isVisited);
	~Node();
	Node* getNorthptr();
	Node* getSouthptr();
	Node* getEastptr();
	Node* getWestptr();
	void setNorthptr(Node* n);
	void setSouthptr(Node* n);
	void setEastptr(Node* n);
	void setWestptr(Node* n);
	bool getIsVisited();
	int getXidx();
	int getYidx();
   	void setIsVisited(bool a);
};