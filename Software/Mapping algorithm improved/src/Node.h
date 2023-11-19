#include <ArxSmartPtr.h>
#include <Vector.h>

//Developer of the entire class (both header file and cpp file), with membervariables and functions: Ask. 

class Node 
{
private: 
	std::shared_ptr<Node> m_north_ptr = nullptr;
	std::shared_ptr<Node> m_east_ptr = nullptr;
	std::shared_ptr<Node> m_south_ptr = nullptr;
	std::shared_ptr<Node> m_west_ptr = nullptr;
	bool m_isVisited;
	double m_xidx;
	double m_yidx;
public: 
	Node(){};
	Node(double x_idx, double y_idx, bool isVisited);

	std::shared_ptr<Node> getNorthptr();
	std::shared_ptr<Node> getSouthptr();
	std::shared_ptr<Node> getEastptr();
	std::shared_ptr<Node> getWestptr();
	void setNorthptr(std::shared_ptr<Node> n);
	void setSouthptr(std::shared_ptr<Node> n);
	void setEastptr(std::shared_ptr<Node> n);
	void setWestptr(std::shared_ptr<Node> n);
	bool getIsVisited();
	double getXidx();
	double getYidx();
   	void setIsVisited(bool a);
};