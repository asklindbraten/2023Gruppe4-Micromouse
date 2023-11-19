#include "Node.h"

	Node::Node(double x_idx, double y_idx, bool isVisited) : //Constructor with parameters.
		m_xidx(x_idx), m_yidx(y_idx), m_isVisited(isVisited){}

    //Standard setter and getter functions:

	std::shared_ptr<Node> Node::getNorthptr() {
		return m_north_ptr;
	}
	std::shared_ptr<Node> Node::getSouthptr() {
		return m_south_ptr;
	}
	std::shared_ptr<Node> Node::getEastptr() {
		return m_east_ptr;
	}
	std::shared_ptr<Node> Node::getWestptr() {
		return m_west_ptr;
	}
	void Node::setNorthptr(std::shared_ptr<Node> n) {
		m_north_ptr = n;
	}
	void Node::setSouthptr(std::shared_ptr<Node> n) {
		m_south_ptr = n;
	}
	void Node::setEastptr(std::shared_ptr<Node> n) {
		m_east_ptr = n;
	}
	void Node::setWestptr(std::shared_ptr<Node> n) {
		m_west_ptr = n;
	}
	bool Node::getIsVisited() {
		return m_isVisited;
	}
	double Node::getXidx() {
		return m_xidx;
	}
	double Node::getYidx() {
		return m_yidx;
	}

    void Node::setIsVisited(bool a){
        m_isVisited = a;
  }