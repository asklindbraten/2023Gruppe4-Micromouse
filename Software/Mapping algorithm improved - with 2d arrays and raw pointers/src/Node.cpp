#include "Node.h"

	Node::Node(int x_idx, int y_idx, bool isVisited){//Constructor with parameters.
		m_xidx = x_idx;
		m_yidx = y_idx; 
		m_isVisited = isVisited;
	}
	
	Node::~Node(){
		delete m_north_ptr;
		delete m_east_ptr;
		delete m_west_ptr;
		delete m_south_ptr;
		m_north_ptr = nullptr;
		m_east_ptr = nullptr;
		m_south_ptr = nullptr;
		m_west_ptr = nullptr;
	}

    //Standard setter and getter functions:

	Node* Node::getNorthptr() {
		return m_north_ptr;
	}
	Node* Node::getSouthptr() {
		return m_south_ptr;
	}
	Node* Node::getEastptr() {
		return m_east_ptr;
	}
	Node* Node::getWestptr() {
		return m_west_ptr;
	}
	void Node::setNorthptr(Node* n) {
		m_north_ptr = n;
	}
	void Node::setSouthptr(Node* n) {
		m_south_ptr = n;
	}
	void Node::setEastptr(Node* n) {
		m_east_ptr = n;
	}
	void Node::setWestptr(Node* n) {
		m_west_ptr = n;
	}
	bool Node::getIsVisited() {
		return m_isVisited;
	}
	int Node::getXidx() {
		return m_xidx;
	}
	int Node::getYidx() {
		return m_yidx;
	}

    void Node::setIsVisited(bool a){
        m_isVisited = a;
  }