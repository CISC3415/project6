#ifndef CUSTGRAPH_H
#define CUSTGRAPH_H
#include <iostream>

struct Node {
  int x, y;
  double cost;
  struct Node *prev;
};

class Graph {
public:
Graph() : nodes(new Node*[512]), capacity(512), length(0) {}
void sort() {quicksort(0, length-1);}

void push(int x, int y, double cost, struct Node *n) {
  struct Node *newnode = (struct Node*) malloc(sizeof(struct Node));
  newnode->x = x;
  newnode->y = y;
  newnode->cost = cost;
  newnode->prev = n;
  nodes[length] = newnode;
  length += 1;
}

void push(struct Node *n) {
  if (length >= 512) {
    capacity *= 2;
    Node **tmp = new Node*[capacity];
    for (int i = 0; i < 512; i++) {
      tmp[i] = nodes[i];
    }
    nodes = tmp;
    delete [] tmp;
  }
  nodes[length] = n;
  length += 1;
}

Node * pop() {
  if (length == 0) throw ("Popped from empty..");
  length -= 1;
  return nodes[length];
}

bool contains(struct Node n) {
  for (int i = 0; i < length; i++) {
    if (nodes[i]->x == n.x && nodes[i]->y == n.y) {
      return true;
    }
  }
  return false;
}

bool isEmpty() {
  return (length == 0);
}

private:
double *costs;
Node **nodes;
int capacity;
int length;

void swap(int i, int j) {
  Node *tmp = nodes[i];
  nodes[i] = nodes[j];
  nodes[j] = tmp;
}

int partition(int lo, int hi) {
  double v = nodes[hi]->cost;
  int i = lo - 1;
  for (int j = lo; j <= hi-1; j++) {
    if (nodes[j]->cost >= v) {
      i++;
      swap(i, j);
    }
  }
  swap(i+1, hi);
  return i+1;
}

void quicksort(int lo, int hi) {
  if (lo < hi) {
    int par = partition(lo, hi);
    quicksort(lo, par-1);
    quicksort(par+1, hi);
  }
}
};


#endif