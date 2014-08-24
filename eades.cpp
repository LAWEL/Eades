#include <iostream>
#include <algorithm>
#include <vector>
#include <random>
#include <complex>
#include <fstream>
#include <memory>
#include <set>

void log(const char *level, const char *file, const int line, const char *format, ...)
{
  va_list argp;
  fprintf(stderr, "%s%s(%d): ", level, file, line);
  va_start(argp, format);
  vfprintf(stderr, format, argp);
  va_end(argp);
  fprintf(stderr, "\n");
}

#define debug(...) log("[DEBUG]", __FILE__, __LINE__, __VA_ARGS__);
#define info(...) log("[INFO]", __FILE__, __LINE__, __VA_ARGS__);
#define warning(...) log("[WARNING]", __FILE__, __LINE__, __VA_ARGS__);
#define error(...) log("[ERROR]", __FILE__, __LINE__, __VA_ARGS__);
#define critical(...) log("[CRITICAL]", __FILE__, __LINE__, __VA_ARGS__);

namespace eades {

  const double MAX_COORDINATE = 1e3;
  const double MAX_NUM_NODE = 1e3;
  const double COULOMB = 5.0 * 1e2;
  const double SPRING = 1e-1;
  const double NATURAL_LENGTH = 1.0 * 1e1;
  const double EPS = 1e-4;
  const double CODF = 3.0 * 1e-1;
  const double THRESHOLD = 1e-1;

  typedef std::complex<double> Velocity;
  typedef Velocity Force;

  class Node {
  public:
    const unsigned int id_;
    std::string label;
    double x;
    double y;
    Velocity v;

    Node(const unsigned int id) : id_(id) {}

    std::string toString() const {
      return label + "(" + std::to_string(id_) + "): (" + std::to_string(x) + ", " + std::to_string(y) + ")";
    }

    bool operator < (const Node& node) const {
      return id_ < node.id_;
    }

    bool operator == (const Node& node) const {
      return id_ == node.id_;
    }

  };

  class Edge {
  public:
    unsigned int from_;
    unsigned int to_;

    Edge(const unsigned int from, const unsigned int to) : from_(from), to_(to) {}

    std::string toString() const {
      return std::to_string(from_) + " -> " + std::to_string(to_);
    }
  };

  class Graph{
  public:
    static const int EXPECTED_NUM_NODE = 1000;
    std::set<Node> nodes;
    std::vector<std::vector<Edge> > adjacency_list;

    Graph() {
      adjacency_list.reserve(EXPECTED_NUM_NODE);
    }

    bool addNode(const Node& node){
      if(nodes.find(node) != nodes.end()) {
        return false;
      }
      nodes.insert(node);
      if(node.id_ >= adjacency_list.size()) {
        adjacency_list.resize(node.id_ + 1);
      }
      return true;
    }

    bool addEdge(const Edge& edge) {
      if(adjacency_list.size() <= edge.from_) {
        return false;
      }
      adjacency_list[edge.from_].push_back(edge);
      return true;
    }

    void init(const unsigned int num_node) {
      std::random_device seed_gen;
      std::mt19937 engine(seed_gen());
      std::uniform_real_distribution<double> pos(-MAX_COORDINATE, MAX_COORDINATE);
      for(unsigned int i = 0; i < num_node; ++i) {
        Node node(i);
        node.x = pos(engine);
        node.y = pos(engine);
        node.label = "p" + std::to_string(i);
        addNode(node);
      }
    }

    void write(const std::string& file_name){
      std::unique_ptr<std::ofstream, decltype(&Graph::myclose)> ofs(new std::ofstream(file_name), Graph::myclose);
      *ofs <<
        R"(digraph G {
  graph[bb="-2000,-2000,2000,2000",size="5,5"];
  node [width="1", height="1",fixedsize="true", fontsize="50"];)" << std::endl;
      for(auto node : nodes) {
        *ofs << "  " << node.label << " [pos=\"" << node.x << "," << node.y << "\"];" << std::endl;
      }
      for(auto node1 : nodes) {
        for(Edge edge : adjacency_list[node1.id_]) {
          auto node2 = nodes.find(Node(edge.to_));
          *ofs << "  " << node1.label << " -> " << node2->label << "[arrowsize = " << 2 << "];" << std::endl;
        }
      }
      *ofs << "}" << std::endl;
    }

  private:
    static void myclose(std::ofstream* ofs) {
      ofs->close();
    }

  };


void eades(Graph& graph) {
  double sum_kinetic_energy;
  do {
    sum_kinetic_energy = 0.0;
    for(auto node1 : graph.nodes) {
      Force force;
      for(Edge edge : graph.adjacency_list[node1.id_]) {
        auto node2 = graph.nodes.find(Node(edge.to_));
/*
  力 := 力 + 定数 / 距離（ノード1, ノード2) ^ 2  // クーロン力
  force += getCoulombsForce(node1, node2);
*/

        force += COULOMB / pow(abs(node1.v - node2->v), 2.0);
/*
  力 := 力 + バネ定数 * (距離 (ノード1, ノード2) - バネの自然長)  // フックの法則による力
  force += getHookeForce(node1, node2);
*/
        force += SPRING * (abs(node1.v - node2->v) - NATURAL_LENGTH);
      }
/*
  ノード１の速度 := (ノード1の速度 + 微小時間 * 力 / ノード1の質量) * 減衰定数
*/
      node1.v = (node1.v + EPS * abs(force) / 1.0) * CODF;

/*
  ノード１の位置 := ノード1の位置 + 微小時間 * ノード1の速度
*/
      node1.x = node1.x + EPS * node1.v.real();
      node1.y = node1.y + EPS * node1.v.imag();

/*
  運動エネルギーの合計 := 運動エネルギーの合計 + ノード1の質量 * ノード1の速度 ^ 2
*/
      sum_kinetic_energy += pow(abs(node1.v), 2.0);
    }
  } while(sum_kinetic_energy < THRESHOLD);
}


}

int main() {
  const unsigned int graph_size = 10;
  eades::Graph graph;
  graph.init(graph_size);
  for(unsigned int i = 0; i < graph_size; ++i) {
    for(unsigned int j = i + 1; j < graph_size; ++j) {
      graph.addEdge(eades::Edge(i, j));
    }
  }
  eades::eades(graph);
  graph.write("test.dot");
  return 0;
}
