#ifndef UNTITLED_FUNCTIONS_H
#define UNTITLED_FUNCTIONS_H

#include "dataStructures/Graph.h"
#include "haversine.h"
#include <vector>
#include <set>
#include <iostream>
#include <unordered_set>
using namespace std;

/**
 * @brief Normalizes the given graph by ensuring connectivity between every pair of vertices.
 *
 * This function modifies the graph by adding edges to connect vertices that are not already
 * connected. It ensures that every pair of vertices is reachable within the graph.
 *
 * @param graph Pointer to the graph to be normalized.
 */
void normalizeGraph(Graph<int>* graph);

/**
 * @brief Implements Prim's algorithm to find the Minimum Spanning Tree (MST) of a graph.
 *
 * @param g Pointer to the graph.
 * @param currentNode The starting node for MST construction.
 * @return Vector of pointers to vertices representing the MST.
 */
std::vector<Vertex<int>*> prim(Graph<int>* g, int currentNode);

/**
 * @brief Backtracking algorithm to solve the Traveling Salesman Problem (TSP).
 *
 * This function recursively explores all possible paths in the graph, starting from the given
 * currentNode. It maintains the visited vertices, the current path being explored, the optimal
 * path found so far, and the minimum cost. At the end of the exploration, the optimal path
 * and its cost are updated if a better solution is found.
 *
 * @param graph Reference to the graph representing cities and distances.
 * @param currentNode The current node being visited in the TSP exploration.
 * @param visited Vector indicating whether each node has been visited.
 * @param path Vector representing the current path being explored.
 * @param optimalPath Vector representing the optimal TSP path found so far.
 * @param minCost Reference to the minimum cost found so far.
 * @param currentCost Reference to the current cost of the path being explored.
 * @param count Reference to the count of explored paths (for performance analysis).
 */
void TSPBackTracking(Graph<int>& graph, int currentNode, std::vector<bool>& visited,
                     std::vector<int>& path, std::vector<int>& optimalPath,
                     double& minCost, double& currentCost, int& count);

/**
 * @brief Solves the Traveling Salesman Problem (TSP) using backtracking.
 *
 * This function initiates the backtracking algorithm to find the optimal TSP path starting
 * from the specified startNode. It returns a vector of pointers to vertices representing
 * the optimal TSP path.
 *
 * @param graph Reference to the graph representing cities and distances.
 * @param startNode The starting node for the TSP.
 * @return Vector of pointers to vertices representing the optimal TSP path.
 */
std::vector<Vertex<int>*> solveTSPBacktracking(Graph<int>& graph, int startNode);

/**
 * @brief Solve the Traveling Salesman Problem (TSP) using the Triangular Inequality heuristic.
 *
 * @param graph The graph representing the cities and their distances.
 * @param currentNode The starting node for the TSP.
 * @return A vector of pointers to vertices representing the optimal TSP path.
 */
vector<Vertex<int>*> solveTSPTriangularInequality(Graph<int>& graph, int currentNode);

/**
 * @brief Form a Minimum Spanning Tree (MST) from the given graph starting from a specific node.
 *
 * @param g The graph from which the MST is to be formed.
 * @param currentNode The starting node for forming the MST.
 * @return The Minimum Spanning Tree graph.
 */
Graph<int> formMST(Graph<int> g, int currentNode);

/**
 * @brief Find the vertices with odd degree in the given graph.
 *
 * @param graph The graph to search for odd degree vertices.
 * @return A vector containing the IDs of vertices with odd degree.
 */
vector<int> findOddDegreeVertices(const Graph<int>& graph);

/**
 * @brief Check if a vertex has an odd degree.
 *
 * @param v The vertex to be checked.
 * @return True if the vertex has an odd degree, otherwise false.
 */
bool isOdd(const Vertex<int>& v);

/**
 * @brief Get the weight of the edge between two vertices in the graph.
 *
 * @param graph The graph containing the edge.
 * @param u The ID of the first vertex.
 * @param v The ID of the second vertex.
 * @return The weight of the edge between the two vertices.
 */
double getEdgeWeight(const Graph<int>& graph, int u, int v);

/**
 * @brief Perform a minimum weight perfect matching on the given graph using the specified odd degree vertices.
 *
 * @param graph The graph on which matching is to be performed.
 * @param odders The vector of IDs of vertices with odd degree.
 * @return A graph representing the minimum weight perfect matching.
 */
Graph<int> MinWeightPerfMatch(Graph<int>& graph, const std::vector<int>& odders);

/**
 * @brief Find an Eulerian path in the given graph.
 *
 * @param graph The graph in which to find the Eulerian path.
 * @return A vector of pointers to vertices representing the Eulerian path.
 */
vector<Vertex<int>*> findEulerianPath(Graph<int>& graph);

/**
 * @brief Convert an Eulerian path into a Hamiltonian cycle.
 *
 * @param eulerianPath The Eulerian path to be converted.
 * @return A vector of pointers to vertices representing the Hamiltonian cycle.
 */
vector<Vertex<int>*> makeHamiltonianCycle(const vector<Vertex<int>*> eulerianPath);

/**
 * @brief Combine a Minimum Spanning Tree (MST) and a matching graph into a single graph.
 *
 * @param mst The Minimum Spanning Tree graph.
 * @param matching The matching graph.
 * @return The combined graph.
 */
Graph<int> combineGraphs(const Graph<int>& mst, const Graph<int>& matching);

/**
 * @brief Solve the Traveling Salesman Problem (TSP) using the Christofides algorithm.
 *
 * @param graph The graph representing the cities and their distances.
 * @return A vector of pointers to vertices representing the optimal TSP path.
 */
vector<Vertex<int>*> christofidesTsp(Graph<int>& graph);



/* 4.4 */
/**
 * @brief Checks if the graph is connected starting from a given node.
 *
 * @param graph Reference to the graph to check.
 * @param startNode The node from which to start the connectivity check.
 * @param reachableNodes Output parameter storing the set of reachable nodes.
 * @return True if the graph is connected, false otherwise.
 */
bool isConnected(Graph<int>& graph, int startNode, std::unordered_set<int>& reachableNodes);

/**
 * @brief Calculates the shortest paths between all pairs of nodes in the graph.
 *
 * @param graph Reference to the graph.
 * @param nodes Set of nodes for which shortest paths are calculated.
 * @return Vector of vectors representing the shortest paths between all pairs of nodes.
 */
std::vector<std::vector<double>> calculateShortestPaths(Graph<int>& graph, const std::unordered_set<int>& nodes);

/**
 * @brief Solves the Traveling Salesman Problem (TSP) using the Nearest Neighbor heuristic.
 *
 * @param graph Reference to the graph representing cities and distances.
 * @param startNode The starting node for the TSP.
 * @param nodes Set of nodes to visit in the TSP.
 * @param totalCost Output parameter storing the total cost of the TSP solution.
 * @return Vector representing the optimal TSP path using the Nearest Neighbor heuristic.
 */
std::vector<int> tspNearestNeighbor(Graph<int>& graph, int startNode, const std::unordered_set<int>& nodes, double& totalCost);

/**
 * @brief Solves the Traveling Salesman Problem (TSP) for real-world scenarios.
 *
 * This function first checks the connectivity of the graph, then uses appropriate heuristics
 * to find an optimal TSP solution. If the graph is disconnected, it calculates the shortest paths
 * between disconnected components and connects them using minimum spanning trees before finding
 * the TSP solution.
 *
 * @param graph Reference to the graph representing cities and distances.
 * @param startNode The starting node for the TSP.
 * @return Vector of pointers to vertices representing the optimal TSP path.
 */
std::vector<Vertex<int>*> solveRealWorldTSP(Graph<int>& graph, int startNode);


#endif //UNTITLED_FUNCTIONS_H

