#include "dataParser.h"
#include <string>
#include <iostream>
#include <stdexcept>
#include <unordered_set>
#include <unordered_map>
#include <vector>

using namespace std;

struct EdgeData {
    int org;
    int dest;
    double distance;
};

/**
 * @brief Parses a toy graph from a CSV file and updates the provided Graph object.
 *
 * This function reads a CSV file containing edges of a toy graph and populates the graph with vertices and edges.
 * The CSV file is expected to be located in the ../data/Toy-Graphs/ directory.
 *
 * @param filename The name of the CSV file to parse.
 * @param graph The graph object to be updated with parsed data.
 */
void ToyGraphParser(const std::string& filename, Graph<int>& graph) {
    ifstream csv("../data/Toy-Graphs/" + filename);
    if (!csv.is_open()) {
        cout << "Error opening file" << endl;
        return;
    }

    unordered_set<int> verticesSeen;
    vector<EdgeData> edges;
    string line;

    // Ignore the first line
    getline(csv, line);

    while (getline(csv, line)) {
        istringstream s(line);
        string org_str, dest_str, distance_str;

        getline(s, org_str, ',');
        getline(s, dest_str, ',');
        getline(s, distance_str, ',');

        int org = stoi(org_str);
        int dest = stoi(dest_str);
        double distance = stod(distance_str);

        if (verticesSeen.insert(org).second) {
            graph.addVertex(org);
        }
        if (verticesSeen.insert(dest).second) {
            graph.addVertex(dest);
        }

        edges.push_back({org, dest, distance});
    }

    for (const auto& edge : edges) {
        graph.addEdge(edge.org, edge.dest, edge.distance);
        graph.addEdge(edge.dest, edge.org, edge.distance);
    }

    csv.close();
}

/**
 * @brief Parses additional toy graph data from a separate set of CSV files.
 *
 * This function reads two CSV files containing vertex and edge data for an additional fully connected toy graph.
 * It updates the provided graph object with the parsed data.
 *
 * @param filename The name of the CSV file containing edge data.
 * @param graph The graph object to be updated with parsed data.
 * @param n The number of vertices to parse from the nodes CSV file.
 */
void ExtraFullyConnectedGraphsParser(const std::string& filename, Graph<int>& graph, int n) {
    string line;

    // Read vertices and their geographical coordinates
    ifstream extra_vertices("../data/Extra_Fully_Connected_Graphs/nodes.csv");
    if (!extra_vertices.is_open()) {
        cout << "Error opening file" << endl;
        return;
    }

    unordered_map<int, Vertex<int>*> vertexMap;
    getline(extra_vertices, line);
    int count = 0;
    while (count < n && getline(extra_vertices, line)) {
        istringstream s(line);
        string iD_str, long_str, lat_str;

        getline(s, iD_str, ',');
        getline(s, long_str, ',');
        getline(s, lat_str, ',');

        int iD = stoi(iD_str);
        double lon = stod(long_str);
        double lat = stod(lat_str);

        vertexMap[iD] = new Vertex<int>(iD);
        vertexMap[iD]->setLong(lon);
        vertexMap[iD]->setLat(lat);

        count++;
    }
    extra_vertices.close();

    // Read edges
    ifstream extra_edges("../data/Extra_Fully_Connected_Graphs/" + filename);
    if (!extra_edges.is_open()) {
        cout << "Error opening file" << endl;
        return;
    }

    vector<EdgeData> edges;
    while (getline(extra_edges, line)) {
        istringstream s(line);
        string org_str, dest_str, distance_str;

        getline(s, org_str, ',');
        getline(s, dest_str, ',');
        getline(s, distance_str, ',');

        int org = stoi(org_str);
        int dest = stoi(dest_str);
        double distance = stod(distance_str);

        edges.push_back({org, dest, distance});
    }
    extra_edges.close();

    // Add vertices and edges to the graph
    for (const auto& vertex : vertexMap) {
        graph.addVertex(vertex.first);
        (*(graph.getVertexSet().end() - 1))->setLong(vertex.second->getLong());
        (*(graph.getVertexSet().end() - 1))->setLat(vertex.second->getLat());
    }
    for (const auto& edge : edges) {
        if (vertexMap.find(edge.org) != vertexMap.end() && vertexMap.find(edge.dest) != vertexMap.end()) {
            graph.addEdge(edge.org, edge.dest, edge.distance);
            graph.addEdge(edge.dest, edge.org, edge.distance);
        }
    }
}

/**
 * @brief Parses real-world graph data including nodes and edges from CSV files.
 *
 * This function reads two CSV files containing node and edge data for a real-world graph. It updates the provided
 * graph object with the parsed data.
 *
 * @param filename The name of the directory containing the nodes and edges CSV files.
 * @param graph The graph object to be updated with parsed data.
 */
void RealWorldGraphParser(const std::string& filename, Graph<int>& graph) {
    ifstream real_nodes("../data/Real_World_Graphs/" + filename + "/nodes.csv");
    if (!real_nodes.is_open()) {
        cout << "Error opening file" << endl;
        return;
    }

    unordered_map<int, Vertex<int>*> vertexMap;
    string line;

    // Parse nodes
    getline(real_nodes, line);
    while (getline(real_nodes, line)) {
        istringstream s(line);
        string iD_str, long_str, lat_str;

        getline(s, iD_str, ',');
        getline(s, long_str, ',');
        getline(s, lat_str, ',');

        int iD = stoi(iD_str);
        double lon = stod(long_str);
        double lat = stod(lat_str);

        vertexMap[iD] = new Vertex<int>(iD);
        vertexMap[iD]->setLong(lon);
        vertexMap[iD]->setLat(lat);
    }
    real_nodes.close();

    ifstream real_edges("../data/Real_World_Graphs/" + filename + "/edges.csv");
    if (!real_edges.is_open()) {
        cout << "Error opening file" << endl;
        return;
    }

    vector<EdgeData> edges;
    getline(real_edges, line);

    // Parse edges
    while (getline(real_edges, line)) {
        istringstream s(line);
        string org_str, dest_str, distance_str;

        getline(s, org_str, ',');
        getline(s, dest_str, ',');
        getline(s, distance_str, ',');

        int org = stoi(org_str);
        int dest = stoi(dest_str);
        double distance = stod(distance_str);

        edges.push_back({org, dest, distance});
    }
    real_edges.close();

    // Add vertices and edges to the graph
    for (const auto& vertex : vertexMap) {
        graph.addVertex(vertex.first);
        (*(graph.getVertexSet().end() - 1))->setLong(vertex.second->getLong());
        (*(graph.getVertexSet().end() - 1))->setLat(vertex.second->getLat());
    }
    for (const auto& edge : edges) {
        if (vertexMap.find(edge.org) != vertexMap.end() && vertexMap.find(edge.dest) != vertexMap.end()) {
            graph.addEdge(edge.org, edge.dest, edge.distance);
            graph.addEdge(edge.dest, edge.org, edge.distance);
        }
    }
}
