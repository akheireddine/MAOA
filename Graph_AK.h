#include <vector>
#include <string>

using namespace std;

class Graph_AK {
private:

    vector< pair <int, int> > x_y_tab;
    vector< int > demands_tab;
    vector< vector<float> > distance_mat;
    int n;
    int capacity;  //vérifier avec const
    int id_depot;

public:
    Graph_AK(string vrp_filename);
    float cost_TSP(vector<int> route);
    float two_opt(vector<int> & route);
    float euclidean_distance(int i,int j);
    void initialize_distance_matrix();
	void print_distance_matrix();
};
