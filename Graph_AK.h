#include <vector>
#include <string>

using namespace std;

class Graph_AK {
private:

    vector< pair <int, int> > x_y_tab;
    vector< int > demands_tab;
    vector< vector<float> > distance_mat;
    int n;
    int capacity;                                  //vérifier avec const !!! 16/12
    int id_depot;
    vector <float> metaheuristic_evaluation_tab;
    vector <int> metaheuristic_position_tab;
    vector < vector<int> > metaheuristic_routes_tab;

public:

    Graph_AK(string vrp_filename);
    float cost_TSP(vector<int> route);
    float two_opt(vector<int> & route);
    float euclidean_distance(int i,int j);
    void initialize_distance_matrix();
	void print_distance_matrix();
	bool is_realizable(vector<int> route);
	void initialize_metaheuristic_tabs();
	float run_metaheuristic();
	void print_solution();
	void write_dot_G(string InstanceName,vector<vector<int> > routes);

	int get_n(){return n;};
	int get_capacity(){return capacity;};
	int get_depot(){ return id_depot;};
	int get_demand(int i){ return demands_tab[i];};

	float get_distance(int i, int j){ return distance_mat[i][j];};
};
