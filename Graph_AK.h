#ifndef __GRAPH_AK__
#define __GRAPH_AK__

#include <vector>
#include <string>
#include <set>

#include <lemon/list_graph.h>
#include <lemon/concepts/maps.h>
#include <lemon/nagamochi_ibaraki.h>

using namespace std;

#define FLT_MAX std::numeric_limits<float>::max()


class Graph_AK {
private:

    vector< pair <int, int> > x_y_tab;
    vector< int > demands_tab;
    vector< vector<float> > distance_mat;
    int n;
    int m;
    int capacity;                                  //vérifier avec const !!! 16/12
    int id_depot;
    vector <float> metaheuristic_evaluation_tab;
    vector <int> metaheuristic_position_tab;
    vector < vector<int> > metaheuristic_routes_tab;
    vector < vector<int> > routes_cplex;

    vector <vector <float> > x_value;
    vector< vector<int> > cplex_routes_tab;


    lemon::ListGraph L_GU;
    vector<lemon::ListGraph::Node> LGU_name_node;
    vector<vector<lemon::ListGraph::Edge> > LGU_name_link;

    //UNUSED
//    map<int, int> L_rtnmap;

public:

    void construct_Undirected_Lemon_Graph();
	double undirected_MinimumCut(vector< int >& W);
	void set_x_value(vector< vector<float> > cost_x);
	void set_routes_cplex(vector<vector<int> > route){ routes_cplex = route;}
	float minDistance(float dist[], bool sptSet[]);

    Graph_AK(string vrp_filename, int upbound);
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
	void write_routes(string InstanceName, vector<vector<int> > routes, float sol);

	bool has_sub_tour(vector<vector<int> > & W);
	void Dijsktra(vector<int> & L, int src, bool atteignable);
	bool is_feasible_tour(vector<vector<int> > & V);



	bool tabu_search(vector<vector<int> > & W);
	int maximum_reached_add_remove(set<int> C_add, set<int> C_remove, set<int> S);
	float sum_of_demands(set<int> S);
	int random_selection_M(set<int> C_add, vector<int> Tabu_list, set<int> S, float M, int ntime);
	float get_max_value_can_get(set<int> C_add, vector<int> Tabu_list, set<int> S);
	float compute_xS(set<int> S, int elem);
	set<int> compute_C_addable(set<int> out_S, float smax, vector<int> Tabu_list);
	set<int> compute_C_removable(set<int> in_S, float smin, vector<int> Tabu_list);
	int select_random_first_node();




	int get_n(){return n;};
	int get_capacity(){return capacity;};
	int get_depot(){ return id_depot;};
	int get_demand(int i){ return demands_tab[i];};
	int get_m(){ return m;};
	float get_distance(int i, int j){ return distance_mat[i][j];};
	float get_x_value(int i,int j){ return x_value[i][j]; };
	vector<vector<int> > get_meta_solution();
	vector<vector<int> > get_routes_cplex(){ return routes_cplex; };

};

#endif
