#include "Graph_AK.h"
#include <string>
#include <stdio.h>
#include <iostream>
#include <fstream>
#include <math.h>

void Graph_AK::print_distance_matrix() {
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			printf("%.1f ", distance_mat[i][j]);
		}
		printf("\n");
	}
	printf("\n");
}

Graph_AK::Graph_AK (string vrp_filename) {

  ifstream fic(vrp_filename);

  if (!(fic)){
    cerr<<"Error occurred"<<endl;
  }

  string line;

  char cstr[100];
  int demand;

  getline(fic, line);
  getline(fic, line);
  getline(fic, line);

  getline(fic, line); //line DIMENSION
  sscanf(line.c_str(), "%s : %d", cstr, &n);

  getline(fic, line);

  getline(fic, line);  //line CAPACITY
  sscanf(line.c_str(), "%s : %d", cstr, &capacity);

  getline(fic, line);   //NODE_COORD_SECTION

  for(int i = 0; i < n ; i++){
	  getline(fic, line);
	  int x,y;
	  sscanf(line.c_str(), "%s %d %d", cstr, &x,&y);
	  x_y_tab.push_back(make_pair(x, y));
  }

  getline(fic, line);    //demand section


  for(int i = 0; i < n ; i++){
	  getline(fic, line);
	  demand = 0;
	  sscanf(line.c_str(), "%s %d", cstr,&demand);
	  demands_tab.push_back(demand);
  }

  getline(fic, line);    //DEPOT SECTION
  getline(fic, line);

  sscanf(line.c_str(), "%d", &id_depot);
  id_depot -= 1;                        //convention depot id egal à 0
  fic.close();

  initialize_distance_matrix();


  print_distance_matrix();
}

void Graph_AK::initialize_distance_matrix(){
	distance_mat.resize(n, vector<float>(n, 0));
    for(int i = 0; i < n; i++){
	  for(int j = i + 1; j < n; j++){
		  float distance_i_j = euclidean_distance(i,j);
		  distance_mat[i][j] = distance_i_j;
		  distance_mat[j][i] = distance_i_j;
	  }
  }
}

float Graph_AK::cost_TSP(vector<int> route){
	int node_inf = id_depot;
	int node_sup;
	float cost = 0.;
	for (unsigned int i = 0; i < route.size(); i++){
		node_sup = route[i];
		cost += distance_mat[node_inf][node_sup];
		node_inf = node_sup;
	}
	cost += distance_mat[node_inf][id_depot];
	return cost;
}

void swap(vector<int> & route, int i, int j){
	int tmp = route[i];
	route[i] = route[j];
	route[j] = tmp;
}

void copy(vector<int> source, vector<int> & destination){
	for (unsigned int i = 0; i < source.size(); i++)
		destination[i] = source[i];
}


float Graph_AK::two_opt(vector<int> & route){

	vector<int> best_route(route.size());
	copy(route, best_route);



	float best_cost = cost_TSP(best_route);
	bool improved = true;
	int size = best_route.size();

	while (improved){
		improved = false;
		for (int i = 0; i < size - 2; i++) {
			for (int j = i + 1; j < size; j++){
				//copie
				vector<int> tmp_route(size);
				copy(route, tmp_route);
				// fin copie
				swap(tmp_route, i, j);
				int k = 1;
				while (i + k < j - k){
					swap(tmp_route, i + k, j - k);
					k++;
				}
				float current_cost = cost_TSP(tmp_route);
				if (best_cost > current_cost){
					best_cost = current_cost;
					copy(tmp_route, best_route);
					improved = true;
				}
			}
		}
		copy(best_route, route);
	}

	return best_cost;
}



float Graph_AK::euclidean_distance(int i, int j){
	pair<int,int> xi = x_y_tab[i];
	pair<int,int> xj = x_y_tab[j];
	return sqrt(pow(xi.first - xj.first,2) + pow(xi.second - xj.second,2) );
}


