#include "Graph_AK.h"
#include <string>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <set>
#include <fstream>
#include <math.h>
#include <limits.h>
#include <lemon/lgf_writer.h>
#include <algorithm>
#include <random>
#include <chrono>
#include <experimental/random>

#include <ctime>

#define GRAPHVIZ "$PATHTUTOMIP/graphviz-2.40.1/bin/"

void Graph_AK::print_distance_matrix() {
	for (int i = 0; i < n; i++) {
		for (int j = 0; j < n; j++) {
			printf("%.1f ", distance_mat[i][j]);
		}
		printf("\n");
	}
	printf("\n");
}

Graph_AK::Graph_AK (string vrp_filename, int upbound) {

  ifstream fic(vrp_filename.c_str());

  if (!(fic)){
    cerr<<"Error occurred"<<endl;
  }

  m = upbound;

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

void copy(vector<int> source, vector<int> & destination){        //suppose que l'allocation de destination est faite
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

bool Graph_AK::is_realizable(vector<int> route){
	float sum_of_demands = 0.;
	for (unsigned int i = 0; i < route.size(); i++)
		sum_of_demands += demands_tab[route[i]];
	return sum_of_demands <= capacity;
}












/////////////////////////////////////////////////////////////////////////////////


int * Graph_AK::trier_clients(){
	int *tab = (int*) malloc(sizeof(int)*(n-1));
	for (int i = 0; i < n-1; i++)
		tab[i] = i+1;

	int i, j;
	int permute = true;
	for (i = (n-1); (i > 1) && permute; i--){
		permute = false;
		for (j = 0; j < i - 1; j++)
		if (get_demand(tab[j + 1]) > get_demand(tab[j])){
			int aux = tab[j+1];
			tab[j + 1] = tab[j];
			tab[j] = aux;
			permute = true;
		}
	}
}

int Graph_AK::update_metaheuristic_weight(int node_id, int tournee_id){
	int global_value = 0;
	for (int t_id = 0; t_id < metaheuristic_routes_tab.size(); t_id++){
		int local_value = 0;
		if (t_id == tournee_id)
			local_value += get_demand(node_id);
		for (int k = 0; k < metaheuristic_routes_tab[t_id].size(); k++){
			if (metaheuristic_routes_tab[t_id][k] == node_id)
				local_value -= get_demand(node_id);
			else
				local_value += get_demand(metaheuristic_routes_tab[t_id][k]);
		}
		if (local_value > get_capacity())
			local_value = local_value - get_capacity();
		else
			local_value = 0;
		global_value += local_value;
	}

	return global_value;
}

void Graph_AK::print_solution() {
	for (int r = 0; r < metaheuristic_routes_tab.size(); r++) {
		printf("\nTournée %d : ", r);
		int weight = 0;
		for (unsigned int c = 0; c < metaheuristic_routes_tab[r].size(); c++){
			printf("%d ", metaheuristic_routes_tab[r][c]);
			weight += get_demand(metaheuristic_routes_tab[r][c]);
		}
		printf("costTSP : (%.1f) W : (%d)\n", cost_TSP(metaheuristic_routes_tab[r]), weight);
	}
}

float Graph_AK::run_metaheuristic(){

	if (!metaheuristic_clustering(m)){
		printf("Clustering failed !!!!\n");
//		exit(1);
		return -1;
	}
	// srand(time(NULL));
	initialize_metaheuristic_tabs();
	/* CALCUL DE LA BEST SOLUTION INITIALE (1 CLIENT = 1 TOURNÉE)*/
	float best_solution_value = 0.;
	for (int i = 0; i < metaheuristic_evaluation_tab.size(); i++)
		best_solution_value += metaheuristic_evaluation_tab[i];
//	printf("\nSolution initiale : %1.f ", best_solution_value);
	/*FIN CALCUL BEST SOLUTION INITIALE */
	bool globally_improved = true;              // après toutes les tentatives de changement de tournées à tous les clients
	// pour l'ordre dans lequel on va tenter les changements de tournées pour les clients, nous retenons l'ordre croissant 1 à n-1 : HYP : 0 est le dépôt.
	vector<int> changing_route_client_order;
	for (int i = 0; i < n; i++)
		if (i != id_depot)
			changing_route_client_order.push_back(i);
	//fin ordre des clients à considérer
 	while (globally_improved){
 		globally_improved = false;
 		for (unsigned int i = 0; i < changing_route_client_order.size(); i++){
			// int client_id = changing_route_client_order[rand()%changing_route_client_order.size()];
 			int client_id = changing_route_client_order[i];
 			int client_position = metaheuristic_position_tab[client_id];
 			for (int r = 0; r < metaheuristic_routes_tab.size(); r++){
 				if (r != client_position /*&& metaheuristic_routes_tab[r].size() != 0*/){           //si la tournée r n'est pas vide et n'est pas la tournée du client
 					/* DÉBUT DE LA SIMULATION*/
 					vector <int> leaving_route(0);
 					vector <int> coming_route(metaheuristic_routes_tab[r].size());
 					/*INSTANCIATION DE LEAVING ROUTE*/
 					for (unsigned int l = 0; l < metaheuristic_routes_tab[client_position].size(); l++)
 						if (metaheuristic_routes_tab[client_position][l] != client_id)
 							leaving_route.push_back(metaheuristic_routes_tab[client_position][l]);
 					/*FIN INSTANCIATION DE LEAVING ROUTE*/
 					/*INSTANCIATION DE COMING ROUTE*/
 					copy(metaheuristic_routes_tab[r], coming_route);					///AUTANT FAIRE LE TEST DE REALISABILITE DE LA TOURNEE AVANT DE COPIER
 					coming_route.push_back(client_id);
 					/*FIN INSTANCIATION DE COMING ROUTE*/
 					if (!is_realizable(coming_route))
 						continue;
 					float leaving_route_2opt_cost = two_opt(leaving_route);
 					float coming_route_2opt_cost = two_opt(coming_route);
 					float new_global_cost = leaving_route_2opt_cost + coming_route_2opt_cost;
 					for (int ev = 0; ev < metaheuristic_evaluation_tab.size(); ev++){
 						if (ev != client_position && ev != r)
 							new_global_cost += metaheuristic_evaluation_tab[ev];               // ON A BESOIN QUE DES 2 NVELLES TOURNEES a comparé avec l'ancienne tournee
 					}
 					if (new_global_cost < best_solution_value){
 						metaheuristic_routes_tab[client_position] = leaving_route;
 						metaheuristic_routes_tab[r] = coming_route;
 						metaheuristic_evaluation_tab[client_position] = leaving_route_2opt_cost;
 						metaheuristic_evaluation_tab[r] = coming_route_2opt_cost;
 						metaheuristic_position_tab[client_id] = r;
 						//if (metaheuristic_routes_tab[client_position].size() == 0) metaheuristic_routes_tab[client_position] = vector<int>(0);
 						globally_improved = true;
 						best_solution_value = new_global_cost;                                  //MAIS ICI LE COST GLOBAL
// 						printf("\nAmélioration : nouveau coût %.1f", best_solution_value);
// 						print_solution();
 					}else{
// 						printf("\nTentative de changement de tournée à %d (non améliorante) : %1.f", client_id, new_global_cost);
 					}
 				}
 				if (globally_improved) break; //premier changement améliorant
 			}
 			if (globally_improved) break;  //premier changement ameliorant
 		}

 	}
//	print_solution();
	return best_solution_value;

}

int Graph_AK::evaluate_weight_penality(){
	int penality = 0;
	for(int k = 0; k < metaheuristic_evaluation_weight.size(); k++){
		if (metaheuristic_evaluation_weight[k] > get_capacity())
			penality +=  metaheuristic_evaluation_weight[k] - get_capacity();
	}
	return penality;
}

bool Graph_AK::metaheuristic_clustering(int m){
	metaheuristic_routes_tab = vector< vector<int> >(m, vector<int>(0));
	metaheuristic_position_tab = vector<int>(n, 0);
	int * tab_client = trier_clients();
//	for(int i= 0; i < n-1; i++)
//		printf("%d ", tab_client[i]);
	int t = 0;
	for (int i = 0; i < n-1; i++){
		int node_id = tab_client[i];
		metaheuristic_routes_tab[t].push_back(node_id);
		metaheuristic_position_tab[node_id] = t;
		t = (t + 1)%m;
	}
//	print_solution();
	// FIN DISTIUTION DES CLIENTS EN VUE DE FORMER m TOURNEES REALISABLES

	//INITIALISATION DES POIDS DES TOURNEES
	vector < int  > eval_routes = vector <int>(m);
	metaheuristic_evaluation_weight = vector<int>(m, 0);
	for(int i = 0; i < m; i++){
		for(int k = 0; k < metaheuristic_routes_tab[i].size(); k++){
			metaheuristic_evaluation_weight[i] += get_demand(metaheuristic_routes_tab[i][k]);
		}
	}
	int nb_tests = n;
	while (nb_tests > 0){
		nb_tests --;
		for (int node_id = 1; node_id < n; node_id++){
			int best_target_route = -1;
			int current_global_cost = evaluate_weight_penality();
			// printf("currebt_cost : %d\n", current_global_cost);
			int position_node_id = metaheuristic_position_tab[node_id];

			for (int new_tournee = 0; new_tournee < metaheuristic_routes_tab.size(); new_tournee++){
				if (new_tournee != position_node_id){
					int new_cost = update_metaheuristic_weight(node_id, new_tournee);
					// printf("new cost : %d\n", new_cost);
					if (new_cost < current_global_cost){
						current_global_cost = new_cost;
						best_target_route = new_tournee;
					}
				}
			}

			if(best_target_route != -1){
				metaheuristic_routes_tab[best_target_route].push_back(node_id);
				metaheuristic_position_tab[node_id] = best_target_route;
				metaheuristic_evaluation_weight[best_target_route] += get_demand(node_id);
				metaheuristic_evaluation_weight[position_node_id] -= get_demand(node_id);
				// modification de metaheuristic_routes_tab[position_node_id]
				vector<int> route_left(0);
				for(int l = 0; l < metaheuristic_routes_tab[position_node_id].size(); l++){
					if (metaheuristic_routes_tab[position_node_id][l] != node_id){
						route_left.push_back(metaheuristic_routes_tab[position_node_id][l]);
					}
				}
				metaheuristic_routes_tab[position_node_id] = route_left;
			}
		}
		if (evaluate_weight_penality() == 0)
			break;
	}
//	print_solution();
	return evaluate_weight_penality() == 0;

	return true;

}

/////////////////////////////////////////////////////////////////////////////////

















void Graph_AK::initialize_metaheuristic_tabs(){
	metaheuristic_evaluation_tab = vector<float>(metaheuristic_routes_tab.size(), 0.);
	for(int r = 0; r < metaheuristic_routes_tab.size(); r++)
		metaheuristic_evaluation_tab[r] = two_opt(metaheuristic_routes_tab[r]);
}





float Graph_AK::euclidean_distance(int i, int j){
	pair<int,int> xi = x_y_tab[i];
	pair<int,int> xj = x_y_tab[j];
	return sqrt(pow(xi.first - xj.first,2) + pow(xi.second - xj.second,2) );
}


vector<vector<int> > Graph_AK::get_metaheuristic_routes_tab(){
	vector<vector<int> > routes;

	for(int i = 0; i < metaheuristic_routes_tab.size(); i++){
		if(metaheuristic_routes_tab[i].size() == 0)
			continue;
		routes.push_back(metaheuristic_routes_tab[i]);
	}
	return routes;
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void Graph_AK::construct_Undirected_Lemon_Graph(){

  LGU_name_node.resize(n);
  for (int i = 0; i < n ; i++){
//	if(i != id_depot){
		LGU_name_node[i] = L_GU.addNode();
//		L_rtnmap[L_GU.id(LGU_name_node[i])] = i;
//	}
  }

  LGU_name_link.resize(n,vector<lemon::ListGraph::Edge>(n));

  for (int i = 0; i < n; i++){
    for (int j = i + 1; j < n; j++ ){
//    	if(i != id_depot or j != id_depot){

    	if( (x_value[i][j] + x_value[j][i]) > 0 )
    		LGU_name_link[i][j] = L_GU.addEdge(LGU_name_node[i],LGU_name_node[j]);
//    	}
    }
  }

}


#define PREC 1000 //Pour la precision dans Cplex


double Graph_AK::undirected_MinimumCut(vector<int >& W){

  construct_Undirected_Lemon_Graph();

  lemon::ListGraph::EdgeMap<float> L_cost(L_GU);
  lemon::ListGraph::NodeMap<bool> mincut(L_GU);

  double mincutvalue;

  for (int i = 0; i < n ; i++){
	  for(int j = i + 1; j < n ; j++){
//		  if(i != id_depot or j != id_depot)
		  if(x_value[i][j] + x_value[j][i] > 0 )
			  L_cost.set(LGU_name_link[i][j],(x_value[i][j] + x_value[j][i]) * PREC);
	  }
  }

  lemon::NagamochiIbaraki<lemon::ListGraph, lemon::ListGraph::EdgeMap<float> > L_NI(L_GU,L_cost);
//  printf("Running\n");

  L_NI.run();

  mincutvalue = L_NI.minCutMap (mincut);  //(PREC*1.0);
//  printf(" mincut %f\n",mincutvalue);
  mincutvalue /= (PREC*1.0);
  W.clear();
  for (int i = 0; i < n; i++){
	if ( /*i != id_depot and */mincut[LGU_name_node[i]]){
		W.push_back(i);
	}
  }


  #ifdef OUTPUT_GRAPH
  cout<<"MinCut value : "<<mincutvalue<<endl;
  cout<<"MinCut induced by : ";
  for (int i=0;i<n;i++)
    if (/*i != id_depot and*/ mincut[LGU_name_node[i]])
    	cout<<i<<" ";
  cout<<endl;
  #endif

  return mincutvalue;

}


void Graph_AK::set_x_value(vector< vector<float> > cost_x){
	x_value.resize(cost_x.size());
	for (int i = 0; i < cost_x.size(); i++){
		x_value[i].resize(cost_x[i].size(), 0.0);
		for (int j = 0; j < cost_x[i].size(); j++){
			x_value[i][j] = cost_x[i][j];
		}
	}

//	printf("MATRIX VAR VALUE :\n");
//	for (int i = 0; i < n; i++) {
//		printf("%d : ",i);
//		for (int j = 0; j < n; j++) {
//			printf("%f ", x_value[i][j]);
//		}
//		printf("\n\n");
//	}
//	printf("\n");
}




void Graph_AK::Dijsktra(vector<int> & L, int src, bool atteignable){

	vector <bool > sptSet(n, false);
	vector<int> E;

	E.push_back(src);
	while (!E.empty()){
		int u = E.back();
		E.pop_back();
		sptSet[u] = true;
		for (int v = 0; v < n; v++){
			if (v != u)
				if ( !sptSet[v] and (x_value[u][v] > 0 or x_value[v][u] > 0)){
					E.push_back(v);
				}
		}
	}

	for(int i = 0; i < n; i++){
		if(!sptSet[i] and !atteignable)
			L.push_back(i);

		if (sptSet[i] and atteignable)
			L.push_back(i);
	}

}


bool Graph_AK::has_sub_tour(vector<vector<int> > & W)
{

	int i = 0, u,v;
	vector<int> L;

	Dijsktra(L, id_depot, false);

	// debug
//	printf("\n Inatteignable from depot : \n\t");
//	for (int j = 0; j < L.size(); j++)
//		printf("%d ", L[j]);
//	printf("\n");

	//No circuit
	if ( L.size() == 0 )
		return false;

	while(L.size() > 0){
		vector<int> new_L;
		vector<int> tmp_l;

		Dijsktra(tmp_l,L[0],true);

//		printf("atteignable from %d\n\t", L[0]);
//
//		for (int j = 0; j < tmp_l.size(); j++)
//			printf(" %d ", tmp_l[j]);
//		printf("\n");

		W.push_back(tmp_l);

		for(int j = 0; j < L.size(); j++){
			int k = 0;
			while (k < tmp_l.size() and tmp_l[k] != L[j]){
				k++;
			}
			if (k == tmp_l.size())
				new_L.push_back(L[j]);
		}
		L = new_L;
	}

	return W.size() > 0;
}



bool Graph_AK::is_feasible_tour(vector<vector<int> > & V){

	int i, u, v;
	vector<int> L;

	Dijsktra(L, id_depot,true);

	for(i = 0; i < L.size(); i++){
		if(L[i] == id_depot){
			L.erase(L.begin() + i);
			break;
		}
	}

	while(L.size() > 0){

		vector<int> tmp_l;
		bool still_exists = true;

		u = id_depot;
		float sum = 0;
		while( still_exists ){
			still_exists = false;

			for(int j = 0; j < L.size(); j++){
				v = L[j];
				if(v != u and (x_value[u][v] > 0) or (x_value[v][u] > 0)){
					still_exists = true;
					sum += demands_tab[v];
					tmp_l.push_back(v);
					u = v;
					L.erase(L.begin() + j);
					break;
				}
			}
		}
		if(sum > capacity){
//			printf(" NO FEASABLE TOUR :\n \t");
//			for(int k = 0; k< tmp_l.size(); k++)
//				printf(" %d ",tmp_l[k]);
//			printf("\n");
			V.push_back(tmp_l);
			i++;
		}
	}

	return V.size() > 0;

}















#define NTIME 2				//between 1 and 6
#define ILIMIT 0.2			// between 0.1 and 0.25
#define ULIMIT 0.4			// between 0.3 and 0.45
#define TOPE 10				// between 5 and 60
#define TLL 8				// between 5 and 15
#define PER 0.4				// between 0.2 and 0.6 if NTIME >=2, else 0

int Graph_AK::select_random_first_node(){

	vector<int> list_id;
	for(int i = 0; i < n; i++){
		if( i != id_depot )
			list_id.push_back(i);
	}

	unsigned seed = std::chrono::system_clock::now()
								   .time_since_epoch()
								   .count();

	shuffle (list_id.begin(), list_id.end(), std::default_random_engine(seed));

	int ind = std::experimental::randint(0, (int) list_id.size() - 1);

	return list_id[ind];
}



set<int> Graph_AK::compute_C_removable(set<int> in_S, float smin, vector<int> Tabu_list){
	set<int> c_remove;
	for(set<int>::iterator elem = in_S.begin(); elem != in_S.end(); ++elem){
		if( (demands_tab[*elem] <= smin)   and (Tabu_list[*elem] != 1))
			c_remove.insert(*elem);
	}

	return c_remove;
}

set<int> Graph_AK::compute_C_addable(set<int> out_S, float smax, vector<int> Tabu_list){
	set<int> c_add;
	for(set<int>::iterator elem = out_S.begin(); elem != out_S.end(); ++elem){
		if( (demands_tab[*elem] <= smax)   and (Tabu_list[*elem] != 2))
			c_add.insert(*elem);
	}

	return c_add;
}

float Graph_AK::compute_xS(set<int> S, int elem){
	float sum = 0.;
	for(set<int>::iterator i = S.begin(); i != S.end(); ++i){
		sum += x_value[*i][elem];
	}

	return sum;
}

float Graph_AK::get_max_value_can_get(set<int> C_add, vector<int> Tabu_list, set<int> S){
	float maxi = -1;
	for(set<int>::iterator elem = C_add.begin(); elem != C_add.end(); ++elem){
		if(Tabu_list[*elem] != 2){
			float val = compute_xS(S,*elem);
			if( maxi == -1 or val > maxi){
				maxi = val;
			}
		}
	}
	return maxi;
}



int Graph_AK::random_selection_M(set<int> C_add, vector<int> Tabu_list, set<int> S, float M, int ntime){
	vector<int> list_node;
	int per_ = 0;
	if (ntime >= 2)
		per_ = PER;

	for(set<int>::iterator elem = C_add.begin(); elem != C_add.end(); ++elem){
		if(Tabu_list[*elem] != 2){
			float val = compute_xS(S,*elem);
			if( (val <= M)  and (val >= M - per_))
				list_node.push_back(*elem);
		}
	}

	int ind = std::experimental::randint(0, (int)list_node.size() - 1);

	return list_node[ind];

}


//TODO
void check_cut_equation(set<int> S, vector<vector<int> > & W){
	vector<int> S_vect(S.begin(), S.end());
	W.push_back(S_vect);
}

float Graph_AK::sum_of_demands(set<int> S){
	float sum = 0.;

	for(set<int>::iterator elem = S.begin(); elem  != S.end(); ++elem){
		sum += demands_tab[*elem];
	}

	return sum;
}


void enable_actions(set<int> C_set, vector<int> Tabu_cpt_list){

	for(set<int>::iterator elem = C_set.begin(); elem != C_set.end(); ++elem){
		if(Tabu_cpt_list[*elem] > TLL)
			Tabu_cpt_list[*elem] = 0;
	}
}

int Graph_AK::maximum_reached_add_remove(set<int> C_add, set<int> C_remove, set<int> S){
	float maxi = -1;
	int v_id = (*C_add.begin());
	for(set<int>::iterator elem = C_add.begin(); elem != C_add.end(); ++elem){
		float val = compute_xS(S,*elem);
		if(maxi == -1 or val > maxi){
			maxi = val;
			v_id = *elem;
		}
	}

	set<int> not_in_S;
	for(int i = 0 ; i < n ; i++){
		if( i != id_depot and  ( find(S.begin(), S.end(), i) != S.end()) )
			not_in_S.insert(i);
	}

	for(set<int>::iterator elem = C_remove.begin(); elem != C_remove.end(); ++elem){
		float val = compute_xS(not_in_S,*elem);
		if(maxi == -1 or val > maxi){
			maxi = val;
			v_id = *elem;
		}
	}
	return v_id;
}




bool Graph_AK::tabu_search(vector<vector<int> > & W){

	float smax, smin, M;
	int v, iter;
	for(int ntime = 0; ntime < NTIME; ntime++){
		set<int> S, out_S, in_S, C_add, C_remove;
		vector<int> Tabu_list(n,0);
		vector<int> Tabu_cpt_list(n,0);

		for(int i = 0; i < n; i++){
			if(i != id_depot)
				out_S.insert(i);
		}

		v = select_random_first_node();

		S.insert(v);
		in_S.insert(v);
		out_S.erase(v);
		Tabu_list[v] = 1;

		for(int p = 1; p < m; p++){
			bool found_expansion = true;

			//EXPANSION PHASE
			while(found_expansion){
				smax = capacity * (p + ULIMIT) - sum_of_demands(S);
				C_add = compute_C_addable(out_S, smax, Tabu_list);

				if(C_add.size() == 0){
					found_expansion = false;
					continue;
				}

				M = get_max_value_can_get(C_add, Tabu_list, S);
				v = random_selection_M(C_add,Tabu_list, S, M, ntime);
				in_S.insert(v);
				out_S.erase(v);
				Tabu_list[v] = 1;
				check_cut_equation(S, W);
			}

			//INTERCHANGE PHASE
			iter = 0;
			while(iter < TOPE){
				float dS = sum_of_demands(S);
				smax = capacity * (p + ULIMIT) - dS;
				smin = dS - capacity * (p - ILIMIT);

				compute_C_addable(out_S, smax, Tabu_list);
				compute_C_removable(in_S, smin, Tabu_list);
				enable_actions(C_add, Tabu_cpt_list);
				enable_actions(C_remove, Tabu_cpt_list);

				if(C_add.size() + C_remove.size() == 0)
					break;

				v = maximum_reached_add_remove(C_add, C_remove, S);

				if( find(out_S.begin(), out_S.end(), v) != out_S.end() ){
					Tabu_list[v] = 1;
					in_S.insert(v);
					out_S.erase(v);
				} else {
					Tabu_list[v] = 2;
					in_S.erase(v);
					out_S.insert(v);
				}
				check_cut_equation(S, W);

				iter++;
			}
		}
	}
	return W.size() > 0;
}






void Graph_AK::write_routes(string InstanceName, vector<vector<int> > routes,float sol){
	ostringstream FileName;
	FileName.str("");
	FileName <<InstanceName.c_str() << ".routes";
	ofstream fic(FileName.str().c_str());


	for(int i = 0; i < routes.size(); i++){
		fic<<"Route #"<<i+1<<": ";
		for(int j = 0; j < routes[i].size(); j++){
			fic<<routes[i][j]<<" ";
		}
		fic<<endl;
	}

	fic<<"cost "<<sol<<endl;
	fic.close();
}





void Graph_AK::write_dot_G(string InstanceName,vector<vector<int> > routes){
  ostringstream FileName;
  FileName.str("");
  FileName <<InstanceName.c_str() << "_G.dot";

  ofstream fic(FileName.str().c_str());
  vector <string> colors;
    colors.push_back("darkorchid");
    colors.push_back("darksalmon");
    colors.push_back("gold");
    colors.push_back("plum");
    colors.push_back("tan");
    colors.push_back("darkorange");
    colors.push_back("rosybrown");
    colors.push_back("darkolivegreen3");
    colors.push_back("lightblue3");
    colors.push_back("firebrick");
    colors.push_back("lightslategray");
    colors.push_back("lightskyblue1");
    colors.push_back("gray36");
    colors.push_back("green");
    colors.push_back("blue");
    colors.push_back("red");
    colors.push_back("cyan");
    colors.push_back("yellow");
    colors.push_back("magenta");
    colors.push_back("violetred");
    colors.push_back("darkorchid");
    colors.push_back("darksalmon");
    colors.push_back("gold");
    colors.push_back("plum");
    colors.push_back("tan");
    colors.push_back("darkorange");
    colors.push_back("rosybrown");
    colors.push_back("darkolivegreen3");
    colors.push_back("lightblue3");
    colors.push_back("firebrick");
    colors.push_back("lightslategray");
    colors.push_back("lightskyblue1");


  if(routes.size() > colors.size()){
    cout<<"We only have "<<colors.size()<<" colors and this solutions needs "<<routes.size()<<" colors... some nodes will have wrong colors!"<<endl;
  }

  fic<<"graph G {"<<endl;
  //depot node
  fic<<"  "<<id_depot<<"[shape = box, label = \"depot\", style = filled ];"<<endl;
  vector< vector<int> > routes_without_empty_route;
  for(unsigned int i = 0; i<routes.size(); i++){
	  if(routes[i].size() > 0){
		  routes_without_empty_route.push_back(routes[i]);
	  }
  }
  routes = routes_without_empty_route;
  int id_color = 0;
  for(unsigned int i=0 ; i<routes_without_empty_route.size() ; i++){
    id_color += 1;
    int node_sup, node_inf = id_depot;

    for(unsigned int j = 0; j < routes_without_empty_route[i].size(); j++){
      int node = routes_without_empty_route[i][j];
//      string label = "\""+to_string(node)+" ("+(int(demands_tab[node])) + ")\"";
      fic<<"  "<<node<<"[shape = ellipse];"<<endl;//, label = "<<label<<", style = filled , fillcolor = "<<colors[id_color]<<" ];"<<endl;
      node_sup = node;
//      string len = "\""+to_string(distance_mat[node_inf][node_sup])+"\"";
	  fic<<"  \""<<node_inf<<"\"--\""<<node_sup<<"\"[color = "<<colors[id_color]<<"];"<<endl;
	  node_inf = node;
    }
//    string len = "\""+to_string(distance_mat[node_inf][id_depot])+"\"";

    fic<<"  \""<<node_inf<<"\"--\""<<id_depot<<"\"[color = "<<colors[id_color]<<"];"<<endl;

  }

  fic<<"}"<<endl;

  fic.close();

  ostringstream commande;
  commande.str("");
  commande<<GRAPHVIZ<<"dot -Tpdf -o "<<InstanceName.c_str() << "_G.pdf "<< FileName.str().c_str()<<endl;
  cout<<commande.str().c_str();
  if(system(commande.str().c_str())){cout<<"PDF generated successfully"<<endl;}
  return;
}


