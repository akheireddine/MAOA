#include "Graph_AK.h"
#include <string>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <fstream>
#include <math.h>
#include <limits.h>
#include <lemon/lgf_writer.h>


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
  construct_Undirected_Lemon_Graph();

//  print_distance_matrix();


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

void Graph_AK::initialize_metaheuristic_tabs(){
	metaheuristic_evaluation_tab = vector<float>(n, 0.);
	metaheuristic_routes_tab = vector< vector<int> >(n, vector<int>(0));
	metaheuristic_position_tab = vector<int>(n, 0);
	for (int i = 0; i < n; i++){
		if (i != id_depot){
			metaheuristic_routes_tab[i] = vector<int>(1, i);
			metaheuristic_evaluation_tab[i] = two_opt(metaheuristic_routes_tab[i]);
			metaheuristic_position_tab[i] = i;
		}
	}
}

void Graph_AK::print_solution() {
	for (int r = 1; r < n; r++) {
		printf("\nTournée %d : ", r);
		if (metaheuristic_routes_tab[r].size() != 0) {
			for (unsigned int c = 0; c < metaheuristic_routes_tab[r].size(); c++)
				printf("%d ", metaheuristic_routes_tab[r][c]);
			printf("(%.1f)\n", cost_TSP(metaheuristic_routes_tab[r]));
		}
	}
}

float Graph_AK::run_metaheuristic(){
	initialize_metaheuristic_tabs();
	/* CALCUL DE LA BEST SOLUTION INITIALE (1 CLIENT = 1 TOURNÉE)*/
	float best_solution_value = 0.;
	for (int i = 0; i < n; i++)
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
 			int client_id = changing_route_client_order[i];
 			int client_position = metaheuristic_position_tab[client_id];
 			for (int r = 0; r < n; r++){
 				if (r != client_position && metaheuristic_routes_tab[r].size() != 0){           //si la tournée r n'est pas vide et n'est pas la tournée du client
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
 					for (int ev = 0; ev < n; ev++){
 						if (ev != client_position && ev != r)
 							new_global_cost += metaheuristic_evaluation_tab[ev];               // ON A BESOIN QUE DES 2 NVELLES TOURNEES a comparé avec l'ancienne tournee
 					}
 					if (new_global_cost < best_solution_value){
 						metaheuristic_routes_tab[client_position] = leaving_route;
 						metaheuristic_routes_tab[r] = coming_route;
 						metaheuristic_evaluation_tab[client_position] = leaving_route_2opt_cost;
 						metaheuristic_evaluation_tab[r] = coming_route_2opt_cost;
 						metaheuristic_position_tab[client_id] = r;
 						if (metaheuristic_routes_tab[client_position].size() == 0) metaheuristic_routes_tab[client_position] = vector<int>(0);
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

float Graph_AK::euclidean_distance(int i, int j){
	pair<int,int> xi = x_y_tab[i];
	pair<int,int> xj = x_y_tab[j];
	return sqrt(pow(xi.first - xj.first,2) + pow(xi.second - xj.second,2) );
}





void Graph_AK::construct_Undirected_Lemon_Graph(){

  LGU_name_node.resize(n);
  for (int i = 0; i < n ; i++){
//	if(i != id_depot){
		LGU_name_node[i] = L_GU.addNode();
//		L_rtnmap[L_GU.id(LGU_name_node[i])] = i;
//	}
  }
  LGU_name_link.resize(n,vector<lemon::ListGraph::Edge>(n));

  for (int i = 0; i < n - 1; i++){
    for (int j = i + 1; j < n; j++ ){
//    	if(i != id_depot or j != id_depot){
    		LGU_name_link[i][j] = L_GU.addEdge(LGU_name_node[i],LGU_name_node[j]);
//    	}
    }
  }

}


#define PREC 1000 //Pour la precision dans Cplex


double Graph_AK::undirected_MinimumCut(vector<int >& W){


  lemon::ListGraph::EdgeMap<float> L_cost(L_GU);
  lemon::ListGraph::NodeMap<bool> mincut(L_GU);

  double mincutvalue;

  for (int i = 0; i < n ; i++){
	  for(int j = i + 1; j < n ; j++){
//		  if(i != id_depot or j != id_depot)
			  L_cost.set(LGU_name_link[i][j],(x_value[i][j] + x_value[j][i]) * PREC);
	  }
  }

  lemon::NagamochiIbaraki<lemon::ListGraph, lemon::ListGraph::EdgeMap<float> > L_NI(L_GU,L_cost);
  printf("Running\n");

  L_NI.run();

  mincutvalue = L_NI.minCutMap (mincut);///(PREC*1.0);
  printf(" mincut %f\n",mincutvalue);
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


//	for (int i = 0; i < n; i++) {
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
				if ( !sptSet[v] and (x_value[u][v] != 0 or x_value[v][u] != 0)){
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
	printf("\n inatteignable from depot \n");
	for (int j = 0; j < L.size(); j++)
		printf("%d ", L[j]);
	printf("\n");
	//No circuit
	if ( L.size() == 0 )
		return false;

	while(L.size() > 0){
		vector<int> new_L;
		vector<int> tmp_l;

		Dijsktra(tmp_l,L[0],true);
		printf("atteignable from %d\n", L[0]);

		for (int j = 0; j < tmp_l.size(); j++)
			printf("%d ", tmp_l[j]);
		printf("\n");

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







//void convert_solution_into_routes(string filename){
//	  ostringstream FileName;
//	  FileName.str("");
//	  FileName <<InstanceName.c_str() << ".vrp";
//	  ofstream fic_write(FileName.str().c_str());
//
//	  FileName.str("");
//	  FileName <<InstanceName.c_str() << ".links";
//	  ifstream fic_read(filename.c_str());
//
//	  int i = 0,index = 1;
//	  int u,v;
//	  string line;
//
//	  vector<int> index_route(n,0);
//	  vector<vector<int> > routes;
//
//	  while(!fic_read.eof()){
//		  getline(fic_read,line);
//		  sscanf(line.c_str(), "%d %d", &u, &v);
//
//		  if()
//		  if( u != id_depot and v != id_depot ){
//			  if( (index_route[u] != 0 || index_route[v] != 0)){
//
//			  }
//		  }
//
//	  }
//
//

//}







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
      string label = "\""+to_string(node)+" ("+(int(demands_tab[node])) + ")\"";
      fic<<"  "<<node<<"[shape = ellipse, label = "<<label<<", style = filled , fillcolor = "<<colors[id_color]<<" ];"<<endl;
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


//// Find shortest path for all vertices
//for (int count = 0; count < n-1; count++)
//{
//	// Pick the minimum distance vertex from the set of vertices not
//	// yet processed. u is always equal to src in the first iteration.
//
//	u = minDistance(dist, sptSet);
//	// Mark the picked vertex as processed
//	sptSet[u] = true;
//
//	// Update dist value of the adjacent vertices of the picked vertex.
//	for (int v = 0; v < n; v++){
//
//	 // Update dist[v] only if is not in sptSet, there is an edge from
//	 // u to v, and total weight of path from src to  v through u is
//	 // smaller than current value of dist[v]
//
//		if (v != u)
//			if ( !sptSet[v] and (x_value[u][v] != 0 or x_value[v][u] != 0) and (dist[u] != FLT_MAX) and (dist[u] + 1 < dist[v]))
//				dist[v] = dist[u] + 1;
//	}
//}

