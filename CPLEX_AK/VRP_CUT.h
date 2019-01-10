#include "../Graph_AK.h"
#include <ilcplex/ilocplex.h>
#include "BC_VRP_CUT.h"


void Formulation_COUPES(Graph_AK * g, string filename, vector<vector<IloNumVar > > & x, bool with_lazycut, bool with_usercut){

	IloEnv   env;
	IloModel model(env);
	int n = g->get_n();
	int nb_var = n*(n - 1);
	int id_depot = g->get_depot();

	x.resize(nb_var,vector< IloNumVar >(nb_var));

	create_binary_var_X(n,x,env);

    IloRangeArray Constraints(env);

    add_out_of_depot_constraint(g, x, env, Constraints);

    add_in_of_depot_constraint(g, x,env, Constraints);

    add_out_of_client_constraint(g, x, env, Constraints);

    add_in_of_client_constraint(g, x, env, Constraints);

    model.add(Constraints);

    ////////////////////
    ///		OBJECTIVE
    ////////////////////
	IloObjective obj=IloAdd(model, IloMinimize(env));

	for(int i = 0; i < n; i++){
	  for(int j = 0; j < n ; j++){
		  if(i != j){
			  obj.setLinearCoef(x[i][j], g->get_distance(i,j));
		  }
	  }
	}


	IloCplex cplex(model);

	if(with_lazycut){
		cplex.use(LazyCutSeparation(env, *g,x));
	}

//	if(with_usercut){
//		cplex.use(LazyCutSeparation(env, *g,x));
//	}

	cout<<"Wrote LP on file"<<endl;
	cplex.exportModel("sortie.lp");

	if ( !cplex.solve() ) {
		 env.error() << "Failed to optimize LP" << endl;
		 exit(1);
	}


	/////////////////////////
	/// 	Print solution
	///////////////////////
	env.out() << "Solution status = " << cplex.getStatus() << endl;
	env.out() << "Solution value  = " << cplex.getObjValue() << endl;



	int i, u, v;
	vector<vector<int> > Tournees;
	vector<int> L;

	for(i = 0; i < n; i++)
		if(i != id_depot)
			L.push_back(i);

	while(L.size() > 0){

		vector<int> tmp_l;
		bool still_exists = true;

		u = id_depot;

//		printf("\n tournee : \n");

		while( still_exists ){
			still_exists = false;

			for(int j = 0; j < L.size(); j++){
				v = L[j];
				if(v != u and (cplex.getValue(x[u][v]) > 0) or (cplex.getValue(x[v][u]) > 0)){
//					printf(" u %d    v %d    ",u,v);

					still_exists = true;
//					if(v != id_depot){
					tmp_l.push_back(v);
					u = v;
//					}
					L.erase(L.begin() + j);

					break;
				}
			}
		}

		Tournees.push_back(tmp_l);
		i++;
	}

//	for(i = 0; i < Tournees.size();i++){
//		for(int j = 0; j < Tournees[i].size(); j++){
//			printf(" %d ",Tournees[i][j]);
//		}
//		printf("\n");
//	}


	env.end();


	g->write_dot_G(filename,Tournees);


}
