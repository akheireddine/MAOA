#include "../Graph_AK.h"
#include <ilcplex/ilocplex.h>
#include "BC_VRP_CUT.h"


void Formulation_COUPES_DIRECTED(Graph_AK * g, string filename, vector<vector<IloNumVar > > & x, bool with_lazycut, bool with_usercut){

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
		cplex.use(LazyDIRECTEDCutSeparation(env, *g,x));
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



void create_binary_undirected_var_X(int n, vector<vector<IloNumVar > > & x, IloEnv env){

	for(int i = 0; i < n; i++){
    	for(int j = i+1; j < n; j++){
			x[i][j] = IloNumVar(env, 0.0, 1.0, ILOINT);
			x[j][i] = x[i][j];
			ostringstream varname;
			varname.str("");
			varname<<"x_"<<i<<"_"<<j;
			x[i][j].setName(varname.str().c_str());
    	}
    }
}


void add_depot_constraint(Graph_AK *g, vector<vector <IloNumVar> > x, IloEnv env, IloRangeArray & Constraints ){
	IloExpr c1(env);
	int nbcst = Constraints.getSize();
	int depot = g->get_depot();

    for(int j = 0; j < g->get_n() ; j++){
    	if(j != depot){
    		c1 += x[depot][j];
    	}
    }
	Constraints.add(c1 <= 2*g->get_m());
	ostringstream nomcst;
	nomcst.str("");
	nomcst<<"(1)";
//	cout << "(2)" << Constraints[nbcst] << endl;
	Constraints[nbcst].setName(nomcst.str().c_str());
}



void add_client_constraint(Graph_AK * g, vector<vector<IloNumVar> > x, IloEnv env, IloRangeArray & Constraints){
	int n = g->get_n();
	int nbcst = Constraints.getSize();

	for(int i = 0; i < n; i++){
		IloExpr c2(env);
		if( (i != g->get_depot())){
			for(int j = 0; j < n ; j++){
				if( (j!=i) /*and j!= depot*/){   // error?
					c2 += x[i][j];
				}
			}

			Constraints.add(c2 == 2);
			ostringstream nomcst;
			nomcst.str("");
			nomcst<<"(2_"<<i<<")";
//			cout << "(3_"<<i<<")"<< Constraints[nbcst] << endl;
			Constraints[nbcst].setName(nomcst.str().c_str());
			nbcst++;
		}
	}

}


void Formulation_COUPES_UNDIRECTED(Graph_AK *g, string filename, vector<vector<IloNumVar > > & x, bool with_lazycut, bool with_usercut){

	IloEnv   env;
	IloModel model(env);
	int n = g->get_n();
	int nb_var = n;
	int id_depot = g->get_depot();

	x.resize(nb_var,vector< IloNumVar >(nb_var));

	create_binary_undirected_var_X(n,x,env);

	IloRangeArray Constraints(env);

	add_depot_constraint(g, x, env, Constraints);
	add_client_constraint(g, x, env, Constraints);

	model.add(Constraints);


	////////////////////
	///		OBJECTIVE
	////////////////////
	IloObjective obj=IloAdd(model, IloMinimize(env));

	for(int i = 0; i < n; i++){
	  for(int j = i+1; j < n ; j++){
			  obj.setLinearCoef(x[i][j], g->get_distance(i,j));
	  }
	}


	IloCplex cplex(model);

	if(with_lazycut){
		cplex.use(LazyUNDIRECTEDCutSeparation(env, *g,x));
	}

//	if(with_usercut){
//		cplex.use(LazyCutSeparation(env, *g,x));
//	}



	//START FROM A HEURISTIC SOLUTION

//	g->run_metaheuristic();
//	vector<vector< int > > starting_solution = g->get_meta_solution();
//
//	// Translate from encoding by a list of nodes to variable x
//	vector<vector<int> > startx;
//	startx.resize(n,vector<int>(n,0));
//
//	for(int t = 0; t < starting_solution.size(); t++){
//		vector<int> tournee = starting_solution[t];
//		for(int k = 1; k < tournee.size() ; k++) {
//			int u = tournee[k-1];
//			int v = tournee[k];
//			if (u < v)
//				startx[u][v] = 1;
//			else
//				startx[v][u] = 1;
//		}
//		if(id_depot < tournee[0])
//			startx[id_depot][tournee[0]] = 1;
//		else
//			startx[tournee[0]][id_depot] = 1;
//	}
//
//	IloNumVarArray startVar(env);
//	IloNumArray startVal(env);
//	for (int i = 0; i < n; i++){
//		for (int j = i+1; j < n; j++) {
//			startVar.add(x[i][j]);
//			startVal.add(startx[i][j]); // startx is your heuristic values
//		}
//	}
//
//	cout<<startVar<<endl;
//	for(int i = 0; i < n;i++){
//		for(int j= i+1; j<n ;j++)
//			printf(" %d ",startx[i][j]);
////		printf("\n")
//	}
//	cplex.addMIPStart(startVar, startVal, IloCplex::MIPStartCheckFeas);
//	startVal.end();
//	startVar.end();




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






   list< pair<int,int> > Lsol;

   for(unsigned int i = 0; i < x.size() ; i++){
	  for (unsigned int j=i+1;j< x.size() ;j++){
	   if (i!=j ){
		   if (cplex.getValue(x[i][j]) >1-epsilon){
			   Lsol.push_back(make_pair(i,j));
		   }
	   }
	  }
   }


//   env.end();

   list<pair<int,int> >::const_iterator itp;


   ofstream ficsol((filename+".ak_cplex").c_str());
   double best_length=0;
   for(itp = Lsol.begin(); itp!=Lsol.end();itp++) {
	 best_length += g->get_distance(itp->first,itp->second);
	 ficsol<<itp->first<<" "<<itp->second<<endl;
   }

   ficsol.close();

   cout<<"Tour found of value : "<<best_length<<endl;






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
				int u_min = min(u,v);
				int v_max = max(u,v);
				if(v != u and (cplex.getValue(x[u_min][v_max]) > 0) ){
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
