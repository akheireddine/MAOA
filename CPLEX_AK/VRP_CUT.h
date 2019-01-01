#include "../Graph_AK.h"
#include <ilcplex/ilocplex.h>
#include "BC_VRP_CUT.h"


void Formulation_COUPES(Graph_AK * g, string filename, vector<vector<IloNumVar > > & x, bool with_lazycut, bool with_usercut){

	IloEnv   env;
	IloModel model(env);
	int n = g->get_n();
	int nb_var = n*(n - 1);

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

	list< pair<int,int> > Lsol;

	for(unsigned int i = 0; i < n ; i++){
	  for (unsigned int j=0;j< n ;j++){
	   if (i!=j ){
		   if (cplex.getValue(x[i][j]) == 1 ){//>1-epsilon){
			   Lsol.push_back(make_pair(i,j));
		   }
	   }
	  }
	}


	env.end();


	list<pair<int,int> >::const_iterator itp;


	ofstream ficsol((filename+".ak_cplex").c_str());
	double best_length=0;
	for(itp = Lsol.begin(); itp!=Lsol.end();itp++) {
	 best_length += g->get_distance(itp->first,itp->second);
	 ficsol<<itp->first<<" "<<itp->second<<endl;
	}

	ficsol.close();

	cout<<"Tour found of value : "<<best_length<<endl;

}
