
#include "../Graph_AK.h"
#include <ilcplex/ilocplex.h>
#include "BC_VRP_MTZ.h"

using namespace std;


void add_out_of_depot_constraint(Graph_AK *g, vector<vector <IloNumVar> > x, IloEnv env, IloRangeArray & Constraints ){
	IloExpr c1(env);
	int nbcst = Constraints.getSize();
	int depot = g->get_depot();

    for(int j = 0; j < g->get_n() ; j++){
    	if(j != depot){
    		c1 += x[depot][j];
    	}
    }

    Constraints.add(c1 <= g->get_m());
    ostringstream nomcst;
	nomcst.str("");
	nomcst<<"(1)";
//	cout << "(1)" << Constraints[nbcst] << endl;
	Constraints[nbcst].setName(nomcst.str().c_str());
}

void add_in_of_depot_constraint(Graph_AK *g, vector<vector <IloNumVar> > x, IloEnv env, IloRangeArray & Constraints ){
	IloExpr c2(env);
	int nbcst = Constraints.getSize();
	int depot = g->get_depot();

    for(int j = 0; j < g->get_n() ; j++){
    	if(j != depot){
    		c2 += x[j][depot];
    	}
    }
	Constraints.add(c2 <= g->get_m());
	ostringstream nomcst;
	nomcst.str("");
	nomcst<<"(2)";
//	cout << "(2)" << Constraints[nbcst] << endl;
	Constraints[nbcst].setName(nomcst.str().c_str());
}

void add_in_of_client_constraint(Graph_AK * g, vector<vector<IloNumVar> > x, IloEnv env, IloRangeArray & Constraints){
	int n = g->get_n();
	int nbcst = Constraints.getSize();

	for(int i = 0; i < n; i++){
		IloExpr c3(env);
		if( (i != g->get_depot())){
			for(int j = 0; j < n ; j++){
				if( (j!=i) /*and j!= depot*/){   // error?
					c3 += x[i][j];
				}
			}
			Constraints.add(c3 == 1);
			ostringstream nomcst;
			nomcst.str("");
			nomcst<<"(3_"<<i<<")";
//			cout << "(3_"<<i<<")"<< Constraints[nbcst] << endl;
			Constraints[nbcst].setName(nomcst.str().c_str());
			nbcst++;
		}
	}

}

void add_out_of_client_constraint(Graph_AK * g, vector<vector<IloNumVar> > x, IloEnv env, IloRangeArray & Constraints){
	int n = g->get_n();
	int nbcst = Constraints.getSize();

	for(int j = 0; j < n; j++){
		IloExpr c4(env);
		if(j != g->get_depot()){
			for(int i = 0; i < n ; i++){
				if((j!=i) /*and i!= depot*/){   //error?
					c4 += x[i][j];
				}
			}
			Constraints.add(c4 == 1);
			ostringstream nomcst;
			nomcst.str("");
			nomcst<<"(4_"<<j<<")";
	//			cout << "(4_"<<j<<")"<< Constraints[nbcst] << endl;
			Constraints[nbcst].setName(nomcst.str().c_str());
		}
	}

}

void add_MTZ_constraints(Graph_AK *g, vector<vector<IloNumVar> > x, vector<IloNumVar> w,  IloEnv env, IloRangeArray & Constraints ){

	int n = g->get_n();
	int depot = g->get_depot();
	int nbcst = Constraints.getSize();

	for(int i = 0; i < n ; i++){
		for(int j = 0; j < n; j++){
			if( (j != depot) and (i != depot) and (i != j)){
				IloExpr mtz(env);
//				mtz = w[i] - w[j] - g->get_demand(i) + (g->get_capacity() + g->get_demand(i))*(1 - x[i][j]) ;
				mtz = w[i] - w[j] + g->get_capacity() * x[i][j] - g->get_capacity() + g->get_demand(j);
				Constraints.add(mtz <= 0);

//				Constraints.add(mtz >= 0);
				ostringstream nomcst;
				nomcst.str("");
				nomcst<<"(5_"<<i<<"_"<<j<<")";
//				cout << Constraints[nbcst]<<endl;
				Constraints[nbcst].setName(nomcst.str().c_str());
				nbcst++;
			}
		}
	}
}

void create_binary_var_X(int n, vector<vector<IloNumVar> > & x, IloEnv env){

	for(int i = 0; i < n; i++){
    	for(int j = 0; j < n; j++){
    		if( i != j){
				x[i][j] = IloNumVar(env, 0.0, 1.0, ILOINT);        //Aaaaaaaaaaaaaahhhhhhhhhhhhhhhh NIsssaaaaaaaaaaaaaaaaaaa
				ostringstream varname;
				varname.str("");
				varname<<"x_"<<i<<"_"<<j;
				x[i][j].setName(varname.str().c_str());
    		}
    	}
    }
}

void create_var_W(Graph_AK *g, vector<IloNumVar> &w, IloEnv env){
    int depot = g->get_depot();
    int n = g->get_n();

	for(int i = 0; i < n; i++){
//		w[i] = IloNumVar(env, 0.0, g->get_capacity(), ILOFLOAT);
		w[i] = IloNumVar(env, g->get_demand(i), g->get_capacity(), ILOFLOAT);

		ostringstream nomvar;
		nomvar.str("");
		nomvar<<"w_"<<i;
		w[i].setName(nomvar.str().c_str());
	}
}

void MTZ_Formulation(Graph_AK * g, string filename, vector<vector<IloNumVar > > & x, bool with_usercut){


    IloEnv   env;
    IloModel model(env);
    int n = g->get_n();
    int nb_var = n*(n - 1);

    x.resize(nb_var,vector< IloNumVar >(nb_var));

    /////////////////////////////////
    ///		Create variables X and W
    /////////////////////////////////
    create_binary_var_X(n, x, env);
    vector<IloNumVar> w(n);
    create_var_W(g, w, env);

    /////////////////////////////////
    ///		add constraints
	/////////////////////////////////

    IloRangeArray Constraints(env);

    add_out_of_depot_constraint(g, x, env, Constraints);

    add_in_of_depot_constraint(g, x,env, Constraints);

    add_out_of_client_constraint(g, x, env, Constraints);

    add_in_of_client_constraint(g, x, env, Constraints);

    add_MTZ_constraints(g, x, w, env, Constraints);

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

	if(with_usercut){
		cplex.use(UsercutCutMinSeparation(env, *g,x));
	}

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




//void write_solution(IloEnv env, IloCplex cplex, vector<vector<IloNumVar> > x, Graph_AK * g , string filename){
//
//  cout<<"Wrote LP on file"<<endl;
//  cplex.exportModel("sortie.lp");
//
//  if ( !cplex.solve() ) {
//	 env.error() << "Failed to optimize LP" << endl;
//	 exit(1);
//   }
//
//   env.out() << "Solution status = " << cplex.getStatus() << endl;
//   env.out() << "Solution value  = " << cplex.getObjValue() << endl;
//
//   list< pair<int,int> > Lsol;
//
//   for(unsigned int i = 0; i < x.size() ; i++){
//	  for (unsigned int j=0;j< x.size() ;j++){
//	   if (i!=j ){
//		   if (cplex.getValue(x[i][j]) == 1 ){//>1-epsilon){
//			   Lsol.push_back(make_pair(i,j));
//		   }
//	   }
//	  }
//   }
//
//
//   env.end();
//
////
////   list<pair<int,int> >::const_iterator itp;
////
////
////   ofstream ficsol((filename+".ak_cplex").c_str());
////   double best_length=0;
////   for(itp = Lsol.begin(); itp!=Lsol.end();itp++) {
////	 best_length += g->get_distance(itp->first,itp->second);
////	 ficsol<<itp->first<<" "<<itp->second<<endl;
////   }
////
////   ficsol.close();
////
////   cout<<"Tour found of value : "<<best_length<<endl;
//}



//void branch_and_cut(Graph_AK & g, vector<vector<IloNumVar > > & x, IloEnv env, IloCplex & cplex){
//
//	  /// ADD SEPARATION CALLBACK
//	  cplex.use(UsercutCutMinSeparation(env,g,x));
//
//}




