
#include "../Graph_AK.h"

#include <ilcplex/ilocplex.h>
#include <list>


//#define epsilon 0.000000001
using namespace std;
#define epsilon_cplex 0.00001

#define epsilon 0.01
#define OUTPUT



bool find_ViolatedCutMinCst(IloEnv env, Graph_AK & G,  vector<vector<IloNumVar> >& x, IloRange & ViolatedCst);



// Usefull inequalities (here are the same as the necessary ones)
ILOUSERCUTCALLBACK2(UsercutCutMinSeparation, Graph_AK &, G, vector<vector<IloNumVar> >&,x ){
	#ifdef OUTPUT
		cout<<"********* UserCut separation Callback *************"<<endl;
	#endif
  int i,j;
  IloRange ViolatedCst;


  // Put the linear relaxation values on the edges of graph G

  vector< vector<float> > cost_x(x.size(),vector<float>(x.size(),0.0));

  for (i = 0; i < G.get_n()  ; i++){
    for (j = 0 ; j < G.get_n(); j++){
    	if( i != j){
    		cost_x[i][j] = getValue(x[i][j]);
    		if(cost_x[i][j] < epsilon_cplex)
    			cost_x[i][j] = 0 ;
    	}
    }
  }

  G.set_x_value(cost_x);
  /* Separation of Cut inequalities */

  if (find_ViolatedCutMinCst(getEnv(),G,x, ViolatedCst)){
	#ifdef OUTPUT
		cout << "Adding constraint : "<<endl;
		cout<< ViolatedCst << endl;
	#endif
    add(ViolatedCst,IloCplex::UseCutPurge);   // UseCutForce UseCutPurge UseCutFilter
  }
    else {
	#ifdef OUTPUT
    	cout<<"No Cst found"<<endl;
	#endif

    }
}




pair<IloEnv,IloCplex> model_plne(Graph_AK * g, string filename, int m, vector<vector<IloNumVar > > & x){



    IloEnv   env;
    IloModel model(env);
    int n = g->get_n();
    int nb_var = n*(n - 1);

    x.resize(nb_var,vector< IloNumVar >(nb_var));



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

    int depot = g->get_depot();

	vector<IloNumVar> w(n);

	for(int i = 0; i < n; i++){
//		w[i] = IloNumVar(env, 0.0, g->get_capacity(), ILOFLOAT);
		w[i] = IloNumVar(env, g->get_demand(i), g->get_capacity(), ILOFLOAT);

		ostringstream nomvar;
		nomvar.str("");
		nomvar<<"w_"<<i;
		w[i].setName(nomvar.str().c_str());
	}


    IloRangeArray Constraints(env);
    int nbcst = 0;

	IloExpr c1(env), c2(env);

    for(int j = 0; j < n ; j++){
    	if(j != depot){
    		c1 += x[depot][j];
    		c2 += x[j][depot];
    	}
    }
    Constraints.add(c1 <= m);
    ostringstream nomcst;
	nomcst.str("");
	nomcst<<"(1)";
	cout << "(1)" << Constraints[nbcst] << endl;
	Constraints[nbcst].setName(nomcst.str().c_str());
	nbcst++;

	Constraints.add(c2 <= m);
	nomcst.str("");
	nomcst<<"(2)";
	cout << "(2)" << Constraints[nbcst] << endl;
	Constraints[nbcst].setName(nomcst.str().c_str());
	nbcst++;


	for(int i = 0; i < n; i++){
		IloExpr c3(env);
		if( (i != depot)){
			for(int j = 0; j < n ; j++){
				if( (j!=i) /*and j!= depot*/){   // error?
					c3 += x[i][j];
				}
			}
			Constraints.add(c3 == 1);
			ostringstream nomcst;
			nomcst.str("");
			nomcst<<"(3_"<<i<<")";
			cout << "(3_"<<i<<")"<< Constraints[nbcst] << endl;
			Constraints[nbcst].setName(nomcst.str().c_str());
			nbcst++;
		}
	}

	for(int j = 0; j < n; j++){
		IloExpr c4(env);
		if(j != depot){
			for(int i = 0; i < n ; i++){
				if((j!=i) /*and i!= depot*/){   //error?
					c4 += x[i][j];
				}
			}
			Constraints.add(c4 == 1);
			ostringstream nomcst;
			nomcst.str("");
			nomcst<<"(4_"<<j<<")";
			cout << "(4_"<<j<<")"<< Constraints[nbcst] << endl;
			Constraints[nbcst].setName(nomcst.str().c_str());
			nbcst++;
		}
	}



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
				cout << Constraints[nbcst]<<endl;
				Constraints[nbcst].setName(nomcst.str().c_str());
				nbcst++;
			}
		}
	}



  model.add(Constraints);

  IloObjective obj=IloAdd(model, IloMinimize(env));

  for(int i = 0; i < n; i++){
	  for(int j = 0; j < n ; j++){
		  if(i != j){
			  obj.setLinearCoef(x[i][j], g->get_distance(i,j));
		  }
	  }
  }


  IloCplex cplex(model);
  cplex.use(UsercutCutMinSeparation(env, *g,x));


  cout<<"Wrote LP on file"<<endl;
  cplex.exportModel("sortie.lp");

  if ( !cplex.solve() ) {
	 env.error() << "Failed to optimize LP" << endl;
	 exit(1);
   }

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



  return make_pair(env,cplex);

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




int main (int argc, char**argv){

    string name;

    if(argc!=3){
        cerr<<"Error arguments"<<endl; 
        return 1;
    }

    name=argv[1];
    int m = atoi(argv[2]);


    Graph_AK * g = new Graph_AK(name+".vrp");

    vector<vector<IloNumVar > > x;


    pair<IloEnv,IloCplex> env_cplex;

//    env_cplex = model_plne(g, name, m, x);

    model_plne(g, name, m, x);
//    branch_and_cut(g, x , env_cplex.first,env_cplex.second);

//    write_solution(env_cplex.first, env_cplex.second, x, g, name);


    return 0;
}
