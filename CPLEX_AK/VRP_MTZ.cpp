
#include <ilcplex/ilocplex.h>
#include "Graph_AK.h"
#include <list>


#define epsilon 0.000000001
using namespace std;



void start_plne(Graph_AK * g, int m,string filename){

    IloEnv   env;
    IloModel model(env);
    int n = g->get_n();
    int nb_var = n*(n - 1);

    vector<vector<IloNumVar > > x(nb_var,vector< IloNumVar >(nb_var));



    for(int i = 0; i < n; i++){
    	for(int j = 0; j < n; j++){
    		if( i != j){
				x[i][j] = IloNumVar(env, 0.0, 1.1, ILOINT);
				ostringstream varname;
				varname.str("");
				varname<<"x_"<<i<<"_"<<j;
				x[i][j].setName(varname.str().c_str());
    		}
    	}
    }

	vector<IloNumVar> w(n);

	for(int i = 0; i < n; i++){
		w[i] = IloNumVar(env, 0.0, g->get_capacity(), ILOFLOAT);
		ostringstream nomvar;
		nomvar.str("");
		nomvar<<"w_"<<i;
		w[i].setName(nomvar.str().c_str());
	}


    IloRangeArray Constraints(env);
    int nbcst = 0;

    int depot = g->get_depot();
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
	Constraints[nbcst].setName(nomcst.str().c_str());
	nbcst++;

	Constraints.add(c2 <= m);
	nomcst.str("");
	nomcst<<"(2)";
	Constraints[nbcst].setName(nomcst.str().c_str());
	nbcst++;


	for(int i = 0; i < n; i++){
		IloExpr c3(env);
		if(i != depot){
			for(int j = 0; j < n ; j++){
				if(j!=i /*and j!= depot*/){   // error?
					c3 += x[i][j];
				}
			}
			Constraints.add(c3 == 1);
			ostringstream nomcst;
			nomcst.str("");
			nomcst<<"(3_"<<i<<")";
			Constraints[nbcst].setName(nomcst.str().c_str());
			nbcst++;
		}
	}

	for(int j = 0; j < n; j++){
		IloExpr c4(env);
		if(j != depot){
			for(int i = 0; i < n ; i++){
				if(j!=i /*and i!= depot*/){   //error?
					c4 += x[i][j];
				}
			}
			Constraints.add(c4 == 1);
			ostringstream nomcst;
			nomcst.str("");
			nomcst<<"(4_"<<j<<")";
			Constraints[nbcst].setName(nomcst.str().c_str());
			nbcst++;
		}
	}



	for(int i = 0; i < n ; i++){
		for(int j = 0; j < n; j++){
			if( (j != depot) and (i != depot) and (i != j)){
				IloExpr mtz(env);
				mtz = w[i] - w[j] - g->get_demand(i) + (g->get_capacity() + g->get_demand(i))*(1 - x[i][j]) ;
				Constraints.add(mtz >= 0);
				ostringstream nomcst;
				nomcst.str("");
				nomcst<<"(5_"<<i<<"_"<<j<<")";
				Constraints[nbcst].setName(nomcst.str().c_str());
				nbcst++;
			}
		}
	}



  model.add(Constraints);

  IloObjective obj=IloAdd(model, IloMinimize(env, 0.0));

  for(int i = 0; i < n; i++){
	  for(int j = 0; j < n ; j++){
		  if(i != j){
			  obj.setLinearCoef(x[i][j], g->get_distance(i,j));
		  }
	  }
  }


  IloCplex cplex(model);


  cout<<"Wrote LP on file"<<endl;
  cplex.exportModel("sortie.lp");

  if ( !cplex.solve() ) {
     env.error() << "Failed to optimize LP" << endl;
     exit(1);
   }


   env.out() << "Solution status = " << cplex.getStatus() << endl;
   env.out() << "Solution value  = " << cplex.getObjValue() << endl;



   list< pair<int,int> > Lsol;

   for(int i = 0; i < n; i++){
      for (int j=0;j< n ;j++){
       if (i!=j  /*and i!= depot and j!= depot*/){
//    	   printf("x_%d_%d = %f\n", i, j, cplex.getValue(x[i][j]));
    	 // cout<<"x_"<<i<<"_"<<j<<" = "<<cplex.getValue(x[i][j])<<endl;
    	   if (cplex.getValue(x[i][j]) == 1 ){//>1-epsilon){
    		   Lsol.push_back(make_pair(i,j));
    	   }

       }
      }
   }


   //////////////
   //////  CPLEX's ENDING
   //////////////

   env.end();

   //////////////
   //////  OUTPUT
   //////////////

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



int main (int argc, char**argv){

    string name, nameext, nameextsol;
    int i,k;

    vector<int> sol;


    if(argc!=3){
        cerr<<"Error arguments"<<endl; 
        return 1;
    }

    name=argv[1];
    nameext=name+".vrp";
//    nameextsol=name+".ak";

    Graph_AK * g = new Graph_AK(nameext);
    int m = atoi(argv[2]);

    start_plne(g,m,name);



    return 0;
}
