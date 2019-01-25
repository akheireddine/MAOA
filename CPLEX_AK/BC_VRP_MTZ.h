

#include <ilcplex/ilocplex.h>
#include "../Graph_AK.h"

#define epsilon_cplex 0.00001
#define epsilonz 0.01



bool  find_ViolatedCutMinCst(IloEnv env, Graph_AK & G,  vector<vector<IloNumVar> >& x,  IloRange & ViolatedCst){

  int i,j;
  vector<int> W;
  vector<int> V_W;
  float test;

  V_W.resize(G.get_n());

  // Find a minimum cut
  test = G.undirected_MinimumCut(W);


  if (test < 1 - epsilonz ) {
    // Found a violated inequality

	bool W_has_depot = false;
    IloExpr expr(env);

    for (i = 0; i < G.get_n() ; i++){
		V_W[i]=0;
    }

    for(i = 0; i < W.size(); i++){
    	if (W[i] == G.get_depot())
    		W_has_depot = true;
    	V_W[W[i]] = 1;
    }

    if(W_has_depot and (G.get_n() - W.size()) < 2){
    	return false;
    }
    if(!W_has_depot and W.size() < 2)
    	return false;

    for (i = 0; i < W.size(); i++){
      for (j = 0; j < G.get_n() ; j++){
    	if (V_W[j] == 0 and (W[i] != j)){
    		if(W_has_depot){
    			expr += x[j][W[i]];
    		}
    		else{
    			expr += x[W[i]][j];
    		}
    	}
      }
    }

    ViolatedCst = IloRange(expr >= 1);

    return true;
  }

  return false;

}



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
