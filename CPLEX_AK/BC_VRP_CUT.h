#include <ilcplex/ilocplex.h>
#include "../Graph_AK.h"

#define epsilon_cplex 0.00001

#define OUTPUT






bool  find_ViolatedCutCst_INTEGER(IloEnv env, Graph_AK & G,  vector<vector<IloNumVar> >& x,  IloRange & ViolatedCst){

  int i,j;
  vector<int> W;
  bool test = false;
  // Find a minimum cut
  test = G.has_sub_tour(W);

  if (test) {
	// Found a violated inequality
	IloExpr expr(env);
	float b = 0.0;


	for (i = 0; i < W.size(); i++){
	  for (j = i + 1; j < W.size() ; j++){
		int u = W[i], v = W[j];
		expr += x[u][v];
		b += G.get_distance(u,v);
	  }
	}

	b /= G.get_capacity();

	ViolatedCst = IloRange(expr >= b);

	return true;
  }

  return false;


}



// Usefull inequalities (here are the same as the necessary ones)
ILOLAZYCONSTRAINTCALLBACK2(LazyCutSeparation, Graph_AK &, G, vector<vector<IloNumVar> >&,x ){
	#ifdef OUTPUT
		cout<<"********* Lazycut separation Callback *************"<<endl;
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
    		if(cost_x[i][j] < 1 - epsilon_cplex)
    			cost_x[i][j] = 1;
    	}
    }
  }

  G.set_x_value(cost_x);
  /* Separation of Cut inequalities */

  if (find_ViolatedCutCst_INTEGER(getEnv(),G,x, ViolatedCst)){
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
