#include <ilcplex/ilocplex.h>

#define epsilon 0.00001



bool find_ViolatedCutMinCst(IloEnv env, Graph_AK & G,  vector<vector<IloNumVar> >& x, IloRange & ViolatedCst);



// Usefull inequalities (here are the same as the necessary ones)
ILOUSERCUTCALLBACK2(UsercutCutMinSeparation, Graph_AK &, G, vector<vector<IloNumVar> >&,x ){
  #ifdef OUTPUT
  cout<<"********* UserCut separation Callback *************"<<endl;
  #endif

  int i,j;
  IloRange ViolatedCst;

  // Put the linear relaxation values on the edges of graph G

  vector< vector<float> > cost_x;
  for (i = 0; i < G.get_n() - 1 ; i++){
    for (j = i + 1; j < G.get_n(); j++){

      cost_x[i][j] = getValue(x[i][j]);
  	  if(cost_x[i][j] < epsilon)
  		cost_x[i][j] = 0 ;

  	  cost_x[j][i] = getValue(x[j][i]);
	  if(cost_x[j][i] < epsilon)
		cost_x[j][i] = 0 ;
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
  #ifdef OUTPUT
    else {
      cout<<"No Cst found"<<endl;
    }
  #endif
}
