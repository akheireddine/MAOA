#include <ilcplex/ilocplex.h>
#include <vector>
#include"../Graph_AK.h"

#define epsilon 0.01

using namespace::std;



bool  find_ViolatedCutMinCst(IloEnv env, Graph_AK & G,  vector<vector<IloNumVar> >& x,  IloRange & ViolatedCst){

  int i,j;
  vector<int> W;
//  vector<int>::const_iterator it;
  vector<int> V_W;
  float test;

  V_W.resize(G.get_n());

  // Find a minimum cut
  test = G.undirected_MinimumCut(W);

  //cout<<"test = "<<test<<endl;

  if (test < 1 - epsilon ) {
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
