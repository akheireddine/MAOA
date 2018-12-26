#include <ilcplex/ilocplex.h>
#include <vector>
#include"../Graph_AK.h"

#define epsilon 0.01

using namespace::std;



bool  find_ViolatedCutMinCst(IloEnv env, Graph_AK & G,  vector<vector<IloNumVar> >& x,  IloRange & ViolatedCst){

  int i,j;
  vector<int> W;
  list<int>::const_iterator it;
  vector<int> V_W;
  float test;

  V_W.resize(G->get_n());

  // Find a minimum cut

  test = G->undirected_MinimumCut(W);

  //cout<<"test = "<<test<<endl;

  if (test < 2-epsilon) {
    // Found a violated inequality

    IloExpr expr(env);
    j = 0;
    for (int i = 0; i < G->get_n() ; i++){
		if (W[j] == i)
			V_W[i] = 1;
		else
			V_W[i]=0;
    }

    for (it = W.begin(); it != W.end(); it++){
      for (j = 0; j < G->get_n() ; j++)
    	if (V_W[j] == 0)
    		expr+=x[*it][j];
    }


    ViolatedCst = IloRange(expr >= 1);
    return true;
  }

  return false;

}
