#include <ilcplex/ilocplex.h>
#include <vector>
#include"../Graph_AK.h"

#define epsilon 0.00001

using namespace::std;

void  find_ViolatedCutMinCst_INTEGER(IloEnv env, Graph_AK & G,  vector<vector<IloNumVar> >& x, vector<float>&fracsol, list<IloRange> & L_ViolatedCst){

  vector<int> sol;
  list<int> L;
  int i;

  sol.resize(G->get_n());

//   Some "integer" value of CPLEX are not exactly integer...
  for (i=0;i<G->get_n();i++)
	if (fracsol[i]>epsilon)
		sol[i]=1;
	else sol[i]=0;

  if (G->return_cutmin(sol,L)){
	// Found a violated inequality -> add to violatedCte structure
	IloExpr expr(env);
	for(list<int>::const_iterator it=L.begin();it!=L.end();it++)
	  expr+=x[*it];

	i=L.size();
	IloRange newCte = IloRange(expr <= i - 1);
	L_ViolatedCst.push_back(newCte);

  }

}
