#include <ilcplex/ilocplex.h>
#include "../Graph_AK.h"

#define epsilon_cplex 0.00001

//#define OUTPUT






bool  find_ViolatedCutCst_INTEGER(IloEnv env, Graph_AK & G,  vector<vector<IloNumVar> >& x,  vector<IloRange> & ViolatedCst){

  int i,j,k;
  vector<vector<int> > W;
  bool test = false;
  // Find a minimum cut

  test = G.has_sub_tour(W);
  cout<<"********* has sub tour *************"<<endl;


  if (test) {

	printf("TRUE B %d\n",W.size());
	for(i = 0; i < W.size(); i++){
		for(j = 0; j < W[i].size(); j++){
			printf("%d ",W[i][j]);
		}
		printf("\n");
	}
	// Found a violated inequality

	for(i = 0; i < W.size(); i++){

		IloExpr expr(env);
		float b = 0.0;
		vector<int> not_in_W;
		for(j = 0; j < G.get_n(); j++){
			int z = 0;
			while( z < W[i].size() and j != W[i][z])
				z++;
			if(z == W[i].size())
				not_in_W.push_back(j);
		}

		for(k = 0; k < W[i].size() ; k++){
			int u = W[i][k];
			for(j = 0; j < not_in_W.size(); j++){
				int v = not_in_W[j];
				expr += x[u][v];
			}
			b += G.get_demand(u);
		}

		b = ceil(b/G.get_capacity());
		ViolatedCst.push_back(expr >= b);
	}

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

//  IloRange ViolatedCst;
  vector<IloRange> ViolatedCst;


  // Put the linear relaxation values on the edges of graph G

//  for (int i = 0; i < G.get_n(); i++) {
//  		for (int j = 0; j < G.get_n(); j++) {
//  			if (i != j)
//  				printf("%f ", getValue(x[i][j]));
//  			else
//  				printf("0.000000 ");
//  		}
//  		printf("\n\n");
//  	}
//  	printf("\n");

  vector< vector<float> > cost_x(G.get_n(),vector<float>(G.get_n(),0.0));

  for (i = 0; i < G.get_n()  ; i++){
    for (j = 0 ; j < G.get_n(); j++){
    	if( i != j and i < j){
    		cost_x[i][j] = getValue(x[i][j]);
//    		cost_x[j][i] = getValue(x[j][i]);
    		if(cost_x[i][j] < epsilon_cplex and getValue(x[j][i]) < epsilon_cplex /* and cost_x[j][i] < epsilon_cplex*/)
    			cost_x[i][j] = 0 ;
    		else
    			cost_x[i][j] = 1;
    	}
    }
  }

//  for (int i = 0; i < G.get_n(); i++) {
//    		for (int j = 0; j < G.get_n(); j++) {
//    			if (i != j)
//    				printf("%f ", cost_x[i][j]);
//    			else
//    				printf("0.00y000 ");
//    		}
//    		printf("\n\n");
//    	}
//    	printf("\n");


  G.set_x_value(cost_x);
  /* Separation of Cut inequalities */

  if (find_ViolatedCutCst_INTEGER(getEnv(),G,x, ViolatedCst)){
	#ifdef OUTPUT
		cout << "Adding constraint : "<<endl;
		for(i = 0; i < ViolatedCst.size(); i++)
			cout<< ViolatedCst[i] << endl;
	#endif
	for(i = 0; i < ViolatedCst.size(); i++){
		add(ViolatedCst[i],IloCplex::UseCutPurge);   // UseCutForce UseCutPurge UseCutFilter
	}
  }
  else {
	#ifdef OUTPUT
    	cout<<"No Cst found"<<endl;
	#endif

    }
}
