#ifndef ____BC_CUT___
#define ____BC_CUT___

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
  test = test or G.is_feasible_tour(W);


  if (test) {

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


ILOLAZYCONSTRAINTCALLBACK2(LazyDIRECTEDCutSeparation, Graph_AK &, G, vector<vector<IloNumVar> >&,x ){

	#ifdef OUTPUT
		cout<<"********* Lazy DIRECTED Cut Separation Callback *************"<<endl;
	#endif
  int i,j;

//  IloRange ViolatedCst;
  vector<IloRange> ViolatedCst;


  // Put the linear relaxation values on the edges of graph G

  vector< vector<float> > cost_x(G.get_n(),vector<float>(G.get_n(),0.0));

  for (i = 0; i < G.get_n()  ; i++){
    for (j = 0 ; j < G.get_n(); j++){
    	if( i != j and i < j){
    		cost_x[i][j] = getValue(x[i][j]);
    		if(cost_x[i][j] < epsilon_cplex and getValue(x[j][i]) < epsilon_cplex /* and cost_x[j][i] < epsilon_cplex*/)
    			cost_x[i][j] = 0 ;
    		else
    			cost_x[i][j] = 1;
    	}
    }
  }

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







bool  find_ViolatedCutCst_INTEGER_UNDIRECTED(IloEnv env, Graph_AK & G,  vector<vector<IloNumVar> >& x,  vector<IloRange> & ViolatedCst){

  int i,j,k;
  vector<vector<int> > W;
  bool test = false,test2 = false;
  // Find a minimum cut

  test = G.has_sub_tour(W);
  test2 = G.is_feasible_tour(W);
  test = test or test2;

  if (test) {

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
		ViolatedCst.push_back(expr >= 2*b);
	}

	return true;
  }

  return false;
}


ILOLAZYCONSTRAINTCALLBACK2(LazyUNDIRECTEDCutSeparation, Graph_AK &, G, vector<vector<IloNumVar> >&,x ){

	#ifdef OUTPUT
		cout<<"********* Lazy UNDIRECTED Cut Separation Callback *************"<<endl;
	#endif
  int i,j;

//  IloRange ViolatedCst;
  vector<IloRange> ViolatedCst;


  // Put the linear relaxation values on the edges of graph G

  vector< vector<float> > cost_x(G.get_n(),vector<float>(G.get_n(),0));

  for (i = 0; i < G.get_n()  ; i++){
    for (j = i + 1 ; j < G.get_n(); j++){
		cost_x[i][j] = getValue(x[i][j]);
		if(cost_x[i][j] < epsilon_cplex ){
			cost_x[i][j] = 0;
			cost_x[j][i] = 0;
		} else{
			cost_x[i][j] = 1;
			cost_x[j][i] = 1;
		}
    }
  }

  G.set_x_value(cost_x);
  /* Separation of Cut inequalities */

  if (find_ViolatedCutCst_INTEGER_UNDIRECTED(getEnv(),G,x, ViolatedCst)){

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




#define NTIME 1				//between 1 and 6
#define ILIMIT 0.2			// between 0.1 and 0.25
#define ULIMIT 0.4			// between 0.3 and 0.45
#define TOPE 5				// between 5 and 60
#define TLL 5				// between 5 and 15
#define per 0.3				// between 0.2 and 0.6 if NTIME >=2, else 0

bool find_Violated_TabuCST(IloEnv env, Graph_AK & G,  vector<vector<IloNumVar> >& x,  vector<IloRange> & ViolatedCst){


  int i,j,k;
  vector<vector<int> > W;
  bool test = false;
//  bool test2 = false;

  test = G.tabu_search(W);
//  test2 = G.is_feasible_tour(W);
//  test = test or test2;

  if (test) {

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
		ViolatedCst.push_back(expr >= 2*b);
	}

	return true;
  }

  return false;
}


ILOUSERCUTCALLBACK2(UserCutTabuSeparation, Graph_AK &, G, vector<vector<IloNumVar> >&,x ){

	#ifdef OUTPUT
		cout<<"********* User Cut Tabu Separation Callback *************"<<endl;
	#endif
  int i,j;

//  IloRange ViolatedCst;
  vector<IloRange> ViolatedCst;


  vector< vector<float> > cost_x(G.get_n(),vector<float>(G.get_n(),0));

  for (i = 0; i < G.get_n()  ; i++){
    for (j = i + 1 ; j < G.get_n(); j++){
		cost_x[i][j] = getValue(x[i][j]);
		if(cost_x[i][j] < epsilon_cplex ){
			cost_x[i][j] = 0;
			cost_x[j][i] = 0;
		}

		if(cost_x[i][j] < 1 - epsilon_cplex){
			cost_x[i][j] = 1;
			cost_x[j][i] = 1;
		}

    }
  }

  G.set_x_value(cost_x);
  /* Separation of Cut inequalities */

  if (find_Violated_TabuCST(getEnv(),G,x, ViolatedCst)){

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


#endif
