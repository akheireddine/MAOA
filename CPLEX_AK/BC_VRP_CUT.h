#ifndef ____BC_CUT___
#define ____BC_CUT___

#include <ilcplex/ilocplex.h>
#include "../Graph_AK.h"
#include <bits/stdc++.h>

#define epsilon_cplex 0.00001

//#define OUTPUT



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


//////////////////////////////////  LAZY CONSTRAINT  //////////////////////////////////

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
		add(ViolatedCst[i],IloCplex::UseCutForce);   // UseCutForce UseCutPurge UseCutFilter
	}
  }
  else {
	#ifdef OUTPUT
    	cout<<"No Cst found"<<endl;
	#endif

    }
}




bool find_Violated_TabuCST(IloEnv env, Graph_AK & G,  vector<vector<IloNumVar> >& x,  vector<IloRange> & ViolatedCst){


  int i,j,k;
  vector<vector<int> > W;
  bool test = false;
  int MAX_CST = max(G.get_n() * 2, 125);
  priority_queue < pair<float,int>, vector<pair<float,int> >, greater<pair<float,int> > > heap_min_violatedCST;
  vector<IloRange> best_CST;
  vector<int> index_CST_to_remove(0);
  test = G.tabu_search(W);

  if (test) {
	for(i = 0; i < W.size(); i++){
		IloExpr expr(env);
		vector<int> not_in_W;
		float value_expr = 0.0;
		float b = 0.0;

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
				value_expr += G.get_x_value(u,v);
			}
			b += G.get_demand(u);
		}


		b = ceil(b/G.get_capacity());
		value_expr = 2*b - value_expr;

		if(value_expr < 0.01){
			if(best_CST.size() >= MAX_CST){
				pair<float,int> least_violated = heap_min_violatedCST.top();
				index_CST_to_remove.push_back(least_violated.second);
			}
			best_CST.push_back(expr >= 2*b);
			int index = best_CST.size() - 1;
			heap_min_violatedCST.push(make_pair(value_expr,index));

		}
	}

	for(int i = 0; i < best_CST.size(); i++){
		if(find(index_CST_to_remove.begin(), index_CST_to_remove.end(), i) == index_CST_to_remove.end()){
			ViolatedCst.push_back(best_CST[i]);
		}
	}

//	printf("UserCut violated constraints : %d (%d removed) \n",ViolatedCst.size(),index_CST_to_remove.size());
	return ViolatedCst.size() > 0;
  }

  return false;
}

//////////////////////////////////  TABU SEARCH  //////////////////////////////////
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
		add(ViolatedCst[i],IloCplex::UseCutForce);   // UseCutForce UseCutPurge UseCutFilter
	}
  }
  else {
	#ifdef OUTPUT
    	cout<<"No Cst found"<<endl;
	#endif

    }
}



bool node_i_in_list(int i, vector<int> L){

	for(int j = 0; j < L.size(); j++)
		if(L[j] == i)
		return true;
	return false;
}



//////////////////////////////////  GREEDY ALGO  //////////////////////////////////

ILOUSERCUTCALLBACK2(UserCutGreedySeparation, Graph_AK &, G, vector<vector<IloNumVar> >&,x ){

   //	cout<<"********* USER user Cut Separation Callback *************"<<endl;
  vector<IloRange> ViolatedCst;

	for(int i = 1; i < G.get_n(); i++){
		vector<int> S;
		S.push_back(i);  //(1) single node

		int v_star;

		do{
			int max_val = 0;
			v_star = -1;
			for (int j = 1; j < G.get_n(); j++){
				if (!(node_i_in_list(j, S))){
					// pour chaque element de S calculer la somme des aretes le liant à j
					float total = 0;
					for(int k = 0; k < S.size(); k++){
						total += getValue(x[S[k]][j]);
					}
					if(total >= max_val){
						v_star = j;
						max_val = total;
					}
				}
			}
			//// MISE A JOUR SI ON A TROUVE UN ENSEMBLE S VIOLANT L'INEGALITE DE COUPES
			if (v_star != -1){        // if v_star = -1 : plus aucun noeud restant
				S.push_back(v_star);
				float sum_demands = 0.;
				float sum_member_gauch = 0.;
				vector<int> W;
				for(int s=0; s < S.size(); s++){
					sum_demands += G.get_demand(S[s]);
					W.push_back(S[s]);
					for(int o_s = 0; o_s < G.get_n(); o_s++){
						if (!(node_i_in_list(o_s, S))){
							float val = getValue(x[S[s]][o_s]);
							sum_member_gauch += val;
						}
					}
				}

				if(sum_member_gauch + epsilon_cplex < 2 * ceil(sum_demands/G.get_capacity())){
//					cout << "MG " << sum_member_gauch + epsilon_cplex << " MD " <<   2 * ceil(sum_demands/G.get_capacity()) << endl;
//					cout<<"********* USER Cut : FIND VIOLATED INEQUALITY *************"<<endl;
					IloExpr expr(getEnv());
					for(int j = 0; j < W.size(); j++){
							int s = W[j];
//							printf("%d ", s);
							for(int o_s = 0; o_s < G.get_n(); o_s++){
								if (!(node_i_in_list(o_s, W))){
									expr += x[s][o_s];
								}
							}
					}

					ViolatedCst.push_back(expr >= 2 * ceil(sum_demands/G.get_capacity()));
				}
			}

		}while(v_star != -1);
	}
//	printf("======================================>>>>>>>>>>>>>>> %d\n", ViolatedCst.size());
	for(int i = 0; i < ViolatedCst.size(); i++){
		add(ViolatedCst[i],IloCplex::UseCutForce);   // UseCutForce UseCutPurge UseCutFilter
	}

}



















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
		add(ViolatedCst[i],IloCplex::UseCutForce);   // UseCutForce UseCutPurge UseCutFilter
	}
  }
  else {
	#ifdef OUTPUT
    	cout<<"No Cst found"<<endl;
	#endif

    }
}





#endif
