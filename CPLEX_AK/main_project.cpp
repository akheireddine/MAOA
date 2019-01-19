#include "../Graph_AK.h"
#include <ilcplex/ilocplex.h>
#include "VRP_MTZ.h"
#include "VRP_CUT.h"
#include <set>



int main (int argc, char**argv){

    string name;

    if(argc!=3){
        cerr<<"Error arguments"<<endl;
        return 1;
    }

    name=argv[1];
    int m = atoi(argv[2]);

    Graph_AK * g = new Graph_AK(name+".vrp", m);

    vector<vector<IloNumVar > > x;


//    MTZ_Formulation(g, name, x,true,true);

//    Formulation_COUPES_UNDIRECTED(g, name, x, true,false);

    set<int> int_set;

    int_set.insert(1);
    int_set.insert(30);
    int_set.insert(2);

    for(set<int>::iterator it = int_set.begin(); it != int_set.end(); ++it)
    	cout<<" "<<*it;
    cout<<endl;

//    cout<<" max size "<<int_set.max_size();

//    int_set.erase(1);
    set<int>::iterator sol = int_set.find(33);
    if (sol != int_set.end())
    	printf(" FIND it %d",*sol);
    else
    	printf(" NOT FOUND :(\n");
    int_set.clear();
    printf(" NEW after erase \n");
    for(set<int>::iterator it = int_set.begin(); it != int_set.end(); ++it)
    	cout<<" "<<*it;
    cout<<endl;

    return 0;
}




