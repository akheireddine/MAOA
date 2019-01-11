#include "../Graph_AK.h"
#include <ilcplex/ilocplex.h>
#include "VRP_MTZ.h"
#include "VRP_CUT.h"





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

    Formulation_COUPES(g, name, x, true,false);

    return 0;
}




