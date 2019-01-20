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

    Formulation_COUPES_UNDIRECTED(g, name, x, true,true,false);

//    std::vector<pair<int,int> > v;
//    v.push_back(make_pair(3, 0));
//    v.push_back(make_pair(9, 1));
//    v.push_back(make_pair(1, 2));
//    v.push_back(make_pair(0, 3));


//	std::cout << "initially, v: ";
//	for (auto i : v) std::cout << i << ' ';
//	std::cout << '\n';

//	std::make_heap(v.begin(), v.end() );

//	priority_queue < pair<int,int>, vector<pair<int,int> >, greater<pair<int,int> > > v;
//    v.push(make_pair(3, 0));
//    v.push(make_pair(9, 1));
//    v.push(make_pair(1, 2));
//    v.push(make_pair(0, 3));
//
//    pair<int, int> top = v.top();
//        cout << top.first << " " << top.second<<endl;
//	std::cout << "after make_heap, v: ";
//	for (vector<pair<int,int> >::iterator i = v.begin(); i != v.end(); ++i)
//		std::cout << "(" <<(*i).first<<", "<<(*i).second<<")  ";
//	std::cout << '\n';

//	std::pop_heap(v.begin(), v.end());
//	pair<int,int> largest = v.back();
//	v.pop_back();
//	std::cout << "largest element: " << largest.first <<" "<<largest.second << '\n';
//
//	std::cout << "after removing the largest element, v: ";
//	for (vector<pair<int,int> >::iterator i = v.begin(); i != v.end(); ++i)
//		std::cout << "(" <<(*i).first<<", "<<(*i).second<<")  ";
//	std::cout << '\n';


    return 0;
}




