#include "Graph_AK.h"



int main(){

	Graph_AK * g = new Graph_AK("A-n5.vrp");
	vector<int> route(4);
	route[0] = 1;
	route[1] = 3;
	route[2] = 2;
	route[3] = 4;
	printf(" BEFORE : ");
	for(unsigned int i = 0; i < route.size(); i++)
			printf("%d ", route[i]);
	printf("  %f \n", g->cost_TSP(route));
	printf("  %f \n",g->two_opt(route));
	printf(" AFTER : ");
	for(unsigned int i = 0; i < route.size(); i++)
		printf("%d ", route[i]);
	return 0;
}
