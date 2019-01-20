#include "../Graph_AK.h"
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <ctime>
#include <time.h> // I don't know which1 exactly :)
#include <string>


void script_metaheuristic(string path){
    ofstream fic;
    fic.open((path+"/metaheuristic.sol").c_str());

    DIR * rep = opendir(path.c_str());

    if(!fic){
    	cerr << "Erreur à l'ouverture !" << endl;
    	return;
    }

    struct dirent *lecture;
    int m = 10;
     while ((lecture = readdir(rep)) != NULL) {
    	 string f_name = (lecture->d_name);
         if( f_name.find(".vrp") !=  std::string::npos ){
        	cout<<" FILE : "<<f_name<<endl;
            Graph_AK * g = new Graph_AK(path+"/"+f_name,m);
        	clock_t t1=clock();
			float value = g->run_metaheuristic();
			clock_t t2=clock() - t1;
			fic<<f_name<<" "<<value<<" "<<double(t2)<<endl;
			//g->write_dot_G(path+"/plots/"+f_name,g->metaheuristic_routes_tab);
			string pathing = path+"/plots_heuristics/";  // && neato -Tpdf -o "+f_name+"_G.pdf"+" "+f_name+"_G.dot";
//			system(cmd.c_str());
			string cmd = "neato -Tpdf -o "+pathing+f_name+"_G.pdf"+" "+pathing+f_name+"_G.dot";
			system(cmd.c_str());
         }
    }
    fic.close();
    closedir(rep);
}

int main(int argc, char**argv){

	string name, nameext;

	if(argc!=3){
		cerr<<"Error arguments"<<endl;
		return 1;
	}
	name=argv[1];
	nameext=name+".vrp";

    int m = atoi(argv[2]);

	Graph_AK * g = new Graph_AK(nameext,m);

	float value = g->run_metaheuristic();
	printf("\nbest solution %.2f\n", value);

//	g->write_dot_G(name,g->get_meta_solution());

	g->print_solution();

//	script_metaheuristic("../Instances/Vrp-Set-X");


	return 0;
}
