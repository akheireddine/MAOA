#include "../Graph_AK.h"
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <time.h> // I don't know which1 exactly :)
#include <string>



int get_number_of_routes(string filename){
	int k = 0, i = 0;

	for(i = 0; i < filename.size(); i++){
		if(filename[i] == 'k'){
			i++;
			break;
		}
	}

	string str_k = "";

	while(isdigit(filename[i])){
		str_k +=filename[i];
		cout<<str_k<<endl;
		i++;
	}

	k = atoi(str_k.c_str());
	return k;
}


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

        	int k = get_number_of_routes(f_name);

        	cout<<" FILE : "<<f_name<<endl;
            Graph_AK * g = new Graph_AK(path+"/"+f_name,k);

        	clock_t t1=clock();
			float value = g->run_metaheuristic();
			clock_t t2=clock() - t1;

			fic<<f_name<<" "<<value<<" "<<((float)t2)/CLOCKS_PER_SEC<<endl;

			string pathing = path+"/plots_heuristics/";

			g->write_dot_G(pathing+f_name,g->get_metaheuristic_routes_tab());
			g->write_routes(pathing+f_name,g->get_metaheuristic_routes_tab(),value);

			string cmd = "neato -Tpdf -o "+pathing+f_name+"_G.pdf"+" "+pathing+f_name+"_G.dot";
			system(cmd.c_str());
         }
    }
    fic.close();
    closedir(rep);
}


int main(int argc, char**argv){


////////////////////////////////////////////// //////////////////////////////////////////////

	string name, nameext;

	if(argc!=3){
		cerr<<"Error arguments"<<endl;
		return 1;
	}
	name=argv[1];
	nameext=name+".vrp";

    int m = atoi(argv[2]);

	Graph_AK * g = new Graph_AK(nameext,m);

	clock_t t1=clock();
	float value = g->run_metaheuristic();
	printf("\nbest solution %.2f (time = %f)\n", value,double(clock() -t1));

	g->write_dot_G(name,g->get_metaheuristic_routes_tab());
	g->write_routes(name,g->get_metaheuristic_routes_tab(),value);



////////////////////////////////////////////// SCRIPT //////////////////////////////////////////////

//
//	if(argc!=2){
//		cerr<<"Error arguments"<<endl;
//		return 1;
//	}
//
//	string path = argv[1];
//
//	script_metaheuristic(path);


	return 0;
}





