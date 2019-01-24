#include "../Graph_AK.h"
#include <ilcplex/ilocplex.h>
#include "VRP_MTZ.h"
#include "VRP_CUT.h"
#include <iostream>
#include <fstream>
#include <dirent.h>
#include <ctime>
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
	printf(" value K %d\n",k);
	return k;
}


void script_PLNE(string path){

    ofstream fic;
    fic.open((path+"/cplex.sol").c_str());

    DIR * rep = opendir(path.c_str());

    if(!fic){
    	cerr << "Erreur à l'ouverture !" << endl;
    	return;
    }

    struct dirent *lecture;

    while ((lecture = readdir(rep)) != NULL) {
    	 string f_name = (lecture->d_name);
         if( f_name.find(".vrp") !=  std::string::npos ){

        	int k = get_number_of_routes(f_name);

        	cout<<" FILE : "<<f_name<<endl;
            Graph_AK * g = new Graph_AK(path+"/"+f_name,k);
            vector<vector<IloNumVar > > x;

        	clock_t t1=clock();
			float value = Formulation_COUPES_UNDIRECTED(g, path+"/"+f_name, x, true,false,false);
			clock_t t2=clock() - t1;

			fic<<f_name<<" "<<value<<" "<<double(t2)<<endl;
			string pathing = path+"/plots_cplex/";  // && neato -Tpdf -o "+f_name+"_G.pdf"+" "+f_name+"_G.dot";
//			system(cmd.c_str());
			string cmd = "neato -Tpdf -o "+pathing+f_name+"_G.pdf"+" "+pathing+f_name+"_G.dot";
			system(cmd.c_str());
         }
    }
    fic.close();
    closedir(rep);
}










int main (int argc, char**argv){




/////////////////////

//    if(argc!=3){
//    	cerr<<"Error arguments"<<endl;
//        return 1;
//    }
//
//    string name=argv[1];
//    int m = atoi(argv[2]);
//    Graph_AK * g = new Graph_AK(name+".vrp", m);
//    vector<vector<IloNumVar > > x;
//
////    MTZ_Formulation(g, name, x,true,true);
//
//    Formulation_COUPES_UNDIRECTED(g, name, x, true,true,false);



//////////////// SCRIPT /////////////////

//
    if(argc!=2){
    	cerr<<"Error arguments"<<endl;
        return 1;
    }
    string path = argv[1];
    script_PLNE(path);


    return 0;
}




