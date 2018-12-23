#include "../Graph_AK.h"
#include <iostream>
#include <fstream>
#include <dirent.h>

void script_metaheuristic(string path){
    ofstream fic;
    fic.open((path+"/A_metaheuristic.sol").c_str());

    DIR * rep = opendir(path.c_str());

    if(!fic){
    	cerr << "Erreur à l'ouverture !" << endl;
    	return;
    }

    struct dirent *lecture;
     while ((lecture = readdir(rep)) != NULL) {
    	 string f_name = (lecture->d_name);
         if( f_name.find(".vrp") !=  std::string::npos ){
        	cout<<" FILE : "<<f_name<<endl;
            Graph_AK * g = new Graph_AK(path+"/"+f_name);
			float value = g->run_metaheuristic();
			fic<<f_name<<" "<<value<<endl;
			g->write_dot_G(path+"/plots/"+f_name,g->metaheuristic_routes_tab);
			string pathing = path+"/plots/";  // && neato -Tpdf -o "+f_name+"_G.pdf"+" "+f_name+"_G.dot";
//			system(cmd.c_str());
			string cmd = "neato -Tpdf -o "+pathing+f_name+"_G.pdf"+" "+pathing+f_name+"_G.dot";
			system(cmd.c_str());
         }
    }
    fic.close();
    closedir(rep);
}

int main(){

	string filename = "Instances/A/A-n55-k9";//Instances/Vrp-Set-X/X\\X-n200-k36";
	Graph_AK * g = new Graph_AK(filename+".vrp");
//
	float value = g->run_metaheuristic();
	printf("\nbest solution %.2f", value);
//
//	g->write_dot_G(filename,g->metaheuristic_routes_tab);



//	script_metaheuristic("Instances/A");
	return 0;
}
