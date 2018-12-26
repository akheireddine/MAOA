#------------------------------------------------------------
#
# Please define an environnement variable PATHTUTOMIP
# equal to the XXX directory where Lemon,Graphviz and Cplex have been installed
# In order to these:
# edit ~/.bashrc add "export PATHTUTOMIP=XXX"
# close your xterm and star new ones
#
#------------------------------------------------------------

CPLEXDIR      = $(PATHTUTOMIP)/cplex-12.7.1/cplex
CONCERTDIR    = $(PATHTUTOMIP)/cplex-12.7.1/concert
LEMONDIR      = $(PATHTUTOMIP)/lemon-1.3.1


SYSTEM     = x86-64_linux
LIBFORMAT  = static_pic


# ---------------------------------------------------------------------
# Compiler selection 
# ---------------------------------------------------------------------

CCC = g++ -O0


# ---------------------------------------------------------------------
# Compiler options 
# ---------------------------------------------------------------------

CCOPT = -m64 -O -fPIC -fno-strict-aliasing -fexceptions -DNDEBUG -DIL_STD

# ---------------------------------------------------------------------
# Link options and libraries
# ---------------------------------------------------------------------

CPLEXBINDIR   = $(CPLEXDIR)/bin/$(BINDIST)
CPLEXLIBDIR   = $(CPLEXDIR)/lib/$(SYSTEM)/$(LIBFORMAT)
CONCERTLIBDIR = $(CONCERTDIR)/lib/$(SYSTEM)/$(LIBFORMAT)
LEMONLIBDIR	= $(LEMONDIR)/lib

CCLNDIRS  = -L$(CPLEXLIBDIR) -L$(CONCERTLIBDIR) -L$(LEMONLIBDIR)


CONCERTINCDIR = $(CONCERTDIR)/include
CPLEXINCDIR   = $(CPLEXDIR)/include
LEMONINCDIR   = $(LEMONDIR)/include

CCLNFLAGS = -lconcert -lilocplex -lcplex -lemon -lm -lpthread

CCFLAGS = $(CCOPT) -I$(CPLEXINCDIR) -I$(CONCERTINCDIR) -I$(LEMONINCDIR)




# ---------------------------------------------------------------------
# Your part
# ---------------------------------------------------------------------



all: VRP_MTZ

Graph_AK.o: ../Graph_AK.cpp ../Graph_AK.h
	$(CCC) -c $(CCFLAGS) $(CCLNDIRS) -c ../Graph_AK.cpp -o ../Graph_AK.o

CutMinSeparation.o: CutMinSeparation.cpp ../Graph_AK.h
	$(CCC) -c $(CCFLAGS) $(CCLNDIRS) -c CutMinSeparation.cpp

VRP_MTZ.o: VRP_MTZ.cpp
	$(CCC) -c $(CCFLAGS) $(CCLNDIRS) VRP_MTZ.cpp

VRP_MTZ: VRP_MTZ.o  ../Graph_AK.o 
	$(CCC) $(CCFLAGS) $(CCLNDIRS) VRP_MTZ.o ../Graph_AK.o  CutMinSeparation.o -o VRP_MTZ $(CCLNFLAGS)

clean:
	rm -f *.o VRP_MTZ
