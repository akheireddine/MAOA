
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

LEMONLIBDIR	= $(LEMONDIR)/lib

CCLNDIRS  = -L$(LEMONLIBDIR)


LEMONINCDIR   = $(LEMONDIR)/include

CCLNFLAGS = -lemon -lm -lpthread -ldl

CCFLAGS = $(CCOPT) -I$(LEMONINCDIR)




# ---------------------------------------------------------------------
# Your part
# ---------------------------------------------------------------------



all: META_HEURISTIC_RESOLUTION

Graph_AK.o: ../Graph_AK.cpp ../Graph_AK.h
	$(CCC) -c $(CCFLAGS) $(CCLNDIRS) -c ../Graph_AK.cpp -o ../Graph_AK.o


meta-heuristic-VRP.o : meta-heuristic-VRP.cpp 
	$(CCC) -c $(CCFLAGS) $(CCLNDIRS)  -c meta-heuristic-VRP.cpp
	
META_HEURISTIC_RESOLUTION : meta-heuristic-VRP.o Graph_AK.o 
	$(CCC) $(CCFLAGS) $(CCLNDIRS) ../Graph_AK.o meta-heuristic-VRP.o  -o META_HEURISTIC_RESOLUTION $(CCLNFLAGS)


clean:
	rm -f *.o META_HEURISTIC_RESOLUTION
