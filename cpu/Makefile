CC = g++
CFLAGS = -O3 -framework GLUT -framework OpenGL
LIBS = -lglut -lGL -lGLU -lboost_graph-mt -lboost_thread-mt -lboost_system-mt

rinne: rinne.hpp rinne.cpp
	$(CC) $(CFLAGS) $(LIBS) -o rinne rinne.cpp

crean:
	rm -f rinne *~
