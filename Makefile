default:
	clang++ -o path main.cpp -lglut -lGLU -lGL -lXmu -lX11 -lXi -lm

clean:
	rm path

fmt:
	astyle --recursive --style=allman "*.cpp"
