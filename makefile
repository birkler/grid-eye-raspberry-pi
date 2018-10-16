all:
	g++ -O3 main.cpp -g -o grid-eye-raspberry-test.exe
	g++ -O3 main_ui.cpp -g `pkg-config --libs opencv` -o grid-eye-raspberry-ui.exe
