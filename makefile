all:
	g++ -O1 main.cpp -g -o grid-eye-raspberry-test.exe
	g++ -O1 main_ui.cpp -g `pkg-config --libs opencv` -o grid-eye-raspberry-ui.exe
