all:
	g++ -O1 main.cpp -o grid-eye-raspberry-test.exe
	g++ -O1 main_ui.cpp `pkg-config --libs opencv` -o grid-eye-raspberry-ui.exe