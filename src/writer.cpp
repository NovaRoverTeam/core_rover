 #include <iomanip>
#include <stdlib.h>
#include <signal.h>
#include <iostream>
#include <math.h>
#include <string>
#include <fstream>
#include <string.h>
#include <unistd.h>

int main () 
{
    char filename[ ] = "../talonPID_data.txt";
    int val = 0;
    std::ofstream myfile;
    while(true){
    myfile.open(filename,std::ios::out|std::ios::app);

        myfile << val;
        val++;
        myfile << " ";
        double value = 11.23444556;

        myfile << value;
        myfile << "\n";
        myfile.close();
        usleep(1000000);

    
    }
    return 0;
}
