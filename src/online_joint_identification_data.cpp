#include <complex>


using namespace std;
#include <vector>
#include <array>
#include <fstream>

#include <complex>
#include <math.h>

#include "fcontrol.h"
#include "IPlot.h"


std::vector<std::array<double, 4>> read_csv(std::string filename){

    std::vector<std::array<double,4>> result;
    ifstream myFile(filename);

    if(!myFile.is_open()) 
        throw std::runtime_error("Could not open file");

    std::array<double,4> line_data = {0,0,0,0};
    std::string line;
    while(std::getline(myFile, line))
    {
        std::stringstream ss(line);
        int i=0;
        double val;
        while(ss >> val){
            line_data[i] = val;
            i++;
            if(ss.peek() == ' ') 
                ss.ignore();
        }
        result.push_back(line_data);
    }
    return result;
}

int main()
{
    std::vector<std::array<double,4>> result = read_csv("../data/joint1_data_identification.csv");
    double start_time = result.at(0)[0];
    IPlot real,id;

    int numOrder = 1;
    int denOrder = 2;
    
    OnlineSystemIdentification Gz(numOrder, denOrder);

    double dts=0.1;
    double tmax=60;
    double t = result.at(0)[0];
    int i = 0;
    while(t-start_time<=tmax)
    {
        Gz.UpdateSystem(result.at(i)[2],result.at(i)[1]);
        real.pushBack(result.at(i)[1]);
        t = result.at(i)[0];
        printf("%f\n", result.at(i+1)[0]-result.at(i)[0]);
        printf("%f\n", t-start_time);
        i++;
    }
    int count_max = i;

    real.Plot();

    Gz.PrintZTransferFunction(dts);

    vector<double> num(numOrder+1),den(denOrder+1);
    Gz.GetZTransferFunction(num,den);
    SystemBlock idsys(num,den);

    double in=0,out=0;
    int count = 0;
    while (count < count_max)
    {
        in=result.at(count)[2];//*(rand() % 10 + 1)-5;
        out =in > idsys;
        id.pushBack(out);
        count++;

    }
    id.Plot();

    return 0;
}