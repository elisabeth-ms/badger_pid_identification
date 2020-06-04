#ifndef PLOT_H
#define PLOT_H


#include <vector>
#include <plotter.h>
#include <plot.h>
#include <fstream>      // std::fstream
#include <algorithm>    // std::min_element, std::max_element

#include <iostream>
//#include <plplot/plplot.h>
//#include <plplot/plstream.h>
#define NSIZE    101
#define STRINGSIZE    255

class IPlot
{
public:
    IPlot();
    IPlot(double sampleTime);

    long pushBack(double new_value);

    long Plot();

    long Plot(std::vector<double> datax, std::vector<double> datay, double scalex, double scaley);
    long PlotAndSave(std::vector<double> datax, std::vector<double> datay, double scalex, double scaley, std::string filename);


private:



    double Ts;
    char xLabel[STRINGSIZE];
    char yLabel[STRINGSIZE];

    std::vector<double> x,y;


    long InitPlot();

    //PlotterParams puPlotParams;
};

#endif // PLOT_H
