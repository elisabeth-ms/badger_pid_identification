#include "IPlot.h"

IPlot::IPlot()
{
    InitPlot();
    Ts=0.01;
}

IPlot::IPlot(double sampleTime)
{
    InitPlot();
    Ts=sampleTime;
}

long IPlot::pushBack(double new_value)
{
    y.push_back(new_value);
    x.push_back(x.back()+Ts);

    return 0;

}

long IPlot::Plot()
{

    double scx,scy;
    scx = *max_element(x.begin(),x.end());
    scy = *max_element(y.begin(),y.end());
    Plot(x,y,1.5*scx,1.5*scy);
    return 0;
}

long IPlot::Plot(std::vector<double> datax, std::vector<double> datay, double scalex, double scaley)
{
    PlotterParams newParams;
    newParams.setplparam("PAGESIZE", (char *)"a4");
    newParams.setplparam("BITMAPSIZE", (char *)"600x600");

    XPlotter plt(newParams);
    plt.fspace(-scalex, scalex, -scaley, scaley);
    //plt.fscale(2,2);
    plt.openpl();
    plt.pencolorname("blue");




    for (ulong i=1; i<datax.size(); i++)
    {
        //plt.fpoint(datax[i]/scalex,datay[i]/scaley);
        plt.fmove(datax[i]/scalex,datay[i]/scaley);
        plt.fline(datax[i-1]/scalex,datay[i-1]/scaley,datax[i]/scalex,datay[i]/scaley);
        //plt.fcircle(datax[i]/scalex,datay[i]/scaley,std::max(scalex,scaley)/10000.);

        //plt.endpath();
        //plt.flushpl();

    }

    //    plt.move(0.5*scalex,0.5*scaley);
        sprintf (yLabel, "   yMax: %f ", scaley);
    //    cout << scalex;
        plt.label(yLabel);

    plt.endpath();
    plt.flushpl();
    plt.closepl();

    return 0;

}

long IPlot::PlotAndSave(std::vector<double> datax, std::vector<double> datay, double scalex, double scaley, std::string filename)
{

    std::fstream datafile;
    datafile.open (filename.c_str(), std::fstream::out);

    for (ulong i=1; i<datax.size(); i++)
    {


        //fprintf (gdata, "%f - %f - %f - %f \n",Ts*i,vel,actualVel,jointPos);
        datafile << datax[i] << " - "
                 << datay[i] << ""

                 << std::endl;


    }


    Plot(datax,datay,scalex,scaley);

    datafile.close();

    return 0;

}

long IPlot::InitPlot()
{
    Ts=0.01;
    x.clear();
    x.push_back(0.);
    y.clear();
    y.push_back(0.);
    return 0;

}
