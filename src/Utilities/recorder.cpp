/** @file recorder.cpp
*
* @author Dominik Belter
*/

#include "Utilities/recorder.h"
#include <iostream>
#include <fstream>
#include <iomanip>

/// plot graph
void Recorder1D::savePlot(void){
    std::ofstream mfile;
    mfile.open (name + ".m");
    mfile << name << "Value = [";
    for (const auto & element : container)
        mfile << element.second << ", ";
    mfile << "];\n";
    mfile << name << "Time = [";
    for (const auto & element : container)
        mfile << element.first << ", ";
    mfile << "];\n";
    mfile << "plot(" << name << "Time, " << name << "Value, " << plotAtr << ");";
    mfile.close();
}
