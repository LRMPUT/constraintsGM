/**
 * Project Walkers
 * @author Dominik Belter
 */

#include "Defs/defs.h"
#include <list>
#include <iostream>

#ifndef _RECORDER_H
#define _RECORDER_H

template <class T> class Recorder{
public:
    Recorder(std::string _plotAtr, int _delay, std::string _name) : plotAtr(_plotAtr), name(_name),
        delay(_delay), currDelay(0){
        maxRecordTime = std::numeric_limits<double>::max();
        isRecording = true;
    }
    /// set max recording time
    void setMaxRecordTime(double _maxRecordTime);
    /// save plot
    virtual void savePlot(void) = 0;
    /// store element
    void store(double time, T value);

    /// max recording time
    double maxRecordTime;
    /// plot atributes
    std::string plotAtr;
    /// is recording
    bool isRecording;

    /// time + value (1D+2D)
    std::list<std::pair<double,T>> container;
    /// name
    std::string name;
private:
  /// store every delay-th element
  int delay;
  /// current delay
  int currDelay;
};

/// set max recording time
template <typename T>
void Recorder<T>::setMaxRecordTime(double _maxRecordTime){
    maxRecordTime = _maxRecordTime;
}

template <typename T>
void Recorder<T>::store (double time, T value){
    if (isRecording&&time<maxRecordTime){
        if (currDelay-1==delay){
            container.push_back(std::make_pair(time,value));
            currDelay = 0;
        }
        else
            currDelay++;
    }
    else if (isRecording&&time>=maxRecordTime){
        isRecording = false;
        savePlot();
    }
}

class Recorder1D : public Recorder<double>{
public:
    Recorder1D(int _delay, std::string _name, std::string _plotAtr) : Recorder<double>(_plotAtr, _delay, _name){
    }
    /// plot graph
    void savePlot(void);
private:
};

#endif //_RECORDER_H
