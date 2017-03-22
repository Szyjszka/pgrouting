#ifndef ALGORITHM_TIME_MEASURE_HPP
#define ALGORITHM_TIME_MEASURE_HPP

#include <chrono>

namespace RouterCH
{

class AlgorithmTimeMeasure
{

private:
    std::chrono::time_point<std::chrono::system_clock> start;
    std::chrono::duration<double> meanTime;
    unsigned int numberOfMeasures;
    void addToMeasures(std::chrono::duration<double> elapsedSeconds)
    {
        ++numberOfMeasures;
        meanTime *= (static_cast<double>(numberOfMeasures-1))/numberOfMeasures;
        meanTime += elapsedSeconds/numberOfMeasures;
    }
public:
    AlgorithmTimeMeasure() :
        meanTime(0),
        numberOfMeasures(0)
    {
    }

    void startMeasurement()
    {
        start = std::chrono::system_clock::now();
    }
    void stopMeasurement()
    {
        std::chrono::duration<double> elapsedSeconds =
                std::chrono::system_clock::now() - start;
        addToMeasures(elapsedSeconds);
    }

    double getMeanTime() const
    {
        return meanTime.count();
    }
};

}
#endif // ALGORITHM_TIME_MEASURE_HPP
