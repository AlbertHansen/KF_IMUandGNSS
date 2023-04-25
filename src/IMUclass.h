// www.moodle.aau.dk/mod/page/view.php?id=1535313
#ifndef IMUclass_H_
#define IMUclass_H_

#include <Arduino.h>
#include <tuple>
#include <array>
#include <iostream>
#include <vector>


constexpr struct OPTIONS
{
    int Windowsize = 10;
} IMUREADER_OPTIONS;


class IMUreader
{
    public:
        void AddMeasurement(const std::vector<std::vector<float>>& AccGyroMeasurement);
        std::vector<std::vector<float>> GetAccMean() const;
        std::vector<std::vector<float>> GetGyroMean() const;

        class Measurement
        {
            public:
                struct AccelerationData
                {
                    float AccX { 0 };
                    float AccY { 0 };
                    float AccZ { 0 };
                } AccData;

                struct GyroData
                {
                    float pitch { 0 };
                    float roll  { 0 };
                    float yaw   { 0 };
                } GyroData;
                
            private:
        };

    private:
        std::array<Measurement, IMUREADER_OPTIONS.Windowsize> Measurements;
        int MeasurementIndex { 0 };
        int MeasurementCount { 0 };
};

#endif

