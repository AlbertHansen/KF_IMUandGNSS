// www.moodle.aau.dk/mod/page/view.php?id=1535313
#include <array>
#include <algorithm>
#include <tuple>
#include <Arduino.h>
#include <string>
#include <Arduino_LSM6DS3.h>


std::string s;

template <size_t N>
class IMUReader
{
  public:
    IMUReader() = default;

    bool AddMeasurement();

    class AccGyroMeasurement
    {
    public:
        struct AccData
        {
          float AccX {0.f};
          float AccY {0.f};
          float AccZ {0.f};
        };
        struct GyroData
        {
          float GyroX {0.f};
          float GyroY {0.f};
          float GyroZ {0.f};
        };
    private:
    };

  private:
    std::array<AccGyroMeasurement, N> Measurements;
    uint64_t index{0};
};

