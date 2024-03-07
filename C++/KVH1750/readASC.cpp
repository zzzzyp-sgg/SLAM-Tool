#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <vector>

void readASCFile(const std::string &filename)
{
    std::ifstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Could not open file: " << filename << std::endl;
        return;
    }

    std::string line;

    std::vector<int> v_gps_week;                // GPS周
    std::vector<double> v_gps_week_sec;         // GPS周秒
    std::vector<double> v_dv_z, v_dv_y, v_dv_x; // 速度增量
    std::vector<double> v_dw_z, v_dw_y, v_dw_x; // 角速度增量

    while (std::getline(file, line))
    {
        std::vector<std::string> tokens;
        std::stringstream ss(line);
        std::string token;
        
        while (std::getline(ss, token, ','))
        {
            tokens.push_back(token);
        }
        
        if (tokens[0] != "%RAWIMUSXA") {    // 找到IMU数据对应的行
            continue;
        }

        v_gps_week.push_back(std::stoi(tokens[4]));
            
        v_gps_week_sec.push_back(std::stod(tokens[5]));

        // acc scale factor: 0.05 / 2^15 (m/s)
        double dvel_z = std::stod(tokens[7]) * 0.05 / 32768;
        v_dv_z.push_back(dvel_z);

        double dvel_y = -std::stod(tokens[8]) * 0.05 / 32768;
        v_dv_y.push_back(dvel_y);

        double dvel_x = std::stod(tokens[9]) * 0.05 / 32768;
        v_dv_x.push_back(dvel_x);

        // gyro scale factor: 0.1 / (3600 * 256) (rad)
        double dw_z = std::stod(tokens[10]) * 0.1 / (3600 * 256);
        v_dw_z.push_back(dw_z);
        
        double dw_y = -std::stod(tokens[11]) * 0.1 / (3600 * 256);
        v_dw_y.push_back(dw_y);

        std::stringstream ss_token(tokens[12]);
        std::string token1;
        std::getline(ss_token, token1, '*');
        double dw_x = std::stod(token1) * 0.1 / (3600 * 256);
        v_dw_x.push_back(dw_x);
    }

    std::ofstream fout("imu_data.txt", std::ios::app);
    fout << std::left << std::setw(10) << "GPS week" << std::setw(15) << "GPS weekSec" << std::setw(20) << "dVel_x" << std::setw(20) << "dVel_y" << std::setw(20) << "dVel_z" << std::setw(20) << "dW_x" << std::setw(20) << "dW_y" << std::setw(20) << "dW_z" << std::endl;
    int length = static_cast<int>(v_gps_week.size());
    for (int i = 0; i < length; i++)
    {
        fout << std::left << std::setw(10) << v_gps_week[i] << std::setw(15) << std::setprecision(10) << v_gps_week_sec[i];
        fout << std::setw(20) << std::setprecision(9) << v_dv_x[i] << std::setw(20) << v_dv_y[i] << std::setw(20) << v_dv_z[i];
        fout << std::setw(20) << v_dw_x[i] << std::setw(20) << v_dw_y[i] << std::setw(20) << v_dw_z[i] << std::endl;
    }
    fout.close();
    std::cout << "IMU data has been written to imu_data.txt" << std::endl;
}

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " <filename>" << std::endl;
        return 1;
    }

    std::ofstream fout("imu_data.txt", std::ios::out);
    fout.close();

    readASCFile(argv[1]);

    return 0;
}