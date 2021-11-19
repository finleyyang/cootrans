#include <iostream>
#include <algorithm>
#include <math.h>
#include "CoordinateTransformation.h"

int main() {

    LocationLLA localla1, localla2;
    LocationECEF locaecef1, locaecef2;
    LocationENU locaenu1, locaenu2;
    localla1.Lon = 116.9395751953;
    localla1.Lat = 36.7399177551;
    localla1.altitude = 0;
    localla2.Lon = 117;
    localla2.Lat = 37;
    localla2.altitude = 10.3;
    LLA2ECEF(localla1, locaecef1);
    LLA2ECEF(localla2, locaecef2);
    std::cout << locaecef1.x<<" "<<locaecef1.y<<" "<<locaecef1.z<< std::endl;
    std::cout << locaecef2.x<<" "<<locaecef2.y<<" "<<locaecef2.z<< std::endl;

    LLA2ENU(localla1, localla2, locaenu2);
    std::cout << locaenu2.x<<" "<<locaenu2.y<<" "<<locaenu2.z<<std::endl;


    return 0;
}
