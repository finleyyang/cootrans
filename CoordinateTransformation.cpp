//
// Created by finley on 17/11/2021.
//

#include "CoordinateTransformation.h"

void LLA2ECEF (LocationLLA &localla, LocationECEF &locaecef){
    double lamda = localla.Lon * d2r;
    double L = localla.Lat * d2r;
    double h = localla.altitude;
    double Rn = a / sqrt(1- pow(e,2) * pow(sin(L), 2));
    locaecef.x = (Rn + h) * cos(L) * cos(lamda);
    locaecef.y = (Rn + h) * cos(L) * sin(lamda);
    locaecef.z = (Rn * (1- pow(e,2)) + h) * sin(L);
}

void ECEF2LLA (LocationECEF &locaecef, LocationLLA &localla){
    double lamda = atan2(locaecef.y, locaecef.x) * r2d;
    double curL = 0;
    double N = 0;
    double calL = atan2(locaecef.z, sqrt(locaecef.x * locaecef.x + locaecef.y * locaecef.y));
    int counter = 0;
    while (abs(curL - calL) * r2d > epsilon  && counter < 25)
    {
        curL = calL;
        N = a / sqrt(1 - e * e * sin(curL) * sin(curL));
        calL = atan2(locaecef.z + N * e * e * sin(curL), sqrt(locaecef.x * locaecef.x + locaecef.y * locaecef.y));
        counter++;
    }
    localla.Lon = lamda;
    localla.Lat = curL * r2d;
    localla.altitude = locaecef.z / sin(curL) - N * (1 - e * e);
}

void LLA2ENU (LocationLLA &locallaref, LocationLLA &localla, LocationENU &locaenu){
    double lamda = locallaref.Lon * d2r;
    double L = locallaref.Lat *d2r;

    LocationECEF locaecefref;
    LLA2ECEF(locallaref, locaecefref);

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0, 3) = -locaecefref.x;
    T(1, 3) = -locaecefref.y;
    T(2, 3) = -locaecefref.z;

    Eigen::Matrix4d R = Eigen::Matrix4d::Identity();
    R(0,0) = -sin(lamda);
    R(0,1) = cos(lamda);
    R(1,0) = -sin(L) * cos(lamda);
    R(1,1) = -sin(L) * sin(lamda);
    R(1,2) = cos(L);
    R(2,0) = cos(L) * cos(lamda);
    R(2,1) = cos(L) * sin(lamda);
    R(2,2) = sin(L);

    LocationECEF locaecef;
    LLA2ECEF(localla, locaecef);
    Eigen::Vector4d locaecefvector;
    locaecefvector << locaecef.x, locaecef.y, locaecef.z, 1;

    Eigen::Vector4d result;

    result = R * T * locaecefvector;

    locaenu.x = result[0];
    locaenu.y = result[1];
    locaenu.z = result[2];
}

void ENU2LLA (LocationLLA &locallaref, LocationENU &locaenu, LocationLLA &localla){
    double lamda = locallaref.Lon * d2r;
    double L = locallaref.Lat *d2r;

    LocationECEF locaecefref;
    LLA2ECEF(locallaref, locaecefref);

    Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
    T(0, 3) = locaecefref.x;
    T(1, 3) = locaecefref.y;
    T(2, 3) = locaecefref.z;

    Eigen::Matrix4d R = Eigen::Matrix4d::Identity();
    R(0,0) = -sin(lamda);
    R(0,1) = -sin(L) * cos(lamda);
    R(0,2) = cos(L) * cos(lamda);
    R(1,0) = cos(lamda);
    R(1,1) = -sin(L) * sin(lamda);
    R(1,2) = cos(L) * sin(lamda);
    R(2,1) = cos(L);
    R(2,2) = sin(L);

    Eigen::Vector4d locaenuvector;
    locaenuvector << locaenu.x, locaenu.y, locaenu.z, 1;

    Eigen::Vector4d result;

    result = T * R * locaenuvector;

    LocationECEF locaecefresult;
    locaecefresult.x = result[0];
    locaecefresult.y = result[1];
    locaecefresult.z = result[2];

    ECEF2LLA(locaecefresult, localla);
}
