//
// Created by finley on 17/11/2021.
//

#ifndef COORDINATE_TRANSFORMATION_COORDINATETRANSFORMATION_H
#define COORDINATE_TRANSFORMATION_COORDINATETRANSFORMATION_H

#include <eigen3/Eigen/Eigen>

const double epsilon = 0.000000000000001;
const double pi = 3.14159265358979323846;
const double d2r = pi / 180;
const double r2d = 180 / pi;

const double a = 6378137.0;		//椭球长半轴
const double f_inverse = 298.257223563;			//扁率倒数
const double b = a - a / f_inverse;
//const double b = 6356752.314245;			//椭球短半轴
const double e = sqrt(a * a - b * b) / a;

struct LocationLLA{
    double Lon;
    double Lat;
    double altitude;
};

typedef struct LocationECEF{
    double x;
    double y;
    double z;
} LocationENU;

void LLA2ECEF (LocationLLA &localla, LocationECEF &locaecef);
void ECEF2LLA (LocationECEF &locaecef, LocationLLA &localla);
void LLA2ENU (LocationLLA &locallaref, LocationLLA &localla, LocationENU &locaenu);
void ENU2LLA (LocationLLA &locallaref, LocationENU &locaenu, LocationLLA &localla);

#endif //COORDINATE_TRANSFORMATION_COORDINATETRANSFORMATION_H
