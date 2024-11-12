#ifndef INTERPOLATION_H
#define INTERPOLATION_H

class Interpolation
{
public:
	static double CatmullRomInterpolation(double t, double p0, double p1, double p2, double p3, double tension = 1);
    static double CatmullRomAngleInterpolation(double t, double a0, double a1, double a2, double a3, double tension = 0);

    static double BSplinePositionInterpolation(double t, double p0, double p1, double p2, double p3);
    static double BSplineAngleInterpolation(double t, double a0, double a1, double a2, double a3);


};

enum SplineType
{
    CatMullRom,
    Bspline,
};

enum AngleInput
{
    Fixed,
    Quaternions,
};
#endif // INTERPOLATION_H