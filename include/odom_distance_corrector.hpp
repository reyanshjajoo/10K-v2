#pragma once

#include "EZ-Template/api.hpp"
#include "pros/distance.hpp"
#include <cmath>
#include <vector>
#include <unordered_map>
#include <optional> // REQUIRED for std::optional

// Info for one distance sensor
struct DistanceSensorInfo
{
    pros::Distance sensor;
    double dx;    // offset from robot center (inches)
    double dy;    // offset right (+) / left (-)
    double angle; // direction sensor faces (radians)
};

class OdomDistanceCorrector
{
public:
    /**
     * @param chassisRef      your ez::Drive instance
     * @param fieldSizeInches field width (12ft = 144)
     *                        coordinate system becomes centered:
     *                        X,Y = [-FIELD/2 ... +FIELD/2]
     */
    OdomDistanceCorrector(ez::Drive &chassisRef,
                          double fieldSizeInches)
        : chassis(chassisRef),
          FIELD(fieldSizeInches)
    {
        minX = -FIELD / 2.0;
        maxX = FIELD / 2.0;
        minY = -FIELD / 2.0;
        maxY = FIELD / 2.0;
    }

    /**
     * Register a distance sensor with offsets from the robot's center.
     *
     * @param port       Distance.sensor port
     * @param dx         forward/back offset (+forward)
     * @param dy         right/left offset (+right)
     * @param angleRad   direction sensor faces relative to robot forward
     */
    void addSensor(int port, double dx, double dy, double angleRad)
    {
        sensors.push_back({pros::Distance(port), dx, dy, angleRad});
    }

    /**
     * Computes corrected robot center position using all valid sensors. DOesn't modify odom.
     *
     * @return std::optional<{x,y}> or std::nullopt if no valid sensors
     */
    std::optional<std::pair<double, double>> computePosition()
    {
        if (sensors.empty())
            return std::nullopt;

        auto pose = chassis.odom_pose_get();
        double rthetaRad = degToRad(pose.theta);

        std::vector<double> xs;
        std::vector<double> ys;

        for (auto &s : sensors)
        {

            double d = getDistanceInches(s.sensor);
            if (d <= 0)
                continue;

            // Sensor world coords
            double sx = pose.x + s.dx * cos(rthetaRad) - s.dy * sin(rthetaRad);
            double sy = pose.y + s.dx * sin(rthetaRad) + s.dy * cos(rthetaRad);

            double dir = rthetaRad + s.angle;

            double bestDist = 1e9;
            double hitX = sx;
            double hitY = sy;

            checkWallIntersectionVertical(minX, sx, sy, dir, bestDist, hitX, hitY);
            checkWallIntersectionVertical(maxX, sx, sy, dir, bestDist, hitX, hitY);
            checkWallIntersectionHorizontal(minY, sx, sy, dir, bestDist, hitX, hitY);
            checkWallIntersectionHorizontal(maxY, sx, sy, dir, bestDist, hitX, hitY);

            if (bestDist >= 1e8)
                continue;

            // Compute corrected sensor location
            double csx = hitX - d * cos(dir);
            double csy = hitY - d * sin(dir);

            // Convert sensor→robot center
            double crx = csx - s.dx * cos(rthetaRad) + s.dy * sin(rthetaRad);
            double cry = csy - s.dx * sin(rthetaRad) - s.dy * cos(rthetaRad);

            xs.push_back(crx);
            ys.push_back(cry);
        }

        if (xs.empty())
            return std::nullopt;

        // Average valid estimates
        double avgx = 0;
        double avgy = 0;

        for (double v : xs)
            avgx += v;
        for (double v : ys)
            avgy += v;

        avgx /= xs.size();
        avgy /= ys.size();

        return std::make_pair(avgx, avgy);
    }

    /**
     * Teleports odometry pose to the sensor-computed position.
     * Does nothing if no sensor data is valid.
     */
    void correctPos()
    {
        auto computed = computePosition();
        if (!computed.has_value())
            return;

        auto pose = chassis.odom_pose_get();
        chassis.odom_xyt_set(computed->first, computed->second, pose.theta);
    }

private:
    ez::Drive &chassis;
    double FIELD;

    // Centered field boundaries
    double minX, maxX, minY, maxY;

    std::vector<DistanceSensorInfo> sensors;

    // Sensor validity (mm)
    const int minValidMm = 80;
    const int maxValidMm = 5000;

    double getDistanceInches(pros::Distance &sensor)
    {
        static std::unordered_map<int, int> lastReading;
        int mm = sensor.get();

        if (mm <= 0)
            return -1;

        int port = sensor.get_port();

        if (lastReading.find(port) == lastReading.end())
            lastReading[port] = mm;

        if (abs(mm - lastReading[port]) > 1500)
            return -1;

        lastReading[port] = mm;

        return mm / 25.4;
    }

    static double degToRad(double d)
    {
        return d * M_PI / 180.0;
    }

    // WALL INTERSECTIONS — centered field
    void checkWallIntersectionVertical(double wallX,
                                       double sx, double sy,
                                       double dir,
                                       double &bestDist,
                                       double &hitX, double &hitY)
    {
        double dx = cos(dir);
        if (fabs(dx) < 1e-6)
            return;

        double t = (wallX - sx) / dx;
        if (t <= 0)
            return;

        double yhit = sy + t * sin(dir);
        if (yhit < minY || yhit > maxY)
            return;

        if (t < bestDist)
        {
            bestDist = t;
            hitX = wallX;
            hitY = yhit;
        }
    }

    void checkWallIntersectionHorizontal(double wallY,
                                         double sx, double sy,
                                         double dir,
                                         double &bestDist,
                                         double &hitX, double &hitY)
    {
        double dy = sin(dir);
        if (fabs(dy) < 1e-6)
            return;

        double t = (wallY - sy) / dy;
        if (t <= 0)
            return;

        double xhit = sx + t * cos(dir);
        if (xhit < minX || xhit > maxX)
            return;

        if (t < bestDist)
        {
            bestDist = t;
            hitX = xhit;
            hitY = wallY;
        }
    }
};
