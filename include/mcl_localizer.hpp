#pragma once

/**
 * @brief Monte Carlo Localization for VEX
 */

#include "EZ-Template/api.hpp"
#include "pros/distance.hpp"
#include <cmath>
#include <vector>
#include <cstdlib>
#include <algorithm>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/**
 * @struct MCLDistanceSensorInfo
 * @brief Mounting geometry of a VEX Distance sensor.
 */
struct MCLDistanceSensorInfo {
    pros::Distance sensor;  ///< Distance sensor
    double dx;              ///< Sensor X offset from robot center (in)
    double dy;              ///< Sensor Y offset from robot center (in)
    double angle;           ///< Sensor direction relative to robot forward (rad)
};

/**
 * @struct MCLParticle
 * @brief Represents a particle (state hypothesis).
 */
struct MCLParticle {
    double x;      ///< X position in inches (0 to FIELD)
    double y;      ///< Y position in inches (0 to FIELD)
    double theta;  ///< Heading in degrees
    double w;      ///< Particle weight (normalized)
};

/**
 * @class MCLLocalizer
 * @brief Full Monte Carlo Localization
 */
class MCLLocalizer {
public:

    /**
     * @struct Pose
     * @brief Simple pose container.
     */
    struct Pose {
        double x;        ///< X in inches
        double y;        ///< Y in inches
        double thetaDeg; ///< Heading in degrees
    };

    /**
     * @brief Construct new MCLLocalizer.
     *
     * @param chassisRef Reference to ez::Drive instance
     * @param fieldSizeInches Field side length in inches
     * @param numParticles Number of particles to maintain
     */
    MCLLocalizer(ez::Drive &chassisRef,
                 double fieldSizeInches,
                 int numParticles = 200)
        : chassis(chassisRef),
          FIELD(fieldSizeInches),
          N(numParticles),
          motionNoiseLin(0.5),
          motionNoiseRot(2.0),
          sensorSigma(3.0),
          initialized(false),
          debugEnabled(false)
    {
        if (N < 1) N = 1;
        particles.resize(N);
    }

    /**
     * @brief Add a distance sensor to be used in the measurement model.
     *
     * @param port Brain port of sensor
     * @param dx X offset from robot center (in)
     * @param dy Y offset from robot center (in)
     * @param angleRad Orientation of sensor relative to robot forward, in (rad)
     */
    void addSensor(int port, double dx, double dy, double angleRad) {
        sensors.push_back({pros::Distance(port), dx, dy, angleRad});
        debugSensors.resize(sensors.size());
    }

    /**
     * @brief Init particles uniformly across field
     */
    void initializeUniform() {
        auto o = chassis.odom_pose_get();
        lastOdom = {o.x + FIELD/2, o.y + FIELD/2, o.theta};

        for (int i = 0; i < N; i++) {
            particles[i].x = randomDouble(0, FIELD);
            particles[i].y = randomDouble(0, FIELD);
            particles[i].theta = randomDouble(0, 360);
            particles[i].w = 1.0 / N;
        }
        initialized = true;
    }

    /**
     * @brief Init particles around current odom with Gaussian noise
     *
     * @param posSpreadInches SD for X/Y noise
     * @param headingSpreadDeg SD for heading noise
     */
    void initializeAroundOdom(double posSpreadInches = 3.0,
                              double headingSpreadDeg = 10.0)
    {
        auto o = chassis.odom_pose_get();
        lastOdom = {o.x + FIELD/2, o.y + FIELD/2, o.theta};

        for (int i = 0; i < N; i++) {
            particles[i].x = lastOdom.x + randomNormal(0, posSpreadInches);
            particles[i].y = lastOdom.y + randomNormal(0, posSpreadInches);
            particles[i].theta = lastOdom.thetaDeg + randomNormal(0, headingSpreadDeg);
            particles[i].w = 1.0 / N;
        }
        initialized = true;
    }

    /**
     * @brief Enable or disable debugging
     *
     * @param on True to enable debug
     */
    void setDebug(bool on) {
        debugEnabled = on;
    }

    /**
     * @brief Perform a complete MCL update cycle.
     */
    void update() {
        if (!initialized)
            initializeAroundOdom();

        auto o = chassis.odom_pose_get();
        Pose curr = {o.x + FIELD/2, o.y + FIELD/2, o.theta};
        debugRawOdom = curr;

        double dxW = curr.x - lastOdom.x;
        double dyW = curr.y - lastOdom.y;
        double dth = normalizeDeg(curr.thetaDeg - lastOdom.thetaDeg);

        double prevRad = degToRad(lastOdom.thetaDeg);

        double dxL =  dxW * cos(prevRad) + dyW * sin(prevRad);
        double dyL = -dxW * sin(prevRad) + dyW * cos(prevRad);

        lastOdom = curr;

        for (auto &p : particles) {
            double th = degToRad(p.theta);

            double ndx = dxL + randomNormal(0, motionNoiseLin);
            double ndy = dyL + randomNormal(0, motionNoiseLin);
            double ndt = dth + randomNormal(0, motionNoiseRot);

            p.x += ndx * cos(th) - ndy * sin(th);
            p.y += ndx * sin(th) + ndy * cos(th);
            p.theta = normalizeDeg(p.theta + ndt);

            p.x = clamp(p.x, 0.0, FIELD);
            p.y = clamp(p.y, 0.0, FIELD);
        }

        if (!sensors.empty()) {
            for (int i = 0; i < sensors.size(); i++) {
                double m = getDistanceInches(sensors[i].sensor);
                debugSensors[i].measured = m;
                debugSensors[i].expected = 0;
                debugSensors[i].valid = (m > 0);
            }

            for (auto &p : particles) {
                double w = 1.0;

                for (int i = 0; i < sensors.size(); i++) {
                    double meas = debugSensors[i].measured;
                    if (meas <= 0) continue;

                    double expDist = expectedDistance(p.x, p.y, p.theta, sensors[i]);
                    if (&p == &particles[0])
                        debugSensors[i].expected = expDist;

                    if (expDist <= 0) continue;

                    if (fabs(expDist - meas) > 15) continue; // TODO: parameterize 15in cutoff

                    double err = expDist - meas;
                    double likelihood = exp(-0.5 * (err * err) / (sensorSigma * sensorSigma));
                    w *= std::max(likelihood, 1e-6);
                }

                p.w *= w;
            }
        }

        double sumW = 0;
        for (auto &p : particles) sumW += p.w;

        if (sumW < 1e-9) {
            initializeUniform();
        } else {
            for (auto &p : particles) p.w /= sumW;
        }

        debugEstimate = getEstimate();

        resample();

        chassis.odom_xyt_set(
            debugEstimate.x - FIELD/2,
            debugEstimate.y - FIELD/2,
            debugEstimate.thetaDeg
        );
    }

    /**
     * @brief Print debug information to the screen
     */
    void debugPrintBrain() {
        if (!debugEnabled) return;

        pros::lcd::set_text(
            0,
            "RAW x=" + fmt(debugRawOdom.x) +
            " y=" + fmt(debugRawOdom.y) +
            " th=" + fmt(debugRawOdom.thetaDeg)
        );

        pros::lcd::set_text(
            1,
            "MCL x=" + fmt(debugEstimate.x) +
            " y=" + fmt(debugEstimate.y) +
            " th=" + fmt(debugEstimate.thetaDeg)
        );

        int line = 3;
        for (int i = 0; i < sensors.size() && line < 7; i++) {
            pros::lcd::set_text(
                line++,
                "S" + std::to_string(i) +
                " m=" + fmt(debugSensors[i].measured) +
                " e=" + fmt(debugSensors[i].expected) +
                (debugSensors[i].valid ? " OK" : " BAD")
            );
        }
    }

private:

    ez::Drive &chassis;
    const double FIELD;
    int N;

    std::vector<MCLParticle> particles;
    std::vector<MCLDistanceSensorInfo> sensors;

    double motionNoiseLin;
    double motionNoiseRot;
    double sensorSigma;

    bool initialized;
    bool debugEnabled;

    struct SensorDebugInfo {
        double measured;
        double expected;
        bool valid;
    };

    std::vector<SensorDebugInfo> debugSensors;

    Pose lastOdom, debugRawOdom, debugEstimate;

    double degToRad(double d) { return d * M_PI / 180.0; }

    double normalizeDeg(double d) {
        while (d < 0) d += 360;
        while (d >= 360) d -= 360;
        return d;
    }

    /**
    * @brief Compute weighted mean pose of all particles
    *
    * @return Pose estimated robot position and heading
    */
    Pose getEstimate() {
        double mx = 0;
        double my = 0;
        double c = 0;
        double s = 0;

        for (auto &p : particles) {
            mx += p.x * p.w;
            my += p.y * p.w;
            c += std::cos(degToRad(p.theta)) * p.w;
            s += std::sin(degToRad(p.theta)) * p.w;
        }

        double heading = std::atan2(s, c) * 180.0 / M_PI;
        heading = normalizeDeg(heading);

        return {mx, my, heading};
    }


    double clamp(double v, double lo, double hi) {
        return std::max(lo, std::min(hi, v));
    }

    double randomDouble(double lo, double hi) {
        return lo + (hi - lo) * (std::rand() / (double)RAND_MAX);
    }

    double randomNormal(double mean, double stddev) {
        double u1 = std::max(1e-9, std::rand() / (double)RAND_MAX);
        double u2 = std::rand() / (double)RAND_MAX;
        double z = std::sqrt(-2 * std::log(u1)) * std::cos(2 * M_PI * u2);
        return mean + z * stddev;
    }

    std::string fmt(double v) {
        return std::to_string((int)(v * 10) / 10.0);
    }

    double getDistanceInches(pros::Distance &sensor) {
        int mm = sensor.get();
        if (mm == 0 || mm == 8191) return -1;
        if (mm < 50 || mm > 3000) return -1;
        return mm / 25.4;
    }

    double expectedDistance(double rx, double ry, double thetaDeg,
                            const MCLDistanceSensorInfo &s)
    {
        double th = degToRad(thetaDeg);

        double sx = rx + s.dx * cos(th) - s.dy * sin(th);
        double sy = ry + s.dx * sin(th) + s.dy * cos(th);

        double dir = th + s.angle;

        double best = 1e9, hx = sx, hy = sy;
        checkVert(0, sx, sy, dir, best, hx, hy);
        checkVert(FIELD, sx, sy, dir, best, hx, hy);
        checkHorz(0, sx, sy, dir, best, hx, hy);
        checkHorz(FIELD, sx, sy, dir, best, hx, hy);

        return (best >= 1e8) ? -1 : best;
    }

    void checkVert(double wallX,
                   double sx, double sy, double dir,
                   double &best, double &hx, double &hy)
    {
        double dx = cos(dir);
        if (fabs(dx) < 1e-6) return;

        double t = (wallX - sx) / dx;
        if (t <= 0) return;

        double y = sy + t * sin(dir);
        if (y < 0 || y > FIELD) return;

        if (t < best) {
            best = t;
            hx = wallX;
            hy = y;
        }
    }

    void checkHorz(double wallY,
                   double sx, double sy, double dir,
                   double &best, double &hx, double &hy)
    {
        double dy = sin(dir);
        if (fabs(dy) < 1e-6) return;

        double t = (wallY - sy) / dy;
        if (t <= 0) return;

        double x = sx + t * cos(dir);
        if (x < 0 || x > FIELD) return;

        if (t < best) {
            best = t;
            hx = x;
            hy = wallY;
        }
    }

    void resample() {
        std::vector<MCLParticle> out(N);

        double step = 1.0 / N;
        double r = randomDouble(0, step);
        double c = particles[0].w;
        int i = 0;

        for (int m = 0; m < N; m++) {
            double U = r + m * step;
            while (U > c && i < N - 1) {
                i++;
                c += particles[i].w;
            }
            out[m] = particles[i];
            out[m].w = 1.0 / N;
        }

        particles.swap(out);
    }
};

