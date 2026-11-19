#pragma once

/**
 * Monte Carlo Localization (particle filter) for 7405B.
 *
 * Particles live in FIELD coordinates (inches, origin at field center,
 * +y = "north", heading in degrees clockwise from +y, same convention as LemLib).
 * Each particle is an (x, y) hypothesis; heading comes from the IMU, which is
 * far more accurate than anything the distance sensors could give us.
 *
 * Every loop:
 *   1. MOTION   - all particles move by the odometry delta (+ random noise)
 *   2. SENSOR   - each particle is scored by how well the 4 distance sensor
 *                 readings match a raycast to the field walls from that particle
 *   3. RESAMPLE - unlikely particles are replaced by copies of likely ones
 *   4. ESTIMATE - weighted mean of particles = robot position
 *
 * Modes:
 *   SUBSTITUTE - MCL reads the tracking wheels + IMU directly (it does not use
 *                LemLib's odom output) and writes its estimate into the chassis
 *                every loop, so moveToPoint etc. follow MCL.
 *   TANDEM     - LemLib odom stays in charge. MCL uses LemLib's pose changes as
 *                its motion model and gently pulls the chassis pose toward the
 *                MCL estimate when the filter is confident.
 *   OBSERVE    - Same as TANDEM, but never touches the chassis. Use this
 *                to test/tune: compare mcl::getPose() against chassis.getPose().
 *
 * Coordinate frames:
 *   Your autons use a robot-relative "chassis frame" (chassis.setPose(0,0,0)).
 *   mcl::start() takes the robot's FIELD pose at that moment and remembers the
 *   transform between the two, so existing autons keep their coordinates.
 *
 * Typical usage (in autonomous):
 *     chassis.setPose(0, 0, 0);                  // your normal chassis frame
 *     mcl::start(mcl::Mode::TANDEM, {-48, -60, 0}); // where that is on the field
 *     ... run auton as normal ...
 */

#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/distance.hpp"
#include <vector>

namespace mcl {

enum class Mode { SUBSTITUTE, TANDEM, OBSERVE };

/** One distance sensor and where it sits on the robot. */
struct DistanceSensorConfig {
        pros::Distance* sensor;
        float forwardOffset; // inches the sensor FACE sits in front (+) / behind (-) the tracking center
        float rightOffset;   // inches the sensor FACE sits right (+) / left (-) of the tracking center
        float facing;        // degrees relative to robot front, clockwise: 0 front, 90 right, 180 back, 270 left
};

/** Axis-aligned rectangle (field coords, inches) that blocks distance sensors, e.g. a fixed goal. */
struct Box {
        float minX, minY, maxX, maxY;
};

struct Config {
        int numParticles = 400;

        // ---- field ----
        float fieldHalfWidth = 70.2f;  // half of the INSIDE wall-to-wall distance (inches). Measure your field!
        std::vector<Box> obstacles {}; // static field elements the sensors can see (optional)

        // ---- motion model noise ----
        float alongTrackNoise = 0.05f; // std dev, as fraction of distance travelled, in the direction of travel
        float crossTrackNoise = 0.03f; // std dev, as fraction of distance travelled, sideways
        float baseNoise = 0.01f;       // std dev (inches) added every loop the robot moves

        // ---- sensor model ----
        float sensorSigmaBase = 0.6f;   // std dev (inches) of a distance reading...
        float sensorSigmaScale = 0.04f; // ...plus this fraction of the expected distance
        float outlierProb = 0.15f;      // chance a reading is junk (another robot, game object, etc.)
        float maxSensorRange = 75.0f;   // inches. Readings beyond this are ignored (sensor is rated to ~78in)
        float minSensorRange = 0.8f;    // inches
        int minConfidence = 30;         // 0-63. Only checked for readings > 200mm (sensor limitation)
        float maxTurnRate = 120.0f;     // deg/s. Skip sensor updates while spinning faster (readings smear)

        // ---- update scheduling ----
        float minMoveForUpdate = 0.25f; // inches moved before re-weighting with sensors...
        int stationaryUpdateMs = 250;   // ...or this long while sitting still
        float resampleThreshold = 0.5f; // resample when effective particle count < this * numParticles
        float roughening = 0.05f;       // std dev (inches) of jitter added after resampling

        // ---- tandem correction ----
        float tandemGain = 0.2f;      // fraction of the (MCL - odom) error applied per correction
        float maxCorrectionStd = 2.0f; // only correct when particle spread (inches) is below this
        float minCorrection = 0.3f;    // ignore disagreements smaller than this (inches)

        int loopMs = 20;
};

/** Tuning parameters. Change these before calling start(). */
extern Config config;

/**
 * Start the MCL task. Call AFTER chassis.calibrate() and AFTER your
 * chassis.setPose(...) for the run.
 *
 * @param mode        SUBSTITUTE, TANDEM, or OBSERVE
 * @param fieldStart  the robot's true pose on the field right now (field coords)
 * @param spread      initial uncertainty (std dev, inches) of fieldStart
 */
void start(Mode mode, lemlib::Pose fieldStart, float spread = 1.0f);

/** Stop the MCL task. The chassis keeps whatever pose it had. */
void stop();

bool isRunning();

void setMode(Mode mode);
Mode getMode();

/**
 * Re-scatter the particles around a field pose (e.g. after a big collision).
 * Doesn't change the chassis<->field transform. In SUBSTITUTE mode the chassis
 * pose is updated to match.
 */
void relocalize(lemlib::Pose fieldPose, float spread = 3.0f);

/** Best estimate in FIELD coordinates. */
lemlib::Pose getPose();

/** Best estimate in your auton's CHASSIS coordinates (comparable to chassis.getPose()). */
lemlib::Pose getChassisPose();

/** Particle standard deviation in inches. Small (< ~1.5) means the filter is confident. */
float getSpread();

/** Converts between field and chassis frames (valid after start()). */
lemlib::Pose fieldToChassis(lemlib::Pose fieldPose);
lemlib::Pose chassisToField(lemlib::Pose chassisPose);

} // namespace mcl
