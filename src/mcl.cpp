#include "mcl.h"
#include "devices.h"
#include "pros/rtos.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <random>

// =============================================================================
//                          ROBOT-SPECIFIC SETUP
// =============================================================================
// !!! SET THESE PORTS AND OFFSETS TO MATCH THE ROBOT !!!
// Offsets are measured from the tracking center (center of rotation) to the
// FACE of each sensor, in inches.

namespace {

pros::Distance dist_north(11); // TODO: real port
pros::Distance dist_east(12);  // TODO: real port
pros::Distance dist_south(13); // TODO: real port
pros::Distance dist_west(14);  // TODO: real port

//                                     sensor        fwd     right   facing
const mcl::DistanceSensorConfig SENSORS[] = {
    {&dist_north,  7.0f,   0.0f,   0.0f}, // front
    {&dist_east,   0.0f,   6.0f,  90.0f}, // right
    {&dist_south, -7.0f,   0.0f, 180.0f}, // back
    {&dist_west,   0.0f,  -6.0f, 270.0f}, // left
};

// Vertical tracking wheel (used in SUBSTITUTE mode).
// Set VERTICAL_ROTATION_PORT to the rotation sensor port of the vertical
// odometry wheel. Negative port = reversed. 0 = fall back to the drive motor
// encoders (which is what LemLib is doing right now, since the vertical wheel
// is nullptr in devices.cpp).
constexpr int VERTICAL_ROTATION_PORT = 0;        // TODO: real port
constexpr float VERTICAL_WHEEL_DIAMETER = lemlib::Omniwheel::NEW_2;
constexpr float VERTICAL_WHEEL_OFFSET = 0.0f;    // inches; same sign convention as LemLib

// =============================================================================
//                               INTERNALS
// =============================================================================

constexpr float DEG2RAD = 3.14159265358979f / 180.0f;

struct Particle {
        float x, y, w;
};

// ---- state (guarded by stateMutex where shared with the outside) ----
pros::Mutex stateMutex;
std::vector<Particle> particles;
std::unique_ptr<pros::Task> task;
volatile bool running = false;
mcl::Mode mode = mcl::Mode::TANDEM;

lemlib::Pose estimate(0, 0, 0); // field frame
float spread = 0;

// chassis frame -> field frame: field = rotate(chassis - chassisAnchor) + fieldAnchor
lemlib::Pose fieldAnchor(0, 0, 0);
lemlib::Pose chassisAnchor(0, 0, 0);
float frameRotation = 0; // degrees, field heading - chassis heading

std::mt19937 rng;

// ---- odometry sources for SUBSTITUTE mode ----
std::unique_ptr<pros::Rotation> verticalRotation;
std::unique_ptr<lemlib::TrackingWheel> verticalWheel;
std::unique_ptr<lemlib::TrackingWheel> leftDriveWheel;
std::unique_ptr<lemlib::TrackingWheel> rightDriveWheel;

float prevVertical = 0;
float prevHorizontal = 0;
float prevImuHeading = 0; // degrees
float imuToField = 0;     // field heading = imu rotation + imuToField

// ---- motion source for TANDEM / OBSERVE ----
lemlib::Pose prevChassis(0, 0, 0);

float gauss(float sigma) {
        if (sigma <= 0) return 0;
        std::normal_distribution<float> d(0.0f, sigma);
        return d(rng);
}

// rotate a vector by `deg` clockwise (compass convention)
void rotateCW(float& x, float& y, float deg) {
        const float c = std::cos(deg * DEG2RAD), s = std::sin(deg * DEG2RAD);
        const float nx = x * c + y * s;
        const float ny = -x * s + y * c;
        x = nx;
        y = ny;
}

float verticalDistance() {
        if (verticalWheel) return verticalWheel->getDistanceTraveled();
        return (leftDriveWheel->getDistanceTraveled() + rightDriveWheel->getDistanceTraveled()) / 2.0f;
}

float verticalOffset() { return verticalWheel ? verticalWheel->getOffset() : 0.0f; }

void initOdomSources() {
        if (VERTICAL_ROTATION_PORT != 0 && !verticalWheel) {
                verticalRotation = std::make_unique<pros::Rotation>(VERTICAL_ROTATION_PORT);
                verticalWheel = std::make_unique<lemlib::TrackingWheel>(verticalRotation.get(), VERTICAL_WHEEL_DIAMETER,
                                                                        VERTICAL_WHEEL_OFFSET);
        }
        if (!verticalWheel && !leftDriveWheel) {
                leftDriveWheel = std::make_unique<lemlib::TrackingWheel>(&left_motor_group, drivetrain.wheelDiameter,
                                                                         -drivetrain.trackWidth / 2, drivetrain.rpm);
                rightDriveWheel = std::make_unique<lemlib::TrackingWheel>(&right_motor_group, drivetrain.wheelDiameter,
                                                                          drivetrain.trackWidth / 2, drivetrain.rpm);
        }
}

/**
 * Distance from (px, py) along direction `headingDeg` to the first wall or
 * obstacle. Returns a negative number if the point is outside the field.
 */
float raycast(float px, float py, float headingDeg) {
        const float H = mcl::config.fieldHalfWidth;
        if (std::fabs(px) >= H || std::fabs(py) >= H) return -1;

        const float dx = std::sin(headingDeg * DEG2RAD);
        const float dy = std::cos(headingDeg * DEG2RAD);
        float best = 1e9f;

        // perimeter walls (we're inside the box, so take the exit distance)
        if (dx > 1e-6f) best = std::min(best, (H - px) / dx);
        else if (dx < -1e-6f) best = std::min(best, (-H - px) / dx);
        if (dy > 1e-6f) best = std::min(best, (H - py) / dy);
        else if (dy < -1e-6f) best = std::min(best, (-H - py) / dy);

        // obstacles (slab method, take the entry distance)
        for (const auto& b : mcl::config.obstacles) {
                float tmin = 0, tmax = 1e9f;
                bool hit = true;
                const float o[2] = {px, py}, d[2] = {dx, dy};
                const float lo[2] = {b.minX, b.minY}, hi[2] = {b.maxX, b.maxY};
                for (int i = 0; i < 2 && hit; i++) {
                        if (std::fabs(d[i]) < 1e-6f) {
                                if (o[i] < lo[i] || o[i] > hi[i]) hit = false;
                        } else {
                                float t1 = (lo[i] - o[i]) / d[i], t2 = (hi[i] - o[i]) / d[i];
                                if (t1 > t2) std::swap(t1, t2);
                                tmin = std::max(tmin, t1);
                                tmax = std::min(tmax, t2);
                                if (tmin > tmax) hit = false;
                        }
                }
                if (hit && tmin > 0) best = std::min(best, tmin);
        }
        return best;
}

void scatter(lemlib::Pose center, float sigma) {
        const int n = std::max(1, mcl::config.numParticles);
        particles.assign(n, Particle {center.x, center.y, 1.0f / n});
        for (auto& p : particles) {
                p.x += gauss(sigma);
                p.y += gauss(sigma);
        }
}

/** Move every particle by a field-frame delta (dx, dy) plus noise. */
void motionUpdate(float dx, float dy, float heading) {
        const float dist = std::hypot(dx, dy);
        if (dist < 1e-4f) return;
        const auto& c = mcl::config;
        const float sAlong = c.alongTrackNoise * dist + c.baseNoise;
        const float sCross = c.crossTrackNoise * dist + c.baseNoise;
        // robot forward / right unit vectors
        const float fx = std::sin(heading * DEG2RAD), fy = std::cos(heading * DEG2RAD);
        const float rx = fy, ry = -fx;
        for (auto& p : particles) {
                const float a = gauss(sAlong), r = gauss(sCross);
                p.x += dx + a * fx + r * rx;
                p.y += dy + a * fy + r * ry;
        }
}

/** Re-weight particles using the distance sensors. Returns false if no sensor gave a usable reading. */
bool sensorUpdate(float heading) {
        const auto& c = mcl::config;
        struct Reading {
                const mcl::DistanceSensorConfig* cfg;
                float z;
        };
        Reading readings[4];
        int count = 0;

        for (const auto& s : SENSORS) {
                const int mm = s.sensor->get_distance();
                if (mm <= 0 || mm >= 9999) continue; // 9999 = nothing seen, PROS_ERR = unplugged
                const float in = mm / 25.4f;
                if (in < c.minSensorRange || in > c.maxSensorRange) continue;
                if (mm > 200 && s.sensor->get_confidence() < c.minConfidence) continue;
                readings[count++] = {&s, in};
        }
        if (count == 0) return false;

        const float fx = std::sin(heading * DEG2RAD), fy = std::cos(heading * DEG2RAD);
        const float rx = fy, ry = -fx;
        const float uniform = c.outlierProb / c.maxSensorRange;

        std::vector<float> logW(particles.size());
        float maxLog = -1e30f;
        for (size_t i = 0; i < particles.size(); i++) {
                const Particle& p = particles[i];
                float lw = 0;
                for (int k = 0; k < count; k++) {
                        const auto* s = readings[k].cfg;
                        const float sx = p.x + s->forwardOffset * fx + s->rightOffset * rx;
                        const float sy = p.y + s->forwardOffset * fy + s->rightOffset * ry;
                        const float expected = raycast(sx, sy, heading + s->facing);
                        if (expected < 0) { // particle (or its sensor) is outside the field
                                lw += -50.0f;
                                continue;
                        }
                        const float sigma = c.sensorSigmaBase + c.sensorSigmaScale * expected;
                        const float err = (readings[k].z - expected) / sigma;
                        const float gaussian = std::exp(-0.5f * err * err) / (sigma * 2.5066283f);
                        lw += std::log((1.0f - c.outlierProb) * gaussian + uniform);
                }
                logW[i] = lw;
                maxLog = std::max(maxLog, lw);
        }

        float total = 0;
        for (size_t i = 0; i < particles.size(); i++) {
                particles[i].w *= std::exp(logW[i] - maxLog);
                total += particles[i].w;
        }
        if (total <= 0 || !std::isfinite(total)) {
                for (auto& p : particles) p.w = 1.0f / particles.size();
        } else {
                for (auto& p : particles) p.w /= total;
        }
        return true;
}

/** Low-variance (systematic) resampling, only when the weights have degenerated. */
void resampleIfNeeded() {
        const int n = particles.size();
        float sumSq = 0;
        for (const auto& p : particles) sumSq += p.w * p.w;
        const float nEff = sumSq > 0 ? 1.0f / sumSq : 0;
        if (nEff >= mcl::config.resampleThreshold * n) return;

        std::vector<Particle> next;
        next.reserve(n);
        std::uniform_real_distribution<float> u(0.0f, 1.0f / n);
        const float r = u(rng);
        float cumulative = particles[0].w;
        int i = 0;
        for (int m = 0; m < n; m++) {
                const float target = r + static_cast<float>(m) / n;
                while (target > cumulative && i < n - 1) cumulative += particles[++i].w;
                next.push_back({particles[i].x + gauss(mcl::config.roughening),
                                particles[i].y + gauss(mcl::config.roughening), 1.0f / n});
        }
        particles.swap(next);
}

void computeEstimate(float heading) {
        float mx = 0, my = 0;
        for (const auto& p : particles) {
                mx += p.w * p.x;
                my += p.w * p.y;
        }
        float var = 0;
        for (const auto& p : particles) var += p.w * ((p.x - mx) * (p.x - mx) + (p.y - my) * (p.y - my));
        stateMutex.take();
        estimate = lemlib::Pose(mx, my, heading);
        spread = std::sqrt(var);
        stateMutex.give();
}

// ---- motion sources: each returns a FIELD-frame delta and the current field heading ----

void substituteMotion(float& dx, float& dy, float& heading, float& dHeading) {
        const float vert = verticalDistance();
        const float horiz = horizontal_tracking_wheel.getDistanceTraveled();
        const float imuDeg = imu.get_rotation();

        const float dV = vert - prevVertical;
        const float dH = horiz - prevHorizontal;
        dHeading = imuDeg - prevImuHeading;
        prevVertical = vert;
        prevHorizontal = horiz;
        prevImuHeading = imuDeg;

        heading = imuDeg + imuToField;
        const float dTheta = dHeading * DEG2RAD;
        const float avgHeading = (heading - dHeading / 2) * DEG2RAD;

        // Same arc math LemLib uses, so the same wheel offsets/signs work for both
        float localX, localY;
        if (std::fabs(dTheta) < 1e-6f) {
                localX = dH;
                localY = dV;
        } else {
                const float chord = 2 * std::sin(dTheta / 2);
                localX = chord * (dH / dTheta + horizontal_tracking_wheel.getOffset());
                localY = chord * (dV / dTheta + verticalOffset());
        }
        dx = localY * std::sin(avgHeading) - localX * std::cos(avgHeading);
        dy = localY * std::cos(avgHeading) + localX * std::sin(avgHeading);
}

void chassisMotion(float& dx, float& dy, float& heading, float& dHeading) {
        const lemlib::Pose now = chassis.getPose();
        dx = now.x - prevChassis.x;
        dy = now.y - prevChassis.y;
        dHeading = now.theta - prevChassis.theta;
        prevChassis = now;
        rotateCW(dx, dy, frameRotation);
        heading = now.theta + frameRotation;
}

void tandemCorrect() {
        const auto& c = mcl::config;
        if (spread > c.maxCorrectionStd) return;
        const lemlib::Pose target = mcl::fieldToChassis(estimate);
        const lemlib::Pose now = chassis.getPose();
        const float ex = target.x - now.x, ey = target.y - now.y;
        if (std::hypot(ex, ey) < c.minCorrection) return;
        const float cx = ex * c.tandemGain, cy = ey * c.tandemGain;
        chassis.setPose(now.x + cx, now.y + cy, now.theta);
        // Don't let the motion model see our own correction as robot movement
        prevChassis.x += cx;
        prevChassis.y += cy;
}

void loop() {
        uint32_t lastSensorTime = pros::millis();
        float movedSinceSensor = 0;
        uint32_t now = pros::millis();

        while (running) {
                const auto& c = mcl::config;
                float dx, dy, heading, dHeading;
                if (mode == mcl::Mode::SUBSTITUTE) substituteMotion(dx, dy, heading, dHeading);
                else chassisMotion(dx, dy, heading, dHeading);

                motionUpdate(dx, dy, heading);
                movedSinceSensor += std::hypot(dx, dy);

                const float turnRate = std::fabs(dHeading) * 1000.0f / c.loopMs;
                const bool due = movedSinceSensor >= c.minMoveForUpdate ||
                                 pros::millis() - lastSensorTime >= static_cast<uint32_t>(c.stationaryUpdateMs);
                if (due && turnRate < c.maxTurnRate) {
                        if (sensorUpdate(heading)) resampleIfNeeded();
                        movedSinceSensor = 0;
                        lastSensorTime = pros::millis();
                }

                computeEstimate(heading);

                if (mode == mcl::Mode::SUBSTITUTE) {
                        chassis.setPose(mcl::fieldToChassis(estimate));
                } else if (mode == mcl::Mode::TANDEM) {
                        tandemCorrect();
                }

                pros::Task::delay_until(&now, c.loopMs);
        }
}

/** Re-sync the "previous" readings of whichever motion source is active. */
void syncMotionSource() {
        prevChassis = chassis.getPose();
        prevVertical = verticalDistance();
        prevHorizontal = horizontal_tracking_wheel.getDistanceTraveled();
        prevImuHeading = imu.get_rotation();
        // IMU heading -> field heading, based on where the filter thinks we're pointed
        imuToField = estimate.theta - prevImuHeading;
}

} // namespace

// =============================================================================
//                                PUBLIC API
// =============================================================================

namespace mcl {

Config config;

void start(Mode m, lemlib::Pose fieldStart, float initialSpread) {
        stop();
        initOdomSources();
        rng.seed(pros::micros());

        mode = m;
        chassisAnchor = chassis.getPose();
        fieldAnchor = fieldStart;
        frameRotation = fieldStart.theta - chassisAnchor.theta;

        scatter(fieldStart, initialSpread);
        estimate = fieldStart;
        spread = initialSpread;
        syncMotionSource();

        running = true;
        task = std::make_unique<pros::Task>(loop, "MCL");
}

void stop() {
        if (!running) return;
        running = false;
        pros::delay(config.loopMs * 2); // let the loop exit
        task.reset();
}

bool isRunning() { return running; }

void setMode(Mode m) {
        // The motion source changes, so pause the loop and re-baseline it
        const bool wasRunning = running;
        if (wasRunning) stop();
        mode = m;
        if (wasRunning) {
                syncMotionSource();
                running = true;
                task = std::make_unique<pros::Task>(loop, "MCL");
        }
}

Mode getMode() { return mode; }

void relocalize(lemlib::Pose fieldPose, float s) {
        // Pause the filter so we don't race the loop over the particle vector
        const bool wasRunning = running;
        const Mode m = mode;
        if (wasRunning) stop();
        scatter(fieldPose, s);
        estimate = fieldPose;
        spread = s;
        if (m == Mode::SUBSTITUTE) chassis.setPose(fieldToChassis(fieldPose));
        if (wasRunning) {
                syncMotionSource();
                running = true;
                task = std::make_unique<pros::Task>(loop, "MCL");
        }
}

lemlib::Pose getPose() {
        stateMutex.take();
        const lemlib::Pose p = estimate;
        stateMutex.give();
        return p;
}

lemlib::Pose getChassisPose() { return fieldToChassis(getPose()); }

float getSpread() {
        stateMutex.take();
        const float s = spread;
        stateMutex.give();
        return s;
}

lemlib::Pose chassisToField(lemlib::Pose p) {
        float dx = p.x - chassisAnchor.x, dy = p.y - chassisAnchor.y;
        rotateCW(dx, dy, frameRotation);
        return lemlib::Pose(fieldAnchor.x + dx, fieldAnchor.y + dy, p.theta + frameRotation);
}

lemlib::Pose fieldToChassis(lemlib::Pose p) {
        float dx = p.x - fieldAnchor.x, dy = p.y - fieldAnchor.y;
        rotateCW(dx, dy, -frameRotation);
        return lemlib::Pose(chassisAnchor.x + dx, chassisAnchor.y + dy, p.theta - frameRotation);
}

} // namespace mcl
