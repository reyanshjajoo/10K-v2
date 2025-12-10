#include "monte.hpp"
#include "liblvgl/lvgl.h"
#include "main.h"
#include "pros/rtos.hpp"
#include "subsystems.hpp"
#include <cmath>
#include <iostream>
#include <random>

// Use EZ-Template drive
// extern ez::Drive chassis;  // Usually declared in main.h

// Constants
const int PARTICLE_QUANTITY = 7500; // tune

namespace
{

  // Global vector of particles; we'll resize it during initialization.
  std::vector<Particle> particles;
  std::random_device rd;
  std::mt19937 gen(rd()); // Mersenne Twister random number generator
  std::normal_distribution<float> noise_x(0.0f,
                                          0.1f); // Mean 0, std dev 0.1 inches
  std::normal_distribution<float> noise_y(0.0f,
                                          0.1f);           // Mean 0, std dev 0.1 inches
  std::normal_distribution<float> noise_theta(0.0f, 5.0f); // Std dev 5.0 degrees

  // Track the last odometry pose for odometry delta calculations
  Pose lastOdomPose(0, 0, 0);

  // Task control
  pros::Task *mclTaskHandle = nullptr;
  bool mclRunning = false;
  ez::Drive *chassisPtr = nullptr;

  // Task timing constants
  const int MCL_DELAY = 20;              // Run at 50Hz
  const float FIELD_DIMENSIONS = 144.0f; // 144 inch square field

  // Sensor offsets relative to tracking center (if you ever want them)
  const float NORTH_SENSOR_X_OFFSET = 0.0f;
  const float NORTH_SENSOR_Y_OFFSET = 0.0f;

  const float SOUTH_SENSOR_X_OFFSET = 0.0f;
  const float SOUTH_SENSOR_Y_OFFSET = 0.0f;

  const float EAST_SENSOR_X_OFFSET = 0.0f;
  const float EAST_SENSOR_Y_OFFSET = 0.0f;

  const float WEST_SENSOR_X_OFFSET = 0.0f;
  const float WEST_SENSOR_Y_OFFSET = 0.0f;

  // Flags to choose which sensors to use
  bool useNorthSensor = false;
  bool useSouthSensor = false;
  bool useEastSensor = false;
  bool useWestSensor = true;

  // Distance dependent noise parameters (currently simplified away)
  const float sigma_close_range = 0.3f;
  const float sigma_far_range = 1.0f;
  const float DISTANCE_THRESHOLD_INCHES = 7.87f; // 200mm in inches

  // Update frequency control
  int lastUpdateTime = 0;           // millis of last MCL update
  const int UPDATE_INTERVAL = 1000; // 1 second between updates
  Pose lastUpdatedPose(0, 0, 0);
  const float MIN_MOTION_THRESHOLD = 0.25f; // inches

  // Motion noise threshold
  const float MOTION_NOISE_THRESHOLD = 0.25f; // inches

  const float UNIFORM_WEIGHT_FACTOR = 0.0001f;
  const float MIN_WEIGHT = 0.1f;
  const int RESAMPLING_INTERVAL = 3;

  // Previous sensor readings
  float prev_north_dist = -1.0f;
  float prev_south_dist = -1.0f;
  float prev_east_dist = -1.0f;
  float prev_west_dist = -1.0f;

  const float DISTANCE_CHANGE_THRESHOLD = 0.25f; // inches

  // Filtered pose and blending constants
  Pose filteredPose(0, 0, 0);
  const float FILTER_ALPHA = 0.3f;          // 30 percent new, 70 percent old
  const float ODOMETRY_TRUST_FACTOR = 0.5f; // 0.5 = equal trust in odom and MCL

  // Helper to read EZ-Template odometry into our Pose struct
  inline Pose getOdomPoseFromChassis(ez::Drive &chassis)
  {
    return Pose(
        static_cast<float>(chassis.odom_x_get()),
        static_cast<float>(chassis.odom_y_get()),
        static_cast<float>(chassis.odom_theta_get()));
  }

  // Helper to write pose back into EZ-Template odometry
  inline void setOdomPoseOnChassis(ez::Drive &chassis, const Pose &p)
  {
    chassis.odom_xyt_set(p.x, p.y, p.theta);
  }

} // namespace

// Initialize particles around an initial pose estimate
void initializeParticles(const Pose &initialPose)
{
  particles.resize(PARTICLE_QUANTITY);
  lastOdomPose = initialPose;

  std::normal_distribution<float> x_dist(initialPose.x, 1.0);
  std::normal_distribution<float> y_dist(initialPose.y, 1.0);
  std::normal_distribution<float> theta_dist(initialPose.theta, 0.0);

  for (auto &particle : particles)
  {
    particle = Particle(
        Pose(x_dist(gen), y_dist(gen), theta_dist(gen)),
        1.0f / PARTICLE_QUANTITY);
  }
}

// Update particles based on robot motion (prediction step)
void motionUpdate(const Pose &localOdomDelta)
{
  float motion_magnitude = std::sqrt(localOdomDelta.x * localOdomDelta.x +
                                     localOdomDelta.y * localOdomDelta.y);

  const float MIN_MOTION_FOR_NOISE = 0.5f;
  bool add_noise = motion_magnitude > MIN_MOTION_FOR_NOISE;

  std::normal_distribution<float> motion_noise(0.0, 0.03);
  std::normal_distribution<float> rotation_noise(0.0, 0.05);

  for (auto &particle : particles)
  {
    float theta_rad = particle.pose.theta * M_PI / 180.0f;

    float dx_global =
        localOdomDelta.x * std::cos(theta_rad) -
        localOdomDelta.y * std::sin(theta_rad);
    float dy_global =
        localOdomDelta.x * std::sin(theta_rad) +
        localOdomDelta.y * std::cos(theta_rad);

    float noise_scale =
        add_noise ? std::min(1.0f, motion_magnitude / 2.0f) : 0.0f;

    particle.pose.x += dx_global + noise_scale * motion_noise(gen);
    particle.pose.y += dy_global + noise_scale * motion_noise(gen);
    particle.pose.theta += localOdomDelta.theta +
                           noise_scale * rotation_noise(gen);

    // Normalize theta to 0..360
    particle.pose.theta = std::fmod(particle.pose.theta, 360.0f);
    if (particle.pose.theta < 0)
      particle.pose.theta += 360.0f;
  }
}

// Calculate expected sensor reading for a particle in a given direction
float predictSensorReading(const Pose &particlePose, const char direction)
{
  float half_dimension = FIELD_DIMENSIONS / 2.0f; // 72 inches
  float sensor_x = particlePose.x;
  float sensor_y = particlePose.y;
  float theta = particlePose.theta * M_PI / 180.0f;

  // FOV of sensor
  const float FOV_HALF_ANGLE = 12.0f * M_PI / 180.0f;

  float sensor_angle = 0.0f;
  switch (direction)
  {
  case 'N':
    sensor_angle = 0.0f;
    break;
  case 'S':
    sensor_angle = M_PI;
    break;
  case 'E':
    sensor_angle = M_PI / 2.0f;
    break;
  case 'W':
    sensor_angle = 3.0f * M_PI / 2.0f;
    break;
  default:
    return -1.0f;
  }

  float center_angle = sensor_angle + theta;
  float left_angle = center_angle - FOV_HALF_ANGLE;
  float right_angle = center_angle + FOV_HALF_ANGLE;

  float distances[4];

  // North boundary (y = 72)
  float t_north_left = (half_dimension - sensor_y) / std::sin(left_angle);
  float t_north_right = (half_dimension - sensor_y) / std::sin(right_angle);
  distances[0] = (t_north_left > 0 && std::isfinite(t_north_left))
                     ? t_north_left
                     : std::numeric_limits<float>::max();
  distances[0] = std::min(
      distances[0],
      (t_north_right > 0 && std::isfinite(t_north_right))
          ? t_north_right
          : std::numeric_limits<float>::max());

  // South boundary (y = -72)
  float t_south_left = (-half_dimension - sensor_y) / std::sin(left_angle);
  float t_south_right = (-half_dimension - sensor_y) / std::sin(right_angle);
  distances[1] = (t_south_left > 0 && std::isfinite(t_south_left))
                     ? t_south_left
                     : std::numeric_limits<float>::max();
  distances[1] = std::min(
      distances[1],
      (t_south_right > 0 && std::isfinite(t_south_right))
          ? t_south_right
          : std::numeric_limits<float>::max());

  // East boundary (x = 72)
  float t_east_left = (half_dimension - sensor_x) / std::cos(left_angle);
  float t_east_right = (half_dimension - sensor_x) / std::cos(right_angle);
  distances[2] = (t_east_left > 0 && std::isfinite(t_east_left))
                     ? t_east_left
                     : std::numeric_limits<float>::max();
  distances[2] = std::min(
      distances[2],
      (t_east_right > 0 && std::isfinite(t_east_right))
          ? t_east_right
          : std::numeric_limits<float>::max());

  // West boundary (x = -72)
  float t_west_left = (-half_dimension - sensor_x) / std::cos(left_angle);
  float t_west_right = (-half_dimension - sensor_x) / std::cos(right_angle);
  distances[3] = (t_west_left > 0 && std::isfinite(t_west_left))
                     ? t_west_left
                     : std::numeric_limits<float>::max();
  distances[3] = std::min(
      distances[3],
      (t_west_right > 0 && std::isfinite(t_west_right))
          ? t_west_right
          : std::numeric_limits<float>::max());

  float min_distance = std::numeric_limits<float>::max();
  for (int i = 0; i < 4; ++i)
  {
    if (distances[i] > 0 && distances[i] < min_distance)
    {
      min_distance = distances[i];
    }
  }

  if (min_distance == std::numeric_limits<float>::max())
  {
    return 72.0f;
  }

  return min_distance;
}

// Update particle weights based on sensor measurements
void measurementUpdate(float north_dist, float south_dist,
                       float east_dist, float west_dist)
{
  bool significant_change = false;
  if (west_dist >= 0 && prev_west_dist >= 0 &&
      std::fabs(west_dist - prev_west_dist) > DISTANCE_CHANGE_THRESHOLD)
  {
    significant_change = true;
  }
  if (!significant_change)
    return;

  float total_weight = 0.0f;

  for (auto &particle : particles)
  {
    float particle_weight = 1.0f;
    int valid_readings = 0;

    auto getSigma = [&](float predicted_distance)
    {
      // Right now you just use a wide sigma because FOV is fat
      return 10.0f;
    };

    if (west_dist >= 0)
    {
      float predicted_west_dist = predictSensorReading(particle.pose, 'W');
      float west_diff = std::fabs(predicted_west_dist - west_dist);
      float sigma = getSigma(predicted_west_dist);
      float west_likelihood =
          std::exp(-(west_diff * west_diff) / (2.0f * sigma * sigma));
      particle_weight *= west_likelihood;
      valid_readings++;
    }

    if (valid_readings > 0)
    {
      particle.weight = std::max(particle_weight, 0.1f);
    }
    else
    {
      particle.weight = 0.1f;
    }
    total_weight += particle.weight;
  }

  if (total_weight > 0)
  {
    for (auto &particle : particles)
    {
      particle.weight /= total_weight;
    }
  }

  prev_west_dist = west_dist;
}

// Systematic resampling
std::vector<Particle> weightedResample(const std::vector<Particle> &particles)
{
  std::vector<Particle> new_particles(PARTICLE_QUANTITY);
  std::uniform_real_distribution<float> dist(0.0f, 1.0f);

  std::vector<float> cumulative_weights(PARTICLE_QUANTITY);
  cumulative_weights[0] = particles[0].weight;
  for (size_t i = 1; i < PARTICLE_QUANTITY; ++i)
  {
    cumulative_weights[i] =
        cumulative_weights[i - 1] + particles[i].weight;
  }

  float step = 1.0f / PARTICLE_QUANTITY;
  float r = dist(gen) * step;
  size_t index = 0;

  for (size_t m = 0; m < PARTICLE_QUANTITY; ++m)
  {
    float U = r + m * step;
    while (index < PARTICLE_QUANTITY - 1 && U > cumulative_weights[index])
    {
      ++index;
    }
    new_particles[m] = particles[index];
    new_particles[m].weight = 1.0f / PARTICLE_QUANTITY;
  }

  return new_particles;
}

// Resample with a bit of noise to avoid impoverishment
void resampleParticles()
{
  std::vector<Particle> new_particles = weightedResample(particles);

  std::normal_distribution<float> n_x(0.0f, 0.1f);
  std::normal_distribution<float> n_y(0.0f, 0.1f);
  std::normal_distribution<float> n_theta(0.0f, 0.05f);

  for (auto &particle : new_particles)
  {
    particle.pose.x += n_x(gen);
    particle.pose.y += n_y(gen);
    particle.pose.theta += n_theta(gen);

    particle.pose.theta = std::fmod(particle.pose.theta, 360.0f);
    if (particle.pose.theta < 0)
      particle.pose.theta += 360.0f;
  }

  particles = new_particles;
}

// Get best estimate of pose from particles, with filtering and odom blending
Pose getEstimatedPose()
{
  Pose rawEstimated(0, 0, 0);
  float total_weight = 0.0f;

  for (const auto &particle : particles)
  {
    rawEstimated.x += particle.weight * particle.pose.x;
    rawEstimated.y += particle.weight * particle.pose.y;
    rawEstimated.theta += particle.weight * particle.pose.theta;
    total_weight += particle.weight;
  }

  if (total_weight > 0)
  {
    rawEstimated.x /= total_weight;
    rawEstimated.y /= total_weight;
    rawEstimated.theta /= total_weight;
  }

  // Low pass on x y
  filteredPose.x = FILTER_ALPHA * rawEstimated.x +
                   (1 - FILTER_ALPHA) * filteredPose.x;
  filteredPose.y = FILTER_ALPHA * rawEstimated.y +
                   (1 - FILTER_ALPHA) * filteredPose.y;

  // Low pass on theta with wrap
  float theta_diff = rawEstimated.theta - filteredPose.theta;
  if (theta_diff > 180)
    theta_diff -= 360;
  if (theta_diff < -180)
    theta_diff += 360;
  filteredPose.theta += FILTER_ALPHA * theta_diff;

  while (filteredPose.theta > 360)
    filteredPose.theta -= 360;
  while (filteredPose.theta < 0)
    filteredPose.theta += 360;

  // Blend with last odometry pose
  Pose blendedPose(0, 0, 0);
  blendedPose.x = ODOMETRY_TRUST_FACTOR * lastOdomPose.x +
                  (1 - ODOMETRY_TRUST_FACTOR) * filteredPose.x;
  blendedPose.y = ODOMETRY_TRUST_FACTOR * lastOdomPose.y +
                  (1 - ODOMETRY_TRUST_FACTOR) * filteredPose.y;

  theta_diff = filteredPose.theta - lastOdomPose.theta;
  if (theta_diff > 180)
    theta_diff -= 360;
  if (theta_diff < -180)
    theta_diff += 360;
  blendedPose.theta =
      lastOdomPose.theta + (1 - ODOMETRY_TRUST_FACTOR) * theta_diff;

  return blendedPose;
}

// Motion delta between current and last odom pose
Pose calculateMotionDelta(const Pose &currentOdomPose)
{
  Pose delta(
      currentOdomPose.x - lastOdomPose.x,
      currentOdomPose.y - lastOdomPose.y,
      currentOdomPose.theta - lastOdomPose.theta);

  lastOdomPose = currentOdomPose;

  printf("Motion Delta: (%.4f, %.4f, %.4f)\n", delta.x, delta.y, delta.theta);
  return delta;
}

// Optional one shot update function if you still want it
void updateMCL(ez::Drive &chassis,
               float north_dist, float south_dist,
               float east_dist, float west_dist)
{
  Pose currentOdomPose = getOdomPoseFromChassis(chassis);
  Pose motionDelta = calculateMotionDelta(currentOdomPose);

  float motion_magnitude =
      std::sqrt(motionDelta.x * motionDelta.x +
                motionDelta.y * motionDelta.y);

  if (motion_magnitude > MOTION_NOISE_THRESHOLD)
  {
    motionUpdate(motionDelta);
  }

  measurementUpdate(north_dist, south_dist, east_dist, west_dist);
  resampleParticles();

  Pose estimatedPose = getEstimatedPose();
  setOdomPoseOnChassis(chassis, estimatedPose);
}

// Background task for MCL
void mclTask(void *param)
{
  if (!chassisPtr)
    return;

  // Initialize particles around current odom pose
  Pose startPose = getOdomPoseFromChassis(*chassisPtr);
  initializeParticles(startPose);
  lastOdomPose = startPose;
  lastUpdateTime = pros::millis();

  int resampleCounter = 0;

  while (mclRunning)
  {
    // Read sensors and convert to inches
    float north = useNorthSensor ? dNorth.get() / 25.4f : -1;
    float south = useSouthSensor ? dNorthW.get() / 25.4f : -1;
    float east = useEastSensor ? dEast.get() / 25.4f : -1;
    float west = useWestSensor ? dWest.get() / 25.4f : -1;

    int north_conf = dNorth.get_confidence();
    int south_conf = dNorthW.get_confidence();
    int east_conf = dEast.get_confidence();
    int west_conf = dWest.get_confidence();

    int north_size = dNorth.get_object_size();
    int south_size = dNorthW.get_object_size();
    int east_size = dEast.get_object_size();
    int west_size = dWest.get_object_size();

    const int MIN_CONFIDENCE = 45;
    const int MIN_OBJECT_SIZE = 50;
    const int MAX_OBJECT_SIZE = 401;

    if (north_conf < MIN_CONFIDENCE || north_size < MIN_OBJECT_SIZE ||
        north_size > MAX_OBJECT_SIZE || north >= 9999 || north > 210)
      north = -1;
    if (south_conf < MIN_CONFIDENCE || south_size < MIN_OBJECT_SIZE ||
        south_size > MAX_OBJECT_SIZE || south >= 9999 || south > 210)
      south = -1;
    if (east_conf < MIN_CONFIDENCE || east_size < MIN_OBJECT_SIZE ||
        east_size > MAX_OBJECT_SIZE || east >= 9999 || east > 210)
      east = -1;
    if (west_conf < MIN_CONFIDENCE || west_size < MIN_OBJECT_SIZE ||
        west_size > MAX_OBJECT_SIZE || west >= 9999 || west > 210)
      west = -1;

    if (!useNorthSensor)
      north = -1;
    if (!useSouthSensor)
      south = -1;
    if (!useEastSensor)
      east = -1;
    if (!useWestSensor)
      west = -1;

    int currentTime = pros::millis();

    if (currentTime - lastUpdateTime >= UPDATE_INTERVAL)
    {
      Pose currentOdomPose = getOdomPoseFromChassis(*chassisPtr);
      Pose motionDelta = calculateMotionDelta(currentOdomPose);

      if (std::sqrt(motionDelta.x * motionDelta.x +
                    motionDelta.y * motionDelta.y) > MOTION_NOISE_THRESHOLD)
      {
        motionUpdate(motionDelta);
      }

      measurementUpdate(north, south, east, west);

      resampleCounter++;
      if (resampleCounter >= RESAMPLING_INTERVAL)
      {
        resampleParticles();
        resampleCounter = 0;
      }

      Pose estimatedPose = getEstimatedPose();

      printf("Estimated: (%.2f, %.2f, %.2f) | Motion: (%.4f, %.4f, %.4f)\n",
             estimatedPose.x, estimatedPose.y, estimatedPose.theta,
             motionDelta.x, motionDelta.y, motionDelta.theta);

      setOdomPoseOnChassis(*chassisPtr, estimatedPose);

      lastUpdateTime = currentTime;
    }

    pros::delay(MCL_DELAY);
  }
}

void startMCL(ez::Drive &chassis)
{
  if (mclTaskHandle != nullptr)
  {
    stopMCL();
  }

  chassisPtr = &chassis;
  mclRunning = true;
  mclTaskHandle = new pros::Task(mclTask, nullptr, "MCL Task");
}

void stopMCL()
{
  if (mclTaskHandle != nullptr)
  {
    mclRunning = false;
    pros::delay(MCL_DELAY * 2);
    delete mclTaskHandle;
    mclTaskHandle = nullptr;
    chassisPtr = nullptr;
  }
}
