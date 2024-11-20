#include "../core/include/utils/math/geometry/translation2d.h"

/**
 * Class representing a rotation in 2d space.
 * Stores theta in radians, as well as cos and sin.
 *
 * Internally this angle is stored continuously,
 * however there are functions that return wrapped angles:
 * "180" is from [-pi, pi), [-180, 180), [-0.5, 0.5)
 * "360" is from [0, 2pi), [0, 360), [0, 1)
 */
class Rotation2d {
public:
  /**
   * Constructs a rotation with the given value in radians.
   *
   * @param radians the value of the rotation in radians.
   */
  Rotation2d(const double &radians);

  /**
   * Constructs a rotation given a translation.
   * Does not have to be normalized.
   * The angle from the x axis to the translation.
   *
   * [theta] = [atan2(y, x)]
   *
   * @param translation the translation
   */
  Rotation2d(const Translation2d &translation);

  /**
   * Constructs a rotation given x and y values.
   * Does not have to be normalized.
   * The angle from the x axis to the point.
   *
   * [theta] = [atan2(y, x)]
   *
   * @param x the x value of the point
   * @param y the y value of the point
   */
  Rotation2d(const double &x, const double &y);

  /**
   * Constructs a rotation given radian angle value.
   *
   * @param radians angle in radians.
   */
  Rotation2d from_radians(const double &radians);

  /**
   * Constructs a rotation given degree angle value.
   *
   * @param degrees angle in degrees.
   */
  Rotation2d from_degrees(const double &degrees);

  /**
   * Constructs a rotation given revolution angle value.
   *
   * @param revolutions angle in revolutions.
   */
  Rotation2d from_revolutions(const double &revolutions);

  /**
   * Returns the radian angle value.
   *
   * @return the radian angle value.
   */
  double radians();

  /**
   * Returns the degree angle value.
   *
   * @return the degree angle value.
   */
  double degrees();

  /**
   * Returns the revolution angle value.
   *
   * @return the revolution angle value.
   */
  double revolutions();

  /**
   * Returns the cosine of the angle value.
   *
   * @return the cosine of the angle value
   */
  double cos();

  /**
   * Returns the sine of the angle value.
   *
   * @return the sine of the angle value.
   */
  double sin();

  /**
   * Returns the tangent of the angle value.
   *
   * @return the tangent of the angle value.
   */
  double tan();

  /**
   * Returns the radian angle value, wrapped from [-pi, pi).
   *
   * @return the radian angle value, wrapped from [-pi, pi)
   */
  double wrapped_radians_180();

  /**
   * Returns the degree angle value, wrapped from [-180, 180).
   *
   * @return the degree angle value, wrapped from [-180, 180)
   */
  double wrapped_degrees_180();

  /**
   * Returns the revolution angle value, wrapped from [-0.5, 0.5).
   *
   * @return the revolution angle value, wrapped from [-0.5, 0.5)
   */
  double wrapped_revolutions_180();

  /**
   * Returns the radian angle value, wrapped from [0, 2pi).
   *
   * @return the radian angle value, wrapped from [0, 2pi)
   */
  double wrapped_radians_360();

  /**
   * Returns the degree angle value, wrapped from [0, 360).
   *
   * @return the degree angle value, wrapped from [0, 360)
   */
  double wrapped_degrees_360();

  /**
   * Returns the revolution angle value, wrapped from [0, 1).
   *
   * @return the revolution angle value, wrapped from [0, 1)
   */
  double wrapped_revolutions_360();

  /**
   * Adds the values of two rotations.
   *
   * @param other the other rotation to add to this rotation.
   *
   * @return the sum of the two rotations.
   */
  Rotation2d operator+(const Rotation2d &other);

  /**
   * Subtracts the values of two rotations.
   *
   * @param other the other rotation to subtract from this rotation.
   *
   * @return the difference between the two rotations.
   */
  Rotation2d operator-(const Rotation2d &other);

  /**
   * Takes the inverse of this rotation by flipping it.
   * Equivalent to adding 180 degrees.
   *
   * @return this inverse of the rotation.
   */
  Rotation2d operator-();

  /**
   * Multiplies this rotation by a scalar.
   *
   * @param scalar the scalar value to multiply the rotation by.
   *
   * @return the rotation multiplied by the scalar.
   */
  Rotation2d operator*(const double &scalar);

  /**
   * Divides this rotation by a scalar.
   *
   * @param scalar the scalar value to divide the rotation by.
   *
   * @return the rotation divided by the scalar.
   */
  Rotation2d operator/(const double &scalar);

  /**
   * Compares two rotations.
   * Returns true if their values are within 1e-9 radians of each other, to account for floating point error.
   *
   * @param other the other rotation to compare to
   *
   * @return whether the values of the rotations are within 1e-9 radians of each other
   */
  bool operator==(const Rotation2d &other);

  /**
   * Calculates the mean of a list of angle values directly.
   * !! DOES NOT WRAP INPUTS !!
   *
   * @param list std::vector containing a list of rotations.
   *
   * @return the single rotation mean of the list of rotations.
   */
  Rotation2d unwrapped_mean(const std::vector<Rotation2d> &list);

  /**
   * Calculates the mean of a list of angle values directly.
   * !! WRAPS INPUTS !!
   *
   * @param list std::vector containing a list of rotations.
   *
   * @return the single rotation mean of the list of rotations.
   */
  Rotation2d wrapped_mean(const std::vector<Rotation2d> &list);

  /**
   * Wraps a radian angle value from [-pi, pi).
   *
   * @param angle the radian angle value to wrap.
   *
   * @return the wrapped radian angle value from [-pi, pi).
   */
  static double wrap_radians_180(const double &angle);

  /**
   * Wraps a degree angle value from [-180, 180).
   *
   * @param angle the degree angle value to wrap.
   *
   * @return the wrapped degree angle value from [-180, 180).
   */
  static double wrap_degrees_180(const double &angle);

  /**
   * Wraps a revolution angle vlue from [-0.5, 0.5).
   *
   * @param angle the revolution angle value to wrap.
   *
   * @return the wrapped revolution angle vlue from [-0.5, 0.5).
   */
  static double wrap_revolutions_180(const double &angle);

  /**
   * Wraps a radian angle value from [0, 2pi).
   *
   * @param angle the radian angle value to wrap.
   *
   * @return the wrapped radian angle value from [0, 2pi).
   */
  static double wrap_radians_360(const double &angle);

  /**
   * Wraps a degree angle value from [0, 360).
   *
   * @param angle the degree angle value to wrap.
   *
   * @return the wrapped degree angle value from [0, 360).
   */
  static double wrap_degrees_360(const double &angle);

  /**
   * Wraps a revolution angle value from [0, 1).
   *
   * @param angle the revolution angle value to wrap.
   *
   * @return the wrapped revolution angle value from [0, 1).
   */
  static double wrap_revolutions_360(const double &angle);

private:
  double m_radians;
  double m_cos;
  double m_sin;
};
