#include "core.h"
#include "vex.h"

#include "wallstake_mech.h"
#include "../core/include/utils/controls/pid.h"

WallStakeMech::WallStakeMech(
  const vex::motor_group &motors, const vex::pot &pot, const Rotation2d &tolerance, const Rotation2d &setpoint,
  const double &pot_offset
)
    : motors(motors), pot(pot), tolerance(tolerance), setpoint(setpoint), pot_offset(pot_offset) { handle = new vex::task(background_task, (void *)this); }

Rotation2d WallStakeMech::get_angle() { return (from_degrees(1.1 * pot.angle(vex::deg) - pot_offset)); }

void WallStakeMech::set_setpoint(const Rotation2d &new_setpoint) { setpoint = new_setpoint; }

bool WallStakeMech::is_at_setpoint() { 
    Rotation2d current_angle = get_angle();
    return (current_angle.wrapped_degrees_180() <= (setpoint + tolerance).wrapped_degrees_180()) && (current_angle.wrapped_degrees_180() >= (setpoint - tolerance).wrapped_degrees_180());
 }

// bool WallStakeMech::is_at_state() {}

void WallStakeMech::update() {
    double kg = 0.9;
    double ffout = kg * (setpoint - from_degrees(2)).f_cos();

    double kp = 0.075;
    double pout = kp * (setpoint.degrees() - get_angle().degrees());
    set_voltage(ffout + pout);

    printf("%f\n", get_angle().degrees());
}

void WallStakeMech::set_voltage(const double &voltage) { motors.spin(vex::fwd, voltage, vex::volt); }

/**
 * Function that runs in the background task. This function pointer is passed
 * to the vex::task constructor.
 *
 * @param ptr Pointer to OdometryBase object
 * @return Required integer return code. Unused.
 */
int WallStakeMech::background_task(void *ptr) {
  WallStakeMech &obj = *((WallStakeMech *)ptr);
  vexDelay(1000);
  while (!obj.end_task) {
    obj.update();
    vexDelay(5);
  }

  return 0;
}

/**
 * End the background task. Cannot be restarted.
 * If the user wants to end the thread but keep the data up to date,
 * they must run the update() function manually from then on.
 */
void WallStakeMech::end_async() { this->end_task = true; }



// void WallStakeMech::set_state(const WallStakeState ) {}
