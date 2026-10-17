// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.RobotBase;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class FeederConstants {
    public static final int FRONT_FEEDER_MOTOR_ID = 21;
    public static final int BACK_FEEDER_MOTOR_ID = 22;
    public static final double feedingSpeedVolts = -12 * 0.6;
    public static final double feederRegurgSpeed = 0.7;
  }

  public static class HoodConstants {
    public static final int HOOD_MOTOR_ID = 69;
    public static final double HoodUpPos = 67;
    public static final double HoodDownPos = -67;
  }

  public static class IntakeConstants {
    public static final int INTAKE_ROLLER_MOTOR_ID = 31;
    public static final int INTAKE_ARM_MOTOR_ID = 30;
    public static final double intakePositionHOME = 0;
    public static final double intakePositionDOWN = -45;
    public static final double intakingSpeed = -8;
  }

  public static class ShooterSuperSystemConstants {
    public static final int FLYWHEEL_MOTOR_ID = 29;
    public static final double flyWheelShootingSpeed = 2000;
    public static final double kFlywheelMomentOfInertia = 0.0015;
    public static final double kFlywheelGearing = 1.0;
    public static final double flyWheelShootingSpeedMultiplier = 0.6;
  }

  public static class BeltFloorConstants {
    public static final int BELT_FLOOR_MOTOR_ID = 67420;
    public static final double beltFeedSpeed = 0.5;
    public static final double beltRegurgSpeed = -0.5;
  }

  public static class AugerConstants {
    public static final int AUGER_MOTOR_ID = 67420;
    public static final double AugerSpeed = 0.5;
  }

  public static class SigmaMathUtils {

    public static double RPMToRps(double Rpm) {
      return Rpm / 60;
    }

    public static double RPMToRadPerSec(double Rpm) {
      return RPMToRps(Rpm) / Math.PI * 2;
    }
  }
}
