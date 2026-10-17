// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

public interface FeederIO {
  @AutoLog
  public static class FeederIOInputs {
    public boolean feederConnected = false;
    public double frontFeederAppliedVoltage = 0.0;
    public double frontFeederCurrentAmps = 0.0;
    public double frontFeederPositionRad = 0.0;
    public double frontFeederVelocityRotPerSec = 0.0;

    public double backFeederAppliedVoltage = 0.0;
    public double backFeederCurrentAmps = 0.0;
    public double backFeederPositionRad = 0.0;
    public double backFeederVelocityRotPerSec = 0.0;
  }

  public default void updateInputs(FeederIOInputs inputs) {}

  public default void setFrontFeederOpenLoop(double output) {}

  public default void setBackFeederOpenLoop(double output) {}
}
