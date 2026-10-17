// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public boolean hoodConnected = false;
    public double hoodAppliedVoltage = 0.0;
    public double hoodCurrentAmps = 0.0;
    public double hoodPositionRad = 0.0;
    public double hoodVelocityRotPerSec = 0.0;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void setHoodOpenLoop(double output) {}

  public default void setHoodPosClosedLoop(double position) {}
}
