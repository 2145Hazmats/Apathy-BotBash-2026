// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.auger;

import org.littletonrobotics.junction.AutoLog;

public interface AugerIO {
  @AutoLog
  public static class AugerIOInputs {
    public boolean augerConnected = false;
    public double augerAppliedVoltage = 0.0;
    public double augerCurrentAmps = 0.0;
    public double augerPositionRad = 0.0;
    public double augerVelocityRotPerSec = 0.0;
  }

  public default void updateInputs(AugerIOInputs inputs) {}

  public default void setAugerOpenLoop(double output) {}
}
