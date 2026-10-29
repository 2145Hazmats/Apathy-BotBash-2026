// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public boolean shooterLeaderConnected = false;
    public double shooterLeaderAppliedVoltage = 0.0;
    public double shooterLeaderCurrentAmps = 0.0;
    public double shooterLeaderVelocityRotPerSec = 0.0;
    public double shooterFollowerAppliedVoltage = 0.0;
    public double shooterFollowerCurrentAmps = 0.0;
    public double shooterFollowerVelocityRotPerSec = 0.0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}

  public default void setShooterVelocity(double velocity) {}

  public default void setShooterOpenLoop(double output) {}
}
