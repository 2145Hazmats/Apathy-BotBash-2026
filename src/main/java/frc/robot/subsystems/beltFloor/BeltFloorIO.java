// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.beltFloor;

import org.littletonrobotics.junction.AutoLog;

public interface BeltFloorIO {
  @AutoLog
  public static class BeltFloorIOInputs {
    public boolean beltFloorConnected = false;
    public double beltFloorAppliedVoltage = 0.0;
    public double beltFloorCurrentAmps = 0.0;
    public double beltFloorPositionRad = 0.0;
    public double beltFloorVelocityRotPerSec = 0.0;
  }

  public default void updateInputs(BeltFloorIOInputs inputs) {}

  public default void setBeltFloorOpenLoop(double output) {}
}
