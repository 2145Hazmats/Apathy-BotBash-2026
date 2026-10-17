// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public boolean intakeRollerConnected = false;
    public double intakeRollerAppliedVoltage = 0.0;
    public double intakeRollerCurrentAmps = 0.0;
    public double intakeRollerPositionRad = 0.0;
    public double intakeRollerVelocityRotPerSec = 0.0;

    public boolean intakeArmConnected = false;
    public double intakeArmAppliedVoltage = 0.0;
    public double intakeArmCurrentAmps = 0.0;
    public double intakeArmPositionRad = 0.0;
    public double intakeArmVelocityRotPerSec = 0.0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setIntakeRollerOpenLoop(double output) {}

  public default void setIntakeArmPosition(double position) {}

  public default void setIntakeArmOpenLoop(double output) {}
}
