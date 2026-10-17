// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  public IntakeIO m_io;
  public IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  /** Creates a new Feeder. */
  public Intake(IntakeIO io) {
    m_io = io;
  }

  public void setIntakeHOME() {
    m_io.setIntakeArmPosition(Constants.IntakeConstants.intakePositionHOME);
  }

  public void setIntakeDOWN() {
    m_io.setIntakeArmPosition(Constants.IntakeConstants.intakePositionDOWN);
  }

  public void intakeRollers() {
    m_io.setIntakeRollerOpenLoop(Constants.IntakeConstants.intakingSpeed);
  }

  public void stopRoller() {
    m_io.setIntakeRollerOpenLoop(0);
  }

  public void stopArm() {
    m_io.setIntakeArmOpenLoop(0);
  }

  @Override
  public void periodic() {
    m_io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }
}
