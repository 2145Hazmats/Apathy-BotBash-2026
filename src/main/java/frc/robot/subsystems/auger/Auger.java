// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.auger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Auger extends SubsystemBase {
  public AugerIO m_io;
  public AugerIOInputsAutoLogged inputs = new AugerIOInputsAutoLogged();

  public Auger(AugerIO io) {
    m_io = io;
  }

  public void AugerRun() {
    m_io.setAugerOpenLoop(Constants.AugerConstants.AugerSpeed);
  }

  public void stopAuger() {
    m_io.setAugerOpenLoop(0);
  }

  @Override
  public void periodic() {
    m_io.updateInputs(inputs);
    Logger.processInputs("Auger", inputs);
  }
}
