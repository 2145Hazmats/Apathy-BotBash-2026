// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {
  public FeederIO m_io;
  public FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  /** Creates a new Feeder. */
  public Feeder(FeederIO io) {
    m_io = io;
  }

  public void feed() {
    m_io.setFrontFeederOpenLoop(Constants.FeederConstants.feedingSpeedVolts);
  }

  public void regurg() {
    m_io.setFrontFeederOpenLoop(Constants.FeederConstants.feederRegurgSpeed);
  }

  public void stop() {
    m_io.setFrontFeederOpenLoop(0);
  }

  @Override
  public void periodic() {
    m_io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
  }
}
