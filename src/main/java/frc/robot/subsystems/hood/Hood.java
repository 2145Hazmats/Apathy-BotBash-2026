// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Hood extends SubsystemBase {
  public HoodIO m_io;
  public HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();

  /** Creates a new Feeder. */
  public Hood(HoodIO io) {
    m_io = io;
  }

  public void up() {
    m_io.setHoodPosClosedLoop(Constants.HoodConstants.HoodUpPos);
  }

  public void down() {
    m_io.setHoodPosClosedLoop(Constants.HoodConstants.HoodDownPos);
  }

  public void stop() {
    m_io.setHoodOpenLoop(0);
  }

  @Override
  public void periodic() {
    m_io.updateInputs(inputs);
    Logger.processInputs("Hood", inputs);
  }
}
