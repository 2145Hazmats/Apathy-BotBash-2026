// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.beltFloor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class BeltFloor extends SubsystemBase {
  public BeltFloorIO m_io;
  public BeltFloorIOInputsAutoLogged inputs = new BeltFloorIOInputsAutoLogged();

  /** Creates a new Feeder. */
  public BeltFloor(BeltFloorIO io) {
    m_io = io;
  }

  public void BeltFeed() {
    m_io.setBeltFloorOpenLoop(Constants.BeltFloorConstants.beltFeedSpeed);
  }

  public void BeltRegurg() {
    m_io.setBeltFloorOpenLoop(Constants.BeltFloorConstants.beltRegurgSpeed);
  }

  public void stopBeltFloor() {
    m_io.setBeltFloorOpenLoop(0);
  }

  @Override
  public void periodic() {
    m_io.updateInputs(inputs);
    Logger.processInputs("BeltFloor", inputs);
  }
}
