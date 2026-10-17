// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  public ShooterIO m_io;
  public ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  // private final LinearSystem<N1, N1, N1> m_flywheelPlant;

  /** Creates a new Shooter. */
  public Shooter(ShooterIO io) {
    m_io = io;
  }

  public void shoot(float RPM) {
    m_io.setShooterVelocity(RPM);
  }

  public void stop() {
    m_io.setShooterOpenLoop(0);
  }

  @Override
  public void periodic() {
    m_io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }
}
