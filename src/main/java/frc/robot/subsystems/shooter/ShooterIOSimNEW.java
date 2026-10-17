// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants.ShooterSuperSystemConstants;
import frc.robot.Constants.SigmaMathUtils;

/** Add your docs here. */
public class ShooterIOSimNEW implements ShooterIO {
  private TalonFX shooterMotor;
  private TalonFXConfiguration shooterMotorConfig;
  private DCMotor motorSim = DCMotor.getKrakenX60(1);
  private FlywheelSim flywheelSim;

  private final LinearSystem<N1, N1, N1> m_flywheelPlant =
      LinearSystemId.createFlywheelSystem(
          motorSim,
          ShooterSuperSystemConstants.kFlywheelMomentOfInertia,
          ShooterSuperSystemConstants.kFlywheelGearing);

  public ShooterIOSimNEW() {
    shooterMotor = new TalonFX(ShooterSuperSystemConstants.FLYWHEEL_MOTOR_ID);
    flywheelSim = new FlywheelSim(m_flywheelPlant, motorSim, 0.01);

    shooterMotorConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(40));
    shooterMotor.getConfigurator().apply(shooterMotorConfig);

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 0.1;
    slot0Configs.kI = 0.0;
    slot0Configs.kD = 0.0;
    shooterMotor.getConfigurator().apply(slot0Configs);

    var shooterMotorSim = shooterMotor.getSimState();
    shooterMotorSim.setMotorType(TalonFXSimState.MotorType.KrakenX60);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    var shooterMotorSim = shooterMotor.getSimState();
    var shooterStatus = shooterMotorSim.getLastStatusCode();

    shooterMotorSim.setSupplyVoltage(RobotController.getBatteryVoltage());

    var simulatedMotorVoltage = shooterMotorSim.getMotorVoltageMeasure();

    flywheelSim.setInputVoltage(simulatedMotorVoltage.in(Volts));
    flywheelSim.update(0.02);

    shooterMotorSim.setRotorVelocity(
        flywheelSim
            .getAngularVelocity()
            .times(ShooterSuperSystemConstants.kFlywheelGearing)
            .in(RotationsPerSecond));

    inputs.shooterConnected = shooterStatus.isOK();
    inputs.shooterAppliedVoltage = shooterMotorSim.getMotorVoltage();
    inputs.shooterCurrentAmps = shooterMotorSim.getSupplyCurrent();
    inputs.shooterVelocityRotPerSec = SigmaMathUtils.RPMToRps(flywheelSim.getAngularVelocityRPM());
  }

  @Override
  public void setShooterVelocity(double velocity) {
    shooterMotor.setControl(new VelocityVoltage(0).withVelocity(velocity / 60));
  }
}
