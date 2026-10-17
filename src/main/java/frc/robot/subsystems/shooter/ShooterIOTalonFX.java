// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;
import frc.robot.Constants.SigmaMathUtils;

public class ShooterIOTalonFX implements ShooterIO {
  // Motors
  private TalonFX shooterMotor;
  // Configs
  private TalonFXConfiguration shooterMotorConfig;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  private final Debouncer shooterConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  // Inputs from drive motor
  private final StatusSignal<AngularVelocity> shooterVelocity;
  private final StatusSignal<Voltage> shooterAppliedVolts;
  private final StatusSignal<Current> shooterCurrent;

  public ShooterIOTalonFX() {
    shooterMotor = new TalonFX(Constants.ShooterSuperSystemConstants.FLYWHEEL_MOTOR_ID);
    shooterMotorConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(40));
    shooterMotor.getConfigurator().apply(shooterMotorConfig);
    // Create feeder status signals
    shooterVelocity = shooterMotor.getVelocity();
    shooterAppliedVolts = shooterMotor.getMotorVoltage();
    shooterCurrent = shooterMotor.getStatorCurrent();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, shooterVelocity, shooterAppliedVolts, shooterCurrent);

    ParentDevice.optimizeBusUtilizationForAll(shooterMotor);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Refresh all signals
    var shooterStatus =
        BaseStatusSignal.refreshAll(shooterVelocity, shooterAppliedVolts, shooterCurrent);

    inputs.shooterConnected = shooterConnectedDebounce.calculate(shooterStatus.isOK());
    inputs.shooterAppliedVoltage = shooterAppliedVolts.getValueAsDouble();
    inputs.shooterCurrentAmps = shooterCurrent.getValueAsDouble();
    inputs.shooterVelocityRotPerSec = shooterVelocity.getValueAsDouble();
  }

  @Override
  public void setShooterVelocity(double output) {
    shooterMotor.setControl(velocityVoltageRequest.withVelocity(SigmaMathUtils.RPMToRps(output)));
  }

  @Override
  public void setShooterOpenLoop(double output) {
    shooterMotor.setControl(voltageRequest.withOutput(output));
  }
}
