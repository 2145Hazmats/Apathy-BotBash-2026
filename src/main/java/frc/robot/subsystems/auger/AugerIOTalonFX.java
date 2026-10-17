// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.auger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

/** Add your docs here. */
public class AugerIOTalonFX implements AugerIO {
  // Motors
  private TalonFX augerMotor;
  // Configs
  private TalonFXConfiguration augerMotorConfig;

  private final VoltageOut voltageRequest = new VoltageOut(0);

  private final Debouncer augerConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  private final StatusSignal<Angle> augerPosition;
  private final StatusSignal<AngularVelocity> augerVelocity;
  private final StatusSignal<Voltage> augerAppliedVolts;
  private final StatusSignal<Current> augerCurrent;

  public AugerIOTalonFX() {
    augerMotor = new TalonFX(Constants.AugerConstants.AUGER_MOTOR_ID);
    augerMotorConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(40));
    augerMotor.getConfigurator().apply(augerMotorConfig);

    augerPosition = augerMotor.getPosition();
    augerVelocity = augerMotor.getVelocity();
    augerAppliedVolts = augerMotor.getMotorVoltage();
    augerCurrent = augerMotor.getStatorCurrent();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, augerVelocity, augerAppliedVolts, augerCurrent, augerPosition);

    ParentDevice.optimizeBusUtilizationForAll(augerMotor);
  }

  @Override
  public void updateInputs(AugerIOInputs inputs) {
    // Refresh all signals
    var augerStatus =
        BaseStatusSignal.refreshAll(augerPosition, augerVelocity, augerAppliedVolts, augerCurrent);

    inputs.augerConnected = augerConnectedDebounce.calculate(augerStatus.isOK());

    inputs.augerAppliedVoltage = augerAppliedVolts.getValueAsDouble();
    inputs.augerPositionRad = augerPosition.getValueAsDouble();
    inputs.augerCurrentAmps = augerCurrent.getValueAsDouble();
    inputs.augerVelocityRotPerSec = augerVelocity.getValueAsDouble();
  }

  @Override
  public void setAugerOpenLoop(double output) {
    augerMotor.setControl(voltageRequest.withOutput(output));
  }
}
