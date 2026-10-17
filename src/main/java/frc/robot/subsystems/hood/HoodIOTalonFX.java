// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.hood;

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
public class HoodIOTalonFX implements HoodIO {
  // Motors
  private TalonFX HoodMotor;
  // Configs
  private TalonFXConfiguration HoodMotorConfig;

  private final VoltageOut voltageRequest = new VoltageOut(0);

  private final Debouncer hoodConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  // Inputs from front hood motor
  private final StatusSignal<Angle> HoodPositionRad;
  private final StatusSignal<AngularVelocity> HoodVelocityRotPerSec;
  private final StatusSignal<Voltage> HoodAppliedVolts;
  private final StatusSignal<Current> HoodCurrentAmps;

  public HoodIOTalonFX() {
    HoodMotor = new TalonFX(Constants.HoodConstants.HOOD_MOTOR_ID);
    HoodMotorConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(40));
    HoodMotor.getConfigurator().apply(HoodMotorConfig);

    // Create front hood status signals
    HoodPositionRad = HoodMotor.getPosition();
    HoodVelocityRotPerSec = HoodMotor.getVelocity();
    HoodAppliedVolts = HoodMotor.getMotorVoltage();
    HoodCurrentAmps = HoodMotor.getStatorCurrent();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, HoodVelocityRotPerSec, HoodAppliedVolts, HoodCurrentAmps, HoodPositionRad);

    ParentDevice.optimizeBusUtilizationForAll(HoodMotor);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    // Refresh all signals
    var feederStatus =
        BaseStatusSignal.refreshAll(
            HoodPositionRad, HoodVelocityRotPerSec, HoodAppliedVolts, HoodCurrentAmps);

    inputs.hoodConnected = hoodConnectedDebounce.calculate(feederStatus.isOK());

    inputs.hoodAppliedVoltage = HoodAppliedVolts.getValueAsDouble();
    inputs.hoodPositionRad = HoodPositionRad.getValueAsDouble();
    inputs.hoodCurrentAmps = HoodCurrentAmps.getValueAsDouble();
    inputs.hoodVelocityRotPerSec = HoodVelocityRotPerSec.getValueAsDouble();
  }

  @Override
  public void setHoodOpenLoop(double output) {
    HoodMotor.setControl(voltageRequest.withOutput(output));
  }

  @Override
  public void setHoodPosClosedLoop(double output) {
    HoodMotor.setControl(voltageRequest.withOutput(output));
  }
}
