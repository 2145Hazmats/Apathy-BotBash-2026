// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.feeder;

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
public class FeederIOTalonFX implements FeederIO {
  // Motors
  private TalonFX frontFeederMotor;
  //private TalonFX backFeederMotor;
  // Configs
  private TalonFXConfiguration frontFeederMotorConfig;
  //private TalonFXConfiguration backFeederMotorConfig;

  private final VoltageOut voltageRequest = new VoltageOut(0);

  private final Debouncer feederConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  // Inputs from front feeder motor
  private final StatusSignal<Angle> frontFeederPosition;
  private final StatusSignal<AngularVelocity> frontFeederVelocity;
  private final StatusSignal<Voltage> frontFeederAppliedVolts;
  private final StatusSignal<Current> frontFeederCurrent;

  // Inputs from back feeder motor
  //private final StatusSignal<Angle> backFeederPosition;
  //private final StatusSignal<AngularVelocity> backFeederVelocity;
  //private final StatusSignal<Voltage> backFeederAppliedVolts;
  //private final StatusSignal<Current> backFeederCurrent;

  public FeederIOTalonFX() {
    frontFeederMotor = new TalonFX(Constants.FeederConstants.FRONT_FEEDER_MOTOR_ID);
    frontFeederMotorConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(40));
    frontFeederMotor.getConfigurator().apply(frontFeederMotorConfig);

    //backFeederMotor = new TalonFX(Constants.FeederConstants.BACK_FEEDER_MOTOR_ID);
    //backFeederMotorConfig =
    //    new TalonFXConfiguration()
    //        .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(40));
    //backFeederMotor.getConfigurator().apply(backFeederMotorConfig);

    // Create front feeder status signals
    frontFeederPosition = frontFeederMotor.getPosition();
    frontFeederVelocity = frontFeederMotor.getVelocity();
    frontFeederAppliedVolts = frontFeederMotor.getMotorVoltage();
    frontFeederCurrent = frontFeederMotor.getStatorCurrent();

    // Create back feeder status signals
    //backFeederPosition = backFeederMotor.getPosition();
    //backFeederVelocity = backFeederMotor.getVelocity();
    //backFeederAppliedVolts = backFeederMotor.getMotorVoltage();
    //backFeederCurrent = backFeederMotor.getStatorCurrent();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        frontFeederVelocity,
        frontFeederAppliedVolts,
        frontFeederCurrent,
        frontFeederPosition);
        //backFeederVelocity,
        //backFeederAppliedVolts,
        //backFeederCurrent,
        //backFeederPosition);

    ParentDevice.optimizeBusUtilizationForAll(frontFeederMotor);
    //ParentDevice.optimizeBusUtilizationForAll(frontFeederMotor, backFeederMotor);
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    // Refresh all signals
    var feederStatus =
        BaseStatusSignal.refreshAll(
            frontFeederPosition,
            frontFeederVelocity,
            frontFeederAppliedVolts,
            frontFeederCurrent);
            //backFeederPosition,
            //backFeederVelocity,
            //backFeederAppliedVolts,
            //backFeederCurrent);

    inputs.feederConnected = feederConnectedDebounce.calculate(feederStatus.isOK());

    inputs.frontFeederAppliedVoltage = frontFeederAppliedVolts.getValueAsDouble();
    inputs.frontFeederPositionRad = frontFeederPosition.getValueAsDouble();
    inputs.frontFeederCurrentAmps = frontFeederCurrent.getValueAsDouble();
    inputs.frontFeederVelocityRotPerSec = frontFeederVelocity.getValueAsDouble();

    //inputs.backFeederAppliedVoltage = frontFeederAppliedVolts.getValueAsDouble();
    //inputs.backFeederPositionRad = frontFeederPosition.getValueAsDouble();
    //inputs.backFeederCurrentAmps = frontFeederCurrent.getValueAsDouble();
    //inputs.backFeederVelocityRotPerSec = frontFeederVelocity.getValueAsDouble();
  }

  @Override
  public void setFrontFeederOpenLoop(double output) {
    frontFeederMotor.setControl(voltageRequest.withOutput(output));
  }

  //@Override
  //public void setBackFeederOpenLoop(double output) {
  //  backFeederMotor.setControl(voltageRequest.withOutput(output));
  //}
}
