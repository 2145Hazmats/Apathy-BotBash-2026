// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.beltFloor;

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
public class BeltFloorIOTalonFX implements BeltFloorIO {
  // Motors
  private TalonFX beltFloorMotor;
  // Configs
  private TalonFXConfiguration beltFloorMotorConfig;

  private final VoltageOut voltageRequest = new VoltageOut(0);

  private final Debouncer beltFloorConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  // Inputs from front feeder motor
  private final StatusSignal<Angle> beltFloorPosition;
  private final StatusSignal<AngularVelocity> beltFloorVelocity;
  private final StatusSignal<Voltage> beltFloorAppliedVolts;
  private final StatusSignal<Current> beltFloorCurrent;

  public BeltFloorIOTalonFX() {
    beltFloorMotor = new TalonFX(Constants.BeltFloorConstants.BELT_FLOOR_MOTOR_ID);
    beltFloorMotorConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(40));
    beltFloorMotor.getConfigurator().apply(beltFloorMotorConfig);

    // Create front feeder status signals
    beltFloorPosition = beltFloorMotor.getPosition();
    beltFloorVelocity = beltFloorMotor.getVelocity();
    beltFloorAppliedVolts = beltFloorMotor.getMotorVoltage();
    beltFloorCurrent = beltFloorMotor.getStatorCurrent();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, beltFloorVelocity, beltFloorAppliedVolts, beltFloorCurrent, beltFloorPosition);

    ParentDevice.optimizeBusUtilizationForAll(beltFloorMotor);
  }

  @Override
  public void updateInputs(BeltFloorIOInputs inputs) {
    // Refresh all signals
    var beltFloorStatus =
        BaseStatusSignal.refreshAll(
            beltFloorPosition, beltFloorVelocity, beltFloorAppliedVolts, beltFloorCurrent);

    inputs.beltFloorConnected = beltFloorConnectedDebounce.calculate(beltFloorStatus.isOK());

    inputs.beltFloorAppliedVoltage = beltFloorAppliedVolts.getValueAsDouble();
    inputs.beltFloorPositionRad = beltFloorPosition.getValueAsDouble();
    inputs.beltFloorCurrentAmps = beltFloorCurrent.getValueAsDouble();
    inputs.beltFloorVelocityRotPerSec = beltFloorVelocity.getValueAsDouble();
  }

  @Override
  public void setBeltFloorOpenLoop(double output) {
    beltFloorMotor.setControl(voltageRequest.withOutput(output));
  }
}
