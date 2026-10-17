// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class IntakeIOTalonFX implements IntakeIO {
  // Motors
  private TalonFX intakeArmMotor;
  private TalonFX intakeRollerMotor;

  // Configs
  private TalonFXConfiguration intakeArmMotorConfig;
  private TalonFXConfiguration intakeRollerMotorConfig;

  // Voltage Control Requests
  private final VoltageOut rollerVoltageRequest = new VoltageOut(0);
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);

  public IntakeIOTalonFX() {
    intakeArmMotor = new TalonFX(Constants.IntakeConstants.INTAKE_ARM_MOTOR_ID);
    intakeArmMotorConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(40));
    intakeArmMotor.getConfigurator().apply(intakeArmMotorConfig);
    intakeRollerMotor = new TalonFX(Constants.IntakeConstants.INTAKE_ROLLER_MOTOR_ID);
    intakeRollerMotorConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(40));
    intakeRollerMotor.getConfigurator().apply(intakeRollerMotorConfig);
    var slot0Configs = new Slot0Configs();
    slot0Configs.kG = 0.4;
    slot0Configs.kP = 0.2;
    intakeArmMotor.getConfigurator().apply(slot0Configs);

    resetIntakePosition();
  }

  public void resetIntakePosition() {
    intakeArmMotor.setPosition(0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.intakeArmAppliedVoltage = intakeArmMotor.getMotorVoltage().getValueAsDouble();
    inputs.intakeRollerAppliedVoltage = intakeRollerMotor.getMotorVoltage().getValueAsDouble();
    Logger.recordOutput("ArmPostition", intakeArmMotor.getPosition().getValueAsDouble());
  }

  @Override
  public void setIntakeRollerOpenLoop(double output) {
    intakeRollerMotor.setControl(rollerVoltageRequest.withOutput(output));
  }

  @Override
  public void setIntakeArmPosition(double position) {
    intakeArmMotor.setControl(positionVoltageRequest.withPosition(position));
  }
}
