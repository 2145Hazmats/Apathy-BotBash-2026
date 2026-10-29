// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants.ShooterSuperSystemConstants;
import frc.robot.Constants.SigmaMathUtils;

public class ShooterIOTalonFX implements ShooterIO {
  // Motors
  private TalonFX shooterLeader;
  private TalonFX shooterFollower;
  // Configs
  private TalonFXConfiguration shooterLeaderMotorConfig;
  private TalonFXConfiguration shooterFollowerMotorConfig;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  private final Debouncer shooterConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  // Inputs from drive motors
  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Voltage> leaderAppliedVolts;
  private final StatusSignal<Current> leaderCurrent;

  private final StatusSignal<AngularVelocity> followerVelocity;
  private final StatusSignal<Voltage> followerAppliedVolts;
  private final StatusSignal<Current> followerCurrent;

  public ShooterIOTalonFX() {
    shooterLeader = new TalonFX(ShooterSuperSystemConstants.FLYWHEEL_LEADER_MOTOR_ID);
    shooterLeaderMotorConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(40));
    shooterLeader.getConfigurator().apply(shooterLeaderMotorConfig);

    shooterFollower = new TalonFX(ShooterSuperSystemConstants.FLYWHEEL_FOLLOWER_MOTOR_ID);
    shooterFollowerMotorConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(40));
    shooterFollower.getConfigurator().apply(shooterFollowerMotorConfig);

    // Create feeder status signals
    leaderVelocity = shooterLeader.getVelocity();
    leaderAppliedVolts = shooterLeader.getMotorVoltage();
    leaderCurrent = shooterLeader.getStatorCurrent();

    followerVelocity = shooterFollower.getVelocity();
    followerAppliedVolts = shooterFollower.getMotorVoltage();
    followerCurrent = shooterFollower.getStatorCurrent();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leaderVelocity,
        leaderAppliedVolts,
        leaderCurrent,
        followerVelocity,
        followerAppliedVolts,
        followerCurrent);

    ParentDevice.optimizeBusUtilizationForAll(shooterLeader, shooterFollower);
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // Refresh all signals
    var shooterStatus =
        BaseStatusSignal.refreshAll(
            leaderVelocity,
            leaderAppliedVolts,
            leaderCurrent,
            followerVelocity,
            followerAppliedVolts,
            followerCurrent);

    inputs.shooterLeaderConnected = shooterConnectedDebounce.calculate(shooterStatus.isOK());

    inputs.shooterLeaderAppliedVoltage = leaderAppliedVolts.getValueAsDouble();
    inputs.shooterLeaderCurrentAmps = leaderCurrent.getValueAsDouble();
    inputs.shooterLeaderVelocityRotPerSec = leaderVelocity.getValueAsDouble();

    inputs.shooterFollowerAppliedVoltage = followerAppliedVolts.getValueAsDouble();
    inputs.shooterFollowerCurrentAmps = followerCurrent.getValueAsDouble();
    inputs.shooterFollowerVelocityRotPerSec = followerVelocity.getValueAsDouble();
  }

  @Override
  public void setShooterVelocity(double output) {
    shooterLeader.setControl(velocityVoltageRequest.withVelocity(SigmaMathUtils.RPMToRps(output)));
    shooterFollower.setControl(
        new Follower(
            ShooterSuperSystemConstants.FLYWHEEL_LEADER_MOTOR_ID, MotorAlignmentValue.Opposed));
  }

  @Override
  public void setShooterOpenLoop(double output) {
    shooterLeader.setControl(voltageRequest.withOutput(output));
    shooterFollower.setControl(
        new Follower(
            ShooterSuperSystemConstants.FLYWHEEL_LEADER_MOTOR_ID, MotorAlignmentValue.Opposed));
  }
}
