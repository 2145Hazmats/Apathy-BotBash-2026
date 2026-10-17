// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.LinearQuadraticRegulator;
import edu.wpi.first.math.estimator.KalmanFilter;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.LinearSystemLoop;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.Constants.SigmaMathUtils;
import org.littletonrobotics.junction.Logger;

/** Add your docs here. */
public class ShooterIOSim implements ShooterIO {
  // Motors
  private TalonFX shooterMotor;
  // Configs
  private TalonFXConfiguration shooterMotorConfig;

  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  private FlywheelSim flywheel;
  private final LinearSystem<N1, N1, N1> m_flywheelPlant =
      LinearSystemId.createFlywheelSystem(
          DCMotor.getKrakenX60(2),
          Constants.ShooterSuperSystemConstants.kFlywheelMomentOfInertia,
          Constants.ShooterSuperSystemConstants.kFlywheelGearing);

  // The observer fuses our encoder data and voltage inputs to reject noise.
  private final KalmanFilter<N1, N1, N1> m_observer =
      new KalmanFilter<>(
          Nat.N1(),
          Nat.N1(),
          m_flywheelPlant,
          VecBuilder.fill(3.0), // How accurate we think our model is
          VecBuilder.fill(0.01), // How accurate we think our encoder
          // data is
          0.020);

  // A LQR uses feedback to create voltage commands.
  private final LinearQuadraticRegulator<N1, N1, N1> m_controller =
      new LinearQuadraticRegulator<>(
          m_flywheelPlant,
          VecBuilder.fill(8.0), // qelms. Velocity error tolerance, in radians per second. Decrease
          // this to more heavily penalize state excursion, or make the controller behave more
          // aggressively.
          VecBuilder.fill(12.0), // relms. Control effort (voltage) tolerance. Decrease this to more
          // heavily penalize control effort, or make the controller less aggressive. 12 is a good
          // starting point because that is the (approximate) maximum voltage of a battery.
          0.020); // Nominal time between loops. 0.020 for TimedRobot, but can be
  // lower if using notifiers.

  // The state-space loop combines a controller, observer, feedforward and plant for easy control.
  private final LinearSystemLoop<N1, N1, N1> m_loop =
      new LinearSystemLoop<>(m_flywheelPlant, m_controller, m_observer, 12.0, 0.020);

  private final Debouncer shooterConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public ShooterIOSim() {
    shooterMotor = new TalonFX(Constants.ShooterSuperSystemConstants.FLYWHEEL_MOTOR_ID);
    shooterMotorConfig =
        new TalonFXConfiguration()
            .withCurrentLimits(new CurrentLimitsConfigs().withSupplyCurrentLimit(40));
    shooterMotor.getConfigurator().apply(shooterMotorConfig);

    flywheel = new FlywheelSim(m_flywheelPlant, DCMotor.getKrakenX60(2));

    m_loop.reset(VecBuilder.fill(SigmaMathUtils.RPMToRadPerSec(flywheel.getAngularVelocityRPM())));
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    var simState = shooterMotor.getSimState();
    // Refresh all signals
    var shooterStatus = simState.getLastStatusCode();

    inputs.shooterConnected = shooterConnectedDebounce.calculate(shooterStatus.isOK());
    inputs.shooterAppliedVoltage = simState.getMotorVoltage();
    inputs.shooterCurrentAmps = simState.getSupplyCurrent();
    inputs.shooterVelocityRotPerSec = SigmaMathUtils.RPMToRps(flywheel.getAngularVelocityRPM());
  }

  @Override
  public void setShooterVelocity(double velocity) {
    m_loop.setNextR(VecBuilder.fill(SigmaMathUtils.RPMToRadPerSec(velocity)));
    m_loop.correct(
        VecBuilder.fill(SigmaMathUtils.RPMToRadPerSec(flywheel.getAngularVelocityRPM())));
    m_loop.predict(0.02);

    double nextVoltage = m_loop.getU(0);
    flywheel.setInputVoltage(nextVoltage);

    // SmartDashboard.putNumber("Flywheel Velocity", nextVoltage);
    Logger.recordOutput("Flywheel Velocity", flywheel.getAngularVelocityRPM());
  }

  @Override
  public void setShooterOpenLoop(double output) {
    flywheel.setInputVoltage(output);
  }
}
