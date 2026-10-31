// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ShooterSuperSystemConstants;
import frc.robot.commands.DriveCommands;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.HubSubsystem;
import frc.robot.subsystems.auger.Auger;
import frc.robot.subsystems.auger.AugerIO;
import frc.robot.subsystems.auger.AugerIOTalonFX;
import frc.robot.subsystems.beltFloor.BeltFloor;
import frc.robot.subsystems.beltFloor.BeltFloorIO;
import frc.robot.subsystems.beltFloor.BeltFloorIOTalonFX;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIO;
import frc.robot.subsystems.feeder.FeederIOTalonFX;
import frc.robot.subsystems.hood.Hood;
import frc.robot.subsystems.hood.HoodIO;
import frc.robot.subsystems.hood.HoodIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSimNEW;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive m_drive;
  private final Shooter m_shooter;
  private final Feeder m_feeder;
  private final Intake m_intake;
  private final Auger m_auger;
  private final BeltFloor m_BeltFloor;
  private final Hood m_hood;
  private final HubSubsystem hub = new HubSubsystem(this);
  // private final Vision vision;

  // Controller
  private final CommandXboxController P1Controller = new CommandXboxController(0);
  private final CommandXboxController P2Controller = new CommandXboxController(1);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
        // a CANcoder
        m_drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                new ModuleIOTalonFX(TunerConstants.FrontRight),
                new ModuleIOTalonFX(TunerConstants.BackLeft),
                new ModuleIOTalonFX(TunerConstants.BackRight));
        m_shooter = new Shooter(new ShooterIOTalonFX());
        m_feeder = new Feeder(new FeederIOTalonFX());
        m_intake = new Intake(new IntakeIOTalonFX());
        m_auger = new Auger(new AugerIOTalonFX());
        m_BeltFloor = new BeltFloor(new BeltFloorIOTalonFX());
        m_hood = new Hood(new HoodIOTalonFX());

        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOLimelight(VisionConstants.camera0Name, drive::getRotation),
        //         new VisionIOLimelight(VisionConstants.camera1Name, drive::getRotation));

        // The ModuleIOTalonFXS implementation provides an example implementation for
        // TalonFXS controller connected to a CANdi with a PWM encoder. The
        // implementations
        // of ModuleIOTalonFX, ModuleIOTalonFXS, and ModuleIOSpark (from the Spark
        // swerve
        // template) can be freely intermixed to support alternative hardware
        // arrangements.
        // Please see the AdvantageKit template documentation for more information:
        // https://docs.advantagekit.org/getting-started/template-projects/talonfx-swerve-template#custom-module-implementations
        //
        // drive =
        // new Drive(
        // new GyroIOPigeon2(),
        // new ModuleIOTalonFXS(TunerConstants.FrontLeft),
        // new ModuleIOTalonFXS(TunerConstants.FrontRight),
        // new ModuleIOTalonFXS(TunerConstants.BackLeft),
        // new ModuleIOTalonFXS(TunerConstants.BackRight));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        m_drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(TunerConstants.FrontLeft),
                new ModuleIOSim(TunerConstants.FrontRight),
                new ModuleIOSim(TunerConstants.BackLeft),
                new ModuleIOSim(TunerConstants.BackRight));
        m_shooter = new Shooter(new ShooterIOSimNEW() {});
        m_feeder = new Feeder(new FeederIO() {});
        m_intake = new Intake(new IntakeIO() {});
        m_auger = new Auger(new AugerIO() {});
        m_BeltFloor = new BeltFloor(new BeltFloorIO() {});
        m_hood = new Hood(new HoodIO() {});

        // vision =
        //     new Vision(
        //         drive::addVisionMeasurement,
        //         new VisionIOPhotonVisionSim(
        //             VisionConstants.camera0Name, VisionConstants.robotToCamera0, drive::getPose),
        //         new VisionIOPhotonVisionSim(
        //             VisionConstants.camera1Name, VisionConstants.robotToCamera1,
        // drive::getPose));
        break;

      default:
        // Replayed robot, disable IO implementations
        m_drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        m_shooter = new Shooter(new ShooterIO() {});
        m_feeder = new Feeder(new FeederIO() {});
        m_intake = new Intake(new IntakeIO() {});
        m_auger = new Auger(new AugerIO() {});
        m_BeltFloor = new BeltFloor(new BeltFloorIO() {});
        m_hood = new Hood(new HoodIO() {});

        // vision = new Vision(drive::addVisionMeasurement, new VisionIO() {}, new VisionIO() {});
        break;
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());
    NamedCommands.registerCommand("Intake DOWN", Commands.run(() -> m_intake.setIntakeDOWN()));
    NamedCommands.registerCommand("Intake MIDDLE", Commands.run(() -> m_intake.setIntakeMIDDLE()));
    NamedCommands.registerCommand("Intake HOME", Commands.run(() -> m_intake.setIntakeHOME()));
    NamedCommands.registerCommand("Shoot", Commands.run(() -> ShootRoutine()));

    // Set up SysId routines
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(m_drive));
    autoChooser.addOption(
        "Drive Simple FF Characterization", DriveCommands.feedforwardCharacterization(m_drive));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Forward)",
        m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Quasistatic Reverse)",
        m_drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    autoChooser.addOption(
        "Drive SysId (Dynamic Forward)", m_drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
    autoChooser.addOption(
        "Drive SysId (Dynamic Reverse)", m_drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Default Commands
    // -----------------------------------------------------------------------------------------------------------
    m_drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            m_drive,
            () -> -P1Controller.getLeftY(),
            () -> -P1Controller.getLeftX(),
            () -> -P1Controller.getRightX()));

    m_shooter.setDefaultCommand(Commands.run(() -> m_shooter.stop(), m_shooter));
    m_feeder.setDefaultCommand(Commands.run(() -> m_feeder.stop(), m_feeder));
    m_intake.setDefaultCommand(Commands.run(() -> m_intake.stopRoller(), m_intake));
    m_auger.setDefaultCommand(Commands.run(() -> m_auger.stop(), m_auger));
    m_BeltFloor.setDefaultCommand(Commands.run(() -> m_BeltFloor.stop(), m_BeltFloor));
    m_hood.setDefaultCommand(Commands.run(() -> m_hood.home(), m_hood));

    // Driver Controls
    // ------------------------------------------------------------------------------------------------------------

    // Slow Mode
    P1Controller.rightBumper().whileTrue(
        DriveCommands.joystickDrive(
            m_drive,
            () -> (-P1Controller.getLeftY() * 0.5),
            () -> (-P1Controller.getLeftX() * 0.5),
            () -> (-P1Controller.getRightX() * 0.75)));

    
    // Lock to 0° when A button is held
    P1Controller.a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                m_drive,
                () -> -P1Controller.getLeftY(),
                () -> -P1Controller.getLeftX(),
                () -> Rotation2d.kZero));

    // Switch to X pattern when X button is pressed
    P1Controller.x().onTrue(Commands.runOnce(m_drive::stopWithX, m_drive));

    // Reset gyro to 0° when B button is pressed
    P1Controller.b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        m_drive.setPose(
                            new Pose2d(m_drive.getPose().getTranslation(), Rotation2d.kZero)),
                    m_drive)
                .ignoringDisable(true));


    // Operator Controls
    // ----------------------------------------------------------------------------------------------------------

    // Intake
    P2Controller.povDown()
        .whileTrue(Commands.run(() -> m_intake.intakeRollers()))
        .whileFalse(Commands.run(() -> m_intake.stopRoller()));

    // Rev Up Flywheel
    P2Controller.rightBumper()
        .whileTrue(Commands.run(() -> m_shooter.shoot(7)))
        .whileFalse(Commands.run(() -> m_shooter.shoot(0)));

    // Shoot
    P2Controller.rightTrigger()
        .whileTrue(
            Commands.run(
                () -> {
                  m_auger.run();
                  m_BeltFloor.feed();
                  m_feeder.feed();
                }));

    // Intake
    P2Controller.povDown().whileTrue(Commands.run(() -> m_intake.setIntakeDOWN()));
    P2Controller.povUp().whileTrue(Commands.run(() -> m_intake.setIntakeHOME()));
 
    P2Controller.leftBumper()
        .whileTrue(
            Commands.run(
                () -> {
                  m_auger.AugerReverse();
                  m_BeltFloor.BeltFeedReverse();
                  m_feeder.regurg();
                }));

    // Auger
    // P2Controller.rightBumper().whileTrue(Commands.run(() -> m_auger.AugerRun()));
    // P2Controller.leftBumper().whileTrue(Commands.run(() -> m_auger.AugerReverse()));

    // BeltFloor
    // P2Controller.rightBumper().whileTrue(Commands.run(() -> m_BeltFloor.BeltFeed()));
    // P2Controller.leftBumper().whileTrue(Commands.run(() -> m_BeltFloor.BeltFeedReverse()));

    // Hood
    P2Controller.leftTrigger().whileTrue(Commands.run(() -> m_hood.up()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public Command ShootRoutine() {
    return Commands.run(
            () -> {
              m_feeder.regurg();
              m_shooter.shoot(ShooterSuperSystemConstants.flyWheelShootingSpeed);
            })
        .withTimeout(ShooterSuperSystemConstants.spinUpSeconds)
        .andThen(
            Commands.run(
                    () -> {
                      m_feeder.feed();
                      m_shooter.shoot(ShooterSuperSystemConstants.flyWheelShootingSpeed);
                      m_auger.run();
                      m_BeltFloor.feed();
                    },
                    m_feeder,
                    m_shooter,
                    m_auger,
                    m_BeltFloor)
                .finallyDo(
                    () -> {
                      m_feeder.stop();
                      m_auger.stop();
                      m_BeltFloor.stop();
                      m_shooter.stop();
                    }));
  }

  public void setControllerRumbles(double value) {
    P1Controller.setRumble(RumbleType.kBothRumble, value);
    P2Controller.setRumble(RumbleType.kBothRumble, value);
  }
}
