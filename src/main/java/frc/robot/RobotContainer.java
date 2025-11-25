// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
// import edu.wpi.first.units.Units;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SwerveSubsystem drivebase = new SwerveSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  // Real world settings
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                              () -> -m_driverController.getLeftY(),
                                              () -> -m_driverController.getLeftX())
                                          .withControllerRotationAxis(() -> -m_driverController.getRightX())
                                          .deadband(OperatorConstants.DEADBAND)
                                          .scaleTranslation(0.8)
                                          .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy()
                                          .withControllerHeadingAxis(m_driverController::getRightX,
                                                                     m_driverController::getRightY)
                                          .headingWhile(true);

  // Simulation
  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                              () -> -m_driverController.getLeftY(),
                                              () -> -m_driverController.getLeftX())
                                          .withControllerRotationAxis(() -> m_driverController.getRawAxis(2))
                                          .deadband(OperatorConstants.DEADBAND)
                                          .scaleTranslation(0.8)
                                          .allianceRelativeControl(true);


  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
                                          .withControllerHeadingAxis(() ->  Math.sin(m_driverController.getRawAxis(2) *
                                                                            Math.PI) * (Math.PI * 2),
                                                                     () ->  Math.cos(m_driverController.getRawAxis(2) *
                                                                            Math.PI) * (Math.PI * 2))
                                          .headingWhile(true)
                                          .translationHeadingOffset(true)
                                          .translationHeadingOffset(Rotation2d.fromDegrees(0));

  // Simulation with joystick
  SwerveInputStream driveAngularVelocitySim = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                          () -> m_driverController.getLeftY(),
                                          () -> m_driverController.getLeftX())
                                          .withControllerRotationAxis(() -> -m_driverController.getRawAxis(2))
                                          .deadband(OperatorConstants.DEADBAND)
                                          .scaleTranslation(0.8)
                                          .allianceRelativeControl(true);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    Command driveFieldOrientedDirectAngle     = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedAngularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    // Simulation
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveFieldOrientedAngularVelocitySim       = drivebase.driveFieldOriented(driveAngularVelocitySim);
    
    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
      // drivebase.setDefaultCommand(driveFieldOrientedAngularVelocitySim);
      // drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocityKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity);
    }

    if (Robot.isSimulation())
    {
      Pose2d target = new Pose2d(new Translation2d(1, 4),
                                 Rotation2d.fromDegrees(90));
      //drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(() -> target,
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5, 2)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(Units.degreesToRadians(360),
                                                                                     Units.degreesToRadians(180))
                                           ));
      m_driverController.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      // m_driverController.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      m_driverController.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                     () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

//      m_driverController.b().whileTrue(
//          drivebase.driveToPose(
//              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
//                              );

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAngularVelocity); // Overrides drive command above!

      m_driverController.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      m_driverController.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      m_driverController.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      m_driverController.back().whileTrue(drivebase.centerModulesCommand());
      m_driverController.leftBumper().onTrue(Commands.none());
      m_driverController.rightBumper().onTrue(Commands.none());
    } else
    {
      m_driverController.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      m_driverController.x().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      m_driverController.start().whileTrue(Commands.none());
      m_driverController.back().whileTrue(Commands.none());
      m_driverController.leftBumper().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      m_driverController.rightBumper().onTrue(Commands.none());
    }

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
