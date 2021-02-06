// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.Timer;
//import static edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SparkMaxSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final SparkMaxSubsystem m_robotDrive = new SparkMaxSubsystem();

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Integer> m_chooser = new SendableChooser<Integer>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    //configureButtonBindings();

    // Setup SmartDashboard options
    Timer.delay(1.);
    m_chooser.addOption("Auto Routine S", Integer.valueOf(1));
    m_chooser.setDefaultOption("Auto Routine Straight", Integer.valueOf(2));
    m_chooser.addOption("Auto Bounce Challenge", Integer.valueOf(3));
    m_chooser.addOption("Auto 2 Straights", Integer.valueOf(4));
    SmartDashboard.putData(m_chooser);
    SmartDashboard.updateValues();

    // Configure default commands
    // Set the default drive command to split-stick arcade drive
    m_robotDrive.setDefaultCommand(
        // A split-stick arcade command, with forward/backward controlled by the left
        // hand, and turning controlled by the right. NO!  NOT ANYMORE!
        new RunCommand(
            () ->
                m_robotDrive.arcadeDrive(
                    -m_driverController.getY(Hand.kLeft),
                    m_driverController.getX(Hand.kLeft)),
            m_robotDrive));
  }

//   /**
//    * Use this method to define your button->command mappings. Buttons can be created by
//    * instantiating a {@link GenericHID} or one of its subclasses ({@link
//    * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
//    * {@link JoystickButton}.
//    */
//   private void configureButtonBindings() {
//     // Drive at half speed when the right bumper is held
//     new JoystickButton(m_driverController, Button.kBumperRight.value)
//         .whenPressed(() -> m_robotDrive.setMaxOutput(0.5))
//         .whenReleased(() -> m_robotDrive.setMaxOutput(1));
//   }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

// all this commented out stuff moved to its own file and made to work right
// junk still here for reference by rkt

//     // Create a voltage constraint to ensure we don't accelerate too fast
//     var autoVoltageConstraint =
//         new DifferentialDriveVoltageConstraint(
//             new SimpleMotorFeedforward(
//                 DriveConstants.ksVolts,
//                 DriveConstants.kvVoltSecondsPerMeter,
//                 DriveConstants.kaVoltSecondsSquaredPerMeter),
//             DriveConstants.kDriveKinematics,
//             10);

//     // Create config for trajectory
//     TrajectoryConfig config =
//         new TrajectoryConfig(
//                 AutoConstants.kMaxSpeedMetersPerSecond,
//                 AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//             // Add kinematics to ensure max speed is actually obeyed
//             .setKinematics(DriveConstants.kDriveKinematics)
//             // Apply the voltage constraint
//             .addConstraint(autoVoltageConstraint);

//     Trajectory exampleTrajectory;

//     // An example trajectory to follow.  All units in meters.
//     int path;
//     path =1 ;
//     switch (path)
//     {
//     case 1: // S
//         exampleTrajectory =
//         TrajectoryGenerator.generateTrajectory(
//             // Start at the origin facing the +X direction
//             new Pose2d(0, 0, new Rotation2d(0)),
//             // Pass through these two interior waypoints, making an 's' curve path
//             List.of(new Translation2d(.9, .5), new Translation2d(1.8, -.5)),
//             // End 3 meters straight ahead of where we started, facing forward
//             new Pose2d(3., 0, new Rotation2d(0)),
//             // Pass config
//             config);
//         break;
//     case 2: //straight
//         exampleTrajectory =
//         TrajectoryGenerator.generateTrajectory(
//             // Start at the origin facing the +X direction
//             new Pose2d(0, 0, new Rotation2d(0)),
//              List.of(new Translation2d(1.5, 0)),
//             // End 3 meters straight ahead of where we started, facing forward
//             new Pose2d(3, 0, new Rotation2d(0)),
//             // Pass config
//             config);
//         break;
//     default:
//         exampleTrajectory = null;
//     }

//     System.out.println(exampleTrajectory);

//     RamseteCommand ramseteCommand =
//         new RamseteCommand(
//             exampleTrajectory,
//             m_robotDrive::getPose,
//             new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
//             new SimpleMotorFeedforward(
//                 DriveConstants.ksVolts,
//                 DriveConstants.kvVoltSecondsPerMeter,
//                 DriveConstants.kaVoltSecondsSquaredPerMeter),
//             DriveConstants.kDriveKinematics,
//             m_robotDrive::getWheelSpeeds,
//             new PIDController(DriveConstants.kPDriveVel, 0, 0),
//             new PIDController(DriveConstants.kPDriveVel, 0, 0),
//             // RamseteCommand passes volts to the callback
//             m_robotDrive::tankDriveVolts,
//             m_robotDrive);

//     // Reset odometry to the starting pose of the trajectory.
//     m_robotDrive.resetOdometry(exampleTrajectory.getInitialPose());

// Run path following command, then stop at the end.
//return ramseteCommand.andThen(() -> m_robotDrive.stopAllMotors());

    return new AutonomousTrajectories(m_robotDrive, m_chooser.getSelected());
  }
}
