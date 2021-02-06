// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

//import static edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
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
    m_chooser.addOption("Auto Routine S", Integer.valueOf(1));
    m_chooser.setDefaultOption("Auto Routine Straight", Integer.valueOf(2));
    m_chooser.addOption("Auto Bounce Challenge", Integer.valueOf(3));
    m_chooser.addOption("Auto 2 Straights", Integer.valueOf(4));
    SmartDashboard.putData(m_chooser);
    //SmartDashboard.updateValues();

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
    // return the autonomous command to run that was built in AutonomousTrajectories class file
    return new AutonomousTrajectories(m_robotDrive, m_chooser.getSelected());
  }
}
