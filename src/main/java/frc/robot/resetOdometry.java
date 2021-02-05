// command I thought might be useful instead of in-lining it but decided not to use it at all
package frc.robot;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SparkMaxSubsystem;

public class resetOdometry extends CommandBase {

  public resetOdometry(SparkMaxSubsystem m_robotDrive) {

    addRequirements(m_robotDrive);
    m_robotDrive.resetOdometry( m_robotDrive.getPose() );

  }
}

// usage by itself
// addCommands(new resetOdometry( m_robotDrive));
// or with others
// addCommands(mycommand, new resetOdometry( m_robotDrive));
// or in-lined
// // addCommands(mycommand.andThen(() -> m_robotDrive.resetOdometry( m_robotDrive.getPose() )));


// import edu.wpi.first.wpilibj.templates.commandbased.subsystems.ExampleSubsystem;
// import edu.wpi.first.wpilibj2.command.CommandBase;

// /**
//  * An example command that uses an example subsystem.
//  */
// public class ExampleCommand extends CommandBase {
//   @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
//   private final ExampleSubsystem m_subsystem;

//   /**
//    * Creates a new ExampleCommand.
//    *
//    * @param subsystem The subsystem used by this command.
//    */
//   public ExampleCommand(ExampleSubsystem subsystem) {
//     m_subsystem = subsystem;
//     // Use addRequirements() here to declare subsystem dependencies.
//     addRequirements(subsystem);
//   }

// package edu.wpi.first.wpilibj.examples.hatchbottraditional.commands;

// import edu.wpi.first.wpilibj2.command.CommandBase;

// import edu.wpi.first.wpilibj.examples.hatchbottraditional.subsystems.HatchSubsystem;

// /**
//  * A simple command that grabs a hatch with the {@link HatchSubsystem}.  Written explicitly for
//  * pedagogical purposes.  Actual code should inline a command this simple with {@link
//  * edu.wpi.first.wpilibj2.command.InstantCommand}.
//  */
// public class GrabHatch extends CommandBase {
//   // The subsystem the command runs on
//   private final HatchSubsystem m_hatchSubsystem;

//   public GrabHatch(HatchSubsystem subsystem) {
//     m_hatchSubsystem = subsystem;
//     addRequirements(m_hatchSubsystem);
//   }

//   @Override
//   public void initialize() {
//     m_hatchSubsystem.grabHatch();
//   }

//   @Override
//   public boolean isFinished() {
//     return true;
//   }
// }



// package edu.wpi.first.wpilibj.examples.hatchbottraditional.commands;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.wpilibj2.command.CommandBase;

// import edu.wpi.first.wpilibj.examples.hatchbottraditional.subsystems.DriveSubsystem;

// /**
//  * A command to drive the robot with joystick input (passed in as {@link DoubleSupplier}s). Written
//  * explicitly for pedagogical purposes - actual code should inline a command this simple with {@link
//  * edu.wpi.first.wpilibj2.command.RunCommand}.
//  */
// public class DefaultDrive extends CommandBase {
//   private final DriveSubsystem m_drive;
//   private final DoubleSupplier m_forward;
//   private final DoubleSupplier m_rotation;

//   /**
//    * Creates a new DefaultDrive.
//    *
//    * @param subsystem The drive subsystem this command wil run on.
//    * @param forward The control input for driving forwards/backwards
//    * @param rotation The control input for turning
//    */
//   public DefaultDrive(DriveSubsystem subsystem, DoubleSupplier forward, DoubleSupplier rotation) {
//     m_drive = subsystem;
//     m_forward = forward;
//     m_rotation = rotation;
//     addRequirements(m_drive);
//   }

//   @Override
//   public void execute() {
//     m_drive.arcadeDrive(m_forward.getAsDouble(), m_rotation.getAsDouble());
//   }
// }
// Notice that this command does not override isFinished(),
//  and thus will never end; this is the norm for commands that are intended to be used as
//   default commands (and, as can be guessed, the library includes tools to make this kind
//    of command easier to write, too!).