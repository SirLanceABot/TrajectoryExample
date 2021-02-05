package frc.robot;

import java.io.BufferedReader;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.SparkMaxSubsystem;


public class AutonomousTrajectories  extends SequentialCommandGroup {

public AutonomousTrajectories(SparkMaxSubsystem m_robotDrive, int path) {
 // Create a voltage constraint to ensure we don't accelerate too fast
 var autoVoltageConstraint =
 new DifferentialDriveVoltageConstraint(
     new SimpleMotorFeedforward(
         DriveConstants.ksVolts,
         DriveConstants.kvVoltSecondsPerMeter,
         DriveConstants.kaVoltSecondsSquaredPerMeter),
     DriveConstants.kDriveKinematics,
     10);

// Create config for trajectory
TrajectoryConfig config =
 new TrajectoryConfig(
         AutoConstants.kMaxSpeedMetersPerSecond,
         AutoConstants.kMaxAccelerationMetersPerSecondSquared)
     // Add kinematics to ensure max speed is actually obeyed
     .setKinematics(DriveConstants.kDriveKinematics)
     // Apply the voltage constraint
     .addConstraint(autoVoltageConstraint);

Trajectory autoTrajectory[];

// trajectories to follow.  All units in meters.

switch (path)
{
case 1: // S
 autoTrajectory = new Trajectory[1];

 autoTrajectory[0] =
 TrajectoryGenerator.generateTrajectory(
     // Start at the origin facing the +X direction
     new Pose2d(0, 0, new Rotation2d(0)),
     // Pass through these two interior waypoints, making an 's' curve path
     List.of(new Translation2d(.9, .5), new Translation2d(1.8, -.5)),
     // End 3 meters straight ahead of where we started, facing forward
     new Pose2d(3., 0, new Rotation2d(0)),
     // Pass config
     config);
 break;

case 2: //straight
 autoTrajectory = new Trajectory[1];

 autoTrajectory[0] =
 TrajectoryGenerator.generateTrajectory(
     // Start at the origin facing the +X direction
     new Pose2d(0, 0, new Rotation2d(0)),
      List.of(new Translation2d(1.5, 0)),
     // End 3 meters straight ahead of where we started, facing forward
     new Pose2d(3, 0, new Rotation2d(0)),
     // Pass config
     config);

 break;

case 3:

/*
    Import trajectory
    then splice in the RamseteCommand stuff
    watch for REVERSE true/false.  may need to fix up
    how to splice together path 1 and path 2
    */
    //String trajectoryJSON = "paths/YourPath.wpilib.json";
    //String trajectoryJSON = "C:\\Users\\RKT\\frc\\FRC2021\\Code\\Romi\\PathWeaver\\output\\1.wpilib.json"; //works
    //String trajectory1JSON = "Romi\\PathWeaver\\output\\1.wpilib.json"; // works

    autoTrajectory = new Trajectory[2];

    String trajectory1JSON = "1.wpilib.json";
    try {
      Path trajectory1Path = Filesystem.getDeployDirectory().toPath().resolve(trajectory1JSON);
      autoTrajectory[0] = TrajectoryUtil.fromPathweaverJson(trajectory1Path);
      //System.out.println(autoTrajectory[0]);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory 1: " + trajectory1JSON, ex.getStackTrace());
    }

    String trajectory2JSON = "2.wpilib.json";
    //Trajectory exampleTrajectory2 = new Trajectory();
    try {
      Path trajectory2Path = Filesystem.getDeployDirectory().toPath().resolve(trajectory2JSON);
      autoTrajectory[1] = TrajectoryUtil.fromPathweaverJson(trajectory2Path);
      //System.out.println(autoTrajectory[1]);
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open trajectory 2: " + trajectory2JSON, ex.getStackTrace());
    }

 break;

case 4:
    autoTrajectory = new Trajectory[2];

    autoTrajectory[0] =
    TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(0, 0, new Rotation2d(0)),
         List.of(new Translation2d(1., 0)),
        // End 1.5 meters straight ahead of where we started, facing forward
        new Pose2d(1.5, 0, new Rotation2d(0)),
        // Pass config
        config);

    autoTrajectory[1] =
    TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        new Pose2d(1.5, 0, new Rotation2d(0)),
            List.of(new Translation2d(2.5, 0)),
        // End 3 meters straight ahead of where we started, facing forward
        new Pose2d(3.5, 0, new Rotation2d(0)),
        // Pass config
        config);

 break;

default:
    autoTrajectory = null;
 break;
}

//System.out.println(autoTrajectory[0]);
RamseteCommand[] ramseteCommand = new RamseteCommand[autoTrajectory.length];

for(int idx = 0; idx < autoTrajectory.length; idx++) {
  ramseteCommand[idx] =
    new RamseteCommand(
     autoTrajectory[idx],
     m_robotDrive::getPose,
     new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
     new SimpleMotorFeedforward(
         DriveConstants.ksVolts,
         DriveConstants.kvVoltSecondsPerMeter,
         DriveConstants.kaVoltSecondsSquaredPerMeter),
     DriveConstants.kDriveKinematics,
     m_robotDrive::getWheelSpeeds,
     new PIDController(DriveConstants.kPDriveVel, 0, 0),
     new PIDController(DriveConstants.kPDriveVel, 0, 0),
     // RamseteCommand passes volts to the callback
     m_robotDrive::tankDriveVolts,
     m_robotDrive);

  if(idx < autoTrajectory.length-1)
      addCommands(ramseteCommand[idx]); // assume each trajectory starts where the last ended; poor assumption but how to fix?; hope for the best that it isn't too tramatic to stay the course
      // addCommands(ramseteCommand[idx].andThen(() -> m_robotDrive.resetOdometry( m_robotDrive.getPose() ))); // useless to reset to where I am; try again below
      // addCommands(ramseteCommand[idx].andThen(() -> m_robotDrive.resetOdometry(autoTrajectory[idx+1].getInitialPose()))); // nope; have to make each command using visible variable
      // addCommands( new resetOdometry( m_robotDrive));

   else // last trajectory is special to stop motors; could make another command to do that but maybe this is better - faster; less overrun?
      addCommands(ramseteCommand[idx].andThen(() -> m_robotDrive.stopAllMotors( )));
 }

// Reset odometry to the starting pose of the trajectory.
m_robotDrive.resetOdometry(autoTrajectory[0].getInitialPose());

}

}
