/////////// SPARKMAX CAN MOTOR CONTROLLER SUBSYSTEM ////////////

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Robot;
import frc.robot.subsystems.MotorConstants;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SparkMaxSubsystem extends SubsystemBase {
    
  // DEFINE AVAILABLE MOTORS
  // choices will be front no back, back no front, front and back with back the follower

  static final boolean front = false;
  static final boolean back = true;

  private static CANSparkMax frontRightMotor;
  private static CANSparkMax frontLeftMotor;
  private static CANSparkMax backRightMotor;
  private static CANSparkMax backLeftMotor;
  private static DifferentialDrive m_drive;

    // LIMIT SWITCHES
    private CANDigitalInput m_forwardLimitFrontRightMotor;
    private CANDigitalInput m_reverseLimitFrontRightMotor;
    private CANDigitalInput m_forwardLimitFrontLeftMotor;
    private CANDigitalInput m_reverseLimitFrontLeftMotor;

    private CANDigitalInput m_forwardLimitBackRightMotor;
    private CANDigitalInput m_reverseLimitBackRightMotor;
    private CANDigitalInput m_forwardLimitBackLeftMotor;
    private CANDigitalInput m_reverseLimitBackLeftMotor;
    
    // ENCODERS
    private CANEncoder m_encoderRight;
    private CANEncoder m_encoderLeft;

    private boolean needToResetEncoder = true;
    private boolean isEncoderResetting = false;

    // Odometry class for tracking robot pose
    private DifferentialDriveOdometry m_odometry;

    private Timer timer = new Timer();

    private double speedFactor = 1.0;

    // constructor for drivetrain class
    public SparkMaxSubsystem()
    {
      System.out.println(this.getClass().getName() + ": Started Constructing");

      // DRIVE MOTOR CONTROLLERS
      if(front) {
      frontRightMotor = new CANSparkMax(Constants.FRONT_RIGHT_MOTOR_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);
      frontLeftMotor = new CANSparkMax(Constants.FRONT_LEFT_MOTOR_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);

      frontRightMotor.restoreFactoryDefaults();
      frontLeftMotor.restoreFactoryDefaults();

      frontRightMotor.setInverted(true); // right motor mounted mirror image - inverted - from the left

      frontRightMotor.setSmartCurrentLimit(MotorConstants.getMotorStallCurrent(MotorConstants.Constants.MotorType.kNeoMotor, 0.3));
      // frontRightMotor.setSecondaryCurrentLimit(Constants.SECONDARY_MOTOR_CURRENT_LIMIT);
      frontRightMotor.setOpenLoopRampRate(Constants.DRIVE_RAMP_TIME);
      frontRightMotor.setIdleMode(IdleMode.kBrake);
      frontRightMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
      frontRightMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
      m_forwardLimitFrontRightMotor = frontRightMotor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
      m_reverseLimitFrontRightMotor = frontRightMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
      m_forwardLimitFrontRightMotor.enableLimitSwitch(false);
      m_reverseLimitFrontRightMotor.enableLimitSwitch(false);

      frontLeftMotor.setSmartCurrentLimit(MotorConstants.getMotorStallCurrent(MotorConstants.Constants.MotorType.kNeoMotor, 0.3));
      // frontLeftMotor.setSecondaryCurrentLimit(Constants.SECONDARY_MOTOR_CURRENT_LIMIT);
      frontLeftMotor.setOpenLoopRampRate(Constants.DRIVE_RAMP_TIME);
      frontLeftMotor.setIdleMode(IdleMode.kBrake);
      frontLeftMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
      frontLeftMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
      m_forwardLimitFrontLeftMotor = frontLeftMotor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
      m_reverseLimitFrontLeftMotor = frontLeftMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
      m_forwardLimitFrontLeftMotor.enableLimitSwitch(false);
      m_reverseLimitFrontLeftMotor.enableLimitSwitch(false);
      m_drive = new DifferentialDrive(frontLeftMotor, frontRightMotor);
      // ENCODER SETUP
      m_encoderRight = frontRightMotor.getEncoder();
      m_encoderLeft = frontLeftMotor.getEncoder();
      }
  
      if(back) {
      backRightMotor = new CANSparkMax(Constants.BACK_RIGHT_MOTOR_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);
      backLeftMotor = new CANSparkMax(Constants.BACK_LEFT_MOTOR_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);

      backRightMotor.restoreFactoryDefaults();
      backLeftMotor.restoreFactoryDefaults();

      backRightMotor.setInverted(true); // right motor mounted mirror image - inverted - from the left

      backRightMotor.setSmartCurrentLimit(MotorConstants.getMotorStallCurrent(MotorConstants.Constants.MotorType.kNeoMotor, 0.3));
      // backRightMotor.setSecondaryCurrentLimit(Constants.SECONDARY_MOTOR_CURRENT_LIMIT);
      backRightMotor.setOpenLoopRampRate(Constants.DRIVE_RAMP_TIME);
      backRightMotor.setIdleMode(IdleMode.kBrake);
      backRightMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
      backRightMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
      m_forwardLimitBackRightMotor = backRightMotor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
      m_reverseLimitBackRightMotor = backRightMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
      m_forwardLimitBackRightMotor.enableLimitSwitch(false);
      m_reverseLimitBackRightMotor.enableLimitSwitch(false);
      
      backLeftMotor.setSmartCurrentLimit(MotorConstants.getMotorStallCurrent(MotorConstants.Constants.MotorType.kNeoMotor, 0.3));
      // backLeftMotor.setSecondaryCurrentLimit(Constants.SECONDARY_MOTOR_CURRENT_LIMIT);
      backLeftMotor.setOpenLoopRampRate(Constants.DRIVE_RAMP_TIME);
      backLeftMotor.setIdleMode(IdleMode.kBrake);
      backLeftMotor.enableSoftLimit(SoftLimitDirection.kForward, false);
      backLeftMotor.enableSoftLimit(SoftLimitDirection.kReverse, false);
      m_forwardLimitBackLeftMotor = backLeftMotor.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
      m_reverseLimitBackLeftMotor = backLeftMotor.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
      m_forwardLimitBackLeftMotor.enableLimitSwitch(false);
      m_reverseLimitBackLeftMotor.enableLimitSwitch(false);
      if(front) {
        backRightMotor.follow(frontRightMotor);
        backLeftMotor.follow(frontLeftMotor);
      } else {
        m_drive = new DifferentialDrive(backLeftMotor, backRightMotor);      
      // ENCODER SETUP
      m_encoderRight = backRightMotor.getEncoder();
      m_encoderLeft = backLeftMotor.getEncoder();
      }
      }
   
      m_drive.setSafetyEnabled(false);
      m_drive.setRightSideInverted(false); // be sure to invert the SparkMax, not here in the differential drive

      // ENCODER SETUP
      m_encoderRight.setVelocityConversionFactor(Constants.ENCODER_METER_PER_REV/60.); // from RPM to distance/second
      m_encoderLeft.setVelocityConversionFactor(Constants.ENCODER_METER_PER_REV/60.);

      m_encoderRight.setPositionConversionFactor(Constants.ENCODER_METER_PER_REV); // from R to distance
      m_encoderLeft.setPositionConversionFactor(Constants.ENCODER_METER_PER_REV);

      m_odometry = new DifferentialDriveOdometry(Robot.m_gyro.gyroImpl.getRotation2d());
      
      System.out.println(this.getClass().getName() + ": Finished Constructing");
    }

    @Override
  public void periodic() {  
    // Update the odometry in the periodic block
    m_odometry.update(
        Robot.m_gyro.gyroImpl.getRotation2d(), m_encoderLeft.getPosition(), m_encoderRight.getPosition());
    
    //System.out.println(this);
  }

  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Returns the current wheel speeds of the robot.
   *
   * @return The current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(m_encoderLeft.getVelocity(), m_encoderRight.getVelocity());
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    m_odometry.resetPosition(pose, Robot.m_gyro.gyroImpl.getRotation2d());
  }

   /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
        m_drive.arcadeDrive(fwd, rot);
        //System.out.println("L R positions " + m_encoderLeft.getPosition() + " " + m_encoderRight.getPosition()); // debugging
    }
    
  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
    public void tankDriveVolts(double leftVolts, double rightVolts) {

//      System.out.println("tankDriveVolts L R " + leftVolts + " " + rightVolts);
      
      if(front) {
      frontLeftMotor.setVoltage(leftVolts);
      frontRightMotor.setVoltage(rightVolts);
      }
      else {
        backLeftMotor.setVoltage(leftVolts);
        backRightMotor.setVoltage(rightVolts);
//        System.out.println("back L R positions " + m_encoderLeft.getPosition() + " " + m_encoderRight.getPosition()); // debugging
      }

      m_drive.feed();
    }

    public void stopAllMotors() {
      if(front) {
        frontRightMotor.stopMotor();
        frontLeftMotor.stopMotor();
      }
      else {
        backRightMotor.stopMotor();
        backLeftMotor.stopMotor();
      }
    }

    //resets both encoder values
    public void resetEncoders()
    {
        m_encoderRight.setPosition(0.);
        m_encoderLeft.setPosition(0.);
    }

  /**
     * Gets the average distance of the encoders converted to inches.
     * 
     * @return Distance traveled.
     */
    public double getAverageEncoderDistance() 
    {
        return (m_encoderLeft.getPosition() + m_encoderRight.getPosition()) / 2.0;
    }

  /**
   * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
   *
   * @param maxOutput the maximum output to which the drive will be constrained
   */
  public void setMaxOutput(double maxOutput) {
    m_drive.setMaxOutput(maxOutput);
  }

    // /**
    //  * Calculates the angle to rotate based on bearing and heading
    //  * @param bearing
    //  * @return angleToRotate
    //  */
    // public double getAngleToRotate(double bearing)
    // {
    //     // direction: 1 = clockwise, -1 = counter-clockwise
    //     double direction = 1;

    //     double bearingX = Math.cos(Math.toRadians(bearing));
    //     double bearingY = Math.sin(Math.toRadians(bearing));
    //     double headingX = Math.cos(Math.toRadians(getHeadingInDegrees()));
    //     double headingY = Math.sin(Math.toRadians(getHeadingInDegrees()));

    //     double angleToRotateRad = Math.acos((bearingX * headingX) + (bearingY * headingY));

    //     double positiveRotateX = (Math.cos(angleToRotateRad) * headingX) - (Math.sin(angleToRotateRad) * headingY);
    //     double positiveRotateY = (Math.sin(angleToRotateRad) * headingX) + (Math.cos(angleToRotateRad) * headingY);
    //     double negativeRotateX = (Math.cos(-angleToRotateRad) * headingX) - (Math.sin(-angleToRotateRad) * headingY);
    //     double negativeRotateY = (Math.sin(-angleToRotateRad) * headingX) + (Math.cos(-angleToRotateRad) * headingY);

    //     double positiveRotateBearing = Math.toDegrees(Math.atan2(positiveRotateY, positiveRotateX));
    //     double negativeRotateBearing = Math.toDegrees(Math.atan2(negativeRotateY, negativeRotateX));

    //     if (Math.abs(bearing - positiveRotateBearing) > Math.abs(bearing - negativeRotateBearing))
    //     {
    //         direction = -1;
    //     }

    //     return Math.toDegrees(angleToRotateRad) * direction;
    // }

    // /**
    //  * Drive the distance passed into the method.
    //  * 
    //  * @return If the robot has completed the drive.
    //  */
    // public boolean driveDistanceInInches(int inches, double maxSpeed, int bearing, int stoppingDistance, OmniEncoder encoder)
    // {
    //     boolean isDoneDriving = false;

    //     if (needToResetEncoder)
    //     {
    //         resetEncoders();
    //         isEncoderResetting = true;
    //         needToResetEncoder = false;
    //     }
    //     else if (isEncoderResetting)
    //     {
    //         if (Math.abs(getLeftDistanceInInches()) < 2 && Math.abs(getRightDistanceInInches()) < 2)
    //         {
    //             isEncoderResetting = false;
    //             // System.out.println("Encoder has Reset");
    //         }

    //     }
    //     else
    //     {
    //         inches = Math.abs(inches);
    //         double startingDistance = maxSpeed * 12.0;
    //         double distanceTravelled;
    //         int driveDirection = maxSpeed < 0 ? -1 : 1;
    //         double angleToRotate = getAngleToRotate(bearing);

    //         double rotate = angleToRotate / 30.0;

    //         if (encoder == OmniEncoder.kLeft)
    //         {
    //             distanceTravelled = Math.abs(getLeftDistanceInInches());
    //         }
    //         else if (encoder == OmniEncoder.kRight)
    //         {
    //             distanceTravelled = Math.abs(getRightDistanceInInches());
    //         }
    //         else
    //         {
    //             distanceTravelled = Math.abs(getDistanceInInches());
    //         } 

    //         if (distanceTravelled <= inches)
    //         {
    //             if (distanceTravelled <= startingDistance)
    //             {
    //                 driveCartesian(0, ((maxSpeed - (Constants.STARTING_SPEED * driveDirection)) / startingDistance)
    //                         * distanceTravelled + (Constants.STARTING_SPEED * driveDirection), rotate);
    //             }
    //             else if (distanceTravelled >= startingDistance && distanceTravelled <= inches - stoppingDistance)
    //             {
    //                 driveCartesian(0, maxSpeed, rotate);
    //             }
    //             else
    //             {
    //                 driveCartesian(0, Constants.STOPPING_SPEED * driveDirection, rotate);
    //             }
    //         }
    //         else
    //         {
    //             driveCartesian(0, 0, 0);
    //             isDoneDriving = true;
    //             needToResetEncoder = true;
    //         }

    //     }

    //     return isDoneDriving;
    // }

    // /**
    //  * Method to return whether the robot should abort autonomous.
    //  * 
    //  * @return Whether to abort autonomous or not.
    //  */
    // public boolean abortAutonomous()
    // {
    //     return abortAutonomous;
    // }

    // /**
    //  * spins the robot to a given bearing based on m_gyro values
    //  * @param bearing
    //  * @param speed
    //  * @return doneSpinning
    //  */
    // public boolean spinToBearing(int bearing, double speed)
    // {
    //     speed = Math.abs(speed);
    //     boolean doneSpinning = false;
    //     boolean isSpinning = true;
    //     double angleToRotate = getAngleToRotate(bearing);
    //     double direction = Math.signum(angleToRotate);

    //     if(needToResetStartingHeading)
    //     {
    //         startingHeading = getHeadingInDegrees();
    //         startingAngleToRotate = angleToRotate;
    //         needToResetStartingHeading = false;
    //         timer.reset();
    //         timer.start();
    //     }
    //     double angleTravelled = Math.abs(startingHeading - getHeadingInDegrees());
    //     double startingAngle = startingAngleToRotate >= 90 ? 30 : speed * 0.3 * startingAngleToRotate;
    //     double stoppingAngle = startingAngleToRotate >= 90 ? 20 : speed * 0.2 * startingAngleToRotate;

    //     double heading = getHeadingInDegrees();

    //     //check out m_gyro.getUpdateCounter
    //     if (timer.get() >= 0.2)
    //     {
    //         if (previousNavXValue == heading)
    //         {
    //             isSpinning = false;
    //         }
    //         else
    //         {
    //             previousNavXValue = heading;
    //             timer.reset();
    //             timer.start();
    //         }
    //     }
    //     else
    //     {
    //         isSpinning = true;
    //     }

    //     if (isSpinning)
    //     {
    //         speed *= direction;

    //         if (Math.abs(angleToRotate) >= Constants.ROTATE_THRESHOLD)
    //         {
    //             if(angleTravelled <= startingAngle)
    //             {
    //                 driveCartesian(0, 0, ((speed - Constants.STARTING_SPEED) * direction / startingAngle)
    //                     * angleTravelled + Constants.STARTING_SPEED * direction);
    //             }
    //             else if(angleTravelled > startingAngle && angleTravelled < startingAngleToRotate - stoppingAngle)
    //             {
    //                 driveCartesian(0, 0, speed);
    //             }
    //             else
    //             {
    //                 driveCartesian(0, 0, Constants.STOPPING_SPEED * direction);
    //             }

    //         }
    //         else
    //         {
    //             driveCartesian(0, 0, 0);
    //             doneSpinning = true;
    //             needToResetStartingHeading = true;
    //         }
    //     }
    //     else
    //     {
    //         abortAutonomous = true;
    //         System.out.println("\nNAVX REPEATED VALUES.\n");
    //         driveCartesian(0, 0, 0);
    //         doneSpinning = true;
    //         needToResetStartingHeading = true;
    //     }

    //     return doneSpinning;
    // }

    public String getFrontRightMotorData()
    {
        return String.format("%6.3f,  %6.0f,  %6.3f,  %5.1f",
         frontRightMotor.get(), frontRightMotor.getEncoder().getPosition(),
          frontRightMotor.getOutputCurrent(), frontRightMotor.getEncoder().getVelocity());
    }

    public String getFrontLeftMotorData()
    {
        return String.format("%6.3f,  %6.0f,  %6.3f,  %5.1f",
         frontLeftMotor.get(), frontLeftMotor.getEncoder().getPosition(),
         frontLeftMotor.getOutputCurrent(), frontLeftMotor.getEncoder().getVelocity());
    }

    public String getBackRightMotorData()
    {
        return String.format("%6.3f,  %6.0f,  %6.3f,  %5.1f",
         backRightMotor.get(), backRightMotor.getEncoder().getPosition(),
         backRightMotor.getOutputCurrent(), backRightMotor.getEncoder().getVelocity());
    }

    public String getBackLeftMotorData()
    {
        return String.format("%6.3f,  %6.0f,  %6.3f,  %5.1f",
         backLeftMotor.get(), backLeftMotor.getEncoder().getPosition(),
         backLeftMotor.getOutputCurrent(), backLeftMotor.getEncoder().getVelocity());
    }

    @Override
    public String toString()
    {
        return
            String.format("LV: %.2f, RV: %.2f LP: %.2f, RP %.2f Yaw: %.2f",
              m_encoderLeft.getVelocity(), m_encoderRight.getVelocity(),
              m_encoderLeft.getPosition(), m_encoderRight.getPosition(),
              Robot.m_gyro.gyroImpl.getAngle());
    }

    //constants class for this class
    public static class Constants
    {
        public static final int FRONT_LEFT_MOTOR_PORT = 4;
        public static final int FRONT_RIGHT_MOTOR_PORT = 1;
        public static final int BACK_RIGHT_MOTOR_PORT = 2;
        public static final int BACK_LEFT_MOTOR_PORT = 3;

        public static final int PRIMARY_MOTOR_CURRENT_LIMIT = 35;
        public static final int SECONDARY_MOTOR_CURRENT_LIMIT = 45;

        public static final double DRIVE_RAMP_TIME = 0.10;

        public static final double MOTOR_DEADBAND = 0.01;

        public static final double STARTING_SPEED = 0.3;
        public static final double STOPPING_SPEED = 0.175;
        public static final int ROTATE_THRESHOLD = 10;

        public static final int LEFT_ENCODER_CHANNEL_A = 18;
        public static final int LEFT_ENCODER_CHANNEL_B = 16;
        public static final int RIGHT_ENCODER_CHANNEL_A = 14;
        public static final int RIGHT_ENCODER_CHANNEL_B = 15;

        // 4096 ticks per motor revolution native NEO brushless
        //public static final double ENCODER_TICKS_PER_INCH = (360.0 * 4.0) / (3.25 * Math.PI);
        public static final double ENCODER_METER_PER_REV = 1./19.1; // approximately
        public static final double ENCODER_METER_PER_TICK = ENCODER_METER_PER_REV/4096.;
    }
}
