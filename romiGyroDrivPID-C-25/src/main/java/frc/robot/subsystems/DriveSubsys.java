// romiGyroDrivPID - C  25           DriveSubsystem. j

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.DriveConstants;
import frc.robot.sensors.RomiGyro;

public class DriveSubsys extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterInch = 2.756; // 70 mm
  private double maxFactor = 1.0; // speed multiplier
  private double rotFactor = 1.0; // rotation multiplier

  // Romi has left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Use differential drive class for its arcade drive method
  // [applies default deadband 0.02, squares inputs]
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  // instance RomiGyro
  public final RomiGyro m_gyro = new RomiGyro();

  // slowing period from 0.02 --> 0.1, calc increments need div. by 5
  private final PIDController p_controller = new PIDController(
      DriveConstants.kTurnP,
      DriveConstants.kTurnI,
      DriveConstants.kTurnD,
                 0.1);

  // slowing period from 0.02 --> 0.1, calc increments need div. by 5
  private final ProfiledPIDController profP_controller = new ProfiledPIDController(
      DriveConstants.kTurnP,
      DriveConstants.kTurnI,
      DriveConstants.kTurnD,
      new TrapezoidProfile.Constraints(
          DriveConstants.kMaxTurnDegPerSec,
          DriveConstants.kMaxTurnAccel),
          0.1);

  // to use FF + FB for drive accuracy, sometime
  //private final SimpleMotorFeedforward m_feedforward =
      // new SimpleMotorFeedforward(
      //     DriveConstants.ksVolts,
      //     DriveConstants.kvVoltSecondsPerInch,
      //     DriveConstants.kaVoltSecondsSquaredPerinch);

  /* Construct a new drivetrain subsyst */
  public DriveSubsys() {
    // must invert one side of the drivetrain so that (+) volts
    // drive both sides forward. Depending on how your robot's
    // drive work, you might have to invert the left motor instead.
    m_rightMotor.setInverted(true);

    // Use inch as unit for encoder distance
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) /
        kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) /
        kCountsPerRevolution);
    resetEncoders();
    resetGyro();

    profP_controller.setTolerance(2);
    p_controller.setTolerance(2);

  } // end constructor

  public void setMaxSpeed(double d) {
    maxFactor = d;
  }

  public void setMaxRotat(double d) {
    rotFactor = d;
  }

  // WPI examples name class method 'arcadeDrive' -- confusing,
  // because top aD() is subsys. class method, second is inherited
  // super's diffDrive method; using same name == sloppy coding
  public void arcaDriv(double xaxisSpeed, double zaxisRotate) {
    // stick input Y need invert
    // diffDrive's aD internally inverts z-rot to make (-) go CW, so..
    m_diffDrive.arcadeDrive(-xaxisSpeed * maxFactor, -zaxisRotate * rotFactor, true);
  }

  // small PID feedback numbers work better if not squared
  public void arcaDrivP(double xaxisSpeed, double zaxisRotate) {
    // PID feedback should not need speed invert ?
    // diffDrive's aD internally inverts z-rot to make (-) go CW, so..
    m_diffDrive.arcadeDrive(xaxisSpeed * maxFactor, -zaxisRotate * 0.9, false);
  }

  // Return a command that drives the robot with arcade control
  // would normal param work as well?
  // public Command arcadeDriveCommand(DoubleSupplier fwd, DoubleSupplier rot) {
  // // A split-stick arcade command, with forward/backward controlled by the left
  // // hand, and turning controlled by the right.
  // return run(() -> m_drive.arcadeDrive(fwd.getAsDouble(), rot.getAsDouble()))
  // .withName("arcadeDrive");
  // } // end Cmd

  /* ? do these inline cmd classes retain all usual method: execute, end ?

   * Return a command that drives the robot at given distance and speed
   * @param distanceMeters The distance to drive forward in inch
   * expect deviant abrupt move
   */
  public Command driveDistanceCommand(double distanceInch, double speed) {
    return runOnce( () -> { // Reset encoders at the start
          m_leftEncoder.reset();
          m_rightEncoder.reset();
        })
        // Drive forward at specified speed
        .andThen(run(() -> arcaDriv(speed, 0)))
        // End command when wheel has turned the specified distance
        .until( () -> 
            Math.max(m_leftEncoder.getDistance(), m_rightEncoder.getDistance()) >= distanceInch)
        // Stop motor when the command ends
        .finallyDo(interrupted -> arcaDriv(0, 0));

  }  // end drivDistCmd

    /* Returns command to turn to an angle using PIDcontroller */
    public Command turnToAngleCmd(double angleDeg) {
      return runOnce( () -> { 
              m_gyro.reset();
              p_controller.reset(); 
            })
          .andThen(run(() -> 
              arcaDrivP(0, (p_controller.calculate(m_gyro.getAngleZ(), angleDeg)) * 0.05)))
          .until(p_controller::atSetpoint)
          .finallyDo(interrupted -> arcaDriv(0, 0));
          } // end tTACmd
          
  /* Returns command to turn to an angle using motion profiled PID control */
  public Command turnToAngleProfCommand(double angleDeg) {
    return runOnce( () -> { 
            m_gyro.reset();
            profP_controller.reset(0); 
          })
        .andThen(run(() -> 
            arcaDrivP(0, (profP_controller.calculate(m_gyro.getAngleZ(), angleDeg)) * 0.05)))
        .until(profP_controller::atGoal)
        .finallyDo(interrupted -> arcaDriv(0, 0));

  } // end TT angle cmd
              // calc need?? Divide FF voltage by battery voltage to normalize to [-1, 1]
            // + m_feedforward.calculate(profP_controller.getSetpoint().velocity)
            // / RobotController.getBatteryVoltage()))

  public void resetEncoders() {
    m_leftEncoder.reset();
    m_rightEncoder.reset();
  }

  // Re'zero' the gyro.
  public void resetGyro() {
    m_gyro.reset();
    System.out.println("gyro reset");
  }

  public int getLeftEncoderCount() {
    return m_leftEncoder.get();
  }

  public int getRightEncoderCount() {
    return m_rightEncoder.get();
  }

  public double getLeftDistanceInch() {
    return m_leftEncoder.getDistance();
  }

  public double getRightDistanceInch() {
    return m_rightEncoder.getDistance();
  }

  public double getAverageDistanceInch() {
    return (getLeftDistanceInch() + getRightDistanceInch()) / 2.0;
  }

  // @return The current X angle of the Romi in degrees
  public double getGyroAngleX() {
    return m_gyro.getAngleX();
  }

  // @return The current Y angle of the Romi in degrees
  public double getGyroAngleY() {
    return m_gyro.getAngleY();
  }

  // @return The current Z angle of the Romi in degrees
  public double getGyroAngleZ() {
    return m_gyro.getAngleZ();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
} // end class
