// romiGyroDrivPID - C  24            DriveSubsystem. j

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.sensors.RomiGyro;

public class DriveSubsys extends SubsystemBase {
  private static final double kCountsPerRevolution = 1440.0;
  private static final double kWheelDiameterInch = 2.75591; // 70 mm
  private double maxFactor = 1.0; // speed multiplier

  // The Romi has the left and right motors set to
  // PWM channels 0 and 1 respectively
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  // The Romi has onboard encoders that are hardcoded
  // to use DIO pins 4/5 and 6/7 for the left and right
  private final Encoder m_leftEncoder = new Encoder(4, 5);
  private final Encoder m_rightEncoder = new Encoder(6, 7);

  // Using differential drive class for its arcade drive method
  // [applies default deadband 0.02,squares inputs]
  private final DifferentialDrive m_diffDrive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  // instance RomiGyro
  public final RomiGyro m_gyro = new RomiGyro();

  /** Construct a new Drivetrain subsyst */
  public DriveSubsys() {
    // must invert one side of the drivetrain so that (+) volts
    // make both sides move forward. Depending on how your robot's
    // drive work, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);

    // Use inches as unit for encoder distances
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) /
        kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) /
        kCountsPerRevolution);
    resetEncoders();
    resetGyro();
  } // end constructor

  public void setMaxOutput(double d) {
    maxFactor = d;
  }

  // WPI examples call class method arcadeDrive -- confusing,
  // because top aD() is subsys. class method, second is inherited
  // super's diffDrive method; using same name sloppy
  public void arcaDriv(double xaxisSpeed, double zaxisRotate) {
    // diffDrive's aD internally inverts z-rot to make (-) go CW, so..
    m_diffDrive.arcadeDrive(xaxisSpeed * maxFactor, -zaxisRotate, true);
  }

  // small PID feedback numbers work better if not squared
  public void arcaDrivP(double xaxisSpeed, double zaxisRotate) {
    // diffDrive's aD internally inverts z-rot to make (-) go CW, so..
    m_diffDrive.arcadeDrive(xaxisSpeed * maxFactor, -zaxisRotate * 0.9, false);
  }

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
