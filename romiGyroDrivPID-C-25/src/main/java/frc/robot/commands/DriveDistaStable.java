// romiGyroDrivPID - C  24 		       DriveDistaStable auto cmd

// subclassing PIDControl cmd seems to give more stable predictable
// control of drive stabliz and turns than freestanding PIDC as in v.D
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsys;

public class DriveDistaStable extends PIDCommand {
  // private final double m_speed;
  private final double m_distance;
  private final DriveSubsys m_drive;

  /** Construct a new DriveDistaStable cmd */
  public DriveDistaStable(double speed, double inch, DriveSubsys drive) {
    super(
        new PIDController(
            DriveConstants.kStabilizP,
            DriveConstants.kStabilizI,
            DriveConstants.kStabilizD),
        // feedback using the turn rate/ or actual angle?
        drive::getGyroAngleZ,
        // Setpoint is 0, v.i. initialize()
        0,
        // Pipe the output to the turning cmd
        output -> drive.arcaDrivP(speed, output),
        // Require the robot drive
        drive); // end super()
    // m_speed = speed;
    m_distance = inch;
    m_drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here
    // the delta tolerance ensures the robot is stationary at the
    // setpoint before it's considered to reach the reference
    getController().setTolerance(DriveConstants.kTurnTolerDeg,
        DriveConstants.kTurnRateTolerVeloc);
  } // end constructor

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.arcaDriv(0, 0);
    m_drive.resetEncoders();
    m_drive.resetGyro();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drive.arcaDriv(0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Compare distance travelled from start to desired distance
    return Math.abs(m_drive.getAverageDistanceInch()) >= m_distance;
  }
} // end class
