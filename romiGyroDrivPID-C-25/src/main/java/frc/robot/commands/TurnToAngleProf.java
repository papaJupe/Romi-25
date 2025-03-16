// romiGyroDrivPID - C 24               turnToAngleProfile.j
// only use here on button 2 press, more stable with new Tolerance
// constant 2/24. First five button presses ratchet up turn, then
// more stabiliz ~ target. Irregularity ??? due to romi hardware,
// possible temp. related ?

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsys;

// A cmnd to turn robot to specified angle using a motion profile
public class TurnToAngleProf extends ProfiledPIDCommand {
  /**
   * set turn speed w/ trapz profile + PID, calc from recd gyro angle
   * 
   * @param targetAngleDegrees == angle setting
   * @param drive              drive subsystem to use
   */
  private final DriveSubsys m_drive;

  public TurnToAngleProf(double targetAngleDegrees, DriveSubsys drive) {
    super(
        new ProfiledPIDController(
            DriveConstants.kTurnP,
            DriveConstants.kTurnI,
            DriveConstants.kTurnD,
            new TrapezoidProfile.Constraints(
                DriveConstants.kMaxTurnVeloc,
                DriveConstants.kMaxTurnAcceler)),
        // Close loop using heading
        drive::getGyroAngleZ,
        // Set reference (setpoint?)to target angle
        targetAngleDegrees,
        // Pipe output to subsys' turn method
        (output, setpoint) -> drive.arcaDriv(0, output),
        // Require the subsystem
        drive); // end super

    m_drive = drive;

    // Set the controller to be continuous (because it is an angle controller)
    getController().enableContinuousInput(-180, 180);
    // Set the controller tolerance - the delta tolerance ensures the robot
    // is stationary at the setpoint before it isFinished
    getController().setTolerance(DriveConstants.kTurnTolerDeg, DriveConstants.kTurnRateTolerVeloc);
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
    // m_drive.resetGyro(); // add if not in calling class already
  }

  @Override
  public boolean isFinished() {
    // End when the controller is at the reference.
    return getController().atGoal();
  }
} // end class