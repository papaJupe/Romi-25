// VSpace\romiGyroDrivPID - C 24  auto cmd   DriveDistance.j   

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsys;

import edu.wpi.first.wpilibj2.command.Command;

public class DriveDistance extends Command {
  private final DriveSubsys m_drive;
  private final double m_distance;
  private final double m_speed;

  /**
   * Creates new DriveDistance command; goes 'straight' fwd/bak at set
   * speed for distance per wheel encoders. No stabiliz unless Rt bumper
   * button held, so I use DDStable in sequential auto
   * 
   * @param speed  The speed [0-1] for the robot to drive
   * @param inches The number of inches the robot to drive
   * @param drive  The drivetrain subsystem for this cmd
   */
  public DriveDistance(double speed, double inches, DriveSubsys drive) {
    m_speed = speed;
    m_distance = inches;
    m_drive = drive;
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_drive.arcaDriv(0, 0);
    m_drive.resetEncoders();
    m_drive.resetGyro();
  }

  // Called every time the scheduler runs it.
  @Override
  public void execute() {
    m_drive.arcaDriv(m_speed, 0);
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
