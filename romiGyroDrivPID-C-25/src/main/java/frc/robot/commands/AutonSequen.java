// romiGyroDrivPID - C  24    AutonSequen.j   auto sequence cmd

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsys;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class AutonSequen extends SequentialCommandGroup {
  /*
   * Creates an Autonomous sequence based on distance. This will drive
   * a specified distance, turn CCW 180, drive back to start, and turn 180.  
   * @param drivetrain the drive subsystem to be controlled
   */
  public AutonSequen(DriveSubsys drivetrain) {
    addCommands(
        new DriveDistaStable(0.6, 42, drivetrain),
        new WaitCommand(4.0),
        new TurnToAngle(-179, drivetrain), // CCW
        new WaitCommand(4.0),
        new DriveDistaStable(0.6, 40, drivetrain),
        new WaitCommand(4.0),
        new TurnToAngle(179, drivetrain)); // CW
  }
}   // end class
