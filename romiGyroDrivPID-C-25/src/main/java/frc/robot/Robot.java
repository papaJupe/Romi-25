// RomigyroDrivPID - C - 25             Robot.j  cmd/subsys framewk

// total restructuring for '25 update. old PID classes now deprecated,
// + had poor function --> condense cmd for Drive & auto into that subsys.
// uses WPI PIDcontrol lib in RC button to activate gyro in teleOp

// For live vision, attach camera to any pi port, its cam server streams
// automatically to pi web interface: wpilibpi.local:1181 or mpeg 
// stream, and (when/if Sim is running) to Shuffleboard.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/* The VM is configured to run this class automatically, and call its
 * methods for each mode. WPI dogma says:
 * "Since Command-based is a "declarative" paradigm [??], very little robot 
*  logic should be handled in Robot periodics besides scheduler calls."
 * Now -- little here specific to any one robot; 
 * but if rI goes away, this class could use an explicit Robot
 * constructor for init/bindings & run project simple ops in periodic
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    // instantiate RobotContainer --> declares, instances, configs the
    // robot specific components and their functionality (methods).
    m_robotContainer = new RobotContainer();
  }

  // This function is called every robot packet, no matter the mode.
  // Does things you want run in all modes, like diagnostics.
  // They run after the mode specific periodic functions, but before
  // LiveWindow and SmartDashboard integrated updating.
  @Override
  public void robotPeriodic() {
    // Calls the Scheduler <-- this is for polling buttons, adding
    // newly-scheduled commands, running now-scheduled commands,
    // removing finished or interrupted commands, and running subsystem periodics.
    // Need CS.run here for anything in the Cmd/Subsys framework to work.

    if (RobotContainer.m_controller.getRawButton(1))
      m_robotContainer.m_drive.resetGyro();

      SmartDashboard.putNumber("Z axis Rot",
                     m_robotContainer.m_drive.m_gyro.getAngleZ());

    CommandScheduler.getInstance().run();
  } // end robotPeriodic

  // function is called once each time robot enters Disabled mode.
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  // autoInit runs the autonomous command set in {RobotContainer <-- chooser}
  @Override
  public void autonomousInit() {
    // RC got selected routine from the SmartDashboard chooser
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_robotContainer.m_drive.resetEncoders();
    m_robotContainer.m_drive.resetGyro();
    // schedule the selected autonomous command (if not empty variable)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  } // end autoInit

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This confirms that the autonomous code has stopped,. If you want
    // auto cmd to continue until interrupted, remove this line.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.m_drive.resetEncoders();
    m_robotContainer.m_drive.resetGyro();
  } // end teleInit

  /** This method is called periodically during operator control. */
  @Override
  public void teleopPeriodic() { // rt bumper button hold activates
    // PID mode to drive straight in teleop [v. RC JoystButton(cmd)
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }
} // end class
