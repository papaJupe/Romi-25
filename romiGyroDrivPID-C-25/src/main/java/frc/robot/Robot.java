// RomigyroDrivPID - C - 24             Robot.j  cmd/subsys framewk

// edit 240118 try to get profiled turn to work, still erratic
// edit 240214 added romi vendordep (no effect, may permit update
// online?) profile turn helped by increasing Tol constant, still
// starts w/ undershoot, then increases to overshoot; same can
// occur w/ auto turn or triggered turn -- likely romi hardware
// irregular response, also temp variation ?

// uses WPI PIDcontrol lib in RC button to activate gyro in teleOp
// drive and DriveDistaStable cmd (used in Auto sequence)

// For live vision, attach camera to any pi port, its cam server streams
// automatically to pi web interface: wpilibpi.local:1181 or mpeg 
// stream, and (when/if Sim is running) to Shuffleboard.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and call the
 * functions corresponding to each mode, as described in TimedRobot doc.
 * -- very little here specific to any one robot.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  public RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    // Instantiate RobotContainer --> declares, instances, configs the
    // robot specific components and their functionality (methods).
    m_robotContainer = new RobotContainer();
  }

  // This function is called every robot packet, no matter the mode.
  // Does things you want run during all modes, like diagnostics.
  // This runs after the mode specific periodic functions, but before
  // LiveWindow and SmartDashboard integrated updating.
  @Override
  public void robotPeriodic() {
    // Calls the Scheduler <-- this is for polling buttons, adding
    // newly-scheduled commands, running now-scheduled commands,
    // removing finished or interrupted commands, and running subsystem periodics.
    // Need CS.run here for anything in the Cmd/Subsys framework to work.
    SmartDashboard.putNumber("Z axis Rot",
        m_robotContainer.m_drivetrain.m_gyro.getAngleZ());

    if (RobotContainer.m_controller.getRawButton(1))
      m_robotContainer.m_drivetrain.resetGyro();
    CommandScheduler.getInstance().run();
  } // end robotPeriodic

  // function is called once each time robot enters Disabled mode.
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  // autoInit runs the autonomous command set in {@link RobotContainer}
  @Override
  public void autonomousInit() {
    // RC got selected routine from the SmartDashboard
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_robotContainer.m_drivetrain.resetEncoders();
    m_robotContainer.m_drivetrain.resetGyro();
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
    m_robotContainer.m_drivetrain.resetEncoders();
    m_robotContainer.m_drivetrain.resetGyro();
  } // end teleInit

  /** This function is called periodically during operator control. */
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
