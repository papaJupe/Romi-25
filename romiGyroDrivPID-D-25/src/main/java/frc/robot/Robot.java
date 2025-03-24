// RomigyroDrivPID - D -25                         Robot.j  flat framewk

// v. D (this) - simplify to flat framework, no cmd or subsys, all in robot.j
// PID control x 3: drive distance, drive straight, turn degr.
// v. '25 eliminate PIDCmd, refactor w/ only PIDController, slowed 50 to 10 hz,
// so also div. feedback increment by 5; much tuning of multiplier needed.

// much improved teleOp drive straight w/ button 6; auto
// drive straight fair, auto turn 180 OK alone, irregular
// 180 turn after straight drive ?? why. No auto's end aP -> tI 

// RCstub kept for reference, does nothing; other unused classes deleted.

// For live vision, attach camera to any pi port; its cam server streams
// automatically to pi web interface: wpilibpi.local:1181 or mpeg stream,
// and (when Sim is running) to Shuffleboard.

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.sensors.OnBoardIO;
import frc.robot.sensors.RomiGyro;
import frc.robot.sensors.OnBoardIO.ChannelMode;
import static frc.robot.Constant.*;

/**
 * all robot config set in Robot.j -- normally few specifics here
 */
public class Robot extends TimedRobot {

  // instance joystick @ 0 --assumes controller plugged into USB 0
  private final XboxController m_controller = new XboxController(0);

  private final OnBoardIO m_onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);

  private final DigitalInput frontIRsensor = new DigitalInput(9); // not yet used

  private final double kCountsPerRevolution = 1440.0;
  private final double kWheelDiameterInch = 2.75591; // 70 mm
  // private double maxFactor = 1.0; // speed multiplier possible

  // Romi has the left and right motors on PWM channels 0 and 1
  private final Spark m_leftMotor = new Spark(0);
  private final Spark m_rightMotor = new Spark(1);

  /// DD class has aD() method, default: deadband 0.02, squares inputs
  public final DifferentialDrive m_Drive = new DifferentialDrive(m_leftMotor, m_rightMotor);

  // The Romi's onboard encoders are hardcoded to DIO pins 4/5 and 6/7
  public static Encoder m_leftEncoder = new Encoder(4, 5);
  public static Encoder m_rightEncoder = new Encoder(6, 7);

  // instance the RomiGyro
  public static RomiGyro m_gyro = new RomiGyro();

  // allows SmartDashboard to pick autonomous routine by name
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  private String m_autoSelected; // returned by chooser for use in autoInit

  // all PID-C might benefit from slower update given poor latency of sim envir
  // here 10 hz vs default 50, so I divide its .calculate value by
  // 5 since it's applied over 5x faster increments?
  // PID to auto drive a set distance 2 setpoint in drive&turn code
  private final PIDController piDist = new PIDController(kDistP, kDistI, kDistD, 0.10);

  // PID control to drive straight on bumper button press; also used
  // in auto drive&turn to go straight - 'turn' param setpoint 0
  private final PIDController piDriv = new PIDController(kStabilP, kStabilI, kStabilD, 0.10);

  // ... turn to some angle, not final, revalued inline
  private PIDController pidTurn = new PIDController(kTurnP, kTurnI, kTurnD, kPIDperiod);

  // private Command stopIt; // attempt cmd to stop autoPeriodic, fails
  private Timer mTimer;
  private Timer twoTimer;

  boolean goBack = false; // used in autoInit, repeat sequence or not
  boolean tooClose = false; // bot front about to bump into something

  /*
   * roboInit runs when the robot is first started and does all init's
   * of robot's specifics (things normally in RC in cmd/subsys framwk)
   */
  @Override
  public void robotInit() {
    // Example of onboard IO buttons doing something
    Trigger onboardButtonA = new Trigger(m_onboardIO::getButtonAPressed);
    onboardButtonA
        .whileTrue(new PrintCommand("onbord A Press"))
        .whileFalse(new PrintCommand("onbord A Release"));

    // invert one side of the drivetrain so that positive voltage
    // results in each side moving forward. Depending on how your robot
    // gearbox is constructed, you might have to invert left side instead.
    m_rightMotor.setInverted(true);

    // auto options set by selected string, run in aP
    m_chooser.setDefaultOption("turn180", "TURN180");
    m_chooser.addOption("Drive&Turn180", "DRIV&TURN");
    m_chooser.addOption("turn&return", "RETURN");
    SmartDashboard.putData("AutoSelect ", m_chooser);

    // Use inches as unit for encoder distance output
    m_leftEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) /
        kCountsPerRevolution);
    m_rightEncoder.setDistancePerPulse((Math.PI * kWheelDiameterInch) /
        kCountsPerRevolution);
    m_leftEncoder.reset();
    m_rightEncoder.reset();

    m_gyro.reset();

    // Stabilize to drive straight w/ gyro when R bumper is held
    // in teleOp, bypassing tP AD cmd; 
    // piDriv is calculating angle error 5x slower than control loop
    // so I need to apply .calc feedbk w/ 5x smaller dX than normal
    new JoystickButton(m_controller, 6)
        .onTrue(new InstantCommand(() -> m_gyro.reset()))
        .whileTrue(new RunCommand(() -> m_Drive.arcadeDrive(-m_controller.getLeftY() * 0.4,
            -piDriv.calculate(m_gyro.getAngleZ(), 0) * 0.05, false)))
        .onFalse(new InstantCommand(() -> teleopPeriodic()));
    // probably better control w/ small feedbk number, if not squared

    // the delta tolerance ensures the robot is stable at the
    // setpoint before it's counted as reaching the reference
    // unclear what units are here,
    // likely inch (encoder reporting unit)
    piDist.setTolerance(2); // delete 2nd accel param

    piDriv.setTolerance(2);
    // degree pos. Tol., deg/sec rate Tol.

    pidTurn.enableContinuousInput(-180, 180);
    pidTurn.setTolerance(2);
  } // end robotInit() // deg, deg/sec

  // ... called every robot packet, no matter the mode.
  @Override
  public void robotPeriodic() { // normally, in cmd/subsys framewk:
    // calls the Scheduler <-- responsible for polling buttons, adding
    // newly-scheduled commands, running now-scheduled commands, removing
    // finished or interrupted commands, and running subsystem periodics.
    // Must be here for anything in the Command-based framework to work.
    // ... in flat framework will run essential stuff regardless of mode

    if (m_controller.getRawButton(1)) // [xbox button A]
      m_gyro.reset();

    SmartDashboard.putNumber("Z axis Rot", m_gyro.getAngleZ());
    // invert value so that true means sensor detects object in front
    SmartDashboard.putBoolean("frontSensor", !frontIRsensor.get());

    // in teleOp, this forces drive straight, no manual turn possible
    // not working ?; piDriv is calculating angle error 5x slower than
    // control loop so I need to apply .calc w/ smaller dX than normal
    // if (m_controller.getRawButton(6)) { // Rt bumper press
    // m_Drive.arcadeDrive(-m_controller.getLeftY() * 0.6,
    // -piDriv.calculate(m_gyro.getAngleZ(), 0) * 0.05, false);
    // } // probably better control w/ small feedbk numbers if not squared

    CommandScheduler.getInstance().run();
  } // end robotPeriodic

  // autoInit gets the autonomous command name set by SmartDashbd
  @Override
  public void autonomousInit() {
    // RC got selected auto from SmartDashboard as CMD, I just get String
    m_autoSelected = m_chooser.getSelected();
    m_leftEncoder.reset();
    m_rightEncoder.reset();
    m_gyro.reset();
    goBack = false;
    SmartDashboard.putNumber("distance", 0.0);
    mTimer = new Timer();
    twoTimer = new Timer();
  } // end autoInit

  /**
   * This function is called periodically during autonomous.pid param only
   * guaranteed for 1st auto run
   */
  @Override
  public void autonomousPeriodic() {
    double distGoal = 36; // inch
    double distNow = (m_leftEncoder.getDistance() +
        m_rightEncoder.getDistance()) / 2;
    SmartDashboard.putNumber("distance", distNow);

    switch (m_autoSelected) { // sequence built by changing this var
      case "RETURN": // D&T end angle too varied for return case to get called
        goBack = true; // repeat wanted
        m_autoSelected = "DRIV&TURN";
        break;
      case "DRIV&TURN": // then stop
        // drive straight distance, angle(0), w/ 2 PID controllers
        m_Drive.arcadeDrive(piDist.calculate(distNow, distGoal) * 0.075,
            -piDriv.calculate(m_gyro.getAngleZ(), 0) * 0.035,
            false);

        if (piDist.atSetpoint()) {
          m_Drive.arcadeDrive(0, 0);
          // new WaitCommand(4); fails here
          mTimer.start(); // simulate wait cmd
          m_gyro.reset(); // should keep @ setpt 0

          if (mTimer.get() > 4.0) { // unclear why different param needed for pidTurn here
            pidTurn = new PIDController(0.032, 0.000015, kTurnD, kPIDperiod);
            pidTurn.enableContinuousInput(-180, 180);
            pidTurn.setTolerance(2);
            m_autoSelected = "TURN180";
            mTimer.stop();
          }
        } // end at setpt
        break;
      case "TURN180":
        // Pipe output to turn bot in place, should turn CCW for (-) setpoint
        // to square / not? ; div increment to correct for slower error update
        m_Drive.arcadeDrive(0, -pidTurn.calculate(m_gyro.getAngleZ(), 179) * 0.03, false);
        if (pidTurn.atSetpoint()) {
          m_Drive.arcadeDrive(0, 0);
          twoTimer.start(); // loop doesn't restart it if running
          // stop turn, simulate wait command after 1st 180 turn
        } // end @ set point

        if (twoTimer.get() > 4.0) {
          // mTimer.reset(); // rezeros, doesn't restart it

          if (goBack) {
            m_gyro.reset();
            m_leftEncoder.reset();
            m_rightEncoder.reset();
            m_autoSelected = "DRIV&TURN"; // do it again
            goBack = false; // don't do again after that
            break;
          } else { // goBk false, so trying to end aP
            m_autoSelected = "endMe";
            break;
          } // end else
        } // end timer > 3

        break;

      case "endMe": // trys to end aP
        new PrintCommand("StoppingAuto");
        this.teleopInit();
        // ? does not stop aP, does not advance to teleopInit
        break;
      default:
        break;
    } // end switch

    // neither stopIt.cancel() or
    // CommandScheduler.getInstance().cancelAll() work to
  } // end autoPeriod so tI can begin

  @Override
  public void teleopInit() {
    // This confirms that the autonomous code has stopped,. If you want
    // auto cmd to continue until interrupted by another command, remove
    // this line or comment it out.
    // if (m_autoSelected != null) { // if auto never activated, don't do resets
    // m_autoSelected.cancel(); // can't cancel a string
    m_leftEncoder.reset();
    m_rightEncoder.reset();
    m_gyro.reset();
    // } // end if
  } // end teleInit

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() { // need gyroPID
    // to drive straight in teleop, just this drifts to L badly
    m_Drive.arcadeDrive(-m_controller.getLeftY() * 0.45,
        -m_controller.getRightX() * 0.35, true);
  }

  // This function is called once each time the robot enters Disabled mode.
  @Override
  public void disabledInit() {
  }

  @Override // incessant (-) drift, so to reset manually in disabled,
            // you can press button 1 [xbox button A]:
  public void disabledPeriodic() { // works from rP, no need to repeat
    // if (m_controller.getRawButton(1))
    // m_gyro.reset();
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    // CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }
} // end class
