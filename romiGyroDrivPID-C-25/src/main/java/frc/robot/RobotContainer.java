// ROMI GYRODRIVpid - C  24                RobotContainer.j

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsys;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutonSequen;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.TurnToAngleProf;
import frc.robot.subsystems.OnBoardIO;
import frc.robot.subsystems.OnBoardIO.ChannelMode;

/* RC is where robot specifics are defined. Since Command-based is a
 * "declarative" paradigm, very little robot logic should be handled in
 * the {@link Robot} periodic methods other than the scheduler calls.
 * Instead, the specifics of operating the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // instance the two subsystems
    protected final DriveSubsys m_drivetrain = new DriveSubsys();
    private final OnBoardIO m_onboardIO = new OnBoardIO(ChannelMode.INPUT,
                                          ChannelMode.INPUT);

    // instance joystick @ 0 --assumes controller plugged into USB 0
    // numerous get()s in AD cmd require public stick
    public static final XboxController m_controller = 
                                      new XboxController (0);

    // allows SmartDashboard to pick autonomous routine
    private final SendableChooser<Command> m_chooser = new SendableChooser<>();

    // N.B. I/O pin function config possible in web interface; v. base code

    /**
     * CONSTRUCT container for robot: its single method, configBB() sets
     * Drivetrain [subsystem's] default Cmd, OperatorInterface (OI)
     * action, Smart Dashbd Autonomous chooser options.
     * --- defines the specifics of this robot
     */
    public RobotContainer() {
        // Configure joystick button bindings et. al.
        configureButtonBindings();
    } // end constructor

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by instantiating a {GenericHID} or one of its subclasses
     * edu.wpi.first.wpilibj.Joystick} or {XboxController}, and then passing
     * it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command is ArcadeDrive. Runs unless another command
        // is scheduled over it.(e.g. runs in teleOp unless overridden)
        // [orig.] m_drivetrain.setDefaultCommand(getArcadeDriveCommand());
        // this has less obscure syntax (v.i.), simpler constructor
        m_drivetrain.setDefaultCommand(new ArcadeDrive(m_drivetrain));

        // m_robotDrive.setDefaultCommand(
        // // A split-stick arcade command, with forward/backward controlled
        // // by the left
        // // hand, and turning controlled by the right.
        // new RunCommand( () -> m_robotDrive.arcadeDrive(
        // -m_driverController.getLeftY(), -m_driverController.getRightX()),
        // m_robotDrive));

        // Example of onboard IO buttons doing something
        Trigger onboardButtonA = new Trigger(m_onboardIO::getButtonAPressed);
        onboardButtonA
                .whileTrue(new PrintCommand("onbord A Press"))
                .whileFalse(new PrintCommand("onbord A Release"));

        // button A (1) resets Gyro (to 0), now done more simply in rP
        // new JoystickButton(m_controller, 1)
        // .onTrue(new InstantCommand(() -> m_drivetrain.resetGyro()))
        // .onTrue(new PrintCommand("Pad A Press"));

        // Stabilize to drive straight w/ gyro when R bumper is held
        // in teleOp, bypassing AD cmd; Auto now has its own PID cmds
        new JoystickButton(m_controller, 6)
                .whileTrue(
                        new PIDCommand(
                                new PIDController(
                                        DriveConstants.kStabilizP,
                                        DriveConstants.kStabilizI,
                                        DriveConstants.kStabilizD),
                                // Close the loop on the turn rate
                                m_drivetrain::getGyroAngleZ,
                                // Setpoint is 0 for straight driving
                                0,
                                // Pipe output to turn param of _P mod method
                                output -> m_drivetrain.arcaDrivP(-m_controller.
                                        getLeftY() * 0.7, output), // unsquared
                                // Require the robot drive
                                m_drivetrain));

        // in v. B pressing during auto activated gyro stabiliz differently
        // new JoystickButton(m_controller, 6)
        // .onTrue(new InstantCommand(() -> m_drivetrain.setGyroMode(true)))
        // .onFalse(new InstantCommand(() -> m_drivetrain.setGyroMode(false)));

        // Drive at reduced speed when the left bumper is held
        new JoystickButton(m_controller, 5)
                .onTrue(new InstantCommand(() -> 
                                   m_drivetrain.setMaxOutput(0.7)))
                .onFalse(new InstantCommand(() -> 
                                   m_drivetrain.setMaxOutput(1.0)));

        // Turn to -90 degrees when the 'X' button is pressed, 5 second timeout
        new JoystickButton(m_controller, 3)
                .onTrue(new TurnToAngle(-90,
                        m_drivetrain).withTimeout(5));

        // Turn to +90 degrees with profile when the B button is pressed,
        // with a 5 second timeout
        new JoystickButton(m_controller, 2)
                .onTrue(new TurnToAngleProf(90, m_drivetrain).withTimeout(5));

        // Setup SmartDashboard options
        m_chooser.setDefaultOption("Auto Turn 90", new TurnToAngle(90, m_drivetrain));
        m_chooser.addOption("Auton Sequen", new AutonSequen(m_drivetrain));

        SmartDashboard.putData(m_chooser);

    } // end configBB()

    // ... passes selected auto command to the scheduling Robot.j class
    // @return the command to run in autonomous
    public Command getAutonomousCommand() {
        return m_chooser.getSelected();
    } // end get.AutoCmd

} // end class
