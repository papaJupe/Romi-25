// ROMI GYRO DRIV pid - C 25                RobotContainer.j

 // rewrite of PID C 24: purging former Cmd, all now inline in drive
 // subsys, controlled from button trigger or auto cmd; straight
 // drive auto improved by rot. tweak; PID improved by slower period;
 // PID profile and regular work OK, not perfectly tuned

package frc.robot;

//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

// import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsys;
import frc.robot.subsystems.OnBoardIO;
import frc.robot.subsystems.OnBoardIO.ChannelMode;

/*  The specifics of operating this robot (subsystem,
 *  command [1 default], and button mapping) are put here.
 */
public class RobotContainer {
        // instance the two subsystems
        public final DriveSubsys m_drive = new DriveSubsys();
        private final OnBoardIO m_onboardIO = new OnBoardIO(ChannelMode.INPUT,
                        ChannelMode.INPUT);

        // instance joystick @ 0 --assumes controller plugged into USB 0 port
        // numerous get()s in AD cmd require public stick <-- still ?;? why static
        public static final XboxController m_controller = new XboxController(0);

        // allows SmartDashboard to pick autonomous routine
        private final SendableChooser<Command> m_chooser = new SendableChooser<>();

        // N.B. I/O pin function config possible in web interface; v. base code

        /*
         * CONSTRUCT 'container' for robot : former method, configBB() not
         * needed (mis-named anyway) -- its inits moved here. they include:
         * Drivetrain [subsystem] default Cmd, OperatorInterface (OI)
         * trigger, Smart Dashbd Auton chooser + its 3 cmd option.
         * --- these define & init the specific controls of this robot
         */
        public RobotContainer() {
                // orig. RC only contained configureButtonBindings() for basic init;
                // phased out formal Cmd class used before -->
                // m_drive.setDefaultCommand(new ArcadeDrive(
                // now, to avoid formal AD Command class, just lambda + subsys method
                m_drive.setDefaultCommand(new RunCommand(  // ? use method refs instead
                        () -> m_drive.arcaDriv(-m_controller.getLeftY(), m_controller.getRightX()), m_drive));  

                // Example of onboard romi IO buttons doing something
                Trigger onboardButtonA = new Trigger(m_onboardIO::getButtonAPressed);
                onboardButtonA
                                .whileTrue(new PrintCommand("onbord A Press"))
                                .whileFalse(new PrintCommand("onbord A Release"));

                // button A (1) reset Gyro (to 0), same done more simply in rP
                // new JoystickButton(m_controller, 1)
                // .onTrue(new InstantCommand(() -> setGyro()))
                // .onTrue(new PrintCommand("Pad A Press"));

                // in v. B pressing during auto activated gyro stabiliz
                // new JoystickButton(m_controller, 6)
                // .onTrue(new InstantCommand(() -> tGyroMode(true)))
                // .onFalse(new InstantCommand(() -> tGyroMode(false)));

                // Drive at reduced speed when the left bumper is held
                new JoystickButton(m_controller, 5)
                        .onTrue(new InstantCommand(() -> m_drive.setMaxSpeed(0.5)))
                        .onFalse(new InstantCommand(() -> m_drive.setMaxSpeed(0.7)));

                // Turn to -90 degrees CW when the 'B' button is pressed, ? 5 sec timeout
                new JoystickButton(m_controller, 2)
                        .onTrue(m_drive.turnToAngleCmd(-90));

                // Turn to +90 degrees CCW with profile when the X button is pressed,
                // ? 5 second timeout
                new JoystickButton(m_controller, 3)
                                .onTrue(m_drive.turnToAngleProfCommand(90));

                // config SmartDashboard auto options
                m_chooser.setDefaultOption("Auto Turn 90", m_drive.turnToAngleProfCommand(90)); 
                m_chooser.addOption("DriveDistance", m_drive.driveDistanceCmd(32, 0.6));
                m_chooser.addOption("Auton Sequen", m_drive.AutoSequComm(32.0,180.0));

                SmartDashboard.putData(m_chooser);

        } // end RC constructor

        // // passes selected auto command to the Robot.j class to schedule
        // // the one & only RC method
        public Command getAutonomousCommand() {
                return m_chooser.getSelected();
        } // end getAutoCmd

        //from rapid react RC   
//   public Command getAutonomousCommand() {
//         // Drive forward for 2 meters at half speed with a 3 second timeout
//         return m_drive
//             .driveDistanceCommand(AutoConstants.kDriveDistanceMeters, AutoConstants.kDriveSpeed)
//             .withTimeout(AutoConstants.kTimeoutSeconds);
//       }

} // end RC class
