package frc.robot.subsystems;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.RobotMap;

public class LiftSubsystem extends Subsystem {

    /*
     * This is the CPP file for Lift Control Current Version: B Button hold to lift.
     * No reverse.
     */

    // static LiftControl *liftcontrol = NULL;
    // LiftControl *LiftControl::getInstance() {
    // if (liftcontrol == NULL) {
    // liftcontrol = new LiftControl();
    // }
    // return liftcontrol;
    // }

    boolean isGoingUp;
    XboxController xbox;
    DriverStation driverStation;
    Solenoid solenoidTop, solenoidBot;
    ShooterSubsystem shooterSubsystem;

    public LiftSubsystem() {
        isGoingUp = true;
        xbox = new XboxController(RobotMap.XBOX_CONTROLLER_PORT);
        driverStation = DriverStation.getInstance();
        solenoidTop = new Solenoid(RobotMap.SOLENOID_MODULE, RobotMap.TOP_CHANNEL);
        solenoidBot = new Solenoid(RobotMap.SOLENOID_MODULE, RobotMap.BOT_CHANNEL);
        shooterSubsystem = new ShooterSubsystem();
    }

    @Override
    public void initDefaultCommand() {
    }

    public void initialize() {
        solenoidTop.set(isGoingUp);
        solenoidBot.set(!isGoingUp);
    }

    public void initializeAutonomous() {
    }

    /**
     * If B is pressed for the first time, the solenoids will go into the up
     * position. Then the bool isGoingUp will be reversed, so the next time B is
     * pressed, the solenoids will go into the down position. Cool.
     */
    public void run() {
        boolean isBPressed = xbox.getBButton();
        if (shooterSubsystem.isLowestAngle()) {
            if (isBPressed) {
                solenoidTop.set(!isGoingUp);
                solenoidBot.set(isGoingUp);
                isGoingUp = !isGoingUp;
            }
        }
    }

    public void runAutonomous() {
    }
}