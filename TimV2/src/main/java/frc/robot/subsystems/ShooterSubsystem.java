package frc.robot.subsystems;

/* This is the CPP file for Shooter Control 
 * Current Version:
 * Hold Right Trigger for Shooting
 * Press X to toggle lift motor
 * Press LBumper to increment speed
 * */
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.*;

public class ShooterSubsystem extends Subsystem {

    // private ShooterSubsystem shooterSubsystem = null;

    /*
     * ShooterControl *ShooterControl::getInstance() { if (shootercontrol == 0) {
     * shootercontrol = new ShooterControl(); } return shootercontrol; }
     */
    XboxController xbox;

    private DoubleSolenoid angleSolenoid;
    private Talon shooterMotor1, shooterMotor2;
    private DoubleSolenoid launcherSolenoid;
    private double shooter1Speed = 0.0, shooter2Speed = 0.0;
    private final double shooterStartSpeed = .4;
    private boolean isLoadingPosition = false;
    private boolean isFiringPosition = false;
    private double angle;
    /*
        SHOOTERMOTORPORT1 5
        SHOOTERMOTORPORT2 6
        SHOOTERANGLEMOTORPORT 7
        SHOOTERSPEEDSTEP .2
        UPPERLIMITPORT 7
        LOWERLIMITPORT 6
        ANGLEMOTORLIFTSPEED 0.3
        SHOOTERSPEEDINCREMENTRESETPT 0.0
        AUTOSPEED .2
        SHOOTERSPEEDSTEPDIFF .1
        DEFAULTSHOOTERSPEED 1.0
        SHOOTERLOADINGSPEED 0.0
        SHOOTERFIRINGSPEED .99
        SHOOTERLOADINGDIRECTION -1.0 // these are for raising and lowering
        SHOOTERFIRINGDIRECTION .99
    */
    final double SHOOTER_SPEED_STEP = 0.25;
    final double SHOOTER_LOADING_SPEED = 0.0;
    final double SHOOTER_LOADING_DIRECTION = -1;
    final double SHOOTER_FIRING_SPEED = 0.99;
    final double SHOOTER_FIRING_DIRECTION = 0.99;
    final double DEFAULT_SHOOTER_SPEED = 1;

    public void initDefaultCommand() {
    }

    public ShooterSubsystem() {

        xbox = new XboxController(1);

        angleSolenoid = new DoubleSolenoid(RobotMap.SHOOTER_UP_SOL, RobotMap.SHOOTER_DOWN_SOL);

        shooterMotor1 = new Talon(RobotMap.SHOOTER_MOTOR_PORT_1);
        shooterMotor2 = new Talon(RobotMap.SHOOTER_MOTOR_PORT_2);

        launcherSolenoid = new DoubleSolenoid(RobotMap.LAUNCHER_EXTEND_SOL, RobotMap.LAUNCHER_RETRACT_SOL);

    }

    public void initialize() {
        angle = 0.0;
        stopMotors();
    }

    public void setMotors() {
        shooterMotor1.set(shooter1Speed);
        shooterMotor2.set(shooter2Speed);
    }

    // this method cycles though the shooter speeds in 4 steps
    public void shooterSeperateCycleSpeed() {
        boolean isLBumperPressed = xbox.getBumper(GenericHID.Hand.kLeft);
        if (isLBumperPressed) {
            shooter1Speed += SHOOTER_SPEED_STEP;
            if (shooter1Speed > 1)
                shooter1Speed = 0;
        }
        boolean isRBumperPressed = xbox.getBumper(GenericHID.Hand.kRight);
        if (isRBumperPressed) {
            shooter2Speed += SHOOTER_SPEED_STEP;
            if (shooter2Speed > 1)
                shooter2Speed = 0;
        }
        setMotors();
    }

    public void stopMotors() {
        shooter1Speed = 0;
        shooter2Speed = 0;
        setMotors();
    }

    public void aPushStop() {
        if (xbox.getAButton()) {
            stopMotors();
            isLoadingPosition = false;
            isFiringPosition = false;
        }
    }

    public void shooterCycleBehindSpeed() {

        boolean isLBumperPressed = xbox.getBumper(GenericHID.Hand.kLeft);

        // if left bumper is pressed reduce the speed for both motors
        if (isLBumperPressed) {
            shooter2Speed = (shooter2Speed -= SHOOTER_SPEED_STEP) < 0 ? 0 : shooter2Speed;
            // if shooter speed is less than shooterstart speed. stop
            if (shooter2Speed < shooterStartSpeed)
                shooter2Speed = 0;
            // Both variables are set to the same speed, but we are using two variables
            // incase we want to set two different values.
            // From final code extra variable can be removed
        }

        // if right bumper is pressed increase the speed for both motors
        boolean isRBumperPressed = xbox.getBumper(GenericHID.Hand.kRight);

        if (isRBumperPressed) {

            shooter2Speed = (shooter2Speed += SHOOTER_SPEED_STEP > 1.0 ? 1 : shooter2Speed);

            // if shooter speed is equal to first speed increment (.25) jump to shooter
            // start speed (.4).
            if (shooter2Speed == SHOOTER_SPEED_STEP)
                shooter2Speed = shooterStartSpeed;

            // Both variables are set to the same speed, but we are using two variables
            // incase we want to set two different values.
            // From final code extra variable can be removed
        }
        if (shooter2Speed < .3) {
            shooter1Speed = shooter2Speed;
        } else {
            shooter1Speed = shooter2Speed - SHOOTER_SPEED_STEP;
        }
        setMotors();
    }

    public void shooterCycleSpeed() {

        boolean isLBumperPressed = xbox.getBumper(GenericHID.Hand.kLeft);

        // if left bumper is pressed reduce the speed for both motors
        if (isLBumperPressed) {
            shooter1Speed = (shooter1Speed -= SHOOTER_SPEED_STEP) < 0 ? 0 : shooter1Speed;
            // if shooter speed is less than shooterstart speed. stop
            if (shooter1Speed < shooterStartSpeed)
                shooter1Speed = 0;

            // Both variables are set to the same speed, but we are using two variables
            // incase we want to set two different values.
            // From final code extra variable can be removed
            shooter2Speed = shooter1Speed;
        }

        // if right bumper is pressed increase the speed for both motors
        boolean isRBumperPressed = xbox.getBumper(GenericHID.Hand.kRight);

        if (isRBumperPressed) {

            shooter1Speed = (shooter1Speed += SHOOTER_SPEED_STEP > 1.0 ? 1 : shooter1Speed);

            // if shooter speed is equal to first speed increment (.25) jump to shooter
            // start speed (.4).
            if (shooter1Speed == SHOOTER_SPEED_STEP)
                shooter1Speed = shooterStartSpeed;

            // Both variables are set to the same speed, but we are using two variables
            // incase we want to set two different values.
            // From final code extra variable can be removed
            shooter2Speed = shooter1Speed;
        }
        setMotors();
    }

    // this method sets the angle of the shooter using a solenoid.
    // erect is up
    public void setErect(boolean erect) {
        if (erect) {
            angleSolenoid.set(Value.kForward);
        } else {
            angleSolenoid.set(Value.kReverse);
        }
    }

    public boolean isRunning() {
        return shooterMotor1.get() == 0;
    }

    public void SetShooterMotors(float speed) {
        shooterMotor1.set(speed);
        shooterMotor2.set(speed);
    }

    // press right trigger to shoot
    public void run() {
        aPushStop();
        // ShooterSeperateCycleSpeed();float axisY = xbox->getAxisRightY();
        goTo();
        double axisY = xbox.getY(GenericHID.Hand.kRight);
        if (Math.abs(axisY) > .2) {
            isLoadingPosition = false;
            isFiringPosition = false;
        }
        if (isLoadingPosition)// loading position
        {
            setErect(false);
            shooter1Speed = SHOOTER_LOADING_SPEED;
            shooter2Speed = SHOOTER_LOADING_SPEED;
        } else if (isFiringPosition) {
            setErect(true);
            shooter1Speed = SHOOTER_FIRING_SPEED;
            shooter2Speed = SHOOTER_FIRING_SPEED;
        } else {

            // ShooterCycleBehindSpeed();
            // why shooterAngle(axisY);
            if (xbox.getXButtonPressed()) {
                shooter2Speed = DEFAULT_SHOOTER_SPEED;
                shooter1Speed = DEFAULT_SHOOTER_SPEED;
            }
        }
        shooterCycleSpeed();
    }

    // void ShooterControl::runAutonomous() {
    // shooterMotor1->Set(AUTOSPEED);
    // shooterMotor2->Set(AUTOSPEED);
    //
    // }
    /*
     * if Y is pressed, goes to loading position, turns motors off If B is pressed,
     * goes to firing position, turns motors on
     */
    public void goTo() {
        if (xbox.getYButtonPressed()) {
            isLoadingPosition = true;
        } else if (xbox.getBButtonPressed()) {
            isFiringPosition = true;
        }
    }
}