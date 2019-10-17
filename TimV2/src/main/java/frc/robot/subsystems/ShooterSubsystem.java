
/* This is the CPP file for Shooter Control 
 * Current Version:
 * Hold Right Trigger for Shooting
 * Press X to toggle lift motor
 * Press LBumper to increment speed
 * */
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.*;

public class ShooterSubsystem extends Subsystem {

    // private ShooterSubsystem shooterSubsystem = null;

    /*
     * ShooterControl *ShooterControl::getInstance() { if (shootercontrol == 0) {
     * shootercontrol = new ShooterControl(); } return shootercontrol; }
     */
    XboxController xbox;

    Victor angleMotor;

    Talon shooterMotor1, shooterMotor2;
    double shooter1Speed = 0.0, shooter2Speed = 0.0;
    DigitalInput upperLimit, lowerLimit;
    final double shooterStartSpeed = .4;
    boolean isLoadingPosition = false;
    boolean isFiringPosition = false;
    double angle;

    public void initDefaultCommand() {
        return;
    }

    public ShooterSubsystem() {

        xbox = new XboxController(3);

        angleMotor = new Victor(RobotMap.SHOOTER_ANGLE_MOTOR_PORT);

        shooterMotor1 = new Talon(RobotMap.SHOOTER_MOTOR_PORT_1);
        shooterMotor2 = new Talon(RobotMap.SHOOTER_MOTOR_PORT_2);

        upperLimit = new DigitalInput(RobotMap.UPPER_LIMIT_PORT);
        lowerLimit = new DigitalInput(RobotMap.LOWER_LIMIT_PORT);

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
            shooter1Speed += RobotMap.SHOOTER_SPEED_STEP;
            if (shooter1Speed > 1)
                shooter1Speed = 0;
        }
        boolean isRBumperPressed = xbox.getBumper(GenericHID.Hand.kRight);
        if (isRBumperPressed) {
            shooter2Speed += RobotMap.SHOOTER_SPEED_STEP;
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
        boolean isAPressed = xbox.getAButton();
        if (isAPressed) {
            stopMotors();
            isLoadingPosition = false;
            isFiringPosition = false;
        }
    }

    public void shooterCycleBehindSpeed() {

        boolean isLBumperPressed = xbox.getBumper(GenericHID.Hand.kLeft);

        // if left bumper is pressed reduce the speed for both motors
        if (isLBumperPressed) {
            shooter2Speed = (shooter2Speed -= RobotMap.SHOOTER_SPEED_STEP) < 0 ? 0 : shooter2Speed;
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

            shooter2Speed = (shooter2Speed += RobotMap.SHOOTER_SPEED_STEP > 1.0 ? 1 : shooter2Speed);

            // if shooter speed is equal to first speed increment (.25) jump to shooter
            // start speed (.4).
            if (shooter2Speed == RobotMap.SHOOTER_SPEED_STEP)
                shooter2Speed = shooterStartSpeed;

            // Both variables are set to the same speed, but we are using two variables
            // incase we want to set two different values.
            // From final code extra variable can be removed
        }
        if (shooter2Speed < .3) {
            shooter1Speed = shooter2Speed;
        } else {
            shooter1Speed = shooter2Speed - RobotMap.SHOOTER_SPEED_STEP;
        }
        setMotors();
    }

    public void shooterCycleSpeed() {

        boolean isLBumperPressed = xbox.getBumper(GenericHID.Hand.kLeft);

        // if left bumper is pressed reduce the speed for both motors
        if (isLBumperPressed) {
            shooter1Speed = (shooter1Speed -= RobotMap.SHOOTER_SPEED_STEP) < 0 ? 0 : shooter1Speed;
            // if shooter speed is less than shooterstart speed. stop
            if (shooter1Speed < shooterStartSpeed)
                shooter1Speed = 0;

            // Both variables are set to the same speed, but we are using two variables
            // incase we want to set two different values.
            // From final code extra variable can be removed
            shooter2Speed = shooter1Speed;
        }

        // if right bumper is pressed increase the speed for both motors
        bool isRBumperPressed = xbox.getBumper(GenericHID.Hand.kRight);

        if (isRBumperPressed) {

            shooter1Speed = (shooter1Speed += RobotMap.SHOOTER_SPEED_STEP > 1.0 ? 1 : shooter1Speed);

            // if shooter speed is equal to first speed increment (.25) jump to shooter
            // start speed (.4).
            if (shooter1Speed == RobotMap.SHOOTER_SPEED_STEP)
                shooter1Speed = shooterStartSpeed;

            // Both variables are set to the same speed, but we are using two variables
            // incase we want to set two different values.
            // From final code extra variable can be removed
            shooter2Speed = shooter1Speed;
        }
        setMotors();
    }

    public boolean isLowestAngle() {
        return !lowerLimit.get();
    }

    // this method sets the angle of the shooter using a motor. elevation is
    // increased when right trigger is pressed, and it is decreased
    public void shooterAngle(double angleDirection) {
        // rightAngle = xbox->getAxisRightY();
        boolean upperOn = !upperLimit.get();
        boolean lowerOn = !lowerLimit.get();

        // if we are hitting the limit, cancel the direction so that we don't move the
        // motor
        if (lowerOn) {
            if (angleDirection < 0) {
                angleDirection = 0;
                isLoadingPosition = false;
            }
        }

        if (upperOn) {
            if (angleDirection > 0)
                angleDirection = 0;
            isFiringPosition = false;
        }

        // check for dead zone i.e +- 1. move motor if beyond dead zone
        if (Math.abs(angleDirection) > .1) {
            AngleMotor.set(-1 * angleDirection);
            Angle += angleDirection;
        } else {
            AngleMotor.set(0.0);
        }
    }

bool ShooterControl::isRunning() {
	float speed = shooterMotor1->Get();
	if (speed == 0)
		return false;
	else
		return true;

}

void ShooterControl::SetShooterMotors(float speed) {
	shooterMotor1->Set(speed);
	shooterMotor2->Set(speed);
}

float ShooterControl::getAngle() {
	// use this for now
	return maxAngleReached();
}

bool ShooterControl::maxAngleReached() {
	return !upperLimit->Get();
}

    // press right trigger to shoot
void ShooterControl::run() {
	APushStop();
	//ShooterSeperateCycleSpeed();float axisY = xbox->getAxisRightY();
	goTo();
	float axisY = xbox->getAxisRightY();
	if (fabs(axisY) > .2) {
		isLoadingPosition = false;
		isFiringPosition = false;
	}
	if (isLoadingPosition)//loading position
	{
		ShooterAngle(SHOOTERLOADINGDIRECTION);
		Shooter1Speed = SHOOTERLOADINGSPEED;
		Shooter2Speed = SHOOTERLOADINGSPEED;
	} else if (isFiringPosition) {
		ShooterAngle(SHOOTERFIRINGDIRECTION);
		Shooter1Speed = SHOOTERFIRINGSPEED;
		Shooter2Speed = SHOOTERFIRINGSPEED;
	} else {

		//ShooterCycleBehindSpeed();
		ShooterAngle(axisY);
		if (xbox->isXPressed()) {
			Shooter2Speed = DEFAULTSHOOTERSPEED;
			Shooter1Speed = DEFAULTSHOOTERSPEED;
		}
	}
	ShooterCycleSpeed();
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
void ShooterControl::goTo() {
	if (xbox->isYPressed()) {
		isLoadingPosition = true;
	} else if (xbox->isBPressed()) {
		isFiringPosition = true;
	}

}
}