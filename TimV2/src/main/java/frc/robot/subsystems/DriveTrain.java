package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.RobotMap;

public class DriveTrain extends Subsystem {

    private Talon frontLeftMotor = new Talon(RobotMap.FL_TALON_CH);
    private Talon frontRightMotor = new Talon(RobotMap.FR_TALON_CH);
    private Talon backLeftMotor = new Talon(RobotMap.BL_TALON_CH);
    private Talon backRightMotor = new Talon(RobotMap.BR_TALON_CH);
    private MecanumDrive drive;

    public DriveTrain() {
        drive = new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new frc.robot.commands.MecanumDrive());
    }

    public void mecanumDrive(double y, double x, double rotation) {
        drive.driveCartesian(y, x, rotation);
    }

    public void stop() {
        drive.stopMotor();
    }

}