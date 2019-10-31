package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
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
    private DifferentialDrive drive2;

    public DriveTrain() {
        drive = new MecanumDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
        frontLeftMotor.setInverted(true);
        frontRightMotor.setInverted(true);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new frc.robot.commands.MecanumDrive());
    }

    public void mecanumDrive(double y, double x, double rotation) {
        //axes are messed up for some reason
        drive.driveCartesian(y, x, rotation);
    }

    public void arcadeDrive(double x, double z) {
        drive2.arcadeDrive(x, z);
    }

    public void stop() {
        drive.stopMotor();
    }

}