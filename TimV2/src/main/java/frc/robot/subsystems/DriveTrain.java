package frc.robot.subsystems;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.commands.MecanumDrive;
import frc.robot.RobotMap;

public class DriveTrain extends Subsystem {

    private Talon frontLeftMotor = new Talon(RobotMap.FL_TALON_CH);
    private Talon frontRightMotor = new Talon(RobotMap.FR_TALON_CH);
    private Talon backLeftMotor = new Talon(RobotMap.BL_TALON_CH);
    private Talon backRightMotor = new Talon(RobotMap.BR_TALON_CH);
    private RobotDrive drive;

    public DriveTrain() {
        drive = new RobotDrive(frontLeftMotor, backLeftMotor, frontRightMotor, backRightMotor);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new MecanumDrive());
    }

    public void mecanumDrive(double x, double y, double rotation) {
        drive.mecanumDrive_Cartesian(x, y, rotation, 0);
    }

    public void stop() {
        drive.stopMotor();
    }

}