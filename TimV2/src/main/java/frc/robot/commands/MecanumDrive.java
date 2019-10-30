package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class MecanumDrive extends Command {
    public MecanumDrive() {
        requires(Robot.driveTrain);
    }

    protected void initialize() {
        execute();
    }

    protected void execute() {
        //TODO: Map input properly (rate limiting/dead zone/scaling/etc)
        Robot.driveTrain.mecanumDrive(Robot.m_oi.driver.getY(Hand.kLeft),Robot.m_oi.driver.getX(Hand.kLeft), Robot.m_oi.driver.getX(Hand.kRight));
    }

    protected boolean isFinished() {
        return false;
    }
}