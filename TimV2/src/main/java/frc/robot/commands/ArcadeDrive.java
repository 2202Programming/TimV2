package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;

public class ArcadeDrive extends Command {
    public ArcadeDrive() {
        requires(Robot.driveTrain);
    }

    protected void initialize() {
        execute();
    }

    protected void execute() {
        //TODO: Map input properly (rate limiting/dead zone/scaling/etc)
        Robot.driveTrain.arcadeDrive(Robot.m_oi.driver.getY(Hand.kLeft), Robot.m_oi.driver.getX(Hand.kLeft));
    }

    protected boolean isFinished() {
        return false;
    }
}