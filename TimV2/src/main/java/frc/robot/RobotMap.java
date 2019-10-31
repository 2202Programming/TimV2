/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // For example to map the left and right motors, you could define the
  // following variables to use with your drivetrain subsystem.
  // public static int leftMotor = 1;
  // public static int rightMotor = 2;

  // If you are using multiple modules, make sure to define both the port
  // number and the module. For example you with a rangefinder:
  // public static int rangefinderPort = 1;
  // public static int rangefinderModule = 1;
  public static int FL_TALON_CH = 2;
  public static int FR_TALON_CH = 1;
  public static int BL_TALON_CH = 4;
  public static int BR_TALON_CH = 3;
  public static final int SHOOTER_MOTOR_PORT_1 = 5;
  public static final int SHOOTER_MOTOR_PORT_2 = 6;
  public static final int UPPER_LIMIT_PORT = 7;
  public static final int LOWER_LIMIT_PORT = 6;
  public static final int XBOX_CONTROLLER_PORT = 3;
  public static final int SOLENOID_MODULE = 1;
  public static final int TOP_CHANNEL = 1;
  public static final int BOT_CHANNEL = 2;

}
