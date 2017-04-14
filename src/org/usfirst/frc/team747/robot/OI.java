package org.usfirst.frc.team747.robot;

//import org.usfirst.frc.team747.robot.commands.ClimberClimbUpFastCommand;
//import org.usfirst.frc.team747.robot.commands.ClimberClimbUpSlowCommand;
//import org.usfirst.frc.team747.robot.commands.ShooterShootCommand;
//import org.usfirst.frc.team747.robot.commands.SimpleVisionDriveCommand;
//import org.usfirst.frc.team747.robot.commands.VisionDriveCommand;
//import org.usfirst.frc.team747.robot.commands.DriveDistanceCommand;
//import org.usfirst.frc.team747.robot.commands.IntakeCommand;
//import org.usfirst.frc.team747.robot.commands.ShooterIndexerReverseCommand;
//import org.usfirst.frc.team747.robot.commands.ShooterReverseCommand;
//import org.usfirst.frc.team747.robot.commands.ShootButton;
//import org.usfirst.frc.team747.robot.commands.IndexerReverseButton;
import org.usfirst.frc.team747.robot.commands.*;
import org.usfirst.frc.team747.robot.maps.DriverStation;
//import org.usfirst.frc.team869.robot.RobotMap;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {

    public static final Joystick JOYSTICK_DRIVER_LEFT = new Joystick(DriverStation.Controller.DRIVER_LEFT.getValue()),
            JOYSTICK_DRIVER_RIGHT = new Joystick(DriverStation.Controller.DRIVER_RIGHT.getValue()),
            CONTROLLER_OPERATOR = new Joystick(DriverStation.Controller.OPERATOR.getValue());

    public static final JoystickButton BUTTON_DRIVE_SLOW_DRIVER
                = new JoystickButton(JOYSTICK_DRIVER_LEFT, DriverStation.Joystick.BUTTON_7.getValue()),
            BUTTON_REV_SHOOTER
                = new JoystickButton(CONTROLLER_OPERATOR, DriverStation.GamePad.BUTTON_RB.getValue()),
            BUTTON_REVERSE_SHOOTER
                = new JoystickButton(CONTROLLER_OPERATOR, DriverStation.GamePad.BUTTON_LB.getValue()),
            BUTTON_INTAKE_FORWARD
                = new JoystickButton(CONTROLLER_OPERATOR, DriverStation.GamePad.BUTTON_A.getValue()),
            BUTTON_INTAKE_BACK
                = new JoystickButton(CONTROLLER_OPERATOR, DriverStation.GamePad.BUTTON_B.getValue()),
            BUTTON_DRIVE_DISTANCE
                = new JoystickButton(CONTROLLER_OPERATOR, DriverStation.GamePad.BUTTON_START.getValue()),
            BUTTON_CLIMB_FAST
                = new JoystickButton(CONTROLLER_OPERATOR, DriverStation.GamePad.BUTTON_X.getValue()),
            BUTTON_CLIMB_SLOW
                = new JoystickButton(CONTROLLER_OPERATOR, DriverStation.GamePad.BUTTON_Y.getValue()),
            BUTTON_GEAR
                = new JoystickButton(JOYSTICK_DRIVER_LEFT, DriverStation.Joystick.BUTTON_2.getValue()),
            BUTTON_SECOND_GEAR
                = new JoystickButton(JOYSTICK_DRIVER_RIGHT, DriverStation.Joystick.BUTTON_2.getValue());
//            BUTTON_INDEXER_FORWARD
//                = new JoystickButton(CONTROLLER_OPERATOR, DriverStation.GamePad.BUTTON_BACK.getValue()),
//            BUTTON_SHOOTER_VOLTAGE
//                = new JoystickButton(CONTROLLER_OPERATOR, DriverStation.GamePad.BUTTON_START.getValue());

            //BUTTON_BOILER
            //    = new JoystickButton(JOYSTICK_DRIVER_RIGHT, DriverStation.Joystick.BUTTON_2.getValue());


    static Preferences prefs;
    
    public OI() {
    	BUTTON_INTAKE_FORWARD.whenPressed(new PIDDriveRevolutions(5));
    }
    
    
}
