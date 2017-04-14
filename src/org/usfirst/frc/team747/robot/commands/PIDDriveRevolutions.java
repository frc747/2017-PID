package org.usfirst.frc.team747.robot.commands;

import org.usfirst.frc.team747.robot.Robot;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Command;

public class PIDDriveRevolutions extends Command {
	
	private final double revolutions;
	
	public PIDDriveRevolutions(double revolutions) {
		this.revolutions = revolutions;
		Robot.DRIVE_TRAIN.enablePositionControl();
		Robot.DRIVE_TRAIN.resetEncoders();
		Robot.resetNavXAngle();
		requires(Robot.DRIVE_TRAIN);
	}

	protected void execute() {
		Robot.DRIVE_TRAIN.set(revolutions, -revolutions);
	}
	
	@Override
	protected boolean isFinished() {
		
		System.out.println("Left: " + Robot.DRIVE_TRAIN.getLeftEncoderPosition() + "\tRight: " + Robot.DRIVE_TRAIN.getRightEncoderPosition());
		// TODO Auto-generated method stub
		return false;
	}
	
	protected void end() {
		Robot.DRIVE_TRAIN.enableVBusControl();
		Robot.DRIVE_TRAIN.resetEncoders();
		Robot.resetNavXAngle();
		Robot.DRIVE_TRAIN.stop();
	}
	
	protected void interrupted() {
		end();
	}

}
