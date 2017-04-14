package org.usfirst.frc.team747.robot.commands;

import org.usfirst.frc.team747.robot.Robot;

import com.ctre.CANTalon;

import edu.wpi.first.wpilibj.command.Command;

public class Foo extends Command {
	
	public Foo() {
		requires(Robot.DRIVE_TRAIN);
	}

	protected void execute() {
	}
	
	@Override
	protected boolean isFinished() {
		return false;
	}

}
