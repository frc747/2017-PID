package org.usfirst.frc.team747.robot.subsystems;

import org.usfirst.frc.team747.robot.commands.Foo;
import org.usfirst.frc.team747.robot.maps.RobotMap;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.command.Subsystem;

public class DriveSubsystem extends Subsystem {

    // static hardware values (Encoder is grayhill 63R128, r128 is 128 pulsePerRevolution)
	private static final double ENCODER_TICKS = 250;
	private static final double WHEEL_CIRCUMFERNCE = 18.875;
	
	private static final double TICKS_PER_INCH = ENCODER_TICKS / WHEEL_CIRCUMFERNCE;
    
    public CANTalon talonDriveLeftPrimary = new CANTalon(RobotMap.DriveTrain.LEFT_FRONT.getValue()),
            talonDriveLeftSlave = new CANTalon(RobotMap.DriveTrain.LEFT_REAR.getValue()),
            talonDriveRightPrimary = new CANTalon(RobotMap.DriveTrain.RIGHT_FRONT.getValue()),
            talonDriveRightSlave = new CANTalon(RobotMap.DriveTrain.RIGHT_REAR.getValue());

    //public RobotDrive autoDrive = new RobotDrive(talonDriveLeftPrimary, talonDriveLeftSlave, talonDriveRightPrimary, talonDriveRightSlave);
    
	StringBuilder sb = new StringBuilder();
	int loops = 0;
    
    public DriveSubsystem() {
        super();
        this.talonDriveLeftPrimary.setInverted(true);
        this.talonDriveLeftSlave.setInverted(true);
        
        this.talonDriveRightPrimary.setInverted(true); // Reverse for Peanut Only?
        this.talonDriveRightSlave.setInverted(true); // Reverse for Peanut Only?
        
       // this.talonDriveRightPrimary.reverseSensor(true);  // Competition Only
        
        //this.talonDriveLeftPrimary.reverseSensor(true); // PeanutOnly!!

        this.talonDriveLeftSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
        this.talonDriveLeftSlave.set(this.talonDriveLeftPrimary.getDeviceID());
        
        this.talonDriveRightSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
        this.talonDriveRightSlave.set(this.talonDriveRightPrimary.getDeviceID());

        talonDriveLeftPrimary.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        talonDriveLeftPrimary.configEncoderCodesPerRev((int) ENCODER_TICKS);
        
        talonDriveLeftPrimary.configNominalOutputVoltage(+0f, -0f);
        talonDriveLeftPrimary.configPeakOutputVoltage(+3f, -3f);
        
        talonDriveLeftPrimary.setAllowableClosedLoopErr(0); // always servo
        
        talonDriveLeftPrimary.setProfile(0);
       // talonDriveLeftPrimary.setP(5.12);
        talonDriveLeftPrimary.setP(8.192);
        talonDriveLeftPrimary.setI(0.0);
      //  talonDriveLeftPrimary.setD(51.2); 
        talonDriveLeftPrimary.setD(81.92); 
        talonDriveLeftPrimary.setF(0.0);
        
        talonDriveRightPrimary.setFeedbackDevice(FeedbackDevice.QuadEncoder);
        talonDriveRightPrimary.configEncoderCodesPerRev((int) ENCODER_TICKS);
        
        talonDriveRightPrimary.configNominalOutputVoltage(+0f, -0f);
        talonDriveRightPrimary.configPeakOutputVoltage(+3f, -3f);
        
        talonDriveRightPrimary.setAllowableClosedLoopErr(0); // always servo
        
        talonDriveRightPrimary.setProfile(0);
      //  talonDriveRightPrimary.setP(5.12);
        talonDriveRightPrimary.setP(8.192);
        talonDriveRightPrimary.setI(0.0);
       // talonDriveRightPrimary.setD(51.2); 
        talonDriveRightPrimary.setD(81.92); 
        talonDriveRightPrimary.setF(0.0);
    }
    
    @Override
    protected void initDefaultCommand() {
        this.setDefaultCommand(new Foo());
    }

    public void changeControlMode(TalonControlMode mode) {
        this.talonDriveLeftPrimary.changeControlMode(mode);
        this.talonDriveRightPrimary.changeControlMode(mode);
    }
    
    public void set(double left, double right) {
        this.talonDriveLeftPrimary.set(left);
        this.talonDriveRightPrimary.set(right);
    }

    public void resetLeftEncoder() {
        talonDriveLeftPrimary.setPosition(0);
        
    }
    
    public void resetRightEncoder() {
        talonDriveRightPrimary.setPosition(0);
    }
    
    public int newGetEncoderPosition() {
        return (talonDriveLeftPrimary.getEncPosition() + talonDriveRightPrimary.getEncPosition())/ 2;
    }
    
    public void resetEncoders() {
    	this.resetLeftEncoder();
    	this.resetRightEncoder();
    	try {
			Thread.sleep(200);
		} catch (InterruptedException e) {
		}
    }
    
    public double getLeftEncoderPosition() {
        return talonDriveLeftPrimary.getPosition();
    }
    
    public double getRightEncoderPosition() {
        return talonDriveRightPrimary.getPosition();
    }
    
    public double getCombindedEncoderPosition() {
        return (getLeftEncoderPosition() + getRightEncoderPosition()) / 2;
    }
    
    public double convertRevsToInches(double revs) {
    	return revs * WHEEL_CIRCUMFERNCE;
    }
    
    public double convertInchesToRevs(double inches) {
    	return inches / WHEEL_CIRCUMFERNCE;
    }
    
    public double convertRevsToTicks(double revs) {
    	return revs * ENCODER_TICKS;
    }
    
    public double convertTicksToRevs(double ticks) {
    	return ticks / ENCODER_TICKS;
    }
    
    public double convertInchesToTicks(double inches) {
    	return convertRevsToTicks(convertInchesToRevs(inches));
    }
    
    public double convertTicksToInches(double ticks) {
    	return convertRevsToInches(convertTicksToRevs(ticks));
    }
    
    public void stop() {
        TalonControlMode mode = this.talonDriveLeftPrimary.getControlMode();

        double left = 0;
        double right = 0;
        
        switch (mode) {
        case Position:
            left = this.talonDriveLeftPrimary.getPosition();
            right = this.talonDriveRightPrimary.getPosition();
            break;
        case PercentVbus:
        case Speed:
        case Voltage:
        default:
            // Values should be 0;
            break;
        }
        
        this.set(left, right);
    }

	public void enablePositionControl() {
		this.changeControlMode(TalonControlMode.Position);
	}

	public void enableVBusControl() {
		this.changeControlMode(TalonControlMode.PercentVbus);
		
	}
}
