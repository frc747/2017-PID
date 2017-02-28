package org.usfirst.frc.team747.robot.maps;

/**
 * Robot
 * 
 * A map file to translate physical hardware address of components to logical
 * address.
 * 
 * The main purpose of this class is to be able to have multiple configurations
 * that can be swapped in and out of the code to allow to use with multiple
 * drive bases. This means that we can develop different bases that have
 * differently addressed components, or missing components, but the code will
 * still execute on the base without issue.
 */
public final class RobotMap {

    /**
     * The drive train uses four talons, two per side.
     */
    public enum DriveTrain {
        LEFT_FRONT(0),
        LEFT_REAR(1),
        RIGHT_FRONT(2),
        RIGHT_REAR(3);

        private int value;

        private DriveTrain(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }
    
    /**
	 * Two talons are used to run the shooter, one for the left wheel and one
	 * for the right wheel.
	 */
	public enum Shooter {
	    LEFT_1(5),
	    LEFT_2(6),
	    RIGHT_1(7),
	    RIGHT_2(8),
	    INDEXER(9);
	
	    private int value;
	
	    private Shooter(int value) {
	        this.value = value;
	    }
	
	    public int getValue() {
	        return value;
	    }
	}
    
    /**
     * This will be the PID values for the Shooter
     */
    public enum ShooterSpeed {
    	SPEED(1500),
    	INDEXER_SPEED(100),
    	PROFILE(0),
    	F(0),
    	P(0),
    	I(0),
    	D(0),
    	RAMPRATE(0),
    	SHOOTER_IZONE(0);

        private int value;
        private double value2;

        private ShooterSpeed(int value) {
            this.value = value;
        }
        
        private ShooterSpeed(double value2){
        	this.value2 = value2;
        }

        public int getValue() {
            return value;
        }
        
        public double getDouble(){
        	return value2;
        }
    }
	
	public enum Climber {
	    CLIMB_1(10),
	    CLIMB_2(11);
	
	    private int value;
	
	    private Climber(int value) {
	        this.value = value;
	    }
	
	    public int getValue() {
	        return value;
	    }
	}

    /**
     * Intake uses 1 talon.
     */
    public enum Intake {
    	INTAKE(9);
    	
    	private int value;

        private Intake(int value) {
            this.value = value;
        }

        public int getValue() {
            return value;
        }
    }
}
