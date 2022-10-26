package com.stuypulse.robot.util;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

public class Constraints {

    private final double trackWidth;

    private final double maxVelocity;
    private final double maxAcceleration;
    
    private final double maxAngularVelocity;
    private final double maxAngularAcceleration;

    private final DifferentialDriveKinematics kinematics;

    public Constraints( double trackWidth, 
                        double maxVelocity, 
                        double maxAcceleration, 
                        double maxAngularVelocity, 
                        double maxAngularAcceleration, 
                        DifferentialDriveKinematics kinematics
    ) {
        this.trackWidth = trackWidth;

        this.maxVelocity = maxVelocity;
        this.maxAcceleration = maxAcceleration;

        this.maxAngularVelocity = maxAngularVelocity;
        this.maxAngularAcceleration = maxAngularAcceleration;

        this.kinematics = kinematics;
    }

    public double getTrackWidth() {
        return trackWidth;
    }

    public double getMaxVelocity() {
        return maxVelocity;
    }

    public double getMaxAcceleration() {
        return maxAcceleration;
    }

    public double getMaxAngularVelocity() {
        return maxAngularVelocity;
    }

    public double getMaxAngularAcceleration() {
        return maxAngularAcceleration;
    }

    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }
}
