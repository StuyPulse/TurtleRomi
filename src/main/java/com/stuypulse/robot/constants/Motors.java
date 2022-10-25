package com.stuypulse.robot.constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;

public interface Motors {

    public interface Edwin {
        Config LEFT_TOP_MOTOR = new Config(false, IdleMode.kBrake, 60);
        Config LEFT_BOTTOM_MOTOR = new Config(false, IdleMode.kBrake, 60);

        Config RIGHT_TOP_MOTOR = new Config(true, IdleMode.kBrake, 60);
        Config RIGHT_BOTTOM_MOTOR = new Config(true, IdleMode.kBrake, 60);
    }
    
    public static class Config {
        public final boolean INVERTED;
        public final IdleMode IDLE_MODE;
        public final int CURRENT_LIMIT_AMPS;
        public final double OPEN_LOOP_RAMP_RATE;

        public Config(
                boolean inverted,
                IdleMode idleMode,
                int currentLimitAmps,
                double openLoopRampRate) {
            this.INVERTED = inverted;
            this.IDLE_MODE = idleMode;
            this.CURRENT_LIMIT_AMPS = currentLimitAmps;
            this.OPEN_LOOP_RAMP_RATE = openLoopRampRate;
        }

        public Config(boolean inverted, IdleMode idleMode, int currentLimitAmps) {
            this(inverted, idleMode, currentLimitAmps, 0.0);
        }

        public Config(boolean inverted, IdleMode idleMode) {
            this(inverted, idleMode, 80);
        }

        public void configure(CANSparkMax motor) {
            motor.setInverted(INVERTED);
            motor.setIdleMode(IDLE_MODE);
            motor.setSmartCurrentLimit(CURRENT_LIMIT_AMPS);
            motor.setOpenLoopRampRate(OPEN_LOOP_RAMP_RATE);
            motor.burnFlash();
        }
    }
}
