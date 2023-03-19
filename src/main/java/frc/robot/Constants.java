// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants {
        public static final int kLeftLeadDeviceID = 2;
        public static final int kLeftFollowDeviceID = 1;
        public static final int kRightLeadDeviceID = 3;
        public static final int kRightFollowDeviceID = 4;

        public static final double kTwistMultiplier = 0.75;

        public static final int kStraightButton = 1;
        public static final int kTurnButton = 2;
    }

    public static final class ArmConstants {
        // How many amps the arm motor can use.
        public static final int CURRENT_LIMIT_A = 20;
        // Percent output to run the arm up/down at
        public static final double OUTPUT_POWER = 0.1;

        public static final int EXTEND_BUTTON = 7;
        public static final int RETRACT_BUTTON = 8;
    }

    public static final class IntakeConstants {
        // How many amps the intake can use while picking up
        public static final int INTAKE_CURRENT_LIMIT_A = 25;
        // How many amps the intake can use while holding
        public static final int INTAKE_HOLD_CURRENT_LIMIT_A = 5;
        // Percent output for intaking
        public static final double INTAKE_OUTPUT_POWER = 0.5;
        // Percent output for holding
        public static final double INTAKE_HOLD_POWER = 0.035;

        public static final int CUBE_IN_BUTTON = 1;
        public static final int CONE_IN_BUTTON = 2;
    }

    public static final class GamePiece {
        public static final int CONE = 1;
        public static final int CUBE = 2;
        public static final int NOTHING = 3;
    }
}
