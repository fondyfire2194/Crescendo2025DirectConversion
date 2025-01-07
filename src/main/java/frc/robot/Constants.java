
package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

import frc.lib.config.SwerveModuleConstants;


/**
 * Field setup important dimensions
 * 
 * *******************WIDTH******************************
 * 
 * Width 26ft 11-1/4 inches - half width 13ft 5-5/8 inches = 161-5/8 inches
 * 
 * Speaker center is offset 57 inches = 4ft 9 inches from center line
 * 
 * Speaker center from source side is 18ft 2-5/8 inches or 218-5/8 inches
 * 
 * Speaker center from amp side is 8ft 8-5/8 inches or 104-5/8 inches
 * 
 * Math check 18ft 2-5/8 + 8ft 8-5/8 = 26ft 11-1/4 = field width
 * 
 * 
 * *******************LENGTH******************************
 * 
 * Length 54ft 3-1/4 inches - center line is 27ft 1-5/8 inches = 325-5/8 inches
 * 
 * Start line is 6ft 4-1/8 inches from speaker face = 76-1/8 inches
 * 
 * 
 * *******************NOTES******************************
 * https://www.firstinspires.org/robotics/frc/playing-field
 * 
 * The 3 start line notes are centered on speaker and 4ft 9 inches = 57 inches
 * either side of that 109.5" from speaker face
 * 
 * Field center line notes are 27ft 1-5/8 inches = 325-5/8 inches from speaker
 * face
 * 
 * One is on the field center line, others are 5ft 6 inches = 66 inches apart
 * from there
 *
 * Outside notes are 2ft 5-5/8 inches = 29-5/8 inches from field edge
 * 
 * 
 * 
 * Math check 4 x 5ft 6 = 22 ft + 2 x 2ft 5-5/8 = 4ft 11-1/4 = 26ft 11-1/4 =
 * field width
 * 
 * Stage layout
 * 
 * Stage center pillar plate begins 120.5" from speaker face and is on field
 * widthcenter line
 * Stage right and laft pillar plates field edge centers are 61-3/8 inches
 * either
 * side of field width center line and are 231.2 inches from speaker face.
 * 
 * 
 * 
 * 
 * 
 * 
 */

public final class Constants {

        public static final class CANIDConstants {
                // can ids 4 through 15 are used for swerve modules see SwerveConstants
                public static final int pdpID = 1;
                public static final int topShooterID = 17;
                public static final int bottomShooterID = 16;

                public static final int intakeID = 18;
                public static final int transferID = 19;
                public static final int armID = 20;
                public static final int armCancoderID = 21;
                public static final int climberIDLeft = 22;
                public static final int climberIDRight = 23;
                public static final int rearLeftSensor = 24;
                public static final int rearRightSensor = 25;
                public static final int transferDistanceSensorID = 26;

        }

        public static final class SwerveConstants {

                public static final double stickDeadband = 0.05;

                public static final boolean invertGyro = true; // Always ensure Gyro is CCW+ CW-

                public static final Distance robotWidthWithBumpers = Meters
                                .of(Meters.convertFrom(36, Inches));
                public static final Distance robotLengthWithBumpers = Meters
                                .of(Meters.convertFrom(32, Inches));

                /* Drivetrain Constants */
                public static final Distance trackWidth = Meters.of(Meters.convertFrom(19.5, Inches));// 9.75 Y
                public static final Distance wheelBase = Meters.of(Meters.convertFrom(17.625, Inches));// 8.8125 X
                public static final Distance wheelDiameter = Meters.of(Meters.convertFrom(3.9, Inches));// 3.86
                public static final Distance wheelCircumference = Meters
                                .of(wheelDiameter.magnitude() * Math.PI);

                public static final double openLoopRamp = 0.25;
                public static final double closedLoopRamp = 0.0;

                public static double mk4iL1DriveGearRatio = 1 / ((14.0 / 50.0) * (25.0 / 19.0) * (15.0 / 45.0));// 8.14.122807

                public static double mk4iL2DriveGearRatio = 1 / ((14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0));// 6.75

                public static double mk4iL1TurnGearRatio = 1 / ((14.0 / 50.0) * (10.0 / 60.0));// 21.43 1/.046667

                public static double mk4iL2TurnGearRatio = 1 / ((14.0 / 50.0) * (10.0 / 60.0));// 21.43 1/.046667

                public static double driveGearRatio = mk4iL2DriveGearRatio;

                public static double angleGearRatio = mk4iL2TurnGearRatio;

                public static final Translation2d flModuleOffset = new Translation2d(wheelBase.magnitude() / 2.0,
                                trackWidth.magnitude() / 2.0);
                public static final Translation2d frModuleOffset = new Translation2d(wheelBase.magnitude() / 2.0,
                                -trackWidth.magnitude() / 2.0);
                public static final Translation2d blModuleOffset = new Translation2d(-wheelBase.magnitude() / 2.0,
                                trackWidth.magnitude() / 2.0);
                public static final Translation2d brModuleOffset = new Translation2d(-wheelBase.magnitude() / 2.0,
                                -trackWidth.magnitude() / 2.0);

                public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                                flModuleOffset, frModuleOffset, blModuleOffset, brModuleOffset);

                /* Swerve Voltage Compensation */
                public static final double voltageComp = 12.0;

                /* Swerve Current Limiting */
                public static final int angleContinuousCurrentLimit = 30;
                public static final int driveContinuousCurrentLimit = 40; // 60

                /* Swerve Profiling Values */
                public static final double kmaxTheoreticalSpeed = 4.6; // 3.7;// mps *1.2 L2
                public static final double kmaxSpeed = 4.0; // meters per second *1.2 L2 3.9
                public static final double kmaxAngularVelocity = 1.0 * Math.PI;

                public static final double maxTranslationalSpeed = Units.feetToMeters(11.5);

                /* Angle Motor PID Values */
                public static final double[] angleKP = { .1, .1, .1, .1 };
                public static final double[] angleKI = { 0.0, 0.0, 0.0, 0.0 };
                public static final double[] angleKD = { 0, 0, 0, 0 };
                public static final double[] angleKFF = { 0, 0, 0, 0 };

                /* Drive Motor PID Values */
                public static final double driveKP = 0.1;
                public static final double driveKI = 0.0;
                public static final double driveKD = 0.0;
                public static final double driveKFF = .95 / kmaxTheoreticalSpeed;

                /*
                 * Drive Motor Feedforward and PP Values
                 * FIELD CARPET
                 */
                public static final double[] driveKS = { 0.2, 0.2, 0.2, 0.2 };
                public static final double[] driveKV = { 2.5, 2.5, 2.5, 2.5 };
                // 2.56/2.38,2.8.2.2,2.1 recorded speeds
                public static final double[] driveKA = { 0.59, 0.59, 0.59, 0.59 };
                public static final double[] driveKP1 = { 0.5, 0.5, 0.5, 0.5 };

                /*
                 * Drive Motor Feedforward and PP Values
                 * SCHOOL LIBRARY CARPET
                 * 
                 * public static final double driveKS = 0.60;//
                 * public static final double driveKV = 2.70;//
                 * public static final double driveKA = 0.59;//
                 * public static final double driveKP1 = 0.01;//
                 * 
                 * public static PIDConstants PPTransConstants = new PIDConstants(.5, 0, 0); //
                 * 2.0 Translation constants 3
                 * public static PIDConstants PPRotConstants = new PIDConstants(.5, 0, 0); //
                 * 2.0 Translation constants 3
                 * ^/
                 * // team 5907 driveKs = 0.22542;driveKv = 2.4829; driveKa = 0.120; driveP =
                 * // 0.08;
                 * 
                 * /* Drive Motor Conversion Factors
                 */
                public static final double driveConversionPositionFactor = (wheelDiameter.magnitude() * Math.PI)
                                / driveGearRatio;
                public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
                public static final double angleConversionFactor = 360.0 / angleGearRatio;

                /* Motor Inverts */
                public static final boolean driveInvert = false;
                public static final boolean angleInvert = true;

                /* Angle Encoder Invert */
                public static final boolean canCoderInvert = false;

                public static String[] modNames = { "FL ", "FR ", "BL ", "BR " };

                /* Module Specific Constants */
                /* Front Left Module - Module 0 */
                public static final class Mod0 {

                        public static final int driveMotorID = 13;
                        public static final int angleMotorID = 14;
                        public static final int cancoderID = 15;

                        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);// 253
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        cancoderID, angleOffset, false);
                }

                /* Front Right Module - Module 1 */
                public static final class Mod1 {
                        public static final int driveMotorID = 10;
                        public static final int angleMotorID = 11;
                        public static final int cancoderID = 12;

                        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);// 108
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        cancoderID, angleOffset, true);
                }

                /* Back Left Module - Module 2 */
                public static final class Mod2 {

                        public static final int driveMotorID = 7;
                        public static final int angleMotorID = 8;
                        public static final int cancoderID = 9;
                        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);// 207
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        cancoderID, angleOffset, false);
                }

                /* Back Right Module - Module 3 */
                public static final class Mod3 {
                        public static final int driveMotorID = 4;
                        public static final int angleMotorID = 5;
                        public static final int cancoderID = 6;

                        public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);// 239
                        public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                                        angleMotorID,
                                        cancoderID, angleOffset, true);
                }

                public static double alignKp = .02;
                public static double alighKd = 0;

                public static double maxTranslationalAcceleration;

                public static double turnToAngleMaxVelocity;

                public static double debounceTime;

                public static double alignNoteKp = .02;

                public static double alignNoteKd = 0;

                public static double odometryUpdateFrequency = 100;

                public static double notePickupSpeed = 0.75;

                public static double wheelRadius = Units.inchesToMeters(4) / 2;

                public static double minLobDistance = Units.feetToMeters(33);// 9 meters

                public static double maxLobDistance = Units.feetToMeters(43);// 13

                public static double rangeLobDistance = maxLobDistance - minLobDistance;

                public static double maxMovingShotDistance = Units.feetToMeters(30);

                public static PathConstraints pfConstraints = new PathConstraints(
                                3., 4.0,
                                Units.degreesToRadians(360), Units.degreesToRadians(540));

                public static PathConstraints pickUpConstraints = new PathConstraints(
                                1, 2.0,
                                Units.degreesToRadians(360), Units.degreesToRadians(540));

                public static boolean useCosineCompensation = false;

        }

        public static final class KeepAngle {
                public static final double kp = 0.30;
                public static final double ki = 0.0;
                public static final double kd = 0.0;
        }

        public static final class FieldConstants {
                public static final double FIELD_WIDTH = 8.21;
                public static final double FIELD_LENGTH = 16.54;

                public static final double noteDiameter = Units.inchesToMeters(14);

                public static final double stageHeight = Units.inchesToMeters(90);
                public static final double stageRadius = 2;

                public static final double speakerSlotHeight = Units.inchesToMeters(80.4375);

                public static final double lobAngleRads = Units.degreesToRadians(50);
                public static final double lobRPM = 2800;
                private static double speakerAimXOffset = Units.inchesToMeters(0);
                private static Transform2d speakerAimAdjustBlue = new Transform2d(speakerAimXOffset, 0.,
                                new Rotation2d());
                public static final Pose2d speakerBlueAlliance = new Pose2d(0., 5.5, Rotation2d.fromDegrees(0.0))
                                .transformBy(speakerAimAdjustBlue);
                private static Transform2d speakerAimAdjustRed = new Transform2d(speakerAimXOffset, 0.,
                                new Rotation2d());
                public static final Pose2d speakerRedAlliance = new Pose2d(16.24, 5.5, Rotation2d.fromDegrees(180.0))
                                .transformBy(speakerAimAdjustRed);

                public static final Pose2d stageBlueAlliance = new Pose2d(Units.inchesToMeters(190), FIELD_WIDTH / 2,
                                Rotation2d.fromDegrees(0.0));// 8 ft high

                public static final Pose2d stageBlueAllianceClimb = new Pose2d(6.17, 4.13,
                                Rotation2d.fromDegrees(0.0));

                // public static final Pose2d stageRedAlliance = new Pose2d(FIELD_LENGTH -
                // Units.inchesToMeters(190),
                // FIELD_WIDTH / 2, Rotation2d.fromDegrees(180.0));

                public static final Pose2d lobBlueAlliance = new Pose2d(0.0, 7.5, Rotation2d.fromDegrees(0.0));
                public static final Pose2d lobRedAlliance = new Pose2d(16.54, 7.5, Rotation2d.fromDegrees(180.0));

                public static Pose2d speakerAimPointX = new Pose2d(.45, 5.55, Rotation2d.fromDegrees(0.0));

                public static Pose2d driverStationBlueAlliance = new Pose2d();
                public static Pose2d driverStationRedAlliance = new Pose2d();

                public static Pose2d ampNoteGappose = new Pose2d(2., 6.2, new Rotation2d(Units.degreesToRadians(180)));

                public static Pose2d[] centerNotes = {
                                new Pose2d(),
                                new Pose2d(8., 7.45, new Rotation2d()),
                                new Pose2d(8., 5.77, new Rotation2d()),
                                new Pose2d(8., 4.10, new Rotation2d()),
                                new Pose2d(8., 2.22, new Rotation2d(Units.degreesToRadians(-150))),
                                new Pose2d(8., 0.77, new Rotation2d(Math.PI)),
                };

                public static Pose2d sbwfrpose = new Pose2d(1.34, 5.55, new Rotation2d(Units.degreesToRadians(180)));

                public static Pose2d[] wingNotePickups = {
                                new Pose2d(),
                                new Pose2d(2.8, 6.9, new Rotation2d(Units.degreesToRadians(180))),
                                new Pose2d(2.8, 5.55, new Rotation2d(Math.PI)),
                                new Pose2d(2.8, 4.09, new Rotation2d(Units.degreesToRadians(180))),

                };

                public static Pose2d[] centerNotesPickup = {
                                new Pose2d(),
                                new Pose2d(8., 7.44, new Rotation2d(Units.degreesToRadians(193))),
                                new Pose2d(8., 5.79, new Rotation2d(Units.degreesToRadians(165))),
                                new Pose2d(8., 4.10, new Rotation2d(Units.degreesToRadians(156))),
                                new Pose2d(8., 2.44, new Rotation2d(Units.degreesToRadians(176.09))),
                                new Pose2d(8., 0.78, new Rotation2d(Units.degreesToRadians(176.09))),
                };

                public static final Pose2d sourceShootBlue = new Pose2d(3.4, 2.7,
                                new Rotation2d(Units.degreesToRadians(150)));

                public static final Pose2d sourceStartPose = new Pose2d(.72, 4.4,
                                new Rotation2d(Units.degreesToRadians(120)));

                public static final Pose2d sourceClearStagePoseBlue = new Pose2d(7, 1.,
                                new Rotation2d(Units.degreesToRadians(180)));

                public static final Pose2d ampStartPose = new Pose2d(.8, 6.68,
                                new Rotation2d(Units.degreesToRadians(-120)));

                public static final Pose2d ampShootBlue = new Pose2d(4.0, 6.25,
                                new Rotation2d(Units.degreesToRadians(-168)));

                public static final Pose2d ampClearStagePoseBlue = new Pose2d(7, 6.8,
                                new Rotation2d(Units.degreesToRadians(180)));

                public static final Pose2d sbwfrStartPose = new Pose2d(1.34, 5.55,
                                new Rotation2d(Units.degreesToRadians(180)));

                // poses for blue stage climb
                public static final Pose2d blueSourceCenter = new Pose2d(4, 2.75,
                                new Rotation2d(Units.degreesToRadians(-120)));
                public static final Pose2d blueSourceSpeaker = new Pose2d(3.06, 2.85,
                                new Rotation2d(Units.degreesToRadians(-120)));
                public static final Pose2d blueSourceMidfield = new Pose2d(4.4, 1.6,
                                new Rotation2d(Units.degreesToRadians(-120)));

                public static final Pose2d blueAmpCenter = new Pose2d(4, 5.38,
                                new Rotation2d(Units.degreesToRadians(120)));
                public static final Pose2d blueAmpSpeaker = new Pose2d(3.06, 5.33,
                                new Rotation2d(Units.degreesToRadians(120)));
                public static final Pose2d blueAmpMidfield = new Pose2d(4.3, 6.5,
                                new Rotation2d(Units.degreesToRadians(120)));

                // poses for red stage climb
                public static final Pose2d redSourceCenter = new Pose2d(12.66, 2.92,
                                new Rotation2d(Units.degreesToRadians(-60)));
                public static final Pose2d redSourceSpeaker = new Pose2d(3.06, 2.85,
                                new Rotation2d(Units.degreesToRadians(-60)));
                public static final Pose2d redSourceMidfield = new Pose2d(12.6, 1.5,
                                new Rotation2d(Units.degreesToRadians(-60)));

                public static final Pose2d redAmpCenter = new Pose2d(12.56, 5.31,
                                new Rotation2d(Units.degreesToRadians(120)));
                public static final Pose2d redAmpSpeaker = new Pose2d(3.06, 5.33,
                                new Rotation2d(Units.degreesToRadians(120)));
                public static final Pose2d redAmpMidfield = new Pose2d(12, 6.5,
                                new Rotation2d(Units.degreesToRadians(120)));

        }

        public static final class AprilTagConstants {

                public static AprilTagFieldLayout layout;
                static {
                        try {
                                layout = AprilTagFieldLayout
                                                .loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
                        } catch (final Exception e) {
                                e.printStackTrace();
                        }
                }

        }

        public static final class GlobalConstants {
                public static final int ROBOT_LOOP_HZ = 50;
                /** Robot loop period */
                public static final double ROBOT_LOOP_PERIOD = 1.0 / ROBOT_LOOP_HZ;
        }

        public class PDPConstants {

                public static final int FRONT_LEFT_DRIVE_CHANNEL = 1;
                public static final int FRONT_RIGHT_DRIVE_CHANNEL = 2;
                public static final int BACK_LEFT_DRIVE_CHANNEL = 3;
                public static final int BACK_RIGHT_DRIVE_CHANNEL = 8;

                public static final int FRONT_LEFT_TURN_CHANNEL = 4;
                public static final int FRONT_RIGHT_TURN_CHANNEL = 5;
                public static final int BACK_LEFT_TURN_CHANNEL = 6;
                public static final int BACK_RIGHT_TURN_CHANNEL = 7;

        }

        public static final class CameraConstants {

                public static class CameraValues {
                        public String camname = "name";
                        public String ipaddress = "ip";
                        public double forward;
                        public double side;
                        public double up;
                        public double roll;
                        public double pitch;
                        public double yaw;
                        public double hfov;
                        public double vfov;
                        public int horpixels;
                        public int vertpixels;
                        public boolean isUsed = true;
                        public boolean isActive = false;

                        public CameraValues(
                                        final String camname,
                                        final String ipaddress,
                                        final double forward, final double side, final double up, final double roll,
                                        final double pitch, final double yaw,
                                        final double hfov, double vfov,
                                        final int horpixels, final int vertpixels,
                                        final boolean isUsed,
                                        final boolean isActive) {
                                this.camname = camname;
                                this.ipaddress = ipaddress;
                                this.forward = forward;
                                this.side = side;
                                this.up = up;
                                this.roll = roll;
                                this.pitch = pitch;
                                this.yaw = yaw;
                                this.hfov = hfov;
                                this.vfov = vfov;
                                this.horpixels = horpixels;
                                this.vertpixels = vertpixels;
                                this.isUsed = isUsed;
                                this.isActive = isActive;
                        }
                }

                public static CameraValues frontLeftCamera = new CameraValues("limelight-frleft", "10.21.94.5",
                                Units.inchesToMeters(10.75),
                                Units.inchesToMeters(-7.25),
                                Units.inchesToMeters(9.0),
                                0,
                                29, // deg
                                7.5,
                                63.3,
                                49.7,
                                1,
                                1,
                                true,
                                false);

                public static CameraValues frontRightCamera = new CameraValues("limelight-frright", "10.21.94.6",
                                Units.inchesToMeters(10.75),
                                Units.inchesToMeters(7.25),
                                Units.inchesToMeters(9.0),
                                0,
                                29, // deg
                                -7.5,
                                63.3,
                                49.7,
                                1,
                                1,
                                true,
                                false);

                public static CameraValues rearCamera = new CameraValues("limelight-rear", "10.21.94.10",
                                0,
                                Units.inchesToMeters(0),
                                0,
                                0,
                                5,
                                0,
                                63.3,
                                49.7,
                                1280,
                                960,
                                true,
                                false);

                public static final double POSE_AMBIGUITY_CUTOFF = 0.05;
                public static final double DISTANCE_CUTOFF = 4.0;

        }

        public static final class ShooterConstants {

                public static final AngularVelocity maxShooterMotorRPM = RPM.of(5700);
                public static final AngularVelocity minShooterMotorRPM = RPM.of(1500);

                public static final double shooterConversionVelocityFactor = 1;
                public static final double shooterConversionPositionFactor = 1;
                public static final double topShooterKP = 3e-4;
                public static final double topShooterKI = 0;
                public static final double topShooterKD = 0.02;
                public static final double topShooterKFF = 1.0 / maxShooterMotorRPM.in(RPM);
                public static final double bottomShooterKP = 3e-4;
                public static final double bottomShooterKI = 0;
                public static final double bottomShooterKD = .02;
                public static final double bottomShooterKFF = 1.0 / maxShooterMotorRPM.in(RPM);
                public static final double voltageComp = 12;

                public static final int shooterContinuousCurrentLimit = 50;

                public static double baseRunVelocity = 1500;
                public static double velocityTolerance = .05;
                public static double gearing;
                public static double circumference;
                public static double jogSpeed = .25;
                public static double debounceTime = .1;
                public static double kAccelCompFactor = 0.100; // in units of seconds
                public static double speedFactor = .25;
                public static double rollerDiameter = Units.inchesToMeters(4);

        }

        public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
       // public static final Matter CHASSIS = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
        public static final double LOOP_TIME = 0.13; // s, 20ms + 110ms sprk max velocity lag
        public static final double MAX_SPEED = 4;

        public static double subwfrArmAngle = 57;// 60;// degrees
        public static double subwfrShooterSpeed = 3000;// rpm

        public static double source_ampShootAngle = 34;
        public static double source_ampShootSpeed = 3550;

        public static double source_ampShootDistance = 4;

        public static double wing1ArmAngle = 34;// degrees
        public static double wing1ShooterSpeed = 3500;// rpm

        public static double wing2ArmAngle = 40;// degrees
        public static double wing2ShooterSpeed = 3500;// rpm

        public static double wing3ArmAngle = 60;// degrees
        public static double wing3ShooterSpeed = 3000;// rpm

        public static double tapeLineArmAngle = 40;
        public static double tapeLineShooterSpeed = 3500;

        // public static double safeStageArmAngle = 37;
        // public static double safeStageShooterSpeed = 3500;

        public static double allianceLineArmAngle = 24;
        public static double allianceLineShooterSpeed = 4500;

        public static double ampStartArmAngle = 27;
        public static double ampStartShooterSpeed = 4000;

        public static final class ArmConstants {

                public static final Angle armAngleOnBottomStopBar = Radians.of(Units.degreesToRadians(20));//

                public static final Angle startupBacklashCompensation = Radians.of(Units.degreesToRadians(2));

                public static final Angle aboveDistanceShootAngleForUDA = Radians.of(Units.degreesToRadians(45));

                public static final Angle UDARads = Radians.of(Units.degreesToRadians(3));

                public static final Angle maxUDAAngleRads = Radians.of(Units.degreesToRadians(60));

                public static final AngularVelocity maxarmMotorRPM = RPM.of(5700);

                public static final double NET_GEAR_RATIO = 200;// 100:1 then 2:1

                public static final Angle DEGREES_PER_ENCODER_REV = Degrees.of(360 / NET_GEAR_RATIO);

                public static final double armConversionPositionFactor = DEGREES_PER_ENCODER_REV.in(Radians);

                public static final double armConversionVelocityFactor = armConversionPositionFactor / 60; //

                public static final AngularVelocity MAX_DEGREES_PER_SEC = RevolutionsPerSecond
                                .of((DEGREES_PER_ENCODER_REV.in(Degrees) * (maxarmMotorRPM.in(RevolutionsPerSecond))));

                public static final AngularVelocity MAX_RADS_PER_SEC = RadiansPerSecond
                                .of(MAX_DEGREES_PER_SEC.in(RadiansPerSecond));// 5 approx

                public static final AngularAcceleration MAX_RADS_PER_SEC_PER_SEC = RadiansPerSecondPerSecond
                                .of(Math.PI);

                public static final double voltageComp = 12;

                public static final int armContinuousCurrentLimit = 40;
                public static Angle armMin = armAngleOnBottomStopBar;
                public static Angle armMax = Degree.of(110);
                public static Angle pickupAngleRadians = Radians.of(Units.degreesToRadians(25));
                public static Angle midRange = Radians.of(Units.degreesToRadians(45));

                public static final double armKg = 0.2;
                public static final double armKs = 0.31;
                public static final double armKv = 2;// volts per deg per sec so 12/max = 12/5=2.4
                public static final double armKa = 0;

                public static final double armKp = 30;
                public static final double armKi = 0.5;
                public static final double armKd = 0.0;

                public static final int currentLimit = 40;

                public static final double angleTolerance = Units.degreesToRadians(1);
                public static final double autoShootAngleTolerance = .5;
                public static double jogSpeed = .05;

                public static double debounceTime = .25;

                public static double kArmOffsetRads;

                public static double armLength = Units.inchesToMeters(21.75);

                public static double armMass = Units.lbsToKilograms(30.0);

                public static double noteAtIntakeAmps = 30;

                public static double maxShootOffsetDegrees = 4;
                public static final double minShootOffsetDegrees = -4;

                public static final double armPivotZ = Units.inchesToMeters(10.3);// 10.3
                public static final double armPivotX = Units.inchesToMeters(-7.5);
                public static final double armPivotOffset = Units.inchesToMeters(3);

        }

        public static final class TransferConstants {

                public static final double maxTransferMotorRPM = 11000;
                public static final double transferConversionVelocityFactor = 1;
                public static final double transferConversionPositionFactor = 1;
                public static final double voltageComp = 12;

                public static final int transferContinuousCurrentLimit = 40;
                public static double clearShooterTime = .75;
                public static double noNoteStopTime = 20;
                public static double jogSpeed = 1;
                public static AngularVelocity intakingSpeed = RPM.of(5500);
                public static AngularVelocity transferToShootSpeed = RPM.of(7000);

                public static final double transferKp = .00002; // P gains caused oscilliation

                public static final double transferPositionKp = .00002; // P gains caused oscilliation
                public static final double transferKi = 0.0;
                public static final double transferKd = 0.00;
                public static final double transferKFF = .95 / maxTransferMotorRPM;
        }

        public static final class IntakeConstants {
                public static final double maxIntakeMotorRPM = 5700;
                public static final double intakeConversionVelocityFactor = 1;
                public static final double intakeConversionPositionFactor = 1;
                public static final double voltageComp = 12;

                public static final int intakeContinuousCurrentLimit = 60;
                public static double jogSpeed = 1;
                public static AngularVelocity reverseRPM = RPM.of(-500);
                public static double reverseTime = 2;
                public static double noteInIntakeAmps = 30;
                public static final AngularVelocity intakeSpeed = RPM.of(4500);
                public static final double intakeKp = 2e-4;
                public static final double intakeKi = 0.0;
                public static final double intakeKd = 0.00;
                public static final double intakeKFF = .95 / maxIntakeMotorRPM;
                public static final double notemissedtime = 1.5;
        }

        public static final class ClimberConstants {
                public static final double maxClimberMotorRPM = 4800;
                public static final double climberConversionPositionFactor = .1;
                public static final double climberConversionVelocityFactor = climberConversionPositionFactor / 60;
                public static final double voltageComp = 12;

                public static final int climberContinuousCurrentLimit = 60;
                public static double jogSpeed = .25;
                public static double minPosition = -1;
                public static double maxPosition = 17;
                public static double maxinchespersec = climberConversionPositionFactor * maxClimberMotorRPM / 60;// 8
                                                                                                                 // ips
        }

        public static class OperatorConstants {

                // Joystick Deadband
                public static final double DEADBAND = 0.1;
                public static final double LEFT_Y_DEADBAND = 0.1;
                public static final double RIGHT_X_DEADBAND = 0.1;
                public static final double TURN_CONSTANT = 6;
        }

}