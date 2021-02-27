package frc.robot.subsystems;

import com.analog.adis16448.frc.ADIS16448_IMU;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.VictorSPXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Drive extends SubsystemBase {

    // parent motors
    private WPI_TalonSRX leftFather = new WPI_TalonSRX(Constants.LEFTFATHER);
    private WPI_TalonSRX rightFather = new WPI_TalonSRX(Constants.RIGHTFATHER);

    // son motors
    private WPI_VictorSPX leftSon = new WPI_VictorSPX(Constants.LEFTSON);
    private WPI_VictorSPX rightSon = new WPI_VictorSPX(Constants.RIGHTSON);

    private final SpeedControllerGroup m_leftMotors = new SpeedControllerGroup(leftFather, leftSon);

    private final SpeedControllerGroup m_rightMotors = new SpeedControllerGroup(rightFather, rightSon);

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry m_odometry;

    public static final ADIS16448_IMU imu = new ADIS16448_IMU();

    // TODO test and find PIDF constants
    private static final double NEUTRAL_DEADBAND = 0.01, kF = 1023 / 3400.0, kP = 0.0, kI = 0.0, kD = 0.0,
            PEAK_OUTPUT = 1.0;

    private static final int I_ZONE = 0;

    private DifferentialDrive drive;

    private PeriodicIO mPeriodicIO;

    public Drive() {

        super();

        mPeriodicIO = new PeriodicIO();

        // Reset to defaults on boot to make sure settings are known
        leftFather.configFactoryDefault();
        rightFather.configFactoryDefault();

        leftFather.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        rightFather.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);

        TalonSRXConfiguration talonSRXConfig = new TalonSRXConfiguration();

        talonSRXConfig.primaryPID.selectedFeedbackSensor = FeedbackDevice.CTRE_MagEncoder_Relative; // Quadrature mag
                                                                                                    // encoder
        talonSRXConfig.neutralDeadband = NEUTRAL_DEADBAND;
        talonSRXConfig.slot0.kF = kF;
        talonSRXConfig.slot0.kP = kP;
        talonSRXConfig.slot0.kI = kI;
        talonSRXConfig.slot0.kD = kD;
        talonSRXConfig.slot0.integralZone = I_ZONE;
        talonSRXConfig.slot0.closedLoopPeakOutput = PEAK_OUTPUT;

        leftFather.configAllSettings(talonSRXConfig);
        rightFather.configAllSettings(talonSRXConfig);

        leftSon.configFactoryDefault();
        rightSon.configFactoryDefault();

        VictorSPXConfiguration victorSPXConfig = new VictorSPXConfiguration();

        victorSPXConfig.neutralDeadband = NEUTRAL_DEADBAND;

        leftSon.configAllSettings(victorSPXConfig);
        rightSon.configAllSettings(victorSPXConfig);

        leftSon.follow(leftFather);
        rightSon.follow(rightFather);

        drive = new DifferentialDrive(leftFather, rightFather);

        m_odometry = new DifferentialDriveOdometry(imu.getRotation2d());

    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        readPeriodicInputs();
        m_odometry.update(imu.getRotation2d(), leftFather.getSelectedSensorPosition(),
                rightFather.getSelectedSensorPosition());

    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return m_odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftFather.getSelectedSensorVelocity(),
                rightFather.getSelectedSensorVelocity());

    }

    /**
     * Zeroes the heading of the robot.
     */
    public void zeroHeading() {
        imu.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return imu.getRotation2d().getDegrees();
    }

    /**
     * Returns the turn rate of the robot.
     *
     * @return The turn rate of the robot, in degrees per second
     */
    public double getTurnRate() {
        return -imu.getRate();
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        rightFather.setSelectedSensorPosition(0);
        leftFather.setSelectedSensorPosition(0);

        m_odometry.resetPosition(pose, imu.getRotation2d());
    }

    public double getLeftEncoderRotations() {
        return mPeriodicIO.left_position_ticks / Constants.DRIVE_ENCODER_PPR;
    }

    public double getRightEncoderRotations() {
        return mPeriodicIO.right_position_ticks / Constants.DRIVE_ENCODER_PPR;
    }

    public double getLeftEncoderDistance() {
        return rotationsToInches(getLeftEncoderRotations());
    }

    public double getRightEncoderDistance() {
        return rotationsToInches(getRightEncoderRotations());
    }

    private static double rotationsToInches(double rotations) {
        return rotations * (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double rpmToInchesPerSecond(double rpm) {
        return rotationsToInches(rpm) / 60;
    }

    private static double inchesToRotations(double inches) {
        return inches / (Constants.kDriveWheelDiameterInches * Math.PI);
    }

    private static double inchesPerSecondToRpm(double inches_per_second) {
        return inchesToRotations(inches_per_second) * 60;
    }

    private static double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s / (Math.PI * 2.0) * 4096.0 / 10.0;
    }

    public static class PeriodicIO {
        // INPUTS
        public int left_position_ticks;
        public int right_position_ticks;
        public double left_distance;
        public double right_distance;
        public int left_velocity_ticks_per_100ms;
        public int right_velocity_ticks_per_100ms;
        // public Rotation2d gyro_heading = Rotation2d.identity();
        // public Pose2d error = Pose2d.identity();

        // OUTPUTS
        public double left_demand;
        public double right_demand;
        public double left_accel;
        public double right_accel;
        public double left_feedforward;
        public double right_feedforward;
        // public TimedState<Pose2dWithCurvature> path_setpoint = new
        // TimedState<Pose2dWithCurvature>(Pose2dWithCurvature.identity());
    }

    public void readPeriodicInputs() {
        double prevLeftTicks = mPeriodicIO.left_position_ticks;
        double prevRightTicks = mPeriodicIO.right_position_ticks;
        mPeriodicIO.left_position_ticks = (int) leftFather.getSelectedSensorPosition(0);
        mPeriodicIO.right_position_ticks = (int) rightFather.getSelectedSensorPosition(0);
        mPeriodicIO.left_velocity_ticks_per_100ms = (int) leftFather.getSelectedSensorVelocity(0);
        mPeriodicIO.right_velocity_ticks_per_100ms = (int) rightFather.getSelectedSensorVelocity(0);
    }
}
