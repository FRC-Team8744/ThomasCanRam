// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

/** Represents a differential drive style drivetrain. */
public class Drivetrain {
  public static final double kMaxSpeed = 3.0; // meters per second
  public static final double kMaxAngularSpeed = 2 * Math.PI; // one rotation per second

  public static final double kConvertInchToMeter = 0.0254;

  public static final double kGearRatio = 8.45;

  private static final double kTrackWidthInch = 18.5;
  private static final double kTrackWidth = kTrackWidthInch * kConvertInchToMeter; // meters

  public static final double kWheelDiameterInches = 6.0;
  public static final double kWheelDiameterMeters = kWheelDiameterInches * kConvertInchToMeter;

  public static final double kEncoderDistancePerRevolution = (kWheelDiameterMeters * Math.PI) / kGearRatio;

  // SysID file used: C:\Users\FabLab9\FRC2024\ThomasCanTrack\sysid_data\sysid_data20231029-134230.json
  public static final double ksVolts = 0.11161; // Don't change!
  public static final double kvVoltSecondsPerMeter = 2.2496; // Don't change!
  public static final double kaVoltSecondsSquaredPerMeter = 0.346; // Don't change!
  
  private final CANSparkMax m_leftFront = new CANSparkMax(9, MotorType.kBrushless);
  private final CANSparkMax m_leftRear = new CANSparkMax(10, MotorType.kBrushless);
  private final CANSparkMax m_rightFront = new CANSparkMax(7, MotorType.kBrushless);
  private final CANSparkMax m_rightRear = new CANSparkMax(8, MotorType.kBrushless);

  private final RelativeEncoder m_leftEncoder = m_leftFront.getEncoder();
  private final RelativeEncoder m_rightEncoder = m_rightFront.getEncoder();

  // The gyro sensor
  private final AHRS m_gyro = new AHRS(SerialPort.Port.kUSB);

  private final MotorControllerGroup m_leftGroup =
      new MotorControllerGroup(m_leftFront, m_leftRear);
  private final MotorControllerGroup m_rightGroup =
      new MotorControllerGroup(m_rightFront, m_rightRear);

  private final PIDController m_leftPIDController = new PIDController(1, 0, 0);
  private final PIDController m_rightPIDController = new PIDController(1, 0, 0);

  private final DifferentialDriveKinematics m_kinematics =
      new DifferentialDriveKinematics(kTrackWidth);

  final DifferentialDriveOdometry m_odometry;

  // Gains are from a SysId check of Thomas - Don't change!
  private final SimpleMotorFeedforward m_feedforward = new SimpleMotorFeedforward(ksVolts, kvVoltSecondsPerMeter);

  /**
   * Constructs a differential drive object. Sets the encoder distance per pulse and resets the
   * gyro.
   */
  public Drivetrain() {
    m_gyro.reset();

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightGroup.setInverted(true);

    // Set the distance per pulse for the drive encoders. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    m_leftEncoder.setPositionConversionFactor(kEncoderDistancePerRevolution);
    m_rightEncoder.setPositionConversionFactor(kEncoderDistancePerRevolution);

    // Reset encoders to zero
    m_leftEncoder.setPosition(0);
    m_rightEncoder.setPosition(0);

    m_odometry =
        new DifferentialDriveOdometry(
            m_gyro.getRotation2d(), getLeftEncoderPosition(), getRightEncoderPosition());
  }

/**
   * Gets the left drive encoder.
   *
   * @return the left drive encoder
   */
  public double getLeftEncoderPosition() {
    return m_leftEncoder.getPosition();
  }

  /**
   * Gets the right drive encoder.
   *
   * @return the right drive encoder
   */
  public double getRightEncoderPosition() {
    return -m_rightEncoder.getPosition();
  }

  public double getLeftEncoderVelocity() {
    return m_leftEncoder.getVelocity();
  }

  public double getRightEncoderVelocity() {
    return -m_rightEncoder.getVelocity();
  }

/**
   * Sets the desired wheel speeds.
   *
   * @param speeds The desired wheel speeds.
   */
  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
    final double leftFeedforward = m_feedforward.calculate(speeds.leftMetersPerSecond);
    final double rightFeedforward = m_feedforward.calculate(speeds.rightMetersPerSecond);

    final double leftOutput =
        m_leftPIDController.calculate(getLeftEncoderVelocity(), speeds.leftMetersPerSecond);
    final double rightOutput =
        m_rightPIDController.calculate(getRightEncoderVelocity(), speeds.rightMetersPerSecond);
    m_leftGroup.setVoltage(leftOutput + leftFeedforward);
    m_rightGroup.setVoltage(rightOutput + rightFeedforward);
  }

  /**
   * Drives the robot with the given linear velocity and angular velocity.
   *
   * @param xSpeed Linear velocity in m/s.
   * @param rot Angular velocity in rad/s.
   */
  public void drive(double xSpeed, double rot) {
    var wheelSpeeds = m_kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }

  /** Updates the field-relative position. */
  public void updateOdometry() {
    m_odometry.update(
        m_gyro.getRotation2d(), getLeftEncoderPosition(), getRightEncoderPosition());
  }

  /**
   * Resets the field-relative position to a specific location.
   *
   * @param pose The position to reset to.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        m_gyro.getRotation2d(), getLeftEncoderPosition(), getRightEncoderPosition(), pose);
  }

  /**
   * Returns the pose of the robot.
   *
   * @return The pose of the robot.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }
}
