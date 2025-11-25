// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeSubsystemConstants;

public class IntakeSubsystem extends SubsystemBase {
  private final SparkMax intakeMotor;

  private final SparkMax indexerMotor;
  private final SparkMax indexerFollower;

  private final RelativeEncoder intakeEncoder;
  private final RelativeEncoder indexerEncoder;
  private final RelativeEncoder indexerFollowerEncoder;

  public IntakeSubsystem() 
  {
    intakeMotor = new SparkMax(IntakeSubsystemConstants.kIntakeMotorPort, MotorType.kBrushless);

    indexerMotor = new SparkMax(IntakeSubsystemConstants.kIndexerMotorPort, MotorType.kBrushless);
    indexerFollower = new SparkMax(IntakeSubsystemConstants.kIndexerFollowerPort, MotorType.kBrushless);

    intakeEncoder = intakeMotor.getEncoder();
    indexerEncoder = indexerMotor.getEncoder();
    indexerFollowerEncoder = indexerFollower.getEncoder();

    inializeIndexerMotors();
    initilizeIntakeMotor();
  }
            
  private void initilizeIntakeMotor() 
  {
    SparkMaxConfig globalConfig = new SparkMaxConfig();

    globalConfig
      .idleMode(IntakeSubsystemConstants.kIntakeIdleMode)
      .smartCurrentLimit(IntakeSubsystemConstants.kIntakeCurrentLimit);
    
    globalConfig.encoder
      .positionConversionFactor(IntakeSubsystemConstants.kIntakeGearBoxRatio)
      .velocityConversionFactor(1);

    intakeMotor.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
        
  private void inializeIndexerMotors() 
  {
    SparkMaxConfig globalConfig = new SparkMaxConfig();
    SparkMaxConfig followerConfig = new SparkMaxConfig();

    globalConfig
    .idleMode(IntakeSubsystemConstants.kIndexerIdleMode)
    .smartCurrentLimit(IntakeSubsystemConstants.kIndexerCurrentLimit);

    globalConfig.encoder
      .positionConversionFactor(IntakeSubsystemConstants.kIntakeGearBoxRatio)
      .velocityConversionFactor(1);

    followerConfig
      .apply(globalConfig)
      .follow(indexerMotor, true);

    indexerMotor.configure(globalConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
    
  @Override
  public void periodic() 
  {
    // This method will be called once per scheduler run

    //**SmartDasboard messueresData**//
    /** Intake Data */
    SmartDashboard.putNumber("intakeSusbsytem/intakeCurrent", getIntakeCurrent());

    /** Indexer Data */
    SmartDashboard.putNumber("indexerSusbsytem/indexerFollowerCurrent", getIndexerFollowerCurrent());
    SmartDashboard.putNumber("indexerSusbsytem/indexerCurrent", getIndexerCurrent());
  }

  public void runSubsystem(double speed) 
  {
    //active intake and indexer motorsq
    intakeMotor.set(speed);
    indexerMotor.set(speed);    
  }

  /**
   * stops the intake and indexer motors
   */
  public void stopSubsystem() 
  {
    //stop intake and indexer motors
    intakeMotor.set(0);
    indexerMotor.set(0);    
  }

  /**
   * runs the intake motor at a given speed
   * @param speed
   */
  public void runIntakeMotor(double speed)
  {
    intakeMotor.set(speed);
  }
  /**
   * stops the intake motor
   */
  public void stopIntakeMotor()
  {
    intakeMotor.set(0);
  }

  /**
   * runs the intake motor at a given speed
   * @param speed
   */
  public void runIndexerMotors(double speed)
  {
    indexerMotor.set(speed);
  }
  /**
   * stops the intake motor
   */
  public void stopIndexerMotors()
  {
    indexerMotor.set(0);
  }

  /** Intake Data */
  /**
   * gets the current being drawn by the intake motor
   *
   * @return
   */
  public double getIntakeCurrent()
  {
    return intakeMotor.getOutputCurrent();
  }

  /**
   * gets the position of the intake encoder
   * @return position from degress
   */
  public double getIntakeEncoderPosition()
  {
    return intakeEncoder.getPosition();
  }

  /** indexer Data */
  /**
   * gets the current being drawn by the indexer motor
   *
   * @return
   */
  public double getIndexerCurrent()
  {
    return indexerMotor.getOutputCurrent();
  }

  /** Indexer follower data  */
  /**
   * gets the position of the indexer encoder
   * @return get from degrees
   */
  public double getIndexerEncoderPosition()
  {
    return indexerEncoder.getPosition();
  }

  /**
   * gets the current being drawn by the indexer follower motor
   *
   * @return get from amps
   */
  public double getIndexerFollowerCurrent()
  {
    return indexerFollower.getOutputCurrent();
  }
  /**
   * gets the position of the indexer follower encoder
   * @return get from degrees
   */
  public double getIndexerFollowerEncoderPosition()
  {
    return indexerFollowerEncoder.getPosition();
  }
}
