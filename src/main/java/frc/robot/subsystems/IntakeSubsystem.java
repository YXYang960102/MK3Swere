// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.ColorSensorV3.ProximitySensorMeasurementRate;
import com.revrobotics.ColorSensorV3.ProximitySensorResolution;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
    private CANSparkMax IntakeMotor = new CANSparkMax(IDConstants.kIntakeMotorPort, MotorType.kBrushless);
    private RelativeEncoder IntakeEncoder = IntakeMotor.getEncoder();

    private I2C.Port i2cPort = I2C.Port.kOnboard;

    private ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

    /** Creates a new IntakeSubsystem. */
    public IntakeSubsystem() {
        IntakeMotor.setInverted(false);
        IntakeMotor.setIdleMode(IdleMode.kCoast);
        IntakeMotor.setSmartCurrentLimit(80);

        colorSensor.configureProximitySensor(ProximitySensorResolution.kProxRes11bit,
                ProximitySensorMeasurementRate.kProxRate12ms);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // SmartDashboard.putNumber("Intake Velocity", getIntakeVelocity());
        SmartDashboard.putBoolean("Intake isPass", isPass());
    }

    public double getIntakeVelocity() {
        return IntakeEncoder.getVelocity();
    }

    public void IntakeFwd() {
        IntakeMotor.setIdleMode(IdleMode.kBrake);
        IntakeMotor.set(IntakeConstants.kIntakeMotorRate);
    }

    public void IntakeRev() {
        IntakeMotor.setIdleMode(IdleMode.kBrake);
        IntakeMotor.set(-IntakeConstants.kIntakeMotorRate);
    }

    public void IntakeAutoRev() {
        IntakeMotor.setIdleMode(IdleMode.kBrake);
        IntakeMotor.set(-IntakeConstants.kIntakeAutoRate);
    }


    public void IntakeStop() {
        IntakeMotor.set(0);
    }

    public double getIR() {
        /**
         * The sensor returns a raw IR value of the infrared light detected.
         */
        double IR = colorSensor.getIR();
        return IR;
    }

    public int getProximity() {
        /**
         * In addition to RGB IR values, the color sensor can also return an
         * infrared proximity value. The chip contains an IR led which will emit
         * IR pulses and measure the intensity of the return. When an object is
         * close the value of the proximity will be large (max 2047 with default
         * settings) and will approach zero when the object is far away.
         * 
         * Proximity can be used to roughly approximate the distance of an object
         * or provide a threshold for when an object is close enough to provide
         * accurate color values.
         */
        int proximity = colorSensor.getProximity();
        return proximity;
    }

    public boolean isPass() {
        return getProximity() > IntakeConstants.kIntakeGateValue;
    }

}
