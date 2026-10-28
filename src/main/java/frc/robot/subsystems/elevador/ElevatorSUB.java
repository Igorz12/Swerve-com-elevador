// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevador;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorSUBConstants;


public class ElevatorSUB extends SubsystemBase {

    private final SparkMax Motor1 = new SparkMax(Constants.ElevatorSUBConstants.MOTOR_ID,MotorType.kBrushless);

    private final SparkMax Motor2 = new SparkMax(Constants.ElevatorSUBConstants.MOTOR_ID_2,MotorType.kBrushless);

    private final RelativeEncoder encoder = Motor1.getEncoder();

    private final SparkMaxConfig configMotor2 = new SparkMaxConfig();

    private final SparkMaxConfig configMotor1 = new SparkMaxConfig();

    private final SparkClosedLoopController closed = Motor1.getClosedLoopController(); //envia setpoints

    public enum ElevatorState {
    PARADO,
    MOVENDO,
    NO_ALVO
}

private ElevatorState estado = ElevatorState.PARADO;
private double posicaoAlvo = 0.0;
private static final double TOLERANCIA = 1.2;



    public ElevatorSUB() {

    encoder.setPosition(0);
    configMotor1.closedLoop.pid(0.040, 0.0, 0.0);
    configMotor1.closedLoopRampRate(0.4);
    configMotor1.smartCurrentLimit(40);
    configMotor1.closedLoop.outputRange(-0.2, 0.2);
    configMotor1.idleMode(IdleMode.kBrake);

    configMotor2.follow(Motor1);
    //adicionei o velocityconversionfactor
    configMotor1.encoder.positionConversionFactor(Constants.ElevatorSUBConstants.CM_POR_ROTACAO_MOTOR);
    configMotor1.encoder.velocityConversionFactor(ElevatorSUBConstants.CM_POR_ROTACAO_MOTOR / 60.0);

    Motor1.configure(configMotor1,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
    
    Motor2.configure(configMotor2,ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
}
   
// ==== GETTERS ====

    public double getPosicao() {
        return encoder.getPosition();
    }
    public double getVelocity(){
        return encoder.getVelocity();
    }

    public ElevatorState getEstado() {
    return estado;
}

    public double getAltura() {     
        return getPosicao(); //ja convertida em cm
    }

    // ==== SETTERS ====

private void atualizarEstado() {
    switch (estado) {

        case PARADO:
            break;

        case MOVENDO:
            if (Math.abs(getAltura() - posicaoAlvo) < TOLERANCIA) {
                estado = ElevatorState.NO_ALVO;
            }
            break;

        case NO_ALVO:
            break;
    }
}

    public void moverPara(double posicao) {
    posicaoAlvo = posicao;
    estado = ElevatorState.MOVENDO;
    closed.setSetpoint(posicaoAlvo,ControlType.kPosition);
}
    
// ==== PERIODIC =====
    @Override
    public void periodic() {
          atualizarEstado();

    
    Logger.recordOutput("Elevator/Posicao", getPosicao());
    Logger.recordOutput("Elevator/Velocidade", getVelocity());
    Logger.recordOutput("Elevator/Altura", getAltura());
    Logger.recordOutput("Elevator/Output", Motor1.getAppliedOutput());
    Logger.recordOutput("Elevator/Estado", estado.name());

    }

  
}