package frc.robot.Limelight;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

public class LimelightSub extends SubsystemBase {

  // Troque pelos índices dos seus pipelines na interface da Limelight
  public static final int PIPELINE_APRILTAG = 3;
  public static final int PIPELINE_DETECTOR = 1;

  private final String nome;

  public LimelightSub(String nome) {
    this.nome = nome;
  }

  // ---- Configuração ----
  public void setAprilTag() {
    LimelightHelpers.setPipelineIndex(nome, PIPELINE_APRILTAG);
  }

  public void setDetector() {
    LimelightHelpers.setPipelineIndex(nome, PIPELINE_DETECTOR);
  }

  // ---- Leitura do detector ----
  public boolean getAlvo() {
    return LimelightHelpers.getTV(nome);
  }

  public double getTx() {
    return LimelightHelpers.getTX(nome);
  }

  public double getTy() {
    return LimelightHelpers.getTY(nome);
  }

  public double getTa() {
    return LimelightHelpers.getTA(nome);
  }

  public String getClasse() {
    return LimelightHelpers.getDetectorClass(nome);
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Limelight/" + nome + "/temAlvo", getAlvo());
    Logger.recordOutput("Limelight/" + nome + "/tx", getTx());
    Logger.recordOutput("Limelight/" + nome + "/ty", getTy());
    Logger.recordOutput("Limelight/" + nome + "/ta", getTa());
    Logger.recordOutput("Limelight/" + nome + "/classe", getClasse());
  }
}