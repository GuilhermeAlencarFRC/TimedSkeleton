package frc.robot;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;

public class Robot extends TimedRobot {

    private SparkMax motorSuperiorDireitoLider;
    private SparkMax motorInferiorDireitoSeg;
    private SparkMax motorSuperiorEsquerdoLider;
    private SparkMax motorInferiorEsquerdoSeg;
    private SparkMax MotorNEO;

    private RelativeEncoder encoderNEO;

    private Joystick logitechTracao;
    private Joystick logitechTorre;
    private Timer timer;
    private boolean motorLigado;
    private boolean isRunning = false;
    private double targetPosition = 100.0;
    private double slowdownThreshold = 20.0;

    public Robot() {
        // Motores da Tração
        motorSuperiorEsquerdoLider = new SparkMax(2, MotorType.kBrushless);
        motorInferiorEsquerdoSeg = new SparkMax(3, MotorType.kBrushless);
        motorSuperiorDireitoLider = new SparkMax(4, MotorType.kBrushless);
        motorInferiorDireitoSeg = new SparkMax(5, MotorType.kBrushless);

        // Motor NEO
        MotorNEO = new SparkMax(6, MotorType.kBrushless);

        // Configuração inicial do MotorNEO
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.idleMode(IdleMode.kCoast); // Inicia em Coast
        motorConfig.smartCurrentLimit(40);
        MotorNEO.configure(motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

        // Encoder integrado do Spark Max
        encoderNEO = MotorNEO.getEncoder();
        encoderNEO.setPosition(0); // Reseta a posição do encoder para 0

        // Configurações gerais para os outros motores
        SparkMaxConfig configGlobal = new SparkMaxConfig();
        SparkMaxConfig configMotorDireitoLider = new SparkMaxConfig();
        SparkMaxConfig configMotorEsquerdoSeg = new SparkMaxConfig();
        SparkMaxConfig configMotorDireitoSeg = new SparkMaxConfig();

        configMotorDireitoLider
            .apply(configGlobal)
            .inverted(true);

        configMotorEsquerdoSeg
            .apply(configGlobal)
            .inverted(false)
            .follow(motorSuperiorEsquerdoLider);

        configMotorDireitoSeg
            .apply(configGlobal)
            .follow(motorSuperiorDireitoLider);

        motorSuperiorEsquerdoLider.configure(configGlobal, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorInferiorEsquerdoSeg.configure(configMotorEsquerdoSeg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorSuperiorDireitoLider.configure(configMotorDireitoLider, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        motorInferiorDireitoSeg.configure(configMotorDireitoSeg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        logitechTracao = new Joystick(0);
        logitechTorre = new Joystick(1);

        timer = new Timer();
        motorLigado = false;
    }

    @Override
    public void robotPeriodic() {
        // Exemplo de leitura da posição e velocidade do encoder
        double position = encoderNEO.getPosition();
        double velocity = encoderNEO.getVelocity();

        // Exibe os valores no console
        System.out.println("Posição do Encoder: " + position);
        System.out.println("Velocidade do Encoder: " + velocity);
    }

    @Override
    public void autonomousInit() {
        timer.reset();
        timer.start();
        motorLigado = true;
    }

    @Override
    public void autonomousPeriodic() {
        if (motorLigado) {
            setMotorSpeed(0.20);  // Liga os motores (velocidade 1)

            if (timer.get() >= 1.0) {  // Se 1 segundo se passou
                motorSuperiorEsquerdoLider.set(0);  // Desliga os motores
                motorInferiorEsquerdoSeg.set(0);
                motorSuperiorDireitoLider.set(0);
                        setMotorSpeed(0);  // Desliga os motores
        }
      }
    }

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {
        double currentPosition = encoderNEO.getPosition();
        double speed = 0;

        // Controle do elevador baseado na posição do encoder
        if (isRunning) {
            // Desaceleração suave conforme se aproxima da posição alvo
            if (Math.abs(targetPosition - currentPosition) < slowdownThreshold) {
                speed = 0.3 * (targetPosition - currentPosition) / slowdownThreshold; // Suaviza a parada
                if (Math.abs(targetPosition - currentPosition) < 1.0) {
                    speed = 0; // Para completamente ao alcançar o alvo
                    isRunning = false;

                    // Configura o modo Brake ao parar
                    SparkMaxConfig brakeConfig = new SparkMaxConfig();
                    brakeConfig.idleMode(IdleMode.kBrake);
                    MotorNEO.configure(brakeConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
                }
            } else {
                // Movimento em velocidade máxima fora do limite de desaceleração
                speed = 0.5;
            }

            MotorNEO.set(speed);
        }

        // Controle para iniciar o movimento 
        if (logitechTorre.getRawButton(1)) {
            isRunning = true;
            encoderNEO.setPosition(0); // Reseta o encoder para iniciar o movimento
            SparkMaxConfig coastConfig = new SparkMaxConfig();
            coastConfig.idleMode(IdleMode.kCoast);
            MotorNEO.configure(coastConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        }
    }

    private void setMotorSpeed(double speed) {
        motorSuperiorEsquerdoLider.set(speed);
        motorInferiorEsquerdoSeg.set(speed);
        motorSuperiorDireitoLider.set(speed);
        motorInferiorDireitoSeg.set(speed);
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void testInit() {}

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {}

    @Override
    public void simulationPeriodic() {}
}
