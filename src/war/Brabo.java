package war;

import java.awt.Color;
import org.neuroph.nnet.MultiLayerPerceptron;
import org.neuroph.util.TransferFunctionType;
import org.neuroph.core.data.DataSet;
import robocode.AdvancedRobot;
import robocode.BulletHitEvent;
import robocode.WinEvent;
import robocode.ScannedRobotEvent;
import robocode.HitWallEvent;
import robocode.BulletMissedEvent;
import robocode.RoundEndedEvent;
import robocode.HitRobotEvent;
import robocode.util.Utils;
import robocode.HitByBulletEvent;

import java.awt.geom.*;
import java.math.BigDecimal;
import java.math.RoundingMode;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

public class Brabo extends AdvancedRobot {
	// Perceptron destinado a lidar com tiros
	static MultiLayerPerceptron firePerceptron = new MultiLayerPerceptron(TransferFunctionType.TANH, 3, 6, 1);
	static double rD, rE, mE, power;
	private double lastTurnEnergy = 100;
	private int moveDirection = 1;
	public static final DataSet fireTraining;
	public static final ArrayList<double[]> treeSet = new ArrayList<double[]>();
    public int[] count = new int[3];
    public int[] correct = new int[3];
    int unpredicted = 0;
    public int disparos = 0;
    public int acertos = 0;
    public int erros = 0;
    public int evade = 0;

    public ScannedRobotEvent Enemy;

    public int gunDirection = 1;

    public double maxDistance = 0;
	public int robotscanned = 0;
	public double distancetorobot = 0;
	public double enemyenergy = 0;
	public double gunaimedtoenemy = 0;
	public double hitbybullet = 0;
	public double hitwall = 0;
	public double hitrobot = 0;
	public double energy = 0;
	public double heat = 0;
	public double positionx = 0;
	public double positiony = 0;
	public int moving = 0;
	public double gunturning = 0;
	public double robotturning = 0;
	public int action = 0;


    static {
        treeSet.add(new double[]{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 3});
        treeSet.add(new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 4});
        treeSet.add(new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 5});
        treeSet.add(new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0.25, 0.75, 0, 0, 0, 6});
        treeSet.add(new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0.25, 0.25, 0, 0, 0, 7});
        treeSet.add(new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0.75, 0.25, 0, 0, 0, 8});
        treeSet.add(new double[]{0, 0, 0, 0, 0, 0, 0, 0, 0, 0.75, 0.75, 0, 0, 0, 9});
        treeSet.add(new double[]{0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 10});
        treeSet.add(new double[]{0, 0, 0, 0, 1, 0, 0, 1.00, 0, 0.27, 0.10, 0, 1, 0, 7});
        treeSet.add(new double[]{1, 0.68, 90.40, 0, 0, 0, 0, 0, 0, 0.27, 0.10, 1, 0, 0, 3});
        treeSet.add(new double[]{1, 0.37, 97.40, 0, 0, 0, 0, 13.20, 1, 0.57, 0.36, 1, 0, 0, 3});
        treeSet.add(new double[]{0, 0, 0, 0, 0, 0, 0, 100.00, 1, 0.40, 0.43, 1, 1, 0, 5});
        treeSet.add(new double[]{0, 0, 0, 0, 0, 0, 0, 100.00, 1, 0.62, 0.26, 1, 1, 0, 5});
        treeSet.add(new double[]{0, 0, 0, 0, 0, 0, 0, 100.00, 1, 0.11, 0.52, 1, 1, 0, 5});
        treeSet.add(new double[]{0, 0, 0, 0, 0, 0, 0, 100.00, 1, 0.84, 0.79, 0, 1, 0, 9});
        treeSet.add(new double[]{1, 0.26, 98.00, 0, 0, 0, 0, 97.00, 1, 0.34, 0.76, 0, 0, 0, 6});
        treeSet.add(new double[]{0, 0, 0, 0, 0, 0, 0, 100.00, 1, 0.18, 0.26, 0, 1, 0, 7});
        treeSet.add(new double[]{1, 0.07, 100.00, 1, 0, 0, 0, 100.00, 1, 0.29, 0.49, 0, 0, 0, 7});
        treeSet.add(new double[]{0, 0, 0, 0, 0, 0, 0, 100.00, 1, 0.64, 0.13, 0, 0, 0, 8});
        treeSet.add(new double[]{0, 0, 0, 0, 0, 1, 0, 74.00, 0, 0.41, 0.03, 0, 0, 1, 5});
        treeSet.add(new double[]{0, 0, 0, 0, 0, 0, 0, 95.00, 0, 0.37, 0.13, 1, 1, 0, 5});
        treeSet.add(new double[]{0, 0, 0, 0, 0, 0, 0, 74.00, 0, 0.41, 0.03, 0, 0, 1, 5});
        treeSet.add(new double[]{0, 0, 0, 0, 0, 0, 0, 100.00, 1, 0.34, 0.11, 1, 1, 0, 5});
        treeSet.add(new double[]{0, 0, 0, 0, 0, 0, 0, 74.00, 0, 0.41, 0.03, 0, 0, 1, 5});

        // ; real data from battle
        treeSet.add(new double[]{0, 0.0000, 0.0000, 0.0, 0.0, 0.0, 0.0, 100.0000, 1.0, 0.9439, 0.3686, 0, 0.0, 0.0, 8.0});
        treeSet.add(new double[]{0, 0.0000, 0.0000, 0.0, 0.0, 0.0, 0.0, 100.0000, 1.0, 0.9473, 0.3706, 1, 0.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.4547, 65.0000, 1.0, 0.0, 0.0, 0.0, 103.0000, 1.0, 0.3921, 0.9054, 0, 1.0, 0.0, 6.0});
        treeSet.add(new double[]{1, 0.2220, 48.0000, 1.0, 0.0, 0.0, 0.0, 99.0000, 1.0, 0.5164, 0.3842, 0, 1.0, 1.0, 7.0});
        treeSet.add(new double[]{1, 0.4457, 48.0000, 1.0, 0.0, 0.0, 0.0, 93.0000, 1.0, 0.8313, 0.5711, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.3275, 30.0000, 1.0, 0.0, 0.0, 0.0, 93.0000, 1.0, 0.7426, 0.2276, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.3305, 30.0000, 1.0, 0.0, 0.0, 0.0, 93.0000, 1.0, 0.7227, 0.2299, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.3355, 30.0000, 1.0, 0.0, 0.0, 0.0, 93.0000, 1.0, 0.7032, 0.2359, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.3419, 30.0000, 1.0, 0.0, 0.0, 0.0, 93.0000, 1.0, 0.6845, 0.2455, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.3523, 30.0000, 1.0, 0.0, 0.0, 0.0, 93.0000, 1.0, 0.6589, 0.2660, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.3587, 30.0000, 1.0, 0.0, 0.0, 0.0, 93.0000, 1.0, 0.6438, 0.2835, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.1117, 27.0000, 1.0, 0.0, 0.0, 0.0, 84.0000, 1.0, 0.3153, 0.7215, 0, 1.0, 0.0, 7.0});
        treeSet.add(new double[]{1, 0.1595, 11.0000, 1.0, 0.0, 0.0, 0.0, 84.0000, 1.0, 0.1695, 0.4108, 0, 1.0, 1.0, 7.0});
        treeSet.add(new double[]{1, 0.4153, 11.0000, 1.0, 0.0, 0.0, 0.0, 72.0000, 1.0, 0.6029, 0.8701, 0, 1.0, 0.0, 6.0});
        treeSet.add(new double[]{1, 0.4352, 11.0000, 1.0, 0.0, 0.0, 0.0, 72.0000, 1.0, 0.6468, 0.8697, 1, 1.0, 1.0, 9.0});
        treeSet.add(new double[]{1, 0.4332, 11.0000, 1.0, 0.0, 0.0, 0.0, 72.0000, 1.0, 0.6655, 0.8606, 1, 1.0, 1.0, 9.0});
        treeSet.add(new double[]{1, 0.4292, 11.0000, 1.0, 0.0, 0.0, 0.0, 72.0000, 1.0, 0.6832, 0.8481, 1, 1.0, 1.0, 9.0});
        treeSet.add(new double[]{1, 0.4230, 11.0000, 1.0, 0.0, 0.0, 0.0, 69.0000, 1.0, 0.6994, 0.8324, 1, 1.0, 1.0, 9.0});
        treeSet.add(new double[]{1, 0.4148, 11.0000, 1.0, 0.0, 0.0, 0.0, 69.0000, 1.0, 0.7137, 0.8139, 1, 1.0, 1.0, 9.0});
        treeSet.add(new double[]{1, 0.4045, 11.0000, 1.0, 0.0, 0.0, 0.0, 69.0000, 1.0, 0.7260, 0.7929, 1, 1.0, 1.0, 9.0});
        treeSet.add(new double[]{1, 0.3923, 11.0000, 1.0, 0.0, 0.0, 0.0, 69.0000, 1.0, 0.7360, 0.7698, 1, 1.0, 1.0, 9.0});
        treeSet.add(new double[]{1, 0.2942, 11.0000, 1.0, 0.0, 0.0, 0.0, 66.0000, 1.0, 0.7202, 0.5685, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.2911, 11.0000, 1.0, 0.0, 0.0, 0.0, 66.0000, 1.0, 0.7068, 0.5487, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.2894, 11.0000, 1.0, 0.0, 0.0, 0.0, 66.0000, 1.0, 0.6915, 0.5315, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.2885, 11.0000, 1.0, 0.0, 0.0, 0.0, 66.0000, 1.0, 0.6745, 0.5174, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.2882, 11.0000, 1.0, 0.0, 0.0, 0.0, 66.0000, 1.0, 0.6563, 0.5066, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.3659, 8.0000, 1.0, 0.0, 0.0, 0.0, 57.0000, 1.0, 0.3489, 0.7605, 0, 1.0, 0.0, 6.0});
        treeSet.add(new double[]{1, 0.2727, 7.0000, 1.0, 0.0, 0.0, 0.0, 48.0000, 1.0, 0.2774, 0.3486, 0, 1.0, 0.0, 7.0});
        treeSet.add(new double[]{1, 0.0486, 5.8000, 1.0, 0.0, 0.0, 1.0, 40.8000, 1.0, 0.4379, 0.4707, 0, 1.0, 0.0, 7.0});
        treeSet.add(new double[]{0, 0.0000, 0.0000, 0.0, 0.0, 0.0, 0.0, 100.0000, 1.0, 0.8455, 0.9278, 0, 0.0, 0.0, 9.0});
        treeSet.add(new double[]{1, 0.5128, 100.0000, 1.0, 0.0, 0.0, 0.0, 100.0000, 1.0, 0.8485, 0.9220, 1, 1.0, 1.0, 9.0});
        treeSet.add(new double[]{1, 0.4944, 100.0000, 1.0, 0.0, 0.0, 0.0, 100.0000, 1.0, 0.8501, 0.9148, 1, 1.0, 1.0, 9.0});
        treeSet.add(new double[]{1, 0.4706, 100.0000, 1.0, 0.0, 0.0, 0.0, 100.0000, 1.0, 0.8504, 0.9007, 1, 1.0, 1.0, 9.0});
        treeSet.add(new double[]{1, 0.4427, 100.0000, 1.0, 0.0, 0.0, 0.0, 100.0000, 1.0, 0.8474, 0.8802, 1, 1.0, 1.0, 9.0});
        treeSet.add(new double[]{1, 0.4120, 100.0000, 1.0, 0.0, 0.0, 0.0, 100.0000, 1.0, 0.8405, 0.8557, 1, 1.0, 1.0, 9.0});
        treeSet.add(new double[]{1, 0.3822, 100.0000, 1.0, 0.0, 0.0, 0.0, 100.0000, 1.0, 0.8308, 0.8324, 1, 1.0, 1.0, 9.0});
        treeSet.add(new double[]{1, 0.3526, 100.0000, 1.0, 0.0, 0.0, 0.0, 100.0000, 1.0, 0.8189, 0.8110, 1, 1.0, 1.0, 9.0});
        treeSet.add(new double[]{1, 0.3233, 100.0000, 1.0, 0.0, 0.0, 0.0, 100.0000, 1.0, 0.8050, 0.7918, 1, 1.0, 0.0, 9.0});
        treeSet.add(new double[]{1, 0.2940, 100.0000, 1.0, 0.0, 0.0, 0.0, 100.0000, 1.0, 0.7908, 0.7730, 1, 1.0, 0.0, 9.0});
        treeSet.add(new double[]{1, 0.2646, 100.0000, 1.0, 0.0, 0.0, 0.0, 100.0000, 1.0, 0.7767, 0.7541, 1, 1.0, 0.0, 9.0});
        treeSet.add(new double[]{1, 0.2355, 100.0000, 1.0, 0.0, 0.0, 0.0, 100.0000, 1.0, 0.7626, 0.7353, 1, 1.0, 0.0, 8.0});
        treeSet.add(new double[]{1, 0.2079, 100.0000, 1.0, 0.0, 0.0, 0.0, 100.0000, 1.0, 0.7470, 0.7185, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.1826, 100.0000, 1.0, 0.0, 0.0, 0.0, 97.0000, 1.0, 0.7299, 0.7048, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.1604, 99.0000, 1.0, 0.0, 0.0, 0.0, 97.0000, 1.0, 0.7115, 0.6943, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.1416, 99.0000, 1.0, 0.0, 0.0, 0.0, 97.0000, 1.0, 0.6922, 0.6874, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.1265, 99.0000, 1.0, 0.0, 0.0, 0.0, 97.0000, 1.0, 0.6724, 0.6842, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.1152, 99.0000, 1.0, 0.0, 0.0, 0.0, 97.0000, 1.0, 0.6524, 0.6847, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.2894, 99.0000, 1.0, 0.0, 0.0, 0.0, 89.2443, 1.0, 0.3910, 0.9700, 0, 1.0, 0.0, 6.0});
        treeSet.add(new double[]{1, 0.2861, 99.0000, 1.0, 0.0, 0.0, 0.0, 89.2443, 1.0, 0.3910, 0.9700, 0, 1.0, 1.0, 6.0});
        treeSet.add(new double[]{1, 0.2814, 99.0000, 1.0, 0.0, 0.0, 0.0, 86.2443, 1.0, 0.3910, 0.9700, 0, 1.0, 1.0, 6.0});
        treeSet.add(new double[]{1, 0.2319, 83.0000, 1.0, 0.0, 0.0, 0.0, 85.2443, 1.0, 0.6497, 0.3488, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.2436, 83.0000, 1.0, 0.0, 0.0, 0.0, 85.2443, 1.0, 0.6688, 0.3411, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.2593, 83.0000, 1.0, 0.0, 0.0, 0.0, 85.2443, 1.0, 0.6870, 0.3300, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.2787, 83.0000, 1.0, 0.0, 0.0, 0.0, 85.2443, 1.0, 0.7038, 0.3156, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.3014, 83.0000, 1.0, 0.0, 0.0, 0.0, 82.2443, 1.0, 0.7189, 0.2982, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.3550, 83.0000, 1.0, 0.0, 0.0, 0.0, 82.2443, 1.0, 0.7431, 0.2559, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.3848, 83.0000, 1.0, 0.0, 0.0, 0.0, 82.2443, 1.0, 0.7516, 0.2318, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.4159, 83.0000, 1.0, 0.0, 0.0, 0.0, 82.2443, 1.0, 0.7576, 0.2063, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.4477, 83.0000, 1.0, 0.0, 0.0, 0.0, 82.2443, 1.0, 0.7608, 0.1800, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.5025, 80.0000, 1.0, 0.0, 0.0, 0.0, 82.2443, 1.0, 0.7673, 0.1327, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.5157, 80.0000, 1.0, 0.0, 0.0, 0.0, 79.2443, 1.0, 0.7713, 0.1066, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.5259, 80.0000, 1.0, 0.0, 0.0, 0.0, 79.2443, 1.0, 0.7725, 0.0800, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.5315, 80.0000, 1.0, 0.0, 0.0, 0.0, 79.2443, 1.0, 0.7709, 0.0534, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.4834, 80.0000, 1.0, 0.0, 0.0, 0.0, 76.7443, 1.0, 0.7655, 0.0301, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.4687, 64.0000, 1.0, 0.0, 0.0, 0.0, 85.4318, 1.0, 0.7653, 0.0300, 0, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.4543, 64.0000, 1.0, 0.0, 0.0, 0.0, 82.4318, 1.0, 0.7616, 0.0311, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.4385, 64.0000, 1.0, 0.0, 0.0, 0.0, 82.4318, 1.0, 0.7540, 0.0367, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.4202, 64.0000, 1.0, 0.0, 0.0, 0.0, 82.4318, 1.0, 0.7438, 0.0490, 1, 1.0, 0.0, 8.0});
        treeSet.add(new double[]{1, 0.4004, 64.0000, 1.0, 0.0, 0.0, 0.0, 82.4318, 1.0, 0.7306, 0.0667, 1, 1.0, 0.0, 8.0});
        treeSet.add(new double[]{1, 0.3818, 64.0000, 1.0, 0.0, 0.0, 0.0, 82.4318, 1.0, 0.7164, 0.0856, 1, 1.0, 0.0, 8.0});
        treeSet.add(new double[]{1, 0.3654, 64.0000, 1.0, 0.0, 0.0, 0.0, 82.4318, 1.0, 0.7023, 0.1044, 1, 1.0, 0.0, 8.0});
        treeSet.add(new double[]{1, 0.3513, 64.0000, 1.0, 0.0, 0.0, 0.0, 82.4318, 1.0, 0.6882, 0.1233, 1, 1.0, 0.0, 8.0});
        treeSet.add(new double[]{1, 0.3394, 64.0000, 1.0, 0.0, 0.0, 0.0, 82.4318, 1.0, 0.6740, 0.1421, 1, 1.0, 0.0, 8.0});
        treeSet.add(new double[]{1, 0.3299, 64.0000, 1.0, 0.0, 0.0, 0.0, 79.4318, 1.0, 0.6599, 0.1610, 1, 1.0, 0.0, 8.0});
        treeSet.add(new double[]{1, 0.3226, 64.0000, 1.0, 0.0, 0.0, 0.0, 79.4318, 1.0, 0.6457, 0.1798, 1, 1.0, 0.0, 8.0});
        treeSet.add(new double[]{1, 0.5825, 45.4000, 1.0, 0.0, 1.0, 0.0, 58.8318, 1.0, 0.6341, 0.9700, 0, 1.0, 0.0, 6.0});
        treeSet.add(new double[]{1, 0.5807, 45.4000, 1.0, 0.0, 0.0, 0.0, 58.8318, 1.0, 0.6341, 0.9700, 0, 1.0, 1.0, 6.0});
        treeSet.add(new double[]{1, 0.5817, 45.4000, 1.0, 0.0, 0.0, 0.0, 58.8318, 1.0, 0.6341, 0.9700, 0, 1.0, 1.0, 6.0});
        treeSet.add(new double[]{1, 0.5939, 45.4000, 1.0, 0.0, 0.0, 0.0, 58.8318, 1.0, 0.6445, 0.9611, 1, 1.0, 1.0, 9.0});
        treeSet.add(new double[]{1, 0.6012, 45.4000, 1.0, 0.0, 0.0, 0.0, 58.8318, 1.0, 0.6531, 0.9468, 1, 1.0, 1.0, 9.0});
        treeSet.add(new double[]{1, 0.6074, 45.4000, 1.0, 0.0, 0.0, 0.0, 58.8318, 1.0, 0.6618, 0.9247, 1, 1.0, 1.0, 9.0});
        treeSet.add(new double[]{1, 0.6128, 45.4000, 1.0, 0.0, 0.0, 0.0, 55.8318, 1.0, 0.6685, 0.8996, 1, 1.0, 1.0, 9.0});
        treeSet.add(new double[]{1, 0.6176, 45.4000, 1.0, 0.0, 0.0, 0.0, 55.8318, 1.0, 0.6725, 0.8735, 1, 1.0, 1.0, 9.0});
        treeSet.add(new double[]{1, 0.6219, 45.4000, 1.0, 0.0, 0.0, 0.0, 55.8318, 1.0, 0.6738, 0.8469, 1, 1.0, 1.0, 9.0});
        treeSet.add(new double[]{1, 0.6254, 45.4000, 1.0, 0.0, 0.0, 0.0, 55.8318, 1.0, 0.6722, 0.8203, 1, 1.0, 1.0, 9.0});
        treeSet.add(new double[]{1, 0.6282, 45.4000, 1.0, 0.0, 0.0, 0.0, 55.8318, 1.0, 0.6679, 0.7943, 1, 1.0, 1.0, 9.0});
        treeSet.add(new double[]{1, 0.6302, 45.4000, 1.0, 0.0, 0.0, 0.0, 55.8318, 1.0, 0.6610, 0.7693, 1, 1.0, 1.0, 9.0});
        treeSet.add(new double[]{1, 0.1152, 67.0000, 1.0, 0.0, 0.0, 0.0, 106.0000, 1.0, 0.6586, 0.8860, 1, 1.0, 1.0, 9.0});
        treeSet.add(new double[]{1, 0.1337, 67.0000, 1.0, 0.0, 0.0, 0.0, 106.0000, 1.0, 0.6689, 0.8632, 1, 1.0, 1.0, 9.0});
        treeSet.add(new double[]{1, 0.1535, 67.0000, 1.0, 0.0, 0.0, 0.0, 106.0000, 1.0, 0.6767, 0.8386, 1, 1.0, 1.0, 9.0});
        treeSet.add(new double[]{1, 0.1708, 64.0000, 1.0, 0.0, 0.0, 0.0, 106.0000, 1.0, 0.6819, 0.8129, 1, 1.0, 1.0, 9.0});
        treeSet.add(new double[]{1, 0.1805, 64.0000, 1.0, 0.0, 0.0, 0.0, 106.0000, 1.0, 0.6843, 0.7864, 1, 1.0, 1.0, 9.0});
        treeSet.add(new double[]{1, 0.0456, 28.2000, 1.0, 0.0, 0.0, 0.0, 109.2000, 1.0, 0.3671, 0.7456, 0, 1.0, 1.0, 7.0});
        treeSet.add(new double[]{1, 0.3886, 38.4000, 1.0, 0.0, 0.0, 0.0, 84.4000, 1.0, 0.4673, 0.1990, 0, 1.0, 1.0, 7.0});
        treeSet.add(new double[]{1, 0.1132, 21.4000, 1.0, 0.0, 0.0, 0.0, 20.4000, 1.0, 0.7453, 0.2716, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.1079, 21.4000, 1.0, 0.0, 0.0, 0.0, 17.4000, 1.0, 0.7254, 0.2739, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.1056, 21.4000, 1.0, 0.0, 0.0, 0.0, 17.4000, 1.0, 0.7059, 0.2799, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.1066, 21.4000, 1.0, 0.0, 0.0, 0.0, 17.4000, 1.0, 0.6872, 0.2895, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.1107, 21.4000, 1.0, 0.0, 0.0, 0.0, 17.4000, 1.0, 0.6697, 0.3024, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.1176, 21.4000, 1.0, 0.0, 0.0, 0.0, 17.4000, 1.0, 0.6538, 0.3184, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.1666, 7.4000, 1.0, 0.0, 0.0, 0.0, 13.4000, 1.0, 0.3318, 0.7471, 0, 1.0, 0.0, 7.0});
        treeSet.add(new double[]{1, 0.4881, 7.4000, 1.0, 0.0, 0.0, 0.0, 4.4000, 1.0, 0.2473, 0.4058, 0, 1.0, 1.0, 7.0});
        treeSet.add(new double[]{0, 0.0000, 0.0000, 0.0, 0.0, 0.0, 0.0, 100.0000, 1.0, 0.0535, 0.6635, 1, 0.0, 1.0, 4.0});
        treeSet.add(new double[]{0, 0.0000, 0.0000, 0.0, 0.0, 0.0, 0.0, 100.0000, 1.0, 0.0584, 0.6538, 1, 0.0, 1.0, 4.0});
        treeSet.add(new double[]{0, 0.0000, 0.0000, 0.0, 0.0, 0.0, 0.0, 100.0000, 1.0, 0.0687, 0.6416, 1, 0.0, 1.0, 4.0});
        treeSet.add(new double[]{1, 0.4093, 100.0000, 1.0, 0.0, 0.0, 0.0, 97.0000, 1.0, 0.0225, 0.5057, 0, 1.0, 1.0, 7.0});
        treeSet.add(new double[]{1, 0.4852, 97.0000, 1.0, 0.0, 0.0, 0.0, 82.0000, 1.0, 0.3663, 0.9700, 0, 1.0, 0.0, 6.0});
        treeSet.add(new double[]{1, 0.4875, 97.0000, 1.0, 0.0, 0.0, 0.0, 82.0000, 1.0, 0.3663, 0.9700, 0, 1.0, 1.0, 6.0});
        treeSet.add(new double[]{1, 0.4924, 97.0000, 1.0, 0.0, 0.0, 0.0, 82.0000, 1.0, 0.3663, 0.9700, 0, 1.0, 1.0, 6.0});
        treeSet.add(new double[]{1, 0.1909, 96.0000, 1.0, 0.0, 0.0, 0.0, 73.0000, 1.0, 0.6418, 0.6135, 1, 1.0, 0.0, 8.0});
        treeSet.add(new double[]{1, 0.4160, 69.0500, 1.0, 0.0, 0.0, 0.0, 62.8000, 1.0, 0.2469, 0.2892, 0, 1.0, 1.0, 7.0});
        treeSet.add(new double[]{1, 0.3487, 69.0500, 1.0, 0.0, 0.0, 0.0, 50.8000, 1.0, 0.6413, 0.7910, 0, 1.0, 0.0, 9.0});
        treeSet.add(new double[]{1, 0.3532, 69.0500, 1.0, 0.0, 0.0, 0.0, 50.8000, 1.0, 0.6465, 0.7951, 1, 1.0, 1.0, 9.0});
        treeSet.add(new double[]{1, 0.3630, 69.0500, 1.0, 0.0, 0.0, 0.0, 50.8000, 1.0, 0.6572, 0.7990, 1, 1.0, 1.0, 9.0});
        treeSet.add(new double[]{1, 0.3767, 69.0500, 1.0, 0.0, 0.0, 0.0, 50.8000, 1.0, 0.6732, 0.8002, 1, 1.0, 1.0, 9.0});
        treeSet.add(new double[]{1, 0.3934, 69.0500, 1.0, 0.0, 0.0, 0.0, 47.8000, 1.0, 0.6930, 0.7972, 1, 1.0, 1.0, 9.0});
        treeSet.add(new double[]{1, 0.5221, 0.5500, 1.0, 0.0, 0.0, 0.0, 29.8000, 1.0, 0.7609, 0.2730, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.4993, 0.5500, 1.0, 0.0, 0.0, 0.0, 29.8000, 1.0, 0.7409, 0.2716, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.4785, 0.5500, 1.0, 0.0, 0.0, 0.0, 29.8000, 1.0, 0.7210, 0.2739, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.4605, 0.5500, 1.0, 0.0, 0.0, 0.0, 29.8000, 1.0, 0.7015, 0.2799, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.4459, 0.5500, 1.0, 0.0, 0.0, 0.0, 29.8000, 1.0, 0.6829, 0.2895, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.4354, 0.5500, 1.0, 0.0, 0.0, 0.0, 29.8000, 1.0, 0.6654, 0.3024, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.4295, 0.5500, 1.0, 0.0, 0.0, 0.0, 29.8000, 1.0, 0.6494, 0.3184, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.3933, 0.0000, 1.0, 0.0, 0.0, 0.0, 20.8000, 1.0, 0.3298, 0.7439, 0, 1.0, 0.0, 7.0});
        treeSet.add(new double[]{1, 0.4603, 61.3210, 1.0, 0.0, 0.0, 0.0, 19.1827, 1.0, 0.8116, 0.6492, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.4380, 61.3210, 1.0, 0.0, 0.0, 0.0, 19.1827, 1.0, 0.8114, 0.6226, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.4143, 61.3210, 1.0, 0.0, 0.0, 0.0, 19.1827, 1.0, 0.8084, 0.5962, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.3894, 61.3210, 1.0, 0.0, 0.0, 0.0, 19.1827, 1.0, 0.8027, 0.5707, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.3634, 61.3210, 1.0, 0.0, 0.0, 0.0, 19.1827, 1.0, 0.7944, 0.5465, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.3365, 61.3210, 1.0, 0.0, 0.0, 0.0, 19.1827, 1.0, 0.7836, 0.5240, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.3089, 61.3210, 1.0, 0.0, 0.0, 0.0, 16.1827, 1.0, 0.7706, 0.5038, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.2806, 59.2802, 1.0, 0.0, 0.0, 0.0, 16.1827, 1.0, 0.7556, 0.4862, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.2520, 59.2802, 1.0, 0.0, 0.0, 0.0, 16.1827, 1.0, 0.7389, 0.4715, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.2232, 59.2802, 1.0, 0.0, 0.0, 0.0, 16.1827, 1.0, 0.7209, 0.4601, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.1944, 59.2802, 1.0, 0.0, 0.0, 0.0, 16.1827, 1.0, 0.7018, 0.4521, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.1659, 43.2802, 1.0, 0.0, 0.0, 0.0, 25.1827, 1.0, 0.6821, 0.4478, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.1380, 43.2802, 1.0, 0.0, 0.0, 0.0, 25.1827, 1.0, 0.6621, 0.4471, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{1, 0.1110, 43.2802, 1.0, 0.0, 0.0, 0.0, 25.1827, 1.0, 0.6422, 0.4502, 1, 1.0, 1.0, 8.0});
        treeSet.add(new double[]{0, 0.0000, 0.0000, 0.0, 1.0, 0.0, 0.0, 0.0000, 1.0, 0.5957, 0.4735, 0, 1.0, 0.0, 7.0});
    }

	public static Object[] root = buildDecisionTree(treeSet, 14, 1, 0.01, 70);

	static{
		fireTraining = new DataSet(3, 1);
		// Entrada: {distancia do inimigo, energia do inimigo, minha energia}
		// Saída: Força do tiro
		fireTraining.addRow(new double[]{0.750, 0.25, 0.30}, new double[]{0.33});
		fireTraining.addRow(new double[]{0.600, 0.99, 0.99}, new double[]{0.33});
		fireTraining.addRow(new double[]{0.700, 0.50, 0.30}, new double[]{0.33});
		fireTraining.addRow(new double[]{0.710, 0.55, 0.76}, new double[]{0.33});
		fireTraining.addRow(new double[]{0.700, 0.45, 0.50}, new double[]{0.33});

		fireTraining.addRow(new double[]{0.300, 1.00, 0.95}, new double[]{0.66});
		fireTraining.addRow(new double[]{0.550, 0.20, 0.89}, new double[]{0.66});
		fireTraining.addRow(new double[]{0.350, 0.73, 0.99}, new double[]{0.66});
		fireTraining.addRow(new double[]{0.500, 0.50, 0.91}, new double[]{0.66});
		fireTraining.addRow(new double[]{0.350, 0.70, 0.50}, new double[]{0.66});
		fireTraining.addRow(new double[]{0.700, 0.70, 0.30}, new double[]{0.66});

		fireTraining.addRow(new double[]{0.030, 1.00, 1.00}, new double[]{0.99});
		fireTraining.addRow(new double[]{0.050, 0.25, 0.88}, new double[]{0.99});
		fireTraining.addRow(new double[]{0.050, 0.10, 0.06}, new double[]{0.99});
		fireTraining.addRow(new double[]{0.050, 0.20, 0.25}, new double[]{0.99});
		fireTraining.addRow(new double[]{0.400, 0.07, 1.00}, new double[]{0.99});
	}


	public void firepercept() {
		firePerceptron.randomizeWeights();
        firePerceptron.getLearningRule().setLearningRate(0.01);
        firePerceptron.getLearningRule().setMaxIterations(100000);
	}

	public void run() {
		// activates radars
		setAdjustRadarForGunTurn(true);
		setAdjustRadarForRobotTurn(true);
		setAdjustGunForRobotTurn(true);

		setTurnRadarRightRadians(Double.POSITIVE_INFINITY);

		maxDistance = Math.sqrt(getBattleFieldHeight()*getBattleFieldHeight() + getBattleFieldWidth()*getBattleFieldWidth());
		setColors(Color.pink, Color.black, Color.red);

		firepercept();

		boolean go = true;

		while(true) {
	        double predict = predict(root, dataToPredict());
//	        if (predict != 5.0) {
//	        	System	.err.println("[" +robotscanned + ", " + formatDecimalNumber(distancetorobot) + ", " + formatDecimalNumber(enemyenergy) + ", " + gunaimedtoenemy + ", " + hitbybullet + ", " + hitwall + ", " + hitrobot + ", " + formatDecimalNumber(energy) + ", " + heat + ", " + formatDecimalNumber(positionx) + ", " + formatDecimalNumber(positiony) + ", " + moving + ", " + gunturning + ", " + robotturning + ", " + predict +"],\n");
//	        }

	        if (predict == 3.0) {
				if (Enemy != null) {
					setTurnGunRight((Enemy.getBearing() + getHeading() + 360)%360 - getGunHeading());
				} else {
					setTurnRadarRightRadians(Double.POSITIVE_INFINITY);
				}
			} else if(predict == 4.0) {
				setTurnRadarRightRadians(Double.POSITIVE_INFINITY);
			} else if (predict == 5.0) {
				if (Enemy != null) {
					setTurnRight(Enemy.getBearing() + 90 - 30 * moveDirection);
					double changeInEnergy = lastTurnEnergy -Enemy.getEnergy();

					if (changeInEnergy > 0 && changeInEnergy<=3) {
						moveDirection = - moveDirection;
						setAhead(120 * moveDirection);
					}
				} else {
					setTurnRadarRightRadians(Double.POSITIVE_INFINITY);
				}
			} else if (predict == 6.0) {
				pressSD();
			} else if (predict == 7.0) {
				pressWD();
			} else if (predict == 8.0) {
				pressWA();
			} else if (predict == 9.0) {
				pressSA();
			} else if (predict == 10.0) {
				setTurnRight(180);
			}

			if(getDistanceRemaining() == 0 && 0 < getTime()){
				double x = 0, y = 0;
				x = Math.random()*getBattleFieldWidth();
				y = Math.random()*getBattleFieldHeight();

				// Radar Turn
				double angle = (Math.toDegrees(Math.atan((x-getX())/(y-getY())))+(y < getY() ? 180 : 360))%360;
				if(Double.isNaN(angle)) {
					angle = x > getX() ? 90 : 270;
				}

				//System.err.println("At x: " + getX() + " y: " + getY() + " Move to x: " + x + " y: " + y + " Angle: " + angle);
				double turn = angle - getHeading();
				if(Math.abs(turn) > 180) {
					turn = (360 - Math.abs(turn))*Math.signum(turn)*-1;
				}

				if(Math.abs(turn) > 90){
					turn = Math.signum(turn)*-1*(180-Math.abs(turn));
					go = false;
				} else {
					go = true;
				}

				setTurnRight(turn);

				// Se mantém procurando por um inimigo enquanto não encontrar
				do{
					scan();
				} while(getTurnRemaining() != 0);

				// Determina o quanto o robô deve andar
				double distance = Math.sqrt(Math.pow(x - getX(), 2) + Math.pow(y - getY(), 2));

				// Informa se o robô deve andar pra frente ou pra trás
				if (go) {
					setAhead(distance);
				} else {
					setBack(distance);
				}
			}

			scan();

			robotscanned = 0;
			distancetorobot = 0;
			enemyenergy = 0;
			gunaimedtoenemy = 0;
			hitbybullet = 0;
			hitwall = 0;
			hitrobot = 0;
			energy = 0;
			heat = 0;
			positionx = 0;
			positiony = 0;
			moving = 0;
			gunturning = 0;
			robotturning = 0;
			Enemy = null;

			scan();
		}
	}

	public double[] dataToPredict() {
		if (getGunHeat() > 0) {
			heat = 1;
		}

		energy = getEnergy();
		positionx = getX()/getBattleFieldWidth();
		positiony = getY()/getBattleFieldHeight();
		if (getDistanceRemaining() != 0) {
			moving = 1;
		}

		if (getGunTurnRemaining() != 0 ) {
			gunturning = 1;
		}

		if (getTurnRemaining() != 0) {
			robotturning = 1;
		}

		return new double[] {
			robotscanned,
			distancetorobot,
			enemyenergy,
			gunaimedtoenemy,
			hitbybullet,
			hitwall,
			hitrobot,
			energy,
			heat,
			positionx,
			positiony,
			moving,
			gunturning,
			robotturning,
			0
		};

	}

	public void onScannedRobot(ScannedRobotEvent e) {
		Enemy = e;
		robotscanned = 1;
		distancetorobot = e.getDistance()/maxDistance;
		enemyenergy = e.getEnergy();

		gunf(e);
		radarf(e);

        if(isInTheSights(e)){
        	gunaimedtoenemy = 1;
        	fired(e);
        } else {
        	gunf(e);
    		double energy = lastTurnEnergy - e.getEnergy();
    		if (energy <= 3) {
    			// zigzag
    			moveDirection = -moveDirection;
    			setTurnRight(e.getBearing() - 30 + 90 * moveDirection);
    			setAhead(120 * moveDirection);
    		} else{
    			// go to enemy
    			setTurnRight(e.getBearing());
    			setAhead(100);
    		}
    		setTurnGunRight(reposition(e));
    		lastTurnEnergy = e.getEnergy();
        }
	}

	/**
	 * POSSIBLE FAULTS EVENTS
	 */

	public void onHitWall(HitWallEvent e) {
		hitwall = 1;
		reverseDirection();
	}

	public void onHitRobot(HitRobotEvent event) {
		hitrobot = 1;
	}

	/**
	 * BULLET EVENTS
	 */

	public void onHitByBullet(HitByBulletEvent e) {
		hitbybullet = 1;
	}

	public void onBulletHit(BulletHitEvent event) {
		acertos = acertos + 1;
		if (fireTraining.size() < 130)
		fireTraining.addRow(new double[]{rD, rE, mE}, new double[]{power});
	}

	public void onBulletMissed(BulletMissedEvent event) {
		erros = erros + 1;
		if (fireTraining.size() < 130)
		fireTraining.addRow(new double[]{rD, rE, mE}, new double[]{1.0});
	}

	/**
	 * END GAME
	 */

	public void onWin(WinEvent e) {
		turnRight(15);
		while(true) {
			turnLeft(30);
			turnRight(30);
		}
	}

	public void onRoundEnded(RoundEndedEvent e) {
//		System.err.println("Tiros: " + disparos + " Acertos: " + acertos + " Erros: " + erros);
	}

	/**
	 *
	 *
	 * HELPERS
	 *
	 *
	 */

	/**
	 * shot
	 * @param e
	 */
	private void fired(ScannedRobotEvent e) {
		rD = Math.min(1d, e.getDistance()/1000d);
    	rE = Math.min(1d, e.getEnergy()/120d);
    	mE = Math.min(1d, getEnergy()/120d);

    	// Perceptron input
        firePerceptron.setInput(rD, rE, mE);
        firePerceptron.calculate();


        firePerceptron.randomizeWeights();
        // System.err.println(formatDecimalNumber(rD) + " " + formatDecimalNumber(rE) + " " + formatDecimalNumber(mE) + " actual = " + Arrays.toString(firePerceptron.getOutput()));
        // System.err.println("força: " + firePerceptron.getOutput()[0]);
        // Define poder do tiro com base no resultado

        power = firePerceptron.getOutput()[0] * 5d;

    	// if it will not disable
		if (power < getEnergy()) {
			disparos = disparos + 1;
			setFire(power);
		}
	}

	/**
	 * Gun Movement
	 * @param e
	 */
	public void gunf(ScannedRobotEvent e) {
		// Recovers enemy bearing
		double degree = e.getBearing();
		// Recovers my info
		double heading = getHeading();
		double gunHeading = getGunHeading();

		// Aim gun to the enemy
		double turn = heading - gunHeading + degree;
		if (Math.abs(turn) > 180) {
			turn = (360 - Math.abs(turn))*Math.signum(turn)*-1;
		}
		setTurnGunRight(turn);
	}

	/**
	 * Radar Movement
	 *
	 * @param e
	 */
	public void radarf(ScannedRobotEvent e) {
		// Radar turns (this movement is a widely used standard)
		double radarTurn = getHeadingRadians() + e.getBearingRadians() - getRadarHeadingRadians();
		setTurnRadarRightRadians(Utils.normalRelativeAngle(radarTurn));
	}

	/**
	 * Back Off
	 */
	public void reverseDirection() {
		if (moveDirection == 1) {
			setBack(100);
		} else {
			setAhead(100);
		}
		moveDirection = -moveDirection;
	}

    //Formating decimal number to have 3 decimal places
    public String formatDecimalNumber(double number) {
        return new BigDecimal(number).setScale(4, RoundingMode.HALF_UP).toString();
    }

    public void keepScore(double prediction, double ideal) {

        count[2]++;

        if (maxOutput(new double[] {prediction}) == ideal) {
            correct[2]++;
        }

        if (prediction < 0) {
            unpredicted++;
        }
    }

    public void move(double x, double y) {
    	setAhead(Math.sqrt(Math.pow(getBattleFieldWidth()*x - getX(), 2) + Math.pow(getBattleFieldHeight()*y - getY(), 2)));
    }

	public void pressWD() {
		setTurnRight(45 - getHeading());
		move(0.7, 0.7);
	}

	public void pressWA() {
		setTurnRight(315 - getHeading());
		move(0.3, 0.7);
	}

	public void pressSD() {
		setTurnRight(135 - getHeading());
		move(0.7, 0.3);
	}

	public void pressSA() {
		setTurnRight(225 - getHeading());
		move(0.3d, 0.3);
	}


	/**
	 * Calcula a rotação necessária para que a arma aponte novamente para o robô inimigo.
	 */
	private double reposition(ScannedRobotEvent e) {
		double degree = e.getBearing();
		double heading = getHeading();
		double gunHeading = getGunHeading();

		// Aponta arma para o inimigo
		double turn = heading - gunHeading + degree;
		//inclui ajuste devido ao movimento esperado do inimigo
		double gunTurnDegrees = Utils.normalRelativeAngleDegrees(turn)
				+ e.getVelocity() * Math.sin(e.getHeading());

		return gunTurnDegrees;
	}

	private boolean isInTheSights(ScannedRobotEvent scannedRobot){
		// 36 is the robot size
		return Math.abs(
				(scannedRobot.getBearing() + getHeading() + 360) % 360 - getGunHeading())
					<=
				Math.abs(2*180*Math.atan(36*0.5d/scannedRobot.getDistance()/Math.PI));
	}

    public static double maxOutput(double[] array) {
        double max = array[0];
        if (max < 0.5) {
            return 0.33;
        } else if (max < 0.75) {
        	return 0.66;
        } else {
        	return 0.99;
        }
    }

    /**
     *
     *
     *
     * DECISION TREE CODE
     *
     *
     *
     *
     */

	/**
	 * Calculates how many objects have the same class
	 *
	 * @param items: Array of objects
	 * @param attr: class column
	 * @return
	 */
	public static Map<Double, Integer> countUniqueValues(ArrayList<double[]> items, int attr) {
		Map<Double, Integer> counter = new HashMap<Double, Integer>();

		// detecting classes
		for (int i = 0; i < items.size(); i++) {
			counter.put(items.get(i)[attr], 0);
		}

	    // counting number of occurrences of each of values
	    // of attribute
	    for (int i = items.size() - 1; i >= 0; i--) {
	      int v = counter.get(items.get(i)[attr]) + 1;
	      counter.put(items.get(i)[attr], v);
	    }
		return counter;
	}

	/**
	 * Calculating entropy of array of objects
	 * by specific attribute
	 *
	 * @return
	 */
	public static double entropy(ArrayList<double[]>items, int attr) {
		double e = 0;
		Map<Double, Integer> counter = countUniqueValues(items, attr);
		double aux = 0;
		for (Entry<Double, Integer> entry : counter.entrySet()) {
			aux = entry.getValue() * 1.0 / items.size();
			// System.out.println(aux + " " + entry.getValue() + " " + items.length);
			e = e + (-aux * Math.log(aux));
		}

		return e;
	}

	/**
	 * Find more frequent class in dataset
	 *
	 * @param items
	 * @param attr
	 */
	public static Double mostFrequentClass(ArrayList<double[]>items, int attr) {
		Map<Double, Integer> counter = countUniqueValues(items, attr);
		int mostFrequentCount = 0;
		Double mostFrequentClass = (double) -1;

		for (Entry<Double, Integer> entry : counter.entrySet()) {
			if (entry.getValue() > mostFrequentCount) {
				mostFrequentCount = entry.getValue();
				mostFrequentClass = entry.getKey();
			}
		}
		return mostFrequentClass;
	}

	/**
	 * Splits array of objects by value of specific attribute
	 * using specific pivot
	 *
	 * @param items
	 * @param attr
	 * @param pivot
	 * @return
	 */
	public static Map<Integer, ArrayList<double[]>> split(ArrayList<double[]> items, int attr, double pivot) {
		double[] item;
		double value;
		ArrayList<double[]> match = new ArrayList<double[]>();
		ArrayList<double[]> noMatch = new ArrayList<double[]>();

		for (int i = items.size() - 1; i >= 0; i--) {
			item = items.get(i);
			value = item[attr];

			if (value >= pivot) {
				match.add(item);
			} else {
				noMatch.add(item);
			}
		}
		Map<Integer, ArrayList<double[]>> result = new HashMap<Integer, ArrayList<double[]>>();
		result.put(0, match);
		result.put(1, noMatch);

		return result;
	}

	public static Object[] buildDecisionTree(ArrayList<double[]> treeset2, int categoryAttr, int minItensCount, double entropyThrehold, int maxTreeDepth) {
		System.out.println(categoryAttr);
		if (maxTreeDepth == 0 || treeset2.size() <= minItensCount) {
			// restriction by maximal depth of tree
			// or size of training set is too small
			// so we have to terminate process of building tree
			return new Object[] {
				mostFrequentClass(treeset2, categoryAttr)
			};
		}

		double initialEntropy = entropy(treeset2, categoryAttr);

		if (initialEntropy <= entropyThrehold) {
			// entropy of training set too small
			// it means that training set is almost homogeneus
			// so we have to terminate process of building tree
			return new Object[] {
				mostFrequentClass(treeset2, categoryAttr)
			};
		}

		// used as hash-set for avoiding the checking of split by rules
	    // with the same 'attribute-predicate-pivot' more than once
		Map<Integer, Map<Double, Integer>> alreadyChecked = new HashMap<Integer, Map<Double, Integer>>();


		double bestSplitGain = 0;
		double bestSplitPivot = 0;
		int bestSplitAttribute = categoryAttr;
		Map<Integer, ArrayList<double[]>> bestSplit = null;


		for (int i = treeset2.size() - 1; i >= 0; i --) {
			double[] item = treeset2.get(i);
			for (int attr = 0; attr < item.length; attr++) {
				if (attr == categoryAttr) {
					continue;
				}

				double pivot = item[attr];

				Map<Double, Integer> check = alreadyChecked.get(attr);

				if (check != null) {
					Integer check2 = check.get(pivot);
					if (check2 != null) {
						continue;
					}
				}

				Map<Double,Integer> entry = new HashMap<Double, Integer>();
				entry.put(pivot, 1);
				Integer key = attr;
				alreadyChecked.put(key, entry);

				Map<Integer, ArrayList<double[]>> currSplit = split(treeset2, attr, pivot);

				// Calculate entropy of subsets
				double matchEntropy = entropy(currSplit.get(0), categoryAttr);
				double notMatchEntropy = entropy(currSplit.get(1), categoryAttr);

				// Calculating Informational Gain
				double newEntropy = 0;
				newEntropy = newEntropy + matchEntropy * currSplit.get(0).size();
				newEntropy = newEntropy + notMatchEntropy * currSplit.get(1).size();
				newEntropy = newEntropy / treeset2.size();

				double currGain = initialEntropy - newEntropy;

				if (currGain > bestSplitGain) {
					bestSplit = currSplit;
					bestSplitPivot = pivot;
					bestSplitGain = currGain;
					bestSplitAttribute = attr;
				}
			}
		}

		if (bestSplitGain == 0) {
			// Can't find optimal split;
			return new Object[] {
				mostFrequentClass(treeset2, categoryAttr)
			};
		}

		maxTreeDepth = maxTreeDepth - 1;

		treeset2 = bestSplit.get(0);
		Object[] matchSubTree = buildDecisionTree(treeset2, categoryAttr, minItensCount, entropyThrehold, maxTreeDepth);

		treeset2 = bestSplit.get(1);
		Object[] notMatchSubTree = buildDecisionTree(treeset2, categoryAttr, minItensCount, entropyThrehold, maxTreeDepth);

		Object[] tree = new Object[] {
			bestSplitAttribute,
			bestSplitPivot,
			matchSubTree,
			notMatchSubTree,
			bestSplit.get(0).size(),
			bestSplit.get(1).size()
		};

		return tree;
	}

	  /**
	   * Classifying item, using decision tree
	   */
	public double predict(Object[] tree, double[] ds) {
		int attr;
		double pivot, value;
		// Traversing tree from the root to leaf
		while (true) {
			if (tree.length == 1) {
				// only leafs contains length 1
				return (double) tree[0];
			}

			attr = (int) tree[0];
			pivot = (double) tree[1];

			value = ds[attr];

			// move to one of subtrees
			if (value >= pivot) {
				tree = (Object[]) tree[2];
			} else {
				tree = (Object[]) tree[3];
			}
		}
	}
}
