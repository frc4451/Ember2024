// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import java.util.ArrayList;
import java.util.List;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkBase;
import com.revrobotics.REVLibError;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.OdometryConstants;

/**
 * Provides an interface for asynchronously reading high-frequency measurements
 * to a set of queues.
 *
 * <p>
 * This version includes an overload for Spark signals, which checks for errors
 * to ensure that
 * all measurements in the sample are valid.
 */
public class SparkOdometryThread {
    private final List<CANSparkBase> sparks = new ArrayList<>();
    private final List<DoubleSupplier> sparkSignals = new ArrayList<>();
    private final List<DoubleSupplier> genericSignals = new ArrayList<>();
    private final List<Queue<Double>> sparkQueues = new ArrayList<>();
    private final List<Queue<Double>> genericQueues = new ArrayList<>();
    private final List<Queue<Double>> timestampQueues = new ArrayList<>();

    private static SparkOdometryThread instance = null;
    private Notifier notifier = new Notifier(this::run);

    /** Used to prevent odometry updates while locked */
    static final Lock odometryLock = new ReentrantLock();

    public static SparkOdometryThread getInstance() {
        if (instance == null) {
            instance = new SparkOdometryThread();
        }
        return instance;
    }

    private SparkOdometryThread() {
        notifier.setName("OdometryThread");
    }

    public void start() {
        if (timestampQueues.size() > 0) {
            notifier.startPeriodic(1.0 / OdometryConstants.kFrequency);
        }
    }

    /** Registers a Spark signal to be read from the thread. */
    public Queue<Double> registerSignal(CANSparkBase spark, DoubleSupplier signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(OdometryConstants.kCacheCapacity);
        odometryLock.lock();
        try {
            sparks.add(spark);
            sparkSignals.add(signal);
            sparkQueues.add(queue);
        } finally {
            odometryLock.unlock();
        }
        return queue;
    }

    /** Registers a generic signal to be read from the thread. */
    public Queue<Double> registerSignal(DoubleSupplier signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(OdometryConstants.kCacheCapacity);
        odometryLock.lock();
        try {
            genericSignals.add(signal);
            genericQueues.add(queue);
        } finally {
            odometryLock.unlock();
        }
        return queue;
    }

    /** Returns a new queue that returns timestamp values for each sample. */
    public Queue<Double> makeTimestampQueue() {
        Queue<Double> queue = new ArrayBlockingQueue<>(OdometryConstants.kCacheCapacity);
        odometryLock.lock();
        try {
            timestampQueues.add(queue);
        } finally {
            odometryLock.unlock();
        }
        return queue;
    }

    private void run() {
        // Save new data to queues
        odometryLock.lock();
        try {
            // Get sample timestamp
            double timestamp = RobotController.getFPGATime() / 1e6;

            // Read Spark values, mark invalid in case of error
            double[] sparkValues = new double[sparkSignals.size()];
            boolean isValid = true;
            for (int i = 0; i < sparkSignals.size(); i++) {
                sparkValues[i] = sparkSignals.get(i).getAsDouble();
                if (sparks.get(i).getLastError() != REVLibError.kOk) {
                    isValid = false;
                }
            }

            // If valid, add values to queues
            if (isValid) {
                for (int i = 0; i < sparkSignals.size(); i++) {
                    sparkQueues.get(i).offer(sparkValues[i]);
                }
                for (int i = 0; i < genericSignals.size(); i++) {
                    genericQueues.get(i).offer(genericSignals.get(i).getAsDouble());
                }
                for (int i = 0; i < timestampQueues.size(); i++) {
                    timestampQueues.get(i).offer(timestamp);
                }
            }
        } finally {
            odometryLock.unlock();
        }
    }
}
