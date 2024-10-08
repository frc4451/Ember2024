
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
import java.util.OptionalDouble;
import java.util.Queue;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.Notifier;
import frc.robot.Constants.OdometryConstants;

/**
 * Provides an interface for asynchronously reading high-frequency measurements
 * to a set of queues.
 *
 * <p>
 * This version is intended for devices like the SparkMax that require polling
 * rather than a
 * blocking thread. A Notifier thread is used to gather samples with consistent
 * timing.
 */
public class SparkMaxOdometryThread {
    private List<Supplier<OptionalDouble>> suppliers = new ArrayList<>();
    private List<Queue<Double>> dataQueues = new ArrayList<>();
    private List<Queue<Double>> timestampQueues = new ArrayList<>();

    private final Notifier notifier;
    private static SparkMaxOdometryThread instance = null;
    protected static final Lock odometryLock = new ReentrantLock();

    public static SparkMaxOdometryThread getInstance() {
        if (instance == null) {
            instance = new SparkMaxOdometryThread();
        }
        return instance;
    }

    private SparkMaxOdometryThread() {
        notifier = new Notifier(this::periodic);
        notifier.setName("SparkMaxOdometryThread");
    }

    public void start() {
        if (timestampQueues.size() > 0) {
            notifier.startPeriodic(1.0 / OdometryConstants.kFrequency);
        }
    }

    public Queue<Double> registerSignal(Supplier<OptionalDouble> signal) {
        Queue<Double> queue = new ArrayBlockingQueue<>(OdometryConstants.kCacheCapacity);
        odometryLock.lock();
        try {
            suppliers.add(signal);
            dataQueues.add(queue);
        } finally {
            odometryLock.unlock();
        }
        return queue;
    }

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

    private void periodic() {
        odometryLock.lock();
        double timestamp = Logger.getRealTimestamp() / 1e6;
        try {
            double[] values = new double[suppliers.size()];
            boolean isValid = true;
            for (int i = 0; i < suppliers.size(); i++) {
                OptionalDouble value = suppliers.get(i).get();
                if (value.isPresent()) {
                    values[i] = value.getAsDouble();
                } else {
                    isValid = false;
                    break;
                }
            }
            if (isValid) {
                for (int i = 0; i < dataQueues.size(); i++) {
                    dataQueues.get(i).offer(values[i]);
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
