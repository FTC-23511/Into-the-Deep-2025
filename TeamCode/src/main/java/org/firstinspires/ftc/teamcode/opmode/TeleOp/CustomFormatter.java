package org.firstinspires.ftc.teamcode.opmode.TeleOp;

import android.annotation.SuppressLint;

import java.util.logging.Formatter;
import java.util.logging.LogRecord;

public class CustomFormatter extends Formatter {
        @Override
        public String format(LogRecord record) {

            String timestamp = String.format("%1$tF %1$tT", new java.util.Date(record.getMillis()));

            String objectType = record.getLevel().getName();

            String value = record.getMessage();

            return timestamp + "," + objectType + "," + value + "\n";
        }
    }

