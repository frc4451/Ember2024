@echo off
rem Script for syncing logs from the robot to the driver station
rem This does not use `/mir` since we don't want to delete logs
rem we have previously collected.

setlocal

rem Define source and destination paths
set "source=D:\logs"
set "destination=%USERPROFILE%\Documents\drive_logs"
set "logfile=%USERPROFILE%\Documents\robocopy_log.txt"

rem Run Robocopy command. We don't have `rsync` in Windows.
robocopy "%source%" "%destination%" /copy:DAT /r:3 /w:3 /log:"%logfile%"

echo "Successfully synced data from %source% to %destination%"
echo "You can read the logs further from %logfile%"

rem Pause so the user can see the pass/fail copy check.
pause

endlocal
