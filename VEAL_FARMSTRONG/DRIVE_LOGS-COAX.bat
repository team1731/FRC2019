set pathToTake=F:\_LOGS\BATTLEFIELD
echo Cleaning up current log files, with user's permission...
del /S "C:\Users\Public\Documents\FRC\Log Files"
echo Coaxing files down from %pathToTake%
xcopy /s %pathToTake% "C:\Users\Public\Documents\FRC\Log Files"
echo Finished Coaxing files.
PAUSE