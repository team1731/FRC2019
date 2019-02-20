set pathToCopy=F:\_LOGS\BATTLEFIELD\
set wantedDate=????_??_?? ??_??_?? ???
::             YYYY_MM_DD HH_MM_SS WKDAY
echo Shoving driver station logs to %pathToCopy%
cd /D "C:\Users\Public\Documents\FRC\Log Files"
del /S F:\_LOGS\BATTLEFIELD\
copy "%wantedDate%.*" %pathToCopy%
echo Finished Shoving files.
PAUSE