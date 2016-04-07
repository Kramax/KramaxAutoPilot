@echo off

set MOD_NAME=KramaxAutoPilot

set DIR=%MOD_NAME%_%1

mkdir Release\%DIR%

xcopy /s /f /y GameData Release\%DIR%\GameData\
copy /y LICENSE.txt Release\%DIR%\GameData\%MOD_NAME%\
copy /y README.md Release\%DIR%\GameData\%MOD_NAME%\

cd Release\%DIR%
rem zip -r ../%DIR%.zip GameData
"c:\Program Files\7-Zip\7z.exe"  a -r ../%DIR%.zip GameData
cd ..\..
