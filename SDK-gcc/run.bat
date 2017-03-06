@echo off
set BASE_PATH=%~dp0
set BIN_PATH=%BASE_PATH%bin\
for /r %BASE_PATH%..\case_example\ %%I in (case*.txt) do (
call %BIN_PATH%cdn.exe %%I ..\case_result\%%~nxI
)
pause