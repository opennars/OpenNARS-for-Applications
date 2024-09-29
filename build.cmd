@echo off
setlocal enabledelayedexpansion

:: delete NAR and src\RuleTable.c
del NAR
del src\RuleTable.c

:: setup error handling
set ERRORLEVEL=0

:: get all .c file path
for /r "src" %%f in (*.c) do (
    set "Str=!Str! %%f"
)

:: print all .c file path and notify user the compilation is started
echo !Str!
echo Compilation started:

:: setup compiler parameters of gcc
set BaseFlags=-flto -g -pthread -lpthread -D_POSIX_C_SOURCE=199506L -pedantic -std=c99 -g3 -O3 !Str! -lm -oNAR

:: mute the output of compiler warnings
set NoWarn=-Wno-unknown-pragmas -Wno-tautological-compare -Wno-unused-parameter -Wno-unused-variable -Wno-strict-prototypes -Wno-implicit-function-declaration -Wno-pointer-to-int-cast -Wno-int-to-pointer-cast

:: 1st stage compilation
gcc %* -DSTAGE=1 -Wall -Wextra -Wformat-security %NoWarn% %BaseFlags%

:: check if compilation was successful
if %ERRORLEVEL% neq 0 (
    echo Error during first stage compilation.
    goto :eof
)

:: notify user that 1st stage is done, and start generating RuleTable.c
echo First stage done, generating RuleTable.c now, and finishing compilation.

:: generate RuleTable.c uses 1st stage binary
call NAR.exe NAL_GenerateRuleTable > src\RuleTable.c

:: try second stage compilation with SSE flag
gcc %* -mfpmath=sse -msse2 -DSTAGE=2 %NoWarn% %BaseFlags% src\RuleTable.c
if %ERRORLEVEL% equ 0 (
    echo Done.
) else (
    echo Error with SSE, hence compiling without SSE:
    :: if error, try compilation without SSE:
    gcc %* -DSTAGE=2 %NoWarn% %BaseFlags% src\RuleTable.c
    if %ERRORLEVEL% equ 0 (
        echo Done.
    ) else (
        echo Compilation failed.
    )
)

:end
endlocal