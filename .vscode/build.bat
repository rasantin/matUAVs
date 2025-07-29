@echo off
setlocal enabledelayedexpansion
cd /d "%~dp0\.."


REM Gera o timestamp com PowerShell
for /f "delims=" %%i in ('powershell -Command "(Get-Date).ToString('MMddyy_HHmmss')"') do set timestamp=%%i

echo Timestamp gerado: %timestamp%

REM Limpeza dos arquivos antigos
echo Limpando binários antigos...
if exist bin\main.exe del /q bin\main.exe
del /q bin\*.obj >nul 2>&1
del /q bin\*.pdb >nul 2>&1
del /q bin\*.ilk >nul 2>&1
del /q bin\*.lib >nul 2>&1
del /q bin\*.exp >nul 2>&1

REM Cria diretórios se não existirem
if not exist bin mkdir bin

REM Inicializa o ambiente do compilador MSVC
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"

REM Caminhos para Gurobi
set GUROBI_INC=C:\gurobi1202\win64\include
set GUROBI_LIB=C:\gurobi1202\win64\lib
set GUROBI_LIBS=%GUROBI_LIB%\gurobi_c++mt2017.lib %GUROBI_LIB%\gurobi120.lib

REM Compila todos os arquivos .cpp da pasta src
set OBJS=
for %%f in (src\*.cpp) do (
    echo Compilando %%f...
    cl /c /Fo:bin\%%~nf.obj /EHsc /std:c++17 /I %GUROBI_INC%  /Zi %%f
    if exist bin\%%~nf.obj (
        set OBJS=!OBJS! bin\%%~nf.obj
    ) else (
        echo ERRO ao compilar %%f
        goto end
    )
)

REM Linka os objetos em um executável
echo Linkando...
link !OBJS! %GUROBI_LIBS% /OUT:bin\main.exe /DEBUG

REM Verifica se o executável foi gerado com sucesso
if exist bin\main.exe (
    echo.
    echo ==== Executando programa ====
    bin\main.exe

    REM Move o log se existir
    if exist gurobi.log (
        move /Y gurobi.log logs\gurobi_%timestamp%.log
    )
) else (
    echo.
    echo ==== ERRO: bin\main.exe não foi gerado ====
)

:end
pause
