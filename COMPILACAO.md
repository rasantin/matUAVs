# Tarefas do Compilador no Projeto matUAVs

## Visão Geral

O processo de compilação neste projeto utiliza o **Microsoft Visual C++ (MSVC)** integrado com a biblioteca de otimização **Gurobi**. O compilador executa uma série de tarefas automatizadas através do script `build.bat`.

## Tarefas Principais do Compilador

### 1. **Limpeza de Arquivos Antigos**
```batch
del /q bin\main.exe
del /q bin\*.obj
del /q bin\*.pdb
del /q bin\*.ilk
del /q bin\*.lib
del /q bin\*.exp
```
- Remove executáveis e objetos compilados anteriormente
- Garante um build limpo sem conflitos de versões

### 2. **Preparação do Ambiente**
```batch
if not exist bin mkdir bin
if not exist logs mkdir logs
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
```
- Cria diretórios necessários (`bin/` e `logs/`)
- Inicializa o ambiente do compilador MSVC para 64-bit

### 3. **Configuração das Bibliotecas Gurobi**
```batch
set GUROBI_INC=C:\gurobi1202\win64\include
set GUROBI_LIB=C:\gurobi1202\win64\lib
set GUROBI_LIBS=%GUROBI_LIB%\gurobi_c++mt2017.lib %GUROBI_LIB%\gurobi120.lib
```
- Define caminhos para headers e bibliotecas do Gurobi
- Especifica as bibliotecas necessárias para otimização

### 4. **Compilação Individual dos Arquivos Fonte**
```batch
for %%f in (src\*.cpp) do (
    cl /c /Fo:bin\%%~nf.obj /EHsc /std:c++17 /I %GUROBI_INC% /Zi %%f
)
```
**Parâmetros de Compilação:**
- `/c` - Compila sem linkar
- `/Fo:bin\%%~nf.obj` - Define o nome e local do arquivo objeto
- `/EHsc` - Habilita tratamento de exceções C++
- `/std:c++17` - Usa o padrão C++17
- `/I %GUROBI_INC%` - Inclui headers do Gurobi
- `/Zi` - Gera informações de debug

**Arquivos Compilados:**
- `Configuration.cpp` → `bin/Configuration.obj`
- `Graph.cpp` → `bin/Graph.obj`
- `Input.cpp` → `bin/Input.obj`
- `MHCP.cpp` → `bin/MHCP.obj` (contém a função main)
- `Node.cpp` → `bin/Node.obj`
- `Output.cpp` → `bin/Output.obj`
- `Rand.cpp` → `bin/Rand.obj`
- `Robot.cpp` → `bin/Robot.obj`
- `Solution.cpp` → `bin/Solution.obj`

### 5. **Linkagem do Executável**
```batch
link !OBJS! %GUROBI_LIBS% /OUT:bin\main.exe /DEBUG
```
- Combina todos os arquivos objeto em um executável
- Linka com as bibliotecas do Gurobi
- Gera `bin/main.exe` com informações de debug

### 6. **Execução Automática**
```batch
bin\main.exe
```
- Executa o programa compilado automaticamente
- Processa dados de entrada e gera soluções de otimização

### 7. **Gerenciamento de Logs**
```batch
if exist gurobi.log (
    move /Y gurobi.log logs\gurobi_%timestamp%.log
)
```
- Move logs do Gurobi para pasta organizada
- Adiciona timestamp para controle de versões

## Dependências Técnicas

### Compilador
- **Microsoft Visual C++ 2022 BuildTools**
- Arquitetura: x64 (64-bit)
- Padrão: C++17

### Bibliotecas Externas
- **Gurobi Optimizer 12.02**
  - `gurobi_c++mt2017.lib` - Interface C++ multi-threaded
  - `gurobi120.lib` - Biblioteca principal do Gurobi

### Integração com VS Code
- Configurado através de `tasks.json`
- Tarefa padrão: "Compilar matUAV com MSVC e Gurobi"
- Atalho: `Ctrl+Shift+P` → "Tasks: Run Build Task"

## Arquivos de Configuração

### `.vscode/c_cpp_properties.json`
- Configura IntelliSense para incluir headers do Gurobi
- Define padrão C++17 e paths de inclusão

### `.vscode/tasks.json`
- Define tarefa de build que executa `build.bat`
- Configurada como tarefa padrão do projeto

## Resultado da Compilação

O processo gera:
- **Executável:** `bin/main.exe`
- **Objetos:** `bin/*.obj`
- **Logs:** `logs/gurobi_*.log`
- **Debug Info:** `.pdb`, `.ilk` files

## Funcionalidade do Programa

O executável final (`main.exe`) implementa algoritmos de otimização para:
- Roteamento de UAVs (Unmanned Aerial Vehicles)
- Problemas de cobertura com restrições de combustível
- Otimização multi-objetivo usando Gurobi Solver

## Como Executar

1. **Via VS Code:** `Ctrl+Shift+P` → "Tasks: Run Build Task"
2. **Via linha de comando:** Execute `.vscode/build.bat`
3. **Manual:** Siga os passos de compilação individualmente

---

*Este documento descreve o processo de compilação automatizado do projeto matUAVs, detalhando cada tarefa executada pelo compilador MSVC integrado com Gurobi.*