# Instruções de Compilação do Projeto matUAVs

## Visão Geral

O projeto matUAVs utiliza **C++17** e pode ser compilado em **Windows (x64)** e **Linux (x64)**. Duas abordagens de build estão disponíveis:

1. **CMake** (recomendado, multiplataforma)
2. **Script nativo** (build.bat para Windows com MSVC)

Ambas as abordagens integram a biblioteca de otimização **Gurobi** para resolver os modelos MILP.

---

## Pré-requisitos

### Windows
- **Compilador**: Microsoft Visual C++ 2022 BuildTools ou Visual Studio 2022
- **CMake**: 3.20 ou superior (opcional, se usar CMake)
- **Gurobi**: Versão 12.0+ instalado em `C:\gurobi1202\win64` (ou outro caminho configurável)
- **Arquitetura**: x64

### Linux
- **Compilador**: GCC 8+ ou Clang 7+
- **CMake**: 3.20 ou superior
- **Gurobi**: Versão 12.0+ instalado em `/opt/gurobi1202/linux64` (ou outro caminho configurável)
- **Bibliotecas do sistema**: pthread, libm
- **Arquitetura**: x64

---

## Método 1: Compilação com CMake (Multiplataforma)

### Configuração do Gurobi

Antes de compilar, certifique-se de que o Gurobi está instalado. Opcionalmente, defina a variável de ambiente `GUROBI_HOME`:

#### Windows (PowerShell)
```powershell
$env:GUROBI_HOME = "C:\gurobi1202\win64"
```

#### Windows (CMD)
```cmd
set GUROBI_HOME=C:\gurobi1202\win64
```

#### Linux (Bash)
```bash
export GUROBI_HOME=/opt/gurobi1202/linux64
export LD_LIBRARY_PATH=$GUROBI_HOME/lib:$LD_LIBRARY_PATH
```

Para tornar permanente no Linux, adicione ao `~/.bashrc`:
```bash
echo 'export GUROBI_HOME=/opt/gurobi1202/linux64' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$GUROBI_HOME/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

### Compilação no Windows

```cmd
# 1. Criar diretório de build
mkdir build
cd build

# 2. Configurar o projeto com CMake
cmake .. -G "Visual Studio 17 2022" -A x64

# 3. Compilar em modo Release
cmake --build . --config Release

# 4. Executar
bin\Release\main.exe ..\input.txt
```

**Alternativa com NMake:**
```cmd
mkdir build
cd build
cmake .. -G "NMake Makefiles" -DCMAKE_BUILD_TYPE=Release
nmake
bin\main.exe ..\input.txt
```

### Compilação no Linux

```bash
# 1. Criar diretório de build
mkdir build
cd build

# 2. Configurar o projeto com CMake
cmake .. -DCMAKE_BUILD_TYPE=Release

# 3. Compilar usando todos os núcleos disponíveis
make -j$(nproc)

# 4. Executar
./bin/main ../input.txt
```

---

## Método 2: Compilação com Script Windows (build.bat)

Este método é específico para Windows e utiliza diretamente o compilador MSVC sem necessidade de CMake.

### Pré-requisitos Específicos
- Microsoft Visual C++ 2022 BuildTools instalado em:
  `C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\`
- Gurobi instalado em: `C:\gurobi1202\win64\`

### Execução do Script

#### Via VS Code (Recomendado)
1. Abra o projeto no VS Code
2. Pressione `Ctrl+Shift+P`
3. Digite: "Tasks: Run Build Task"
4. O programa será compilado e executado automaticamente

#### Via Linha de Comando
```cmd
.vscode\build.bat
```

### O que o Script Faz

### O que o Script Faz

O script `build.bat` executa as seguintes etapas automatizadas:

#### 1. Limpeza de Arquivos Antigos
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

#### 2. Preparação do Ambiente
```batch
if not exist bin mkdir bin
if not exist logs mkdir logs
call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
```
- Cria diretórios necessários (`bin/` e `logs/`)
- Inicializa o ambiente do compilador MSVC para 64-bit

#### 3. Configuração das Bibliotecas Gurobi
```batch
set GUROBI_INC=C:\gurobi1202\win64\include
set GUROBI_LIB=C:\gurobi1202\win64\lib
set GUROBI_LIBS=%GUROBI_LIB%\gurobi_c++mt2017.lib %GUROBI_LIB%\gurobi120.lib
```
- Define caminhos para headers e bibliotecas do Gurobi
- Especifica as bibliotecas necessárias para otimização

#### 4. Compilação Individual dos Arquivos Fonte
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

#### 5. Linkagem do Executável
```batch
link !OBJS! %GUROBI_LIBS% /OUT:bin\main.exe /DEBUG
```
- Combina todos os arquivos objeto em um executável
- Linka com as bibliotecas do Gurobi
- Gera `bin/main.exe` com informações de debug

#### 6. Execução Automática
```batch
bin\main.exe
```
- Executa o programa compilado automaticamente
- Processa dados de entrada e gera soluções de otimização

#### 7. Gerenciamento de Logs
```batch
if exist gurobi.log (
    move /Y gurobi.log logs\gurobi_%timestamp%.log
)
```
- Move logs do Gurobi para pasta organizada
- Adiciona timestamp para controle de versões

---

## Detalhes Técnicos por Plataforma

### Windows (MSVC)

**Compilador**: Microsoft Visual C++ 2022
**Flags de compilação**:
- `/c` - Compilação sem linkagem
- `/EHsc` - Tratamento de exceções C++
- `/std:c++17` - Padrão C++17
- `/Zi` - Informações de debug

**Bibliotecas Gurobi**:
- `gurobi_c++mt2017.lib` - Interface C++ multi-threaded
- `gurobi120.lib` - Biblioteca principal do Gurobi

**Executável gerado**: `bin/main.exe`

### Linux (GCC/Clang)

**Compiladores suportados**: 
- GCC 8 ou superior
- Clang 7 ou superior

**Flags de compilação**:
- `-std=c++17` - Padrão C++17
- `-Wall -Wextra` - Avisos do compilador
- `-O2` ou `-O3` - Otimização (modo Release)

**Bibliotecas Gurobi**:
- `libgurobi_c++.a` - Interface C++ estática
- `libgurobi120.so` - Biblioteca principal do Gurobi (dinâmica)

**Bibliotecas do sistema**:
- `pthread` - Suporte a threads POSIX
- `m` - Biblioteca matemática

**Executável gerado**: `bin/main`

---

## Diferenças entre Plataformas

| Aspecto | Windows | Linux |
|---------|---------|-------|
| **Compilador** | MSVC (Visual Studio) | GCC 8+ ou Clang 7+ |
| **Executável** | `main.exe` | `main` |
| **Libs Gurobi** | `.lib` (estáticas) | `.so` (dinâmicas) + `.a` |
| **Separador de caminho** | `\` (mas `/` também funciona) | `/` |
| **Build script nativo** | `.vscode\build.bat` | Makefile ou CMake |
| **Bibliotecas do sistema** | Incluídas automaticamente | Requer `-lpthread -lm` |

**Nota importante**: O código-fonte C++ é 100% compatível entre plataformas, utilizando apenas recursos padrão do C++17.

---

## Estrutura de Arquivos Gerados

Após a compilação, a seguinte estrutura é criada:

```
matUAVs/
├── bin/
│   ├── main.exe         # Windows
│   ├── main             # Linux
│   ├── *.obj            # Windows (arquivos objeto)
│   └── *.o              # Linux (arquivos objeto)
├── build/               # Diretório de build do CMake (se usado)
├── logs/
│   └── gurobi_*.log     # Logs timestampados do Gurobi
└── output/              # Resultados das execuções
```

---

## Resolução de Problemas

### Erro: "Gurobi não encontrado"

**Windows:**
1. Verificar instalação em `C:\gurobi1202\win64`
2. Confirmar licença ativa do Gurobi
3. Se instalado em outro local, editar paths no `build.bat` ou definir `GUROBI_HOME`

**Linux:**
1. Verificar instalação em `/opt/gurobi1202/linux64`
2. Confirmar licença ativa do Gurobi
3. Definir `GUROBI_HOME` e `LD_LIBRARY_PATH`

### Erro: "filesystem: No such file or directory"

**Causa**: Compilador não suporta C++17

**Solução**:
- Windows: Atualizar para Visual Studio 2017 15.7 ou superior
- Linux: Usar GCC 8+ ou Clang 7+

### Windows: "LINK : fatal error LNK1181"

**Causa**: Bibliotecas do Gurobi não encontradas

**Solução**:
1. Verificar se o Gurobi está instalado corretamente
2. Usar "Developer Command Prompt for VS 2022"
3. Ou executar `vcvars64.bat` antes de compilar

### Linux: "error while loading shared libraries: libgurobi120.so"

**Causa**: Sistema não encontra as bibliotecas compartilhadas do Gurobi

**Solução**:
```bash
export LD_LIBRARY_PATH=/opt/gurobi1202/linux64/lib:$LD_LIBRARY_PATH
```

Ou adicione ao `~/.bashrc` para tornar permanente.

### Erro de permissão no Linux

**Causa**: Executável não tem permissão de execução

**Solução**:
```bash
chmod +x bin/main
```

---

## Verificação da Compilação

Para verificar se a compilação foi bem-sucedida:

### Windows
```cmd
# Verificar se o executável foi gerado
dir bin\main.exe

# Executar
bin\main.exe input.txt
```

### Linux
```bash
# Verificar se o executável foi gerado
ls -lh bin/main

# Executar
./bin/main input.txt
```

**Saída esperada:**
```
Program: main
Start Reading: input.txt
Node Information:
Base nodes: 1
Target nodes: X
Depot nodes: Y
Robot Information:
[... informações dos robôs ...]
```

---

## Integração com VS Code

O projeto inclui configurações do VS Code para facilitar o desenvolvimento:

### Arquivos de Configuração

#### `.vscode/c_cpp_properties.json`
- Configura IntelliSense para incluir headers do Gurobi
- Define padrão C++17 e paths de inclusão

#### `.vscode/tasks.json`
- Define tarefa de build que executa `build.bat` (Windows)
- Configurada como tarefa padrão do projeto

### Como Usar no VS Code

1. Abrir o projeto: `code .`
2. Compilar: `Ctrl+Shift+P` → "Tasks: Run Build Task"
3. Ou usar o atalho: `Ctrl+Shift+B`

---

## Resultado da Compilação

O processo gera os seguintes arquivos:

**Windows:**
- **Executável**: `bin/main.exe`
- **Objetos**: `bin/*.obj`
- **Logs**: `logs/gurobi_*.log`
- **Debug Info**: `.pdb`, `.ilk` files

**Linux:**
- **Executável**: `bin/main`
- **Objetos**: `bin/*.o`
- **Logs**: `logs/gurobi_*.log`

---

## Funcionalidade do Programa

O executável final implementa algoritmos de otimização para:
- Roteamento de UAVs (Unmanned Aerial Vehicles)
- Problemas de cobertura com restrições de combustível
- Otimização multi-objetivo usando:
  - Metaheurísticas (MOVNS - Multi-Objective Variable Neighborhood Search)
  - Métodos exatos (MILP com Gurobi Solver)
  - Matheurísticas que combinam ambas as abordagens

---

## Documentação Adicional

- **[FLUXO_EXECUCAO.md](FLUXO_EXECUCAO.md)** - Fluxo completo de execução do projeto
- **[GUIA_PRATICO.md](GUIA_PRATICO.md)** - Guia prático com exemplos e troubleshooting
- **[ADAPTACOES_MULTIPLATAFORMA.md](ADAPTACOES_MULTIPLATAFORMA.md)** - Detalhes técnicos das adaptações multiplataforma
- **[BUILD.md](BUILD.md)** - Instruções detalhadas de build com CMake

---

*Este documento descreve o processo de compilação do projeto matUAVs em ambas as plataformas suportadas (Windows e Linux), detalhando as tarefas executadas pelos compiladores e as diferenças específicas de cada sistema.*