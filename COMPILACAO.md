# Instruções de Compilação do Projeto matUAVs

## Visão Geral

O projeto matUAVs utiliza **C++17** e pode ser compilado em **Windows (x64)** e **Linux (x64)**. O sistema de build canônico é o **CMake**, que garante consistência e portabilidade entre plataformas.

O CMake integra automaticamente a biblioteca de otimização **Gurobi** para resolver os modelos MILP.

---

## Pré-requisitos

### Windows
- **Compilador**: Microsoft Visual C++ 2022 BuildTools ou Visual Studio 2022
- **CMake**: 3.20 ou superior
- **Gurobi**: Versão 12.0+ instalado em `C:\gurobi1202\win64` (ou outro caminho configurável)
- **Arquitetura**: x64

### Linux
- **Compilador**: GCC 8+ ou Clang 7+
- **CMake**: 3.20 ou superior
- **Gurobi**: Versão 12.0+ instalado em `/opt/gurobi1202/linux64` (ou outro caminho configurável)
- **Bibliotecas do sistema**: pthread, libm
- **Arquitetura**: x64

---

## Compilação com CMake

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

## Detalhes Técnicos por Plataforma

### Windows (MSVC)

**Compilador**: Microsoft Visual C++ 2022
**Flags de compilação** (via CMake):
- `/EHsc` - Tratamento de exceções C++
- `/std:c++17` - Padrão C++17
- `/W3` - Avisos do compilador

**Bibliotecas Gurobi**:
- `gurobi_c++mt2017.lib` - Interface C++ multi-threaded
- `gurobi120.lib` - Biblioteca principal do Gurobi

**Executável gerado**: `bin/Release/matUAVs.exe` ou `bin/Debug/matUAVs.exe`

### Linux (GCC/Clang)

**Compiladores suportados**: 
- GCC 8 ou superior
- Clang 7 ou superior

**Flags de compilação** (via CMake):
- `-std=c++17` - Padrão C++17
- `-Wall -Wextra` - Avisos do compilador
- `-O2` ou `-O3` - Otimização (modo Release)

**Bibliotecas Gurobi**:
- `libgurobi_c++.a` - Interface C++ estática
- `libgurobi120.so` - Biblioteca principal do Gurobi (dinâmica)

**Bibliotecas do sistema**:
- `pthread` - Suporte a threads POSIX
- `m` - Biblioteca matemática

**Executável gerado**: `bin/matUAVs`

---

## Diferenças entre Plataformas

| Aspecto | Windows | Linux |
|---------|---------|-------|
| **Compilador** | MSVC (Visual Studio) | GCC 8+ ou Clang 7+ |
| **Executável** | `matUAVs.exe` | `matUAVs` |
| **Libs Gurobi** | `.lib` (estáticas) | `.so` (dinâmicas) + `.a` |
| **Separador de caminho** | `\` (mas `/` também funciona) | `/` |
| **Build system** | CMake | CMake |
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
3. Se instalado em outro local, definir `GUROBI_HOME` antes de executar CMake

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
2. Executar CMake em "Developer Command Prompt for VS 2022"
3. Ou usar "x64 Native Tools Command Prompt" do Visual Studio

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
chmod +x build/bin/matUAVs
```

---

## Verificação da Compilação

Para verificar se a compilação foi bem-sucedida:

### Windows
```cmd
# Verificar se o executável foi gerado
dir build\bin\Release\matUAVs.exe

# Executar
build\bin\Release\matUAVs.exe input.txt
```

### Linux
```bash
# Verificar se o executável foi gerado
ls -lh build/bin/matUAVs

# Executar
./build/bin/matUAVs input.txt
```

**Saída esperada:**
```
Program: matUAVs
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
- Define tarefa de build usando CMake
- Configurada como tarefa padrão do projeto

#### `.vscode/launch.json`
- Configurações de debug para Windows e Linux
- Suporte a debugging com VS Code e WSL

### Como Usar no VS Code

1. Abrir o projeto: `code .`
2. Compilar: `Ctrl+Shift+P` → "Tasks: Run Build Task"
3. Ou usar o atalho: `Ctrl+Shift+B`

---

## Resultado da Compilação

O processo CMake gera os seguintes arquivos:

**Windows:**
- **Executável**: `build/bin/Release/matUAVs.exe` ou `build/bin/Debug/matUAVs.exe`
- **Objetos**: arquivos intermediários em `build/`
- **Logs**: `logs/gurobi_*.log` (após execução)

**Linux:**
- **Executável**: `build/bin/matUAVs`
- **Objetos**: arquivos intermediários em `build/`
- **Logs**: `logs/gurobi_*.log` (após execução)

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