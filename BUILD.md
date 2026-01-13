# Build Instructions - matUAVs (Cross-Platform)

## Pré-requisitos

### Comum a todas as plataformas:
- **C++17 compatible compiler**
- **CMake 3.12+** (recomendado) ou usar scripts específicos
- **Gurobi Optimizer 12.0+** instalado

---

## Opção 1: Build com CMake (Recomendado - Multiplataforma)

### Windows

```batch
# 1. Criar diretório de build
mkdir build
cd build

# 2. Gerar arquivos de projeto (Visual Studio)
cmake .. -G "Visual Studio 17 2022" -A x64

# 3. Compilar
cmake --build . --config Release

# 4. Executar
bin\Release\main.exe ..\input.txt
```

### Linux

```bash
# 1. Criar diretório de build
mkdir build
cd build

# 2. Gerar Makefiles
cmake ..

# 3. Compilar (use todos os cores disponíveis)
make -j$(nproc)

# 4. Executar
./bin/main ../input.txt
```

---

## Opção 2: Build com Scripts Nativos

### Windows (MSVC)

Use o script batch existente:

```batch
.vscode\build.bat
```

Este script:
- Limpa builds anteriores
- Compila com MSVC usando C++17
- Linka com Gurobi
- Executa automaticamente

### Linux (Makefile customizado)

Se preferir não usar CMake, você pode criar um Makefile:

```makefile
CXX = g++
CXXFLAGS = -std=c++17 -Wall -Wextra -O2
GUROBI_INC = /opt/gurobi1202/linux64/include
GUROBI_LIB = /opt/gurobi1202/linux64/lib
LDFLAGS = -L$(GUROBI_LIB) -lgurobi_c++ -lgurobi120 -lpthread -lm

SOURCES = $(wildcard src/*.cpp)
OBJECTS = $(SOURCES:src/%.cpp=bin/%.o)
TARGET = bin/main

all: $(TARGET)

$(TARGET): $(OBJECTS)
	$(CXX) $(OBJECTS) $(LDFLAGS) -o $@

bin/%.o: src/%.cpp | bin
	$(CXX) $(CXXFLAGS) -I$(GUROBI_INC) -c $< -o $@

bin:
	mkdir -p bin

clean:
	rm -rf bin/*.o $(TARGET)

.PHONY: all clean
```

Depois execute:
```bash
make -j$(nproc)
./bin/main input.txt
```

---

## Configuração do Gurobi

### Definir GUROBI_HOME (Opcional)

Se o Gurobi não estiver nos caminhos padrão:

**Windows (PowerShell):**
```powershell
$env:GUROBI_HOME = "C:\gurobi1202\win64"
```

**Windows (CMD):**
```cmd
set GUROBI_HOME=C:\gurobi1202\win64
```

**Linux (Bash):**
```bash
export GUROBI_HOME=/opt/gurobi1202/linux64
export LD_LIBRARY_PATH=$GUROBI_HOME/lib:$LD_LIBRARY_PATH
```

**Permanente (Linux - adicione ao ~/.bashrc):**
```bash
echo 'export GUROBI_HOME=/opt/gurobi1202/linux64' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$GUROBI_HOME/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

---

## Estrutura de Diretórios após Build

```
matUAVs/
├── bin/              # Executáveis compilados
│   └── main.exe      # Windows
│   └── main          # Linux
├── build/            # Arquivos de build do CMake
├── logs/             # Logs do Gurobi
├── output/           # Resultados de execução
└── src/              # Código-fonte
```

---

## Resolução de Problemas

### Erro: "Gurobi libraries not found"

**Solução:**
1. Verifique se o Gurobi está instalado
2. Defina a variável GUROBI_HOME
3. Ou edite o CMakeLists.txt com o caminho correto

### Erro: "filesystem: No such file or directory"

**Solução:**
- Certifique-se de usar C++17: `-std=c++17` (GCC/Clang) ou `/std:c++17` (MSVC)
- GCC/Clang: requer GCC 8+ ou Clang 7+
- MSVC: requer Visual Studio 2017 15.7+

### Windows: "LINK : fatal error LNK1181"

**Solução:**
- Verifique se as bibliotecas do Gurobi estão no caminho correto
- Use o "Developer Command Prompt" ou execute vcvars64.bat

### Linux: "error while loading shared libraries: libgurobi120.so"

**Solução:**
```bash
export LD_LIBRARY_PATH=/opt/gurobi1202/linux64/lib:$LD_LIBRARY_PATH
```

---

## Diferenças entre Plataformas

| Aspecto | Windows | Linux |
|---------|---------|-------|
| Compilador | MSVC (Visual Studio) | GCC/Clang |
| Executável | `main.exe` | `main` |
| Libs Gurobi | `.lib` estáticas | `.so` dinâmicas |
| Separador de caminho | `\` (mas `/` também funciona) | `/` |
| Build script | `.vscode\build.bat` | `make` ou `cmake` |

**Nota:** O código C++ é 100% compatível - usa apenas padrões C++17.

---

## VS Code - Integração

Para compilar diretamente no VS Code:

1. Abra o projeto no VS Code
2. Pressione `Ctrl+Shift+P`
3. Digite: "Tasks: Run Build Task"
4. Selecione "Build matUAVs"

Alternativamente:
- `Ctrl+Shift+B` (atalho para build)

---

## Verificar se Funcionou

Após compilar com sucesso:

```bash
# Windows
bin\main.exe input.txt

# Linux
./bin/main input.txt
```

Você deverá ver:
```
Program: main
Start Reading: input.txt
[... output do algoritmo ...]
```

E os resultados em:
```
output/<instance_name>/<timestamp>/
```

---

## Performance

**Flags de otimização recomendadas:**

CMake (Release mode):
```bash
cmake .. -DCMAKE_BUILD_TYPE=Release
```

Manual:
- GCC/Clang: `-O3 -march=native -DNDEBUG`
- MSVC: `/O2 /DNDEBUG`

---

## Mais Informações

- Veja `ADAPTACOES_MULTIPLATAFORMA.md` para detalhes técnicos
- Veja `README.md` para informações sobre o projeto
- Veja `CMakeLists.txt` para configurações de build
