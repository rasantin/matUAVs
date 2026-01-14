# matUAVs — Otimização de Roteamento de UAVs

## Descrição do Projeto

O matUAVs é um projeto acadêmico resultante do meu doutorado, focado no planejamento e otimização de rotas para frotas heterogêneas de Veículos Aéreos Não Tripulados (UAVs) em missões de cobertura completa de áreas. O objetivo é gerar rotas viáveis e eficientes sob restrições operacionais, considerando explicitamente a autonomia dos veículos e a localização de estações de recarga.

A solução adota uma abordagem matheurística multiobjetivo, combinando metaheurísticas (MOVNS) com métodos exatos baseados em Programação Inteira Mista (MILP), resolvidos com Gurobi, para explorar soluções não dominadas que equilibram o tempo total da missão e o número de estações de recarga, com melhor eficiência computacional.

A abordagem integra:

- Modelagem do problema em grafos (nós, arestas, custos e restrições de autonomia);
- Formulações MILP para decisões conjuntas de roteamento e recarga;
- Matheurísticas multiobjetivo que utilizam métodos exatos como operadores de rota no processo de busca.

**O projeto compila e executa em Windows (x64) e Linux (x64)**, utilizando C++17, CMake e Gurobi. O build system está estruturado para facilitar a compilação e execução em ambas as plataformas, com tarefas automatizadas que cobrem desde a preparação do ambiente até a execução dos experimentos e a geração de logs.

---

## Pipeline de Build e Execução

O projeto suporta dois métodos de build:

### Método 1: CMake (Recomendado - Multiplataforma)
- Funciona em **Windows** e **Linux**
- Configuração automática de dependências
- Build otimizado para cada plataforma

### Método 2: Script Windows (build.bat)
- Específico para Windows com MSVC
- Build direto sem necessidade de CMake

---

### Pipeline de Build Windows (MSVC + build.bat)

O script `.vscode/build.bat` executa 7 etapas principais:

### 🧹 1) Limpeza
- Remove executáveis e objetos antigos (`*.exe`, `*.obj`, `*.pdb`);
- Garante um build limpo e sem conflitos.

### 📁 2) Preparação
- Cria os diretórios `bin/` e `logs/`;
- Inicializa o ambiente MSVC 64-bit.

### 🔗 3) Configuração do Gurobi
- Define os caminhos (paths) para a instalação do Gurobi;
- Configura as bibliotecas `gurobi_c++mt2017.lib` e `gurobi120.lib` para linkagem.

### ⚙️ 4) Compilação (C++17)
Compila individualmente 9 arquivos-fonte:
- `Configuration.cpp` — Configurações do sistema;
- `Graph.cpp` — Estruturas e operações em grafos;
- `Input.cpp` — Processamento de dados de entrada;
- `MHCP.cpp` — Programa principal (contém `main()`);
- `Node.cpp` — Representação de nós do grafo;
- `Output.cpp` — Geração e formatação de resultados;
- `Rand.cpp` — Geração de números aleatórios;
- `Robot.cpp` — Lógica dos UAVs/robôs;
- `Solution.cpp` — Métodos de construção e melhoria de soluções.

### 🔗 5) Linkagem
- Une todos os objetos e gera `bin/main.exe`;
- Realiza a linkagem com as bibliotecas do Gurobi.

### ▶️ 6) Execução Automática
- Executa `main.exe` ao final do build;
- Processa os dados de entrada e produz as soluções correspondentes.

### 📊 7) Gerenciamento de Logs
- Move os logs do Gurobi para `logs/` com timestamp;
- Organiza saídas para análise posterior.

---

## Como Usar

### Windows

#### Via VS Code (recomendado):
- Pressione: `Ctrl+Shift+P` → "Tasks: Run Build Task"

#### Via script batch:
```cmd
.vscode\build.bat
```

#### Via CMake:
```cmd
mkdir build && cd build
cmake .. -G "Visual Studio 17 2022" -A x64
cmake --build . --config Release
bin\Release\main.exe input.txt
```

### Linux

#### Via CMake:
```bash
mkdir build && cd build
cmake ..
make -j$(nproc)
./bin/main input.txt
```

---

## Tecnologias

- **Linguagem**: C++17
- **Build System**: CMake 3.20+ / MSVC (Windows)
- **Compiladores**: 
  - Windows: Microsoft Visual C++ 2022 (MSVC)
  - Linux: GCC 8+ ou Clang 7+
- **Otimizador**: Gurobi 12.0+
- **Arquitetura**: x64 (64-bit)
- **Plataformas**: Windows (x64) e Linux (x64)

---

## Arquivo Principal

O ponto de entrada é `src/MHCP.cpp`, que implementa os algoritmos de otimização para roteamento de UAVs com restrições de combustível.

---

## 📖 Documentação Completa

- **[COMPILACAO.md](COMPILACAO.md)** - Instruções detalhadas de configuração e compilação
- **[FLUXO_EXECUCAO.md](FLUXO_EXECUCAO.md)** - Fluxo completo de execução do projeto
- **[GUIA_PRATICO.md](GUIA_PRATICO.md)** - Guia prático com exemplos e troubleshooting
- **[DIAGRAMA_FLUXO.md](DIAGRAMA_FLUXO.md)** - Diagramas visuais do fluxo de execução

---

Para começar rapidamente, consulte o [Guia Prático](GUIA_PRATICO.md).
