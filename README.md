# matUAVs - Otimização de Roteamento de UAVs

O compilador (MSVC) neste projeto executa **7 tarefas principais**:

### 🧹 1. Limpeza
- Remove executáveis e objetos antigos (`*.exe`, `*.obj`, `*.pdb`)
- Garante build limpo sem conflitos

### 📁 2. Preparação
- Cria diretórios `bin/` e `logs/`
- Inicializa ambiente MSVC 64-bit

### 🔗 3. Configuração Gurobi
- Define paths para biblioteca de otimização Gurobi
- Configura bibliotecas `gurobi_c++mt2017.lib` e `gurobi120.lib`

### ⚙️ 4. Compilação C++17
Compila 9 arquivos fonte individualmente:
- `Configuration.cpp` - Configurações do sistema
- `Graph.cpp` - Estruturas de grafos
- `Input.cpp` - Processamento de entrada
- `MHCP.cpp` - **Programa principal** (contém `main()`)
- `Node.cpp` - Nós do grafo
- `Output.cpp` - Saída de resultados
- `Rand.cpp` - Geração de números aleatórios
- `Robot.cpp` - Lógica dos UAVs/robôs
- `Solution.cpp` - Algoritmos de solução

### 🔗 5. Linkagem
- Combina todos os objetos em `bin/main.exe`
- Linka com bibliotecas Gurobi para otimização

### ▶️ 6. Execução Automática
- Executa `main.exe` automaticamente
- Processa dados de entrada e gera soluções

### 📊 7. Gerenciamento de Logs
- Move logs do Gurobi para `logs/` com timestamp
- Organiza saídas para análise posterior

## Como Usar

**Via VS Code (Recomendado):**
```
Ctrl+Shift+P → "Tasks: Run Build Task"
```

**Via Script:**
```cmd
.vscode\build.bat
```

## Tecnologias

- **Compilador:** Microsoft Visual C++ 2022 (MSVC)
- **Padrão:** C++17
- **Otimizador:** Gurobi 12.02
- **Arquitetura:** x64

## Arquivo Principal

O ponto de entrada é `src/MHCP.cpp` que implementa algoritmos de otimização para roteamento de UAVs com restrições de combustível.

---

*Para detalhes técnicos completos, consulte [COMPILACAO.md](COMPILACAO.md)*
