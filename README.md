# matUAVs — Otimização de Roteamento de UAVs

## Descrição do Projeto

O matUAVs é um projeto acadêmico focado no planejamento e otimização de rotas para Veículos Aéreos Não Tripulados (UAVs). O objetivo é encontrar rotas viáveis e eficientes sob restrições operacionais — com destaque para limites de combustível — utilizando técnicas de otimização com Gurobi e heurísticas em C++17.

A abordagem combina:
- Modelagem do problema em grafos (nós, arestas, custos e restrições);
- Otimização com Gurobi para resolver subproblemas estruturados;
- Heurísticas para melhorar a qualidade das soluções e reduzir tempo de execução.

O projeto foi organizado para oferecer uma experiência de build e execução simples no Windows (x64), com tarefas automatizadas que cuidam desde a limpeza do ambiente até a geração de logs.

---

## Pipeline de Build e Execução

O processo de build (MSVC + scripts) executa 7 etapas principais:

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

- Via VS Code (recomendado):
  - Pressione: `Ctrl+Shift+P` → "Tasks: Run Build Task"

- Via script:
  - `cmd`:
    ```
    .vscode\build.bat
    ```

---

## Tecnologias

- Compilador: Microsoft Visual C++ 2022 (MSVC)
- Padrão: C++17
- Otimizador: Gurobi 12.0.2
- Arquitetura: x64

---

## Arquivo Principal

O ponto de entrada é `src/MHCP.cpp`, que implementa os algoritmos de otimização para roteamento de UAVs com restrições de combustível.

---

Para instruções detalhadas de configuração e compilação, consulte [COMPILACAO.md](COMPILACAO.md).
