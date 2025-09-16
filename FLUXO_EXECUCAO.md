# Fluxo de Execução do Projeto matUAVs

## Visão Geral
Este documento detalha o fluxo completo de execução do projeto matUAVs, desde a preparação dos dados de entrada até a geração dos resultados de otimização de roteamento de UAVs.

## 📋 Índice
1. [Preparação do Ambiente](#preparação-do-ambiente)
2. [Estrutura de Dados de Entrada](#estrutura-de-dados-de-entrada)
3. [Fluxo Principal de Execução](#fluxo-principal-de-execução)
4. [Algoritmo de Otimização](#algoritmo-de-otimização)
5. [Geração de Resultados](#geração-de-resultados)
6. [Exemplos Práticos](#exemplos-práticos)

---

## 🛠️ Preparação do Ambiente

### Pré-requisitos
- Windows x64
- Microsoft Visual C++ 2022 BuildTools
- Gurobi Optimizer 12.02
- Visual Studio Code (recomendado)

### Processo de Build
```batch
# Via VS Code (Recomendado)
Ctrl+Shift+P → "Tasks: Run Build Task"

# Via linha de comando
.vscode\build.bat
```

O processo executa 7 etapas automatizadas:
1. **Limpeza** - Remove executáveis e objetos antigos
2. **Preparação** - Cria diretórios `bin/` e `logs/`
3. **Configuração Gurobi** - Define paths e bibliotecas
4. **Compilação** - Compila 9 arquivos C++17 individuais
5. **Linkagem** - Gera `bin/main.exe`
6. **Execução** - Executa automaticamente o programa
7. **Logs** - Organiza logs do Gurobi com timestamp

---

## 📄 Estrutura de Dados de Entrada

### Formato do Arquivo de Entrada
```txt
@base
Base_1@<x> <y>

@target
<x1> <y1>
<x2> <y2>
...

@robots_configuration
configId:<nome>
maxVel:<velocidade>
maxFuel:<combustivel>
prop:<propriedade>

@robot
robotId:<id>
configId:<config_ref>

@robot_base
<robot_id>@<base_id>

@param
nexec:<num_execucoes>
m:<parametro_m>
n:<parametro_n>
cvl_subset:<max_linhas_cobertura>
```

### Exemplo Prático
```txt
@base
Base_1@7650 -50

@target
425 0
850 0
1275 0

@robots_configuration
configId:Config_1
maxVel:15
maxFuel:1200
prop:3.00

@robot
robotId:Robot_1
configId:Config_1

@robot_base
Robot_1@Base_1

@param
nexec:1
m:15
n:4
cvl_subset:3
```

---

## 🔄 Fluxo Principal de Execução

### 1. Inicialização (`main()`)
```cpp
Input input(fileName);        // Lê arquivo de entrada
input.printNodes();          // Exibe nós carregados
input.printRobots();         // Exibe robôs configurados
Output output(input);        // Inicializa saída
```

### 2. Loop Principal de Execuções
```cpp
while(exec <= input.getNExec()) {
    output.createOutput(datetime());        // Cria diretório timestampado
    Solution s(input, cvl_subset_num);      // Inicializa solução
    
    // Algoritmo de otimização (VNS/VND)
    while(s.HasSolutionNotVisited() && m <= targetsNum) {
        // Perturbação + Busca Local
    }
    
    exec++;
}
```

### 3. Estrutura de Controle
- **Execuções** (`nexec`): Número de runs independentes
- **Parâmetro m**: Máximo de iterações por targets
- **Parâmetro n**: Número de aplicações VND
- **cvl_subset**: Linhas de cobertura por subconjunto

---

## 🎯 Algoritmo de Otimização

### Estratégia Híbrida VNS/VND

#### VNS (Variable Neighborhood Search)
```cpp
s.perturbation(&s.currentSol, maxDepots);
```
- **Objetivo**: Escapar de ótimos locais
- **Método**: Gera soluções perturbadas aleatoriamente
- **Parâmetro**: `maxDepots` controla intensidade da perturbação

#### VND (Variable Neighborhood Descent)
```cpp
while(n <= input.getN()) {
    // Operadores locais em sequência:
    if(s.shift(&s.currentSol))         continue;  // Move tarefas
    if(s.swap(&s.currentSol))          continue;  // Troca tarefas
    if(s.improveSol(&s.currentSol))    continue;  // Melhoria geral
    if(s.swapRobots(&s.currentSol))    continue;  // Troca rotas
    if(s.closeRandomDepot(&s.currentSol)) continue; // Remove depósito
    
    // Avaliação e atualização
    if(s.IsBetterSol(s.currentSol, s.best_sol))
        s.best_sol = s.currentSol;
    else
        s.currentSol = s.best_sol;
    
    n++;
}
```

#### Operadores de Melhoria
1. **Shift**: Move tarefas entre robôs diferentes
2. **Swap**: Troca tarefas entre robôs
3. **ImproveSol**: Aplicação de melhorias gerais
4. **SwapRobots**: Troca rotas completas entre robôs
5. **CloseRandomDepot**: Remove depósito aleatório (reduz custos)

### Avaliação Multi-objetivo
```cpp
s.eval_VecSol();        // Avalia conjunto de soluções
s.print_paretoSet();    // Exibe Pareto Front
```

**Critérios de Otimização**:
- Minimização do custo total de roteamento
- Maximização da cobertura de targets
- Respeito às restrições de combustível
- Balanceamento de carga entre robôs

---

## 📊 Geração de Resultados

### Estrutura de Saída
```
output/
├── <timestamp>/
│   ├── nodes/
│   │   └── nodes.txt          # Coordenadas dos nós
│   └── solutions/
│       ├── sol_1/             # Solução 1
│       ├── sol_2/             # Solução 2
│       └── ...
└── gurobi_<timestamp>.log     # Logs do Gurobi
```

### Informações Geradas
- **Nós**: Coordenadas e tipos (base, target, depot)
- **Soluções**: Rotas otimizadas para cada robô
- **Métricas**: Custos, tempos, cobertura
- **Logs**: Informações detalhadas do solver

### Saída no Console
```
Program: main.exe
Start Reading: input.txt
Node Information:
Base nodes: 1
Target nodes: 37
Depot nodes: 0
Robot Information:
Robot_1 (Config_3): Vel=15, Fuel=1320
Robot_2 (Config_2): Vel=16, Fuel=1800
Robot_3 (Config_1): Vel=15, Fuel=1200
```

---

## 🚀 Exemplos Práticos

### Execução Básica
```bash
# 1. Compilar projeto
.vscode\build.bat

# 2. Executar com arquivo padrão
bin\main.exe input.txt

# 3. Executar com arquivo personalizado
bin\main.exe meu_cenario.txt
```

### Modificando Parâmetros
Para alterar comportamento, edite o arquivo de entrada:

```txt
@param
nexec:5      # 5 execuções independentes
m:20         # Máximo 20 iterações por target
n:6          # 6 aplicações do VND
cvl_subset:5 # 5 linhas de cobertura por subset
```

### Configurando Múltiplos Robôs
```txt
@robots_configuration
configId:Config_Rapido
maxVel:20
maxFuel:2000
prop:1.50

configId:Config_Economico
maxVel:12
maxFuel:3000
prop:2.80

@robot
robotId:Robot_Rapido
configId:Config_Rapido

robotId:Robot_Economico
configId:Config_Economico
```

### Cenários de Teste Incluídos
- `input.txt` - Cenário padrão (37 targets, 3 robôs)
- `C_T20_R6.txt` - 20 targets, 6 robôs
- `C_T140_R9.txt` - 140 targets, 9 robôs
- `D_T30_R6.txt` - 30 targets, 6 robôs

---

## 📈 Monitoramento e Análise

### Verificação de Resultados
1. **Durante execução**: Observe logs no console
2. **Pós-execução**: Analise arquivos em `output/`
3. **Performance**: Verifique `logs/gurobi_*.log`

### Indicadores de Qualidade
- **Convergência**: Redução progressiva do custo
- **Cobertura**: Percentual de targets atendidos
- **Balanceamento**: Distribuição equilibrada entre robôs
- **Feasibilidade**: Respeito às restrições de combustível

### Troubleshooting
- **Erro de compilação**: Verificar instalação do Gurobi
- **Solução inviável**: Ajustar parâmetros de combustível
- **Performance lenta**: Reduzir número de targets ou robôs

---

## 🔧 Personalização Avançada

### Modificando Algoritmo
- Ajustar pesos na função objetivo (`Solution.cpp`)
- Alterar operadores de vizinhança
- Modificar critérios de parada

### Adicionando Restrições
- Janelas de tempo
- Capacidade de carga
- Zonas proibidas
- Prioridades de targets

---

*Este documento serve como guia completo para execução e personalização do projeto matUAVs. Para detalhes técnicos de compilação, consulte [COMPILACAO.md](COMPILACAO.md).*