# Resumo do Fluxo de Execução - matUAVs

## 🎯 Visão Executiva

O projeto **matUAVs** implementa um sistema de otimização para roteamento de UAVs (Veículos Aéreos Não Tripulados) usando técnicas avançadas de otimização combinatória. Este documento apresenta o fluxo de execução completo do sistema.

## 📋 Fluxo Resumido de Execução

### 1. **Preparação** (Automática)
```
Build System → Gurobi Setup → Compilation → Execution
```

### 2. **Entrada de Dados**
```
Input File → Parse Nodes → Parse Robots → Parse Parameters
```

### 3. **Otimização** (Núcleo do Algoritmo)
```
VNS Perturbation → VND Local Search → Solution Evaluation → Pareto Front Update
```

### 4. **Saída de Resultados**
```
Solution Export → Log Generation → Performance Metrics
```

---

## 🔄 Algoritmo Principal: VNS/VND Híbrido

### Variable Neighborhood Search (VNS)
- **Objetivo**: Exploração global do espaço de soluções
- **Método**: Perturbações sistemáticas para escapar de ótimos locais
- **Controle**: Parâmetro `maxDepots` regula intensidade

### Variable Neighborhood Descent (VND)
- **Objetivo**: Refinamento local das soluções
- **Operadores**:
  1. **Shift**: Realocação de tarefas entre robôs
  2. **Swap**: Troca de tarefas entre robôs
  3. **ImproveSol**: Otimização geral da solução
  4. **SwapRobots**: Troca de rotas inteiras
  5. **CloseRandomDepot**: Remoção estratégica de depósitos

### Critérios de Otimização
- ✅ Minimização do custo total de roteamento
- ✅ Maximização da cobertura de targets
- ✅ Respeito às restrições de combustível
- ✅ Balanceamento de carga entre robôs

---

## 📊 Estrutura de Controle

### Loops Aninhados
```cpp
for(exec = 1; exec <= nexec; exec++) {          // Múltiplas execuções
    while(m <= targetNum) {                     // Iterações por target
        while(n <= N) {                         // Aplicações VND
            // Operadores de melhoria local
        }
    }
}
```

### Parâmetros de Configuração
- **`nexec`**: Número de execuções independentes
- **`m`**: Máximo de iterações por conjunto de targets
- **`n`**: Número de aplicações do VND
- **`cvl_subset`**: Linhas de cobertura por subconjunto

---

## 🛠️ Comandos de Execução

### Compilação e Execução Automática
```bash
# Via VS Code (Recomendado)
Ctrl+Shift+P → "Tasks: Run Build Task"

# Via linha de comando
.vscode\build.bat

# Execução manual
bin\main.exe input.txt
```

### Parâmetros de Linha de Comando
```bash
bin\main.exe <arquivo_entrada>
```

---

## 📁 Estrutura de Entrada e Saída

### Arquivo de Entrada (Exemplo)
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

### Estrutura de Saída
```
output/
├── <timestamp>/
│   ├── nodes/
│   │   └── nodes.txt
│   └── solutions/
│       ├── sol_1/
│       └── sol_2/
└── logs/
    └── gurobi_<timestamp>.log
```

---

## ⚡ Execução Rápida (Tutorial de 5 Minutos)

### Passo 1: Verificar Pré-requisitos
- Windows x64
- Visual Studio Code
- Gurobi 12.02 instalado

### Passo 2: Abrir Projeto
```bash
cd matUAVs
code .
```

### Passo 3: Executar
```
Ctrl+Shift+P → "Tasks: Run Build Task"
```

### Passo 4: Verificar Resultados
- Console: Progresso da otimização
- `output/`: Arquivos de solução
- `logs/`: Logs detalhados

---

## 🎛️ Configurações Típicas

### Desenvolvimento/Teste
```txt
@param
nexec:1    # Execução única
m:5        # Iterações reduzidas
n:3        # VND simplificado
cvl_subset:1
```

### Produção/Pesquisa
```txt
@param
nexec:10   # Múltiplas execuções
m:50       # Exploração extensa
n:8        # VND completo
cvl_subset:5
```

### Análise de Performance
```txt
@param
nexec:100  # Estatísticas robustas
m:30       # Balanceamento
n:6        # Refinamento médio
cvl_subset:3
```

---

## 📈 Interpretação de Resultados

### Indicadores de Qualidade
- **Convergência**: Redução progressiva do custo
- **Cobertura**: Percentual de targets atendidos (meta: >90%)
- **Feasibilidade**: Todas as restrições respeitadas
- **Balanceamento**: Distribuição equilibrada entre robôs

### Saída no Console
```
Program: main.exe
Start Reading: input.txt
Node Information: Base=1, Target=37, Depot=0
Robot Information: 3 robots configured
Optimization Progress: [Iteração X] Cost=Y | Coverage=Z%
Final Best Solution: Cost=1420.45 | Coverage=95%
```

---

## 🔧 Troubleshooting Comum

### Problema: Erro de Compilação
**Causa**: Gurobi não configurado
**Solução**: Verificar instalação em `C:\gurobi1202\`

### Problema: Solução Inviável
**Causa**: Restrições de combustível muito restritivas
**Solução**: Aumentar `maxFuel` ou reduzir distâncias

### Problema: Performance Lenta
**Causa**: Parâmetros muito altos
**Solução**: Reduzir `m`, `n` e `nexec`

---

## 🚀 Próximos Passos

1. **Familiarização**: Executar cenários de exemplo
2. **Customização**: Criar arquivos de entrada personalizados
3. **Análise**: Estudar resultados e métricas
4. **Otimização**: Ajustar parâmetros para seu caso de uso
5. **Extensão**: Modificar algoritmo conforme necessário

---

## 📚 Documentação Complementar

- **[FLUXO_EXECUCAO.md](FLUXO_EXECUCAO.md)** - Detalhes técnicos completos
- **[GUIA_PRATICO.md](GUIA_PRATICO.md)** - Exemplos práticos e troubleshooting
- **[DIAGRAMA_FLUXO.md](DIAGRAMA_FLUXO.md)** - Diagramas visuais
- **[COMPILACAO.md](COMPILACAO.md)** - Processo de build detalhado

---

*Este resumo fornece uma visão executiva do fluxo de execução do projeto matUAVs. Para informações detalhadas, consulte a documentação específica.*