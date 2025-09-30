# Diagrama de Fluxo - Algoritmo MHCP (VNS/VND)

## Fluxo Principal do Algoritmo

```mermaid
flowchart TD
    A[Início] --> B[Ler entrada e inicializar]
    B --> C[exec = 1]
    C --> D{exec <= nExec?}
    D -->|Não| Z[Fim]
    D -->|Sim| E[Criar solução inicial S]
    E --> F[m = 1, n = 1]
    F --> G{HasSolutionNotVisited && m <= targetsNum?}
    G -->|Não| H[Avaliar VecSol e Pareto]
    H --> I[Incrementar exec]
    I --> D
    G -->|Sim| J[s.currentSol = s.best_sol]
    J --> K[Perturbação VNS]
    K --> L[n = 1]
    L --> M{n menor ou igual a input.getN}
    M -->|Não| N[Avaliar soluções]
    N --> O[Obter próxima solução não visitada]
    O --> P[m++]
    P --> G
    M -->|Sim| Q[Iniciar VND - Busca Local]
    
    %% VND - Variable Neighborhood Descent
    Q --> R[Shift]
    R --> S{Melhoria encontrada?}
    S -->|Sim| R
    S -->|Não| T[Swap]
    T --> U{Melhoria encontrada?}
    U -->|Sim| R
    U -->|Não| V[ImproveSol]
    V --> W{Melhoria encontrada?}
    W -->|Sim| R
    W -->|Não| X[SwapRobots]
    X --> Y{Melhoria encontrada?}
    Y -->|Sim| R
    Y -->|Não| AA[CloseRandomDepot]
    AA --> BB{Melhoria encontrada?}
    BB -->|Sim| R
    BB -->|Não| CC[Verificar se s.currentSol é melhor que s.best_sol]
    CC --> DD{É melhor?}
    DD -->|Sim| EE[s.best_sol = s.currentSol]
    DD -->|Não| FF[s.currentSol = s.best_sol<br/>solutionToNodesSet]
    EE --> GG[Perturbação VNS]
    FF --> GG
    GG --> HH[n++]
    HH --> M

    %% Estilo único para todos os nós
    classDef tudo fill:#e0f7fa,stroke:#006064,stroke-width:2px,color:#000000
    class A,B,C,D,E,F,G,H,I,J,K,L,M,N,O,P,Q,R,S,T,U,V,W,X,Y,Z,AA,BB,CC,DD,EE,FF,GG,HH tudo
```

## Comportamento Real dos Operadores Locais (VND)

### Sequência de Execução:
1. **Shift** → Se encontrar melhoria, **retorna ao Shift**
2. **Swap** → Se encontrar melhoria, **retorna ao Shift** 
3. **ImproveSol** → Se encontrar melhoria, **retorna ao Shift**
4. **SwapRobots** → Se encontrar melhoria, **retorna ao Shift**
5. **CloseRandomDepot** → Se encontrar melhoria, **retorna ao Shift**

### Comportamento Chave:
- **Quando um operador encontra melhoria**: O algoritmo executa `continue`, que reinicia o ciclo VND desde o primeiro operador (Shift)
- **Apenas quando NENHUM operador encontra melhoria**: O algoritmo incrementa `n` e aplica uma nova perturbação VNS

### Código Correspondente (src/MHCP.cpp, linhas 104-185):
```cpp
while(n <= input.getN()){
    // VND - Variable Neighborhood Descent
    if(s.shift(&s.currentSol)){
        continue;  // ← RETORNA AO INÍCIO DO LOOP (linha 104)
    }
    
    if(s.swap(&s.currentSol)){
        continue;  // ← RETORNA AO INÍCIO DO LOOP (linha 104)
    }
    
    if(s.improveSol(&s.currentSol)){
        continue;  // ← RETORNA AO INÍCIO DO LOOP (linha 104)
    }
    
    if(s.swapRobots(&s.currentSol)){
        continue;  // ← RETORNA AO INÍCIO DO LOOP (linha 104)
    }
    
    if(s.closeRandomDepot(&s.currentSol)){
        continue;  // ← RETORNA AO INÍCIO DO LOOP (linha 104)
    }
    
    // Só chega aqui se NENHUM operador encontrou melhoria
    // Atualiza melhor solução e aplica nova perturbação
    if(s.IsBetterSol(s.currentSol,s.best_sol))
        s.best_sol = s.currentSol;
    else{
        s.currentSol = s.best_sol;
        s.solutionToNodesSet(s.best_sol);
    }
    
    s.perturbation(&s.currentSol,maxDepots);
    n++;  // ← INCREMENTA APENAS QUANDO NÃO HÁ MELHORIAS
}
```

## Diferença em Relação ao Comportamento Inicialmente Descrito

### Comportamento Inicial (Incorreto):
- Aplicar operador → Se encontrar melhoria, incrementar `n` e ir para próximo operador
- Cada operador seria testado apenas uma vez por iteração

### Comportamento Real (Correto):
- Aplicar operador → Se encontrar melhoria, **voltar ao primeiro operador**
- Cada operador é reaplicado **até esgotar todas as melhorias possíveis**
- Só incrementa `n` quando **nenhum** operador consegue mais melhorias

Este comportamento implementa uma estratégia de **intensificação local mais agressiva**, onde cada tipo de movimento é explorado até a exaustão antes de passar para outros tipos de movimento ou aplicar novas perturbações.

