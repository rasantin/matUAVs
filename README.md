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

O CMake é o método canônico de build deste projeto, suportando Windows e Linux de forma consistente.

### Método: CMake 
- Funciona em **Windows** e **Linux**
- Independente de IDE
- Facilita builds reproduzíveis e automação de experimentos

---

## Como Usar

### Windows

#### Via VS Code (recomendado):
- Pressione: `Ctrl+Shift+P` → "Tasks: Run Build Task"

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
