# Guia Prático de Execução - matUAVs

## 🚀 Guia Rápido de Início

### Pré-requisitos Essenciais

#### Windows (x64)
- ✅ Windows 10/11 (x64)
- ✅ Visual Studio Code (recomendado)
- ✅ Microsoft Visual C++ 2022 BuildTools
- ✅ Gurobi Optimizer 12.0+
- ✅ CMake 3.20+ (opcional)

#### Linux (x64)
- ✅ Distribuição Linux (Ubuntu 20.04+, Fedora, etc.)
- ✅ Visual Studio Code (recomendado)
- ✅ GCC 8+ ou Clang 7+
- ✅ Gurobi Optimizer 12.0+
- ✅ CMake 3.20+
- ✅ Bibliotecas: pthread, libm

### ⚡ Execução em 3 Passos

#### Windows

1. **Abrir o Projeto**
   ```cmd
   cd matUAVs
   code .
   ```

2. **Compilar e Executar**
   ```
   Ctrl+Shift+P → "Tasks: Run Build Task"
   ```
   ou
   ```cmd
   .vscode\build.bat
   ```

3. **Aguardar Resultados**
   - O programa executa automaticamente após a compilação
   - Resultados aparecem em `output/`

#### Linux

1. **Abrir o Projeto**
   ```bash
   cd matUAVs
   code .
   ```

2. **Compilar**
   ```bash
   mkdir build && cd build
   cmake .. -DCMAKE_BUILD_TYPE=Release
   make -j$(nproc)
   cd ..
   ```

3. **Executar e Aguardar Resultados**
   ```bash
   ./bin/main input.txt
   ```
   - Resultados aparecem em `output/`

---

## 📝 Preparação dos Dados de Entrada

### Estrutura Básica do Arquivo
```txt
@base
Base_1@<coordenada_x> <coordenada_y>

@target
<x1> <y1>
<x2> <y2>

@robots_configuration
configId:<nome_config>
maxVel:<velocidade_maxima>
maxFuel:<combustivel_maximo>
prop:<propriedade>

@robot
robotId:<id_robo>
configId:<referencia_config>

@robot_base
<id_robo>@<id_base>

@param
nexec:<numero_execucoes>
m:<iteracoes_por_target>
n:<aplicacoes_vnd>
cvl_subset:<linhas_cobertura>
```

### ✏️ Exemplo: Cenário Simples
```txt
@base
Base_Central@5000 5000

@target
1000 1000
2000 2000
3000 3000
4000 4000

@robots_configuration
configId:Drone_Padrao
maxVel:20
maxFuel:2000
prop:2.5

@robot
robotId:Drone_01
configId:Drone_Padrao

robotId:Drone_02
configId:Drone_Padrao

@robot_base
Drone_01@Base_Central
Drone_02@Base_Central

@param
nexec:3
m:10
n:5
cvl_subset:2
```

---

## 🎮 Modos de Execução

### 1. Execução Padrão

#### Windows
```cmd
# Usar arquivo input.txt padrão
bin\main.exe

# Ou especificar explicitamente
bin\main.exe input.txt
```

#### Linux
```bash
# Usar arquivo input.txt padrão
./bin/main

# Ou especificar explicitamente
./bin/main input.txt
```

### 2. Execução com Arquivo Personalizado

#### Windows
```cmd
# Especificar arquivo customizado
bin\main.exe meu_cenario.txt
```

#### Linux
```bash
# Especificar arquivo customizado
./bin/main meu_cenario.txt
```

### 3. Execução via VS Code
- **F5**: Debug mode (requer configuração de launch.json)
- **Ctrl+F5**: Run sem debug
- **Ctrl+Shift+P**: Build Task (compila e pode executar)

---

## 🔧 Configurações Avançadas

### Ajuste de Performance
```txt
@param
nexec:1      # ⚡ Execução única (rápida)
m:5          # 🔄 Poucas iterações (teste)
n:3          # 🔧 VND simplificado
cvl_subset:1 # 📏 Cobertura mínima
```

### Configuração de Produção
```txt
@param
nexec:10     # 🎯 Múltiplas execuções
m:50         # 🔄 Exploração extensa
n:8          # 🔧 VND completo
cvl_subset:5 # 📏 Cobertura máxima
```

### Cenários de Teste Rápido
```txt
@param
nexec:1
m:3
n:2
cvl_subset:1
```

---

## 📊 Interpretando Resultados

### Saída no Console

A saída é idêntica em ambas as plataformas:

```
Program: main.exe              # Windows
Program: main                  # Linux

Start Reading: input.txt

Node Information:
Base nodes: 1
Target nodes: 4
Depot nodes: 0

Robot Information:
Drone_01 (Drone_Padrao): Vel=20, Fuel=2000
Drone_02 (Drone_Padrao): Vel=20, Fuel=2000

Optimization Progress:
[Iteração 1] Cost: 1500.23 | Coverage: 85%
[Iteração 2] Cost: 1450.67 | Coverage: 90%
...
Final Best Solution: Cost=1420.45 | Coverage=95%
```

### Estrutura de Arquivos Gerados

A estrutura é idêntica em ambas as plataformas (usando `std::filesystem`):

```
output/
├── 16-09-2024-14-30-25/    # Timestamp da execução
│   ├── nodes/
│   │   └── nodes.txt        # Coordenadas e tipos dos nós
│   └── solutions/
│       ├── sol_1/           # Primeira solução
│       │   ├── path_0/      # Rota do robô 0
│       │   └── path_1/      # Rota do robô 1
│       └── sol_2/           # Segunda solução
└── logs/
    └── gurobi_160924_143025.log  # Log detalhado do solver
```

### 📈 Métricas de Qualidade

#### Indicadores de Sucesso
- ✅ **Convergência**: Custo diminui ao longo das iterações
- ✅ **Cobertura Alta**: > 90% dos targets visitados
- ✅ **Balanceamento**: Distribuição equilibrada entre robôs
- ✅ **Feasibilidade**: Todas as restrições respeitadas

#### Sinais de Problemas
- ❌ **Estagnação**: Custo não melhora por muitas iterações
- ❌ **Cobertura Baixa**: < 70% dos targets visitados
- ❌ **Infeasibilidade**: Restrições de combustível violadas
- ❌ **Desbalanceamento**: Um robô com toda a carga

---

## 🛠️ Troubleshooting

### Windows

#### Problema: Erro de Compilação - Gurobi não encontrado
```
ERRO: Gurobi não encontrado
```
**Solução**:
1. Verificar instalação do Gurobi em `C:\gurobi1202\win64`
2. Confirmar licença ativa
3. Atualizar paths no `build.bat` se instalado em local diferente
4. Ou definir variável de ambiente:
   ```cmd
   set GUROBI_HOME=C:\gurobi1202\win64
   ```

#### Problema: LINK error LNK1181
```
LINK : fatal error LNK1181: cannot open input file 'gurobi120.lib'
```
**Solução**:
1. Usar "Developer Command Prompt for VS 2022"
2. Ou executar antes da compilação:
   ```cmd
   call "C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\VC\Auxiliary\Build\vcvars64.bat"
   ```

#### Problema: Executável não encontrado
```
'bin\main.exe' não é reconhecido como um comando interno
```
**Solução**:
1. Verificar se a compilação foi bem-sucedida
2. Executar de dentro do diretório do projeto
3. Ou usar caminho completo

---

### Linux

#### Problema: Erro de Compilação - Gurobi não encontrado
```
fatal error: gurobi_c++.h: No such file or directory
```
**Solução**:
1. Verificar instalação do Gurobi em `/opt/gurobi1202/linux64`
2. Definir variável GUROBI_HOME:
   ```bash
   export GUROBI_HOME=/opt/gurobi1202/linux64
   ```
3. Recompilar com CMake

#### Problema: Biblioteca compartilhada não encontrada
```
error while loading shared libraries: libgurobi120.so: cannot open shared object file
```
**Solução**:
```bash
export LD_LIBRARY_PATH=/opt/gurobi1202/linux64/lib:$LD_LIBRARY_PATH
```

Para tornar permanente, adicione ao `~/.bashrc`:
```bash
echo 'export GUROBI_HOME=/opt/gurobi1202/linux64' >> ~/.bashrc
echo 'export LD_LIBRARY_PATH=$GUROBI_HOME/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc
```

#### Problema: Compilador não suporta C++17
```
error: 'filesystem' in namespace 'std' does not name a type
```
**Solução**:
1. Atualizar GCC para versão 8 ou superior:
   ```bash
   # Ubuntu/Debian
   sudo apt update
   sudo apt install g++-8
   
   # Verificar versão
   g++ --version
   ```
2. Ou instalar Clang 7+:
   ```bash
   sudo apt install clang-7
   ```

#### Problema: Permissão negada ao executar
```
bash: ./bin/main: Permission denied
```
**Solução**:
```bash
chmod +x bin/main
./bin/main input.txt
```

---

### Problemas Comuns (Ambas Plataformas)

### Problemas Comuns (Ambas Plataformas)

#### Problema: Solução Inviável
```
No feasible solution found
```
**Solução**:
1. Aumentar `maxFuel` dos robôs
2. Reduzir número de targets
3. Adicionar mais robôs
4. Verificar coordenadas dos nós

#### Problema: Performance Lenta
```
Execution time > 1 hour
```
**Solução**:
1. Reduzir parâmetros `m` e `n` no arquivo de entrada
2. Diminuir `nexec`
3. Usar `cvl_subset` menor
4. Simplificar cenário de teste
5. Compilar em modo Release:
   - Windows CMake: `cmake --build . --config Release`
   - Linux CMake: `cmake .. -DCMAKE_BUILD_TYPE=Release`

#### Problema: Arquivo de Entrada Inválido
```
Error reading input file
```
**Solução**:
1. Verificar formato das seções (@base, @target, etc.)
2. Confirmar sintaxe dos parâmetros
3. Validar coordenadas numéricas
4. Usar template como base

---

## 🎯 Cenários de Uso Comum

### 1. Teste de Desenvolvimento

#### Windows
```cmd
# Cenário mínimo para validar mudanças
bin\main.exe input_test_minimal.txt
```

#### Linux
```bash
# Cenário mínimo para validar mudanças
./bin/main input_test_minimal.txt
```

### 2. Análise de Performance

#### Windows
```cmd
# Múltiplas execuções para estatísticas
bin\main.exe input_performance.txt
```

#### Linux
```bash
# Múltiplas execuções para estatísticas
./bin/main input_performance.txt
```

### 3. Validação de Algoritmo

#### Windows
```cmd
# Cenário conhecido com resultado esperado
bin\main.exe input_validation.txt
```

#### Linux
```bash
# Cenário conhecido com resultado esperado
./bin/main input_validation.txt
```

### 4. Produção Final

#### Windows
```cmd
# Configuração otimizada para resultado final
bin\main.exe input_production.txt
```

#### Linux
```bash
# Configuração otimizada para resultado final
./bin/main input_production.txt
```

---

## 📚 Exemplos Práticos Completos

### Cenário 1: Delivery Urbano
```txt
@base
Centro_Distribuicao@0 0

@target
Endereco_1@100 200
Endereco_2@-150 300
Endereco_3@250 -100
Endereco_4@-200 -150

@robots_configuration
configId:Drone_Delivery
maxVel:25
maxFuel:1500
prop:2.0

@robot
robotId:Drone_A
configId:Drone_Delivery

robotId:Drone_B
configId:Drone_Delivery

@robot_base
Drone_A@Centro_Distribuicao
Drone_B@Centro_Distribuicao

@param
nexec:5
m:15
n:6
cvl_subset:3
```

### Cenário 2: Inspeção Industrial
```txt
@base
Base_Operacoes@1000 1000

@target
Torre_1@500 800
Torre_2@1200 600
Linha_A@800 1200
Linha_B@1400 1400
Edificio_1@600 600

@robots_configuration
configId:Drone_Inspecao
maxVel:18
maxFuel:3000
prop:1.8

@robot
robotId:Inspector_1
configId:Drone_Inspecao

@robot_base
Inspector_1@Base_Operacoes

@param
nexec:3
m:20
n:8
cvl_subset:4
```

---

## 📋 Checklist de Execução

### Antes de Executar
- [ ] Arquivo de entrada validado
- [ ] Gurobi configurado corretamente (incluindo `LD_LIBRARY_PATH` no Linux)
- [ ] Espaço em disco suficiente
- [ ] Parâmetros ajustados para o cenário
- [ ] Compilador e CMake instalados (se usar CMake)

### Durante a Execução
- [ ] Monitorar logs no console
- [ ] Verificar progresso das iterações
- [ ] Observar métricas de qualidade
- [ ] Acompanhar tempo de execução

### Após a Execução
- [ ] Revisar arquivos gerados em `output/`
- [ ] Analisar logs do Gurobi em `logs/`
- [ ] Validar qualidade das soluções
- [ ] Documentar resultados

---

## 🔍 Diferenças entre Plataformas

| Aspecto | Windows | Linux |
|---------|---------|-------|
| **Executável** | `bin\main.exe` | `./bin/main` |
| **Separador de caminho** | `\` (mas `/` funciona) | `/` |
| **Build recomendado** | Script `.vscode\build.bat` ou CMake | CMake |
| **Variáveis de ambiente** | Opcionais | `LD_LIBRARY_PATH` necessária |
| **Permissões de arquivo** | Automáticas | Pode precisar `chmod +x` |

**Importante**: O código C++ e os arquivos de entrada/saída são 100% compatíveis entre plataformas.

---

*Para detalhes técnicos sobre o algoritmo, consulte [FLUXO_EXECUCAO.md](FLUXO_EXECUCAO.md)*

*Para informações de compilação, consulte [COMPILACAO.md](COMPILACAO.md)*