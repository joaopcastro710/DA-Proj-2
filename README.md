# DA-Proj-2

Projeto em C++ desenvolvido para explorar e comparar algoritmos para o Problema do Caixeiro Viajante (TSP) aplicados a vários conjuntos de grafos: toy graphs, grafos totalmente conectados de teste e grafos do "mundo real" com coordenadas geográficas.

O projecto inclui parsers para ficheiros CSV, cálculo de distâncias geográficas (Haversine), várias heurísticas e algoritmos exactos/aproximados para TSP e um menu interativo em consola para seleccionar datasets e algoritmos.

---

## Funcionalidades principais

- Parsers para três tipos de datasets:
  - Toy Graphs (`data/Toy-Graphs/`) — ficheiros CSV com arestas simples.
  - Extra Fully Connected Graphs (`data/Extra_Fully_Connected_Graphs/`) — `nodes.csv` + `edges_*.csv`.
  - Real World Graphs (`data/Real_World_Graphs/<graphX>/`) — `nodes.csv` + `edges.csv`.
- Implementação do cálculo Haversine para distâncias geográficas (km).
- Algoritmos/Técnicas implementadas:
  - Solução exacta por Backtracking (só para grafos muito pequenos).
  - Heurística baseada na Desigualdade Triangular (MST + pre-order walk).
  - Versão do algoritmo de Christofides (uso de matching heurístico, não Blossom).
  - Heurística Nearest Neighbor para casos reais e grafos não totalmente conectados.
  - Prim (MST), Dijkstra (caminhos mais curtos), detecção conectividade, Euler -> Hamiltonian conversion.
- Menu interativo com medição de tempo de execução e gravação de resultados em `output.txt`.

---

## Estrutura do repositório (principais ficheiros)

- `main.cpp` — ponto de entrada; arranca o menu.
- `menu.h` — interface de consola, leitura de inputs e gravação em `output.txt`.
- `dataParser.h` / `dataParser.cpp` — loaders para CSVs.
- `dataStructures/Graph.h` — estrutura de grafo (vértices, arestas, priority queue).
- `functions.h` / `functions.cpp` — implementação de algoritmos e utilitários TSP.
- `haversine.h` / `haversine.cpp` — cálculo de distância geográfica.
- `CMakeLists.txt` — configuração de build CMake.
- `.gitignore` — exclui a pasta `data/` e builds locais.

> Observação: a pasta `data/` está ignorada e não está incluída no repositório. Tem de fornecer ou gerar os ficheiros CSV para testar.

---

## Como compilar e executar

1. Criar diretório de build e entrar nele:
   - mkdir build && cd build
2. Gerar ficheiros de construção com CMake:
   - cmake ..
3. Compilar:
   - make
4. Executar:
   - ./untitled
5. Use o menu para seleccionar grupo de datasets, dataset e algoritmo. Os resultados serão mostrados em consola e gravados em `output.txt` (caminho relativo conforme execução).

---

## Formato esperado dos ficheiros CSV

- Toy graphs (`data/Toy-Graphs/<file>.csv`): (header opcional na primeira linha, é ignorada)
  Exemplo de linhas:
  ```
  origin,destination,distance
  0,1,12.5
  1,2,8.3
  2,0,15.0
  ```

- Extra Fully Connected:
  - `nodes.csv`:
    ```
    id,longitude,latitude
    0,-8.611,41.149
    1,-9.142,38.711
    ```
  - `edges_XXX.csv`:
    ```
    org,dest,distance
    0,1,250.3
    1,2,120.0
    ```

- Real World Graphs (`data/Real_World_Graphs/graphX/`):
  - `nodes.csv` (mesmo formato que acima)
  - `edges.csv` (mesmo formato que toy graphs, primeira linha de header é ignorada)

---


